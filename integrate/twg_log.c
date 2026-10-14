#define _POSIX_C_SOURCE 200809L

#include "twg_log.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define TWG_LOG_WORKER_PAUSE_NS UINT64_C(1000000)

static const char *phase_name(TwgLogPhase phase)
{
    switch (phase) {
    case TWG_LOG_PHASE_ZERO:
        return "zero";

    case TWG_LOG_PHASE_RUN:
        return "run";

    case TWG_LOG_PHASE_CALIBRATION:
        return "calibration";

    case TWG_LOG_PHASE_TARE_Z:
        return "tare_z";

    case TWG_LOG_PHASE_TARE_FULL:
        return "tare_full";

    case TWG_LOG_PHASE_CLEAR:
        return "clear";

    case TWG_LOG_PHASE_RESTORE:
        return "restore";

    case TWG_LOG_PHASE_PRE:
    default:
        return "pre";
    }
}

static bool make_stamp(char *out, size_t capacity)
{
    struct timespec now;
    struct tm local;

    if (out == NULL || capacity == 0u ||
        clock_gettime(CLOCK_REALTIME, &now) != 0 ||
        localtime_r(&now.tv_sec, &local) == NULL) {
        return false;
    }

    return snprintf(
               out,
               capacity,
               "%04d%02d%02d_%02d%02d%02d",
               local.tm_year + 1900,
               local.tm_mon + 1,
               local.tm_mday,
               local.tm_hour,
               local.tm_min,
               local.tm_sec) > 0;
}

static bool open_unique(TwgLog *log)
{
    char stamp[32];
    unsigned suffix;

    if (!make_stamp(stamp, sizeof(stamp))) {
        return false;
    }

    for (suffix = 0u; suffix <= 999u; ++suffix) {
        int fd;

        if (suffix == 0u) {
            (void)snprintf(
                log->filename,
                sizeof(log->filename),
                "twg_acq_%s.csv",
                stamp);
        } else {
            (void)snprintf(
                log->filename,
                sizeof(log->filename),
                "twg_acq_%s_%03u.csv",
                stamp,
                suffix);
        }

        fd = open(
            log->filename,
            O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC,
            0644);

        if (fd >= 0) {
            log->stream = fdopen(fd, "w");
            if (log->stream == NULL) {
                int saved = errno;
                (void)close(fd);
                (void)unlink(log->filename);
                errno = saved;
                return false;
            }
            return true;
        }

        if (errno != EEXIST) {
            return false;
        }
    }

    errno = EEXIST;
    return false;
}

static const char *text(const char *s)
{
    return s != NULL ? s : "none";
}

static bool write_header(FILE *stream)
{
    return fprintf(
        stream,
        "schema,row,sched_ns,sched_valid,actual_ns,actual_valid,"
        "deadline,gates_skipped,phase,logger_drops,"
        "imu_epoch,imu_reader_state,imu_ready,imu_valid_mask,"
        "imu_fresh_mask,imu_stale_mask,imu_missing_mask,imu_multi_mask,"
        "rv_seq,rv_host_ns,rv_sensor_us,rv_report_seq,rv_status,"
        "rv_age_valid,rv_age_ns,qw,qx,qy,qz,yaw_rad,pitch_rad,"
        "roll_rad,orientation_err_rad,"
        "acc_seq,acc_host_ns,acc_sensor_us,acc_report_seq,acc_status,"
        "acc_age_valid,acc_age_ns,ax,ay,az,"
        "gyro_seq,gyro_host_ns,gyro_sensor_us,gyro_report_seq,"
        "gyro_status,gyro_age_valid,gyro_age_ns,gx,gy,gz,"
        "pressure_ready,pressure_fresh,pressure_stale,"
        "pressure_age_valid,pressure_age_ns,pressure_driver_state,"
        "pressure_contract,pressure_flags,pressure_seq,"
        "pressure_complete_ns,raw_d1,raw_d2,pressure_mbar,"
        "temperature_c,zero_requested,zero_valid,"
        "zero_valid_samples,zero_rejected_samples,"
        "surface_pressure_mbar,fluid_density_kg_m3,depth_m,"
        "stage_active,stage_event,last_stage,last_result,"
        "last_reason,last_warning,cal_valid,cal_phase,cal_pose,"
        "cal_pose_total,cal_accel_status,cal_gyro_status,"
        "cal_mag_status,cal_rv_status,cal_rv_err_rad,"
        "cal_mag_x_ut,cal_mag_y_ut,cal_mag_z_ut,cal_gate\n") >= 0;
}

static bool write_record(
    FILE *stream,
    const TwgLogRecord *r)
{
    const ImuPublicationRecord_t *i = &r->imu;

    return fprintf(
        stream,
        "%" PRIu32 ",%" PRIu64 ","
        "%" PRIu64 ",%u,%" PRIu64 ",%u,%u,%" PRIu32 ",%s,"
        "%" PRIu64 ","
        "%" PRIu32 ",%d,%u,%u,%u,%u,%u,%u,"
        "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%u,%u,%u,"
        "%" PRIu64 ","
        "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
        "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%u,%u,%u,"
        "%" PRIu64 ",%.9g,%.9g,%.9g,"
        "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%u,%u,%u,"
        "%" PRIu64 ",%.9g,%.9g,%.9g,"
        "%u,%u,%u,%u,%" PRIu64 ",%" PRId32 ","
        "%" PRIu32 ",%" PRIu32 ",%" PRIu64 ",%" PRIu64 ","
        "%" PRIu32 ",%" PRIu32 ",%.9g,%.9g,"
        "%u,%u,%" PRIu64 ",%" PRIu64 ",%.9g,%.9g,%.9g"
        ",%s,%u,%s,%s,%s,%u,%u,%u,%u,%u,%u,%u,%u,%u"
        ",%.9g,%.9g,%.9g,%.9g,%u\n",

        r->schemaVersion,
        r->rowSequence,
        r->scheduledNs,
        r->scheduledValid,
        r->actualNs,
        r->actualValid,
        r->deadlineMissed,
        r->gatesSkippedBefore,
        phase_name(r->phase),
        r->loggerDropsBefore,

        i->configurationEpoch,
        (int)i->readerState,
        i->sampleReady,
        i->validMask,
        i->freshMask,
        i->staleMask,
        i->missingMask,
        i->multiUpdateMask,

        i->rotation.groupEventSeq,
        i->rotation.hostDecodeNs,
        i->rotation.sensorTimeUs,
        i->rotation.deviceReportSeq,
        i->rotation.rawStatus,
        i->rotation.ageValid,
        i->rotation.ageNs,

        (double)i->qw,
        (double)i->qx,
        (double)i->qy,
        (double)i->qz,
        (double)i->yawRad,
        (double)i->pitchRad,
        (double)i->rollRad,
        (double)i->orientationErrRad,

        i->accel.groupEventSeq,
        i->accel.hostDecodeNs,
        i->accel.sensorTimeUs,
        i->accel.deviceReportSeq,
        i->accel.rawStatus,
        i->accel.ageValid,
        i->accel.ageNs,

        (double)i->ax,
        (double)i->ay,
        (double)i->az,

        i->gyro.groupEventSeq,
        i->gyro.hostDecodeNs,
        i->gyro.sensorTimeUs,
        i->gyro.deviceReportSeq,
        i->gyro.rawStatus,
        i->gyro.ageValid,
        i->gyro.ageNs,

        (double)i->gx,
        (double)i->gy,
        (double)i->gz,

        r->pressureReady,
        r->pressureFresh,
        r->pressureStale,
        r->pressureAgeValid,
        r->pressureAgeNs,
        r->pressureDriverState,

        r->pressure.contract_version,
        r->pressure.status_flags,
        r->pressure.measurement_sequence,
        r->pressure.measurement_complete_time_ns,
        r->pressure.raw_pressure_d1,
        r->pressure.raw_temperature_d2,
        r->pressure.pressure_mbar,
        r->pressure.temperature_c,

        r->zeroRequested,
        r->zeroValid,
        r->zeroValidSamples,
        r->zeroRejectedSamples,
        r->surfacePressureMbar,
        r->fluidDensityKgM3,
        r->depthM,

        text(r->stageActive),
        r->stageEvent,
        text(r->lastStage),
        text(r->lastResult),
        text(r->lastReason),
        r->lastWarning,
        r->calValid,
        r->calPhase,
        r->calPose,
        r->calPoseTotal,
        r->calAccel,
        r->calGyro,
        r->calMag,
        r->calRv,
        (double)r->calRvErrRad,
        (double)r->calMagXuT,
        (double)r->calMagYuT,
        (double)r->calMagZuT,
        r->calGate) >= 0;
}

static void update_high_water(TwgLog *log, uint32_t value)
{
    uint32_t current =
        atomic_load_explicit(
            &log->highWater, memory_order_relaxed);

    while (value > current &&
           !atomic_compare_exchange_weak_explicit(
               &log->highWater,
               &current,
               value,
               memory_order_relaxed,
               memory_order_relaxed)) {
    }
}

static void pause_worker(void)
{
    struct timespec pause;

    pause.tv_sec = 0;
    pause.tv_nsec = (long)TWG_LOG_WORKER_PAUSE_NS;

    while (nanosleep(&pause, &pause) != 0 &&
           errno == EINTR) {
    }
}

static void *worker_main(void *opaque)
{
    TwgLog *log = opaque;

    for (;;) {
        uint64_t tail = atomic_load_explicit(
            &log->tail, memory_order_relaxed);
        uint64_t head = atomic_load_explicit(
            &log->head, memory_order_acquire);

        if (tail == head) {
            if (atomic_load(&log->stopRequested)) {
                break;
            }

            pause_worker();
            continue;
        }

        if (!atomic_load(&log->writeFailed) &&
            !write_record(
                log->stream,
                &log->ring[tail % TWG_LOG_QUEUE_CAPACITY])) {
            atomic_store(&log->writeFailed, true);
        } else if (!atomic_load(&log->writeFailed)) {
            atomic_fetch_add(&log->written, 1u);
        }

        atomic_store_explicit(
            &log->tail, tail + 1u, memory_order_release);
    }

    if (fflush(log->stream) != 0) {
        atomic_store(&log->writeFailed, true);
    }

    atomic_store(&log->workerDone, true);
    return NULL;
}

bool twg_log_init(TwgLog *log)
{
    if (log == NULL) {
        return false;
    }

    memset(log, 0, sizeof(*log));

    atomic_init(&log->head, 0u);
    atomic_init(&log->tail, 0u);
    atomic_init(&log->enqueued, 0u);
    atomic_init(&log->written, 0u);
    atomic_init(&log->dropped, 0u);
    atomic_init(&log->highWater, 0u);

    atomic_init(&log->stopRequested, false);
    atomic_init(&log->workerDone, false);
    atomic_init(&log->writeFailed, false);

    log->initialized = true;
    return true;
}

bool twg_log_start(TwgLog *log)
{
    pthread_attr_t attr;
    struct sched_param param;
    int rc;

    if (log == NULL ||
        !log->initialized ||
        log->workerStarted ||
        !open_unique(log)) {
        return false;
    }

    if (!write_header(log->stream) ||
        fflush(log->stream) != 0) {
        (void)fclose(log->stream);
        log->stream = NULL;
        return false;
    }

    memset(&param, 0, sizeof(param));

    if (pthread_attr_init(&attr) != 0) {
        (void)fclose(log->stream);
        log->stream = NULL;
        return false;
    }

    rc = pthread_attr_setinheritsched(
        &attr, PTHREAD_EXPLICIT_SCHED);

    if (rc == 0) {
        rc = pthread_attr_setschedpolicy(&attr, SCHED_OTHER);
    }

    if (rc == 0) {
        rc = pthread_attr_setschedparam(&attr, &param);
    }

    if (rc == 0) {
        rc = pthread_create(
            &log->worker, &attr, worker_main, log);
    }

    (void)pthread_attr_destroy(&attr);

    if (rc != 0) {
        (void)fclose(log->stream);
        log->stream = NULL;
        return false;
    }

    log->workerStarted = true;
    return true;
}

bool twg_log_enqueue(
    TwgLog *log,
    const TwgLogRecord *record)
{
    uint64_t head;
    uint64_t tail;
    uint64_t count;

    if (log == NULL ||
        record == NULL ||
        !log->workerStarted ||
        record->schemaVersion != TWG_LOG_SCHEMA_VERSION ||
        atomic_load(&log->stopRequested)) {
        return false;
    }

    head = atomic_load_explicit(
        &log->head, memory_order_relaxed);
    tail = atomic_load_explicit(
        &log->tail, memory_order_acquire);

    if (head - tail >= TWG_LOG_QUEUE_CAPACITY) {
        atomic_fetch_add(&log->dropped, 1u);
        return false;
    }

    log->ring[head % TWG_LOG_QUEUE_CAPACITY] = *record;

    atomic_store_explicit(
        &log->head, head + 1u, memory_order_release);
    atomic_fetch_add(&log->enqueued, 1u);

    count = head + 1u - tail;
    if (count > UINT32_MAX) {
        count = UINT32_MAX;
    }

    update_high_water(log, (uint32_t)count);
    return true;
}

bool twg_log_stop(TwgLog *log)
{
    bool ok = true;

    if (log == NULL || !log->initialized) {
        return false;
    }

    if (log->workerStarted) {
        atomic_store(&log->stopRequested, true);

        if (pthread_join(log->worker, NULL) != 0) {
            return false;
        }

        log->workerStarted = false;
    }

    if (log->stream != NULL) {
        int fd = fileno(log->stream);

        if (fflush(log->stream) != 0) {
            ok = false;
        }

        if (fd >= 0 && fsync(fd) != 0) {
            ok = false;
        }

        if (fclose(log->stream) != 0) {
            ok = false;
        }

        log->stream = NULL;
    }

    if (atomic_load(&log->writeFailed)) {
        ok = false;
    }

    return ok;
}

bool twg_log_write_failed(const TwgLog *log)
{
    return log != NULL &&
           atomic_load(&log->writeFailed);
}

const char *twg_log_filename(const TwgLog *log)
{
    return log != NULL ? log->filename : "";
}

uint64_t twg_log_enqueued(const TwgLog *log)
{
    return log != NULL
             ? atomic_load(&log->enqueued)
             : 0u;
}

uint64_t twg_log_written(const TwgLog *log)
{
    return log != NULL
             ? atomic_load(&log->written)
             : 0u;
}

uint64_t twg_log_dropped(const TwgLog *log)
{
    return log != NULL
             ? atomic_load(&log->dropped)
             : 0u;
}

uint32_t twg_log_high_water(const TwgLog *log)
{
    return log != NULL
             ? atomic_load(&log->highWater)
             : 0u;
}
