/*
 * validate_logger.c — bounded-ring CSV logger implementation.
 *
 * Uses a single-producer single-consumer ring buffer between the RT
 * 100 Hz capture thread and the SCHED_OTHER file writer thread.
 * Push drops the incoming record if the ring is full, incrementing a
 * drop counter that is logged in every row so loss is auditable.
 *
 * Part of the sensor_validate binary (see Makefile).
 */

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <inttypes.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "validation/validate_logger.h"

/* Ring capacity: 2048 records @ 100 Hz = ~20 seconds of buffering */
#define RING_SIZE 2048u
#define RING_MASK (RING_SIZE - 1u)

static CsvRecord_t sRing[RING_SIZE];
static volatile uint32_t sHead = 0;  /* write pointer (RT thread only) */
static volatile uint32_t sTail = 0;  /* read pointer (logger thread only) */
static volatile uint64_t sDrops = 0;

static pthread_t       sThread;
static volatile bool   sRunning = false;
static FILE           *sFp      = NULL;

static uint64_t sRecordsWritten = 0;
static uint64_t sT0_ns          = 0;
static uint64_t sTLast_ns       = 0;

static uint64_t nowns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000000000ull) + (uint64_t)ts.tv_nsec;
}

static void writeRow(FILE *fp, const CsvRecord_t *r)
{
    fprintf(fp,
            "%" PRIu64 ","
            "%" PRId64 ",%.4f,%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ","
            "%" PRIu64 ",%" PRIu64 ",%" PRIu32 ",%" PRIu8 ",%" PRIu8 ","
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,"
            "%" PRIu64 ",%" PRIu64 ",%" PRIu32 ",%" PRIu8 ",%" PRIu8 ","
            "%.4f,%.4f,%.4f,"
            "%" PRIu64 ",%" PRIu64 ",%" PRIu32 ",%" PRIu8 ",%" PRIu8 ","
            "%.4f,%.4f,%.4f,"
            "0x%02x,%" PRIu64 "\n",
            /* tick timestamp */
            r->host_ts_ns,
            /* encoder */
            r->enc_count,
            r->enc_angle_deg,
            r->enc_event_ts_ns,
            r->enc_x_pulses,
            r->enc_invalid,
            r->enc_edges_ab,
            /* RV metadata */
            r->imu.rv.sensor_ts_us,
            r->imu.rv.host_ts_ns,
            r->imu.rv.seq,
            r->imu.rv.report_seq,
            r->imu.rv.status,
            /* RV payload */
            r->imu.rv_qw, r->imu.rv_qx, r->imu.rv_qy, r->imu.rv_qz,
            r->imu.yaw,   r->imu.pitch, r->imu.roll,
            r->imu.rvErrRad,
            /* Accel metadata */
            r->imu.accel.sensor_ts_us,
            r->imu.accel.host_ts_ns,
            r->imu.accel.seq,
            r->imu.accel.report_seq,
            r->imu.accel.status,
            /* Accel payload */
            r->imu.ax, r->imu.ay, r->imu.az,
            /* Gyro metadata */
            r->imu.gyro.sensor_ts_us,
            r->imu.gyro.host_ts_ns,
            r->imu.gyro.seq,
            r->imu.gyro.report_seq,
            r->imu.gyro.status,
            /* Gyro payload */
            r->imu.gx, r->imu.gy, r->imu.gz,
            /* Flags & drops */
            r->imu.validMask,
            r->drops);
}

static void *loggerThread(void *arg)
{
    (void)arg;

    while (sRunning || (sTail != sHead)) {
        if (sTail == sHead) {
            /* Ring empty: yield briefly */
            usleep(2000); /* 2 ms */
            continue;
        }

        uint32_t t = sTail;
        writeRow(sFp, &sRing[t & RING_MASK]);
        sTLast_ns = sRing[t & RING_MASK].host_ts_ns;
        sRecordsWritten++;
        __sync_synchronize();
        sTail = t + 1;
    }

    return NULL;
}

int csv_log_start(const char *path, double arm_len_m, const char *notes)
{
    if (sRunning || sFp != NULL) {
        return -1;
    }

    sFp = fopen(path, "w");
    if (!sFp) {
        return -1;
    }

    /* Line buffering so tail -f works cleanly during capture */
    setvbuf(sFp, NULL, _IOLBF, 0);

    /* Capture current time for the header */
    time_t now = time(NULL);
    struct tm tm_buf;
    char time_str[64] = "unknown";
    if (localtime_r(&now, &tm_buf)) {
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S %z", &tm_buf);
    }

    /* Metadata comment block */
    fprintf(sFp,
            "# AMT102 Encoder vs BNO085 IMU Validation Capture\n"
            "# Date:            %s\n"
            "# Encoder:         AMT102-V, 2048 PPR (8192 counts/rev in x4)\n"
            "# IMU:             BNO085 (SH-2) over SPI @ 3 MHz\n"
            "# Clocks:          host_ts_ns, rv_host_ts_ns, accel_host_ts_ns, gyro_host_ts_ns, enc_event_ts_ns\n"
            "#                  are all CLOCK_MONOTONIC nanoseconds.\n"
            "#                  sensor_ts_us fields are BNO085 device timestamps (microseconds).\n"
            "# Angle units:     enc_angle_deg in degrees; yaw, pitch, roll in radians; rvErrRad in radians.\n"
            "# Motion units:    ax,ay,az in m/s^2; gx,gy,gz in rad/s.\n"
            "# Mechanical arm:  %.4f m (axis to IMU center)\n"
            "# Run notes:       %s\n"
            "# ------------------------------------------------------------------\n",
            time_str,
            arm_len_m,
            (notes && notes[0]) ? notes : "none");

    /* Column header row (no '#' prefix — pandas read_csv reads this as header) */
    fprintf(sFp,
            "host_ts_ns,"
            "enc_count,enc_angle_deg,enc_event_ts_ns,enc_x_pulses,enc_invalid,enc_edges_ab,"
            "rv_sensor_ts_us,rv_host_ts_ns,rv_seq,rv_report_seq,rv_status,"
            "rv_qw,rv_qx,rv_qy,rv_qz,yaw,pitch,roll,rv_accuracy_rad,"
            "accel_sensor_ts_us,accel_host_ts_ns,accel_seq,accel_report_seq,accel_status,"
            "ax,ay,az,"
            "gyro_sensor_ts_us,gyro_host_ts_ns,gyro_seq,gyro_report_seq,gyro_status,"
            "gx,gy,gz,"
            "valid_mask,drops\n");

    sHead           = 0;
    sTail           = 0;
    sDrops          = 0;
    sRecordsWritten = 0;
    sT0_ns          = nowns();
    sTLast_ns       = sT0_ns;
    sRunning        = true;

    if (pthread_create(&sThread, NULL, loggerThread, NULL) != 0) {
        sRunning = false;
        fclose(sFp);
        sFp = NULL;
        return -1;
    }

    return 0;
}

int csv_log_push(const CsvRecord_t *rec)
{
    uint32_t h = sHead;
    uint32_t t = sTail;

    if ((h - t) >= RING_SIZE) {
        /* Full: drop the record, bump drop counter */
        __sync_fetch_and_add(&sDrops, 1);
        return 1;
    }

    sRing[h & RING_MASK] = *rec;
    __sync_synchronize();
    sHead = h + 1;
    return 0;
}

uint64_t csv_log_dropped(void)
{
    return sDrops;
}

void csv_log_stop(CsvLogStats_t *out)
{
    if (!sFp) {
        if (out) memset(out, 0, sizeof(*out));
        return;
    }

    /* Signal logger thread to drain and exit */
    sRunning = false;
    pthread_join(sThread, NULL);

    /* Compute duration */
    double duration = 0.0;
    if (sRecordsWritten > 0 && sTLast_ns > sT0_ns) {
        duration = (double)(sTLast_ns - sT0_ns) / 1e9;
    }

    /* Write footer metadata */
    fprintf(sFp,
            "# ------------------------------------------------------------------\n"
            "# Capture complete.\n"
            "# Records written: %" PRIu64 "\n"
            "# Records dropped: %" PRIu64 "\n"
            "# Duration:        %.3f s\n",
            sRecordsWritten,
            sDrops,
            duration);

    fclose(sFp);
    sFp = NULL;

    if (out) {
        out->records_written = sRecordsWritten;
        out->records_dropped = sDrops;
        out->duration_sec    = duration;
    }
}
