#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "app/imu_cli.h"
#include "app/imu_contract.h"
#include "app/imu_publish.h"
#include "app/imu_session.h"

#include "ms5837_contract.h"
#include "ms5837_driver.h"
#include "ms5837_hal_rpi.h"
#include "ms5837_math.h"

#include "rt/realtime.h"
#include "rt/rt_policy.h"

#include "twg_log.h"
#include "twg_stage.h"
#include "twg_term.h"

#define TWG_RT_PRIORITY         90
#define TWG_SERVICE_PERIOD_SEC  0.001
#define TWG_SERVICE_PERIOD_NS   UINT64_C(1000000)
#define TWG_PUBLISH_TICKS       UINT64_C(10)

#define TWG_STARTUP_ATTEMPTS    3u
#define TWG_STARTUP_RETRY_NS    UINT64_C(200000000)

#define TWG_ZERO_DURATION_NS    UINT64_C(5000000000)
#define TWG_ZERO_MIN_SAMPLES    350u
#define TWG_ZERO_MAX_SAMPLES    512u
#define TWG_ZERO_TRIM_FRACTION  0.10
#define TWG_DENSITY_KGM3        1000.0

static volatile sig_atomic_t sStop;
static volatile sig_atomic_t sSignal;

typedef struct {
    Ms5837Driver_t driver;
    Ms5837HalRpi_t halContext;
    Ms5837Hal_t hal;

    BaroDriverStats_t stats;
    BaroSample_t latest;
    bool haveLatest;

    uint64_t previousSequence;
    bool previousValid;

    bool zeroActive;
    uint64_t zeroStartedNs;
    double zeroSamples[TWG_ZERO_MAX_SAMPLES];
    size_t zeroValidSamples;
    size_t zeroRejectedSamples;

    bool zeroValid;
    double surfacePressureMbar;
    double fluidDensityKgM3;
} PressureContext;

typedef struct {
    ImuPublisherState_t imuPublisher;
    ImuPublishGate_t gate;

    uint64_t rowSequence;
    uint64_t nextTurnTicks;
    uint64_t acquiredNs;
    uint64_t acquisitionDeadlineNs;

    bool windowOpen;
    bool windowJustOpened;
    bool windowClosed;
} PublicationContext;

static void request_stop(int number)
{
    sSignal = number;
    sStop = 1;
}

static bool install_signals(void)
{
    struct sigaction action;

    memset(&action, 0, sizeof(action));
    action.sa_handler = request_stop;
    sigemptyset(&action.sa_mask);

    return sigaction(SIGINT, &action, NULL) == 0 &&
           sigaction(SIGTERM, &action, NULL) == 0;
}

static bool monotonic_ns(uint64_t *out)
{
    struct timespec now;

    if (out == NULL ||
        clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return false;
    }

    *out =
        (uint64_t)now.tv_sec * UINT64_C(1000000000) +
        (uint64_t)now.tv_nsec;

    return true;
}

static int sleep_relative(uint64_t durationNs)
{
    struct timespec pause;

    pause.tv_sec =
        (time_t)(durationNs / UINT64_C(1000000000));
    pause.tv_nsec =
        (long)(durationNs % UINT64_C(1000000000));

    while (nanosleep(&pause, &pause) != 0) {
        if (errno != EINTR) {
            return -errno;
        }

        if (sStop) {
            return -EINTR;
        }
    }

    return 0;
}

static bool pressure_recovering(
    const PressureContext *pressure)
{
    Ms5837DriverState_t state =
        ms5837_driver_state(&pressure->driver);

    return state >= MS5837_STATE_RECOVERY_BACKOFF &&
           state <= MS5837_STATE_RECOVERY_VALIDATE;
}

static bool pressure_initialize(
    PressureContext *pressure)
{
    unsigned attempt;
    int rc = MS5837_DRIVER_ERR_NOT_INITIALIZED;

    ms5837_hal_rpi_configure(
        &pressure->halContext,
        MS5837_DEFAULT_I2C_BUS,
        MS5837_I2C_ADDRESS);

    pressure->hal =
        ms5837_hal_rpi_make(&pressure->halContext);

    for (attempt = 1u;
         attempt <= TWG_STARTUP_ATTEMPTS;
         ++attempt) {
        rc = ms5837_driver_init(
            &pressure->driver,
            &pressure->hal);

        if (rc == MS5837_DRIVER_OK) {
            return true;
        }

        fprintf(
            stderr,
            "pressure init attempt %u/%u failed: %d\n",
            attempt,
            TWG_STARTUP_ATTEMPTS,
            rc);

        if (attempt < TWG_STARTUP_ATTEMPTS &&
            sleep_relative(TWG_STARTUP_RETRY_NS) < 0) {
            break;
        }
    }

    return false;
}

static bool pressure_service(
    PressureContext *pressure,
    uint64_t nowNs,
    bool *sampleCompleted)
{
    BaroSample_t completed;
    int rc;

    *sampleCompleted = false;

    rc = ms5837_driver_service(
        &pressure->driver,
        nowNs,
        &pressure->stats,
        &completed);

    if (rc == MS5837_SERVICE_SAMPLE_READY) {
        pressure->latest = completed;
        pressure->haveLatest = true;
        *sampleCompleted = true;
        return true;
    }

    if (rc < 0 &&
        !pressure_recovering(pressure) &&
        ms5837_driver_state(&pressure->driver) !=
            MS5837_STATE_STOPPED) {
        return false;
    }

    return true;
}

static bool pressure_trigger_if_idle(
    PressureContext *pressure)
{
    int rc;

    if (ms5837_driver_state(&pressure->driver) !=
        MS5837_STATE_IDLE) {
        return true;
    }

    rc = ms5837_driver_trigger(
        &pressure->driver,
        &pressure->stats);

    return rc >= 0 || pressure_recovering(pressure);
}

static bool pressure_sample_acceptable(
    const BaroSample_t *sample)
{
    uint32_t rejectMask =
        BARO_STATUS_NOT_READY |
        BARO_STATUS_RAW_INVALID |
        BARO_STATUS_PRESSURE_INVALID |
        BARO_STATUS_TEMPERATURE_EXTENDED;

    return sample != NULL &&
           sample->contract_version ==
               BARO_SAMPLE_CONTRACT_VERSION &&
           (sample->status_flags & rejectMask) == 0u &&
           isfinite(sample->pressure_mbar);
}

static int compare_double(
    const void *left,
    const void *right)
{
    double a = *(const double *)left;
    double b = *(const double *)right;

    return a < b ? -1 : a > b ? 1 : 0;
}

static void pressure_zero_begin(
    PressureContext *pressure,
    uint64_t nowNs)
{
    pressure->zeroActive = true;
    pressure->zeroStartedNs = nowNs;
    pressure->zeroValidSamples = 0u;
    pressure->zeroRejectedSamples = 0u;

    puts("pressure zero collecting for five seconds...");
}

static void pressure_zero_clear(
    PressureContext *pressure)
{
    pressure->zeroActive = false;
    pressure->zeroStartedNs = 0u;
    pressure->zeroValidSamples = 0u;
    pressure->zeroRejectedSamples = 0u;
    pressure->zeroValid = false;
    pressure->surfacePressureMbar = NAN;

    puts("pressure reference cleared; depth is now NaN.");
}

static bool pressure_zero_finish(
    PressureContext *pressure)
{
    size_t trim;
    size_t begin;
    size_t end;
    size_t i;
    double sum = 0.0;

    pressure->zeroActive = false;

    if (pressure->zeroValidSamples <
        TWG_ZERO_MIN_SAMPLES) {
        pressure->zeroValid = false;
        pressure->surfacePressureMbar = NAN;

        fprintf(
            stderr,
            "pressure zero failed: only %zu valid samples"
            " (need %u)\n",
            pressure->zeroValidSamples,
            TWG_ZERO_MIN_SAMPLES);

        return false;
    }

    qsort(
        pressure->zeroSamples,
        pressure->zeroValidSamples,
        sizeof(pressure->zeroSamples[0]),
        compare_double);

    trim = (size_t)(
        (double)pressure->zeroValidSamples *
        TWG_ZERO_TRIM_FRACTION);

    begin = trim;
    end = pressure->zeroValidSamples - trim;

    if (begin >= end) {
        pressure->zeroValid = false;
        pressure->surfacePressureMbar = NAN;
        return false;
    }

    for (i = begin; i < end; ++i) {
        sum += pressure->zeroSamples[i];
    }

    pressure->surfacePressureMbar =
        sum / (double)(end - begin);

    pressure->zeroValid =
        isfinite(pressure->surfacePressureMbar);

    printf(
        "pressure zero complete:"
        " valid=%zu rejected=%zu"
        " surface_mbar=%.4f\n",
        pressure->zeroValidSamples,
        pressure->zeroRejectedSamples,
        pressure->surfacePressureMbar);

    return pressure->zeroValid;
}

static void pressure_zero_service(
    PressureContext *pressure,
    bool sampleCompleted,
    uint64_t nowNs)
{
    if (!pressure->zeroActive) {
        return;
    }

    if (sampleCompleted) {
        if (pressure_sample_acceptable(&pressure->latest) &&
            pressure->zeroValidSamples <
                TWG_ZERO_MAX_SAMPLES) {
            pressure->zeroSamples[
                pressure->zeroValidSamples++] =
                    pressure->latest.pressure_mbar;
        } else {
            ++pressure->zeroRejectedSamples;
        }
    }

    if (nowNs - pressure->zeroStartedNs >=
        TWG_ZERO_DURATION_NS) {
        (void)pressure_zero_finish(pressure);
    }
}

static bool publication_begin(
    PublicationContext *publication,
    uint64_t nowNs)
{
    ImuSampleSnapshot_t baseline;

    if (!imu_session_get_snapshot(&baseline) ||
        baseline.readerState !=
            IMU_READER_STATE_OPERATIONAL ||
        !imu_publish_begin_window(
            &publication->imuPublisher,
            &baseline) ||
        !imu_publish_gate_begin(
            &publication->gate,
            nowNs + TWG_SERVICE_PERIOD_NS,
            true,
            UINT64_C(86400) * UINT64_C(1000000000))) {
        return false;
    }

    publication->windowOpen = true;
    publication->windowJustOpened = true;
    publication->nextTurnTicks = 1u;

    return true;
}

static TwgLogPhase log_phase(
    TwgStageMode mode)
{
    switch (mode) {
    case TWG_STAGE_MODE_CALIBRATION:
        return TWG_LOG_PHASE_CALIBRATION;

    case TWG_STAGE_MODE_TARE_Z:
        return TWG_LOG_PHASE_TARE_Z;

    case TWG_STAGE_MODE_TARE_FULL:
        return TWG_LOG_PHASE_TARE_FULL;

    case TWG_STAGE_MODE_PRESSURE_ZERO:
        return TWG_LOG_PHASE_ZERO;

    case TWG_STAGE_MODE_CLEAR_CONFIRM:
    case TWG_STAGE_MODE_CLEAR_ALL:
        return TWG_LOG_PHASE_CLEAR;

    case TWG_STAGE_MODE_RESTORE:
        return TWG_LOG_PHASE_RESTORE;

    case TWG_STAGE_MODE_ACQUISITION:
    default:
        return TWG_LOG_PHASE_RUN;
    }
}

static void classify_pressure(
    PressureContext *pressure,
    TwgLogRecord *record)
{
    if (!pressure->haveLatest ||
        !pressure_sample_acceptable(
            &pressure->latest)) {
        return;
    }

    record->pressure = pressure->latest;
    record->pressureReady = 1u;

    if (pressure->previousValid &&
        pressure->previousSequence ==
            pressure->latest.measurement_sequence) {
        record->pressureStale = 1u;
    } else {
        record->pressureFresh = 1u;
    }

    pressure->previousValid = true;
    pressure->previousSequence =
        pressure->latest.measurement_sequence;

    if (record->actualValid &&
        pressure->latest.measurement_complete_time_ns != 0u &&
        record->actualNs >=
            pressure->latest.measurement_complete_time_ns) {
        record->pressureAgeValid = 1u;
        record->pressureAgeNs =
            record->actualNs -
            pressure->latest.measurement_complete_time_ns;
    }
}

static bool publication_emit(
    PublicationContext *publication,
    PressureContext *pressure,
    TwgLog *log,
    TwgStageMode mode)
{
    ImuPublishGateDecision_t decision;
    ImuPublishGateStatus_t gateStatus;
    ImuSampleSnapshot_t snapshot;
    TwgLogRecord record;
    uint64_t actualNs = 0u;
    bool actualValid;

    gateStatus = imu_publish_gate_advance(
        &publication->gate,
        publication->nextTurnTicks,
        &decision);

    if (gateStatus == IMU_PUBLISH_GATE_IDLE) {
        return true;
    }

    if (gateStatus != IMU_PUBLISH_GATE_DUE) {
        return false;
    }

    if (!imu_session_get_snapshot(&snapshot)) {
        return false;
    }

    memset(&record, 0, sizeof(record));

    record.schemaVersion = TWG_LOG_SCHEMA_VERSION;
    record.rowSequence = ++publication->rowSequence;
    record.phase = log_phase(mode);

    actualValid = monotonic_ns(&actualNs);

    if (!imu_publish_evaluate_snapshot(
            &publication->imuPublisher,
            &snapshot,
            decision.scheduledNs,
            decision.scheduledValid != 0u,
            actualNs,
            actualValid,
            &record.imu) ||
        !imu_publish_apply_timing(
            &record.imu,
            decision.gatesSkippedBefore)) {
        return false;
    }

    record.scheduledNs = record.imu.scheduledNs;
    record.actualNs = record.imu.actualNs;
    record.scheduledValid = record.imu.scheduledValid;
    record.actualValid = record.imu.actualValid;
    record.deadlineMissed = record.imu.deadlineMissed;
    record.gatesSkippedBefore =
        record.imu.gatesSkippedBefore;

    record.pressureDriverState =
        (int32_t)ms5837_driver_state(
            &pressure->driver);

    record.zeroRequested = pressure->zeroActive ? 1u : 0u;
    record.zeroValid = pressure->zeroValid ? 1u : 0u;
    record.zeroValidSamples =
        pressure->zeroValidSamples;
    record.zeroRejectedSamples =
        pressure->zeroRejectedSamples;

    record.surfacePressureMbar =
        pressure->zeroValid
          ? pressure->surfacePressureMbar
          : NAN;

    record.fluidDensityKgM3 =
        pressure->fluidDensityKgM3;

    classify_pressure(pressure, &record);

    record.depthM =
        record.pressureReady &&
        pressure->zeroValid
          ? ms5837_depth_from_reference(
                record.pressure.pressure_mbar,
                pressure->surfacePressureMbar,
                pressure->fluidDensityKgM3)
          : NAN;

    record.loggerDropsBefore = twg_log_dropped(log);

    if (!twg_log_enqueue(log, &record)) {
        return false;
    }

    if (!pressure_trigger_if_idle(pressure)) {
        return false;
    }

    if ((record.rowSequence % UINT64_C(50)) == 0u) {
        printf(
            "row=%" PRIu64
            " phase=%d"
            " imu_fresh=0x%02x imu_valid=0x%02x"
            " pressure_seq=%" PRIu64
            " pressure_fresh=%u"
            " pressure_mbar=%.2f depth_m=%.4f"
            " qw=%.4f qx=%.4f qy=%.4f qz=%.4f"
            " yaw=%.4f pitch=%.4f roll=%.4f"
            " ax=%.4f ay=%.4f az=%.4f"
            " gx=%.4f gy=%.4f gz=%.4f"
            " drops=%" PRIu64 "\n",

            record.rowSequence,
            (int)record.phase,
            record.imu.freshMask,
            record.imu.validMask,
            record.pressure.measurement_sequence,
            record.pressureFresh,
            record.pressure.pressure_mbar,
            record.depthM,

            (double)record.imu.qw,
            (double)record.imu.qx,
            (double)record.imu.qy,
            (double)record.imu.qz,

            (double)record.imu.yawRad,
            (double)record.imu.pitchRad,
            (double)record.imu.rollRad,

            (double)record.imu.ax,
            (double)record.imu.ay,
            (double)record.imu.az,

            (double)record.imu.gx,
            (double)record.imu.gy,
            (double)record.imu.gz,

            record.loggerDropsBefore);
    }

    return true;
}

static void print_usage(void)
{
    puts("usage: twg_app [--duration 1..3600]");
    puts("       twg_app --help");
    puts("");
    puts("During acquisition:");
    puts("  c  start IMU calibration");
    puts("  z  start Z-axis IMU tare");
    puts("  a  start full-axis IMU tare");
    puts("  w  start pressure zero/reference");
    puts("  x  request full sensor clear");
    puts("  y  confirm tare or requested full clear");
    puts("  q  cancel current action");
}

int main(int argc, char **argv)
{
    ImuCliParse_t parsed;
    PressureContext pressure;
    PublicationContext publication;
    TwgStage stage;
    TwgTerm term;
    TwgLog log;

    RT_StartStatus_t rtStatus;
    AppRtPolicy_t rtPolicy;

    uint64_t nowNs;
    uint64_t bodyStartNs;
    uint64_t bodyEndNs;
    uint64_t maximumBodyNs = 0u;
    uint64_t serviceTicks = 0u;
    uint64_t serviceOverruns = 0u;
    uint64_t acquisitionNs = 0u;

    bool termReady = false;
    bool termStarted = false;
    bool logReady = false;
    bool logStarted = false;
    bool imuOpened = false;
    bool pressureOpened = false;
    bool ownerFailure = false;
    bool completed = false;

    int exitStatus = 0;

    memset(&pressure, 0, sizeof(pressure));
    memset(&publication, 0, sizeof(publication));
    memset(&stage, 0, sizeof(stage));
    memset(&term, 0, sizeof(term));
    memset(&log, 0, sizeof(log));

    pressure.surfacePressureMbar = NAN;
    pressure.fluidDensityKgM3 = TWG_DENSITY_KGM3;

    if (imu_cli_parse(argc, argv, &parsed) !=
        IMU_CLI_STATUS_OK) {
        if (parsed.status == IMU_CLI_STATUS_HELP) {
            print_usage();
            return 0;
        }

        fprintf(
            stderr,
            "CLI error=%d argv_index=%d\n",
            (int)parsed.error,
            parsed.errorIndex);

        return 2;
    }

    /*
     * Startup command options are deliberately rejected. Runtime keys now
     * own calibration/tare/clear/zero selection.
     */
    if (parsed.plan.slot1 != IMU_CMD_ID_NONE ||
        parsed.plan.slot2 != IMU_CMD_ID_NONE ||
        parsed.plan.slot3 != IMU_CMD_ID_NONE) {
        fprintf(
            stderr,
            "runtime controls replace startup"
            " calibration/tare/check/clear options\n");

        return 2;
    }

    if (!install_signals()) {
        fprintf(stderr, "signal setup failed\n");
        return 1;
    }

    if (!twg_term_init(&term, STDIN_FILENO)) {
        fprintf(stderr, "terminal init failed\n");
        return 1;
    }
    termReady = true;

    if (!twg_term_start(&term)) {
        fprintf(stderr, "terminal worker start failed\n");
        exitStatus = 1;
        goto cleanup;
    }
    termStarted = true;

    if (!twg_log_init(&log)) {
        fprintf(stderr, "logger init failed\n");
        exitStatus = 1;
        goto cleanup;
    }
    logReady = true;

    if (!twg_log_start(&log)) {
        fprintf(stderr, "logger start failed\n");
        exitStatus = 1;
        goto cleanup;
    }
    logStarted = true;

    if (!imu_session_open()) {
        fprintf(stderr, "IMU session open failed\n");
        exitStatus = 1;
        goto cleanup;
    }
    imuOpened = true;

    if (!pressure_initialize(&pressure)) {
        fprintf(stderr, "pressure initialization failed\n");
        exitStatus = 1;
        goto cleanup;
    }
    pressureOpened = true;

    rtStatus = StartRT(
        TWG_RT_PRIORITY,
        TWG_SERVICE_PERIOD_SEC);

    rtPolicy =
        app_rt_policy_from_start(rtStatus);

    if (rtPolicy == APP_RT_POLICY_FATAL) {
        fprintf(
            stderr,
            "fatal StartRT status=%u\n",
            (unsigned)rtStatus);

        exitStatus = 1;
        goto cleanup;
    }

    if (rtPolicy ==
        APP_RT_POLICY_WARN_AND_CONTINUE) {
        fprintf(
            stderr,
            "StartRT warning status=%u;"
            " continuing with fallback\n",
            (unsigned)rtStatus);
    }

    if (!twg_stage_init(
            &stage,
            parsed.plan.flightCalMask)) {
        fprintf(stderr, "stage initialization failed\n");
        exitStatus = 1;
        goto cleanup;
    }

    acquisitionNs =
        (uint64_t)parsed.plan.acquisitionDurationS *
        UINT64_C(1000000000);

    if (RT_Reset() < 0) {
        fprintf(stderr, "RT reset failed\n");
        exitStatus = 1;
        goto cleanup;
    }

    while (!sStop && !completed) {
        TwgStageStatus stageStatus;
        TwgStageMode mode;
        bool pressureCompleted = false;
        bool wasZeroing;
        int sleepRc;

        if (!monotonic_ns(&nowNs)) {
            ownerFailure = true;
            break;
        }

        bodyStartNs = nowNs;

        stageStatus = twg_stage_turn(
            &stage,
            &term,
            nowNs,
            sStop != 0);

        if (stageStatus == TWG_STAGE_ERROR) {
            ownerFailure = true;
            break;
        }

        if (stageStatus == TWG_STAGE_PROCESS_STOP) {
            break;
        }

        if (twg_stage_take_pressure_zero_request(&stage)) {
            pressure_zero_begin(&pressure, nowNs);
        }

        if (twg_stage_take_pressure_zero_cancel(&stage)) {
            pressure.zeroActive = false;
        }

        if (twg_stage_take_pressure_clear(&stage)) {
            pressure_zero_clear(&pressure);
        }

        if (!pressure_service(
                &pressure,
                nowNs,
                &pressureCompleted)) {
            ownerFailure = true;
            break;
        }

        wasZeroing = pressure.zeroActive;

        pressure_zero_service(
            &pressure,
            pressureCompleted,
            nowNs);

        if (wasZeroing && !pressure.zeroActive) {
            twg_stage_pressure_zero_finished(&stage);
        }

        ++serviceTicks;

        mode = twg_stage_mode(&stage);

        if (!publication.windowOpen &&
            mode == TWG_STAGE_MODE_ACQUISITION) {
            if (!publication_begin(&publication, nowNs)) {
                fprintf(stderr,
                        "publication initialization failed\n");
                ownerFailure = true;
                break;
            }
        }

        if (publication.windowOpen &&
            !publication.windowJustOpened) {
            if (!publication_emit(
                    &publication,
                    &pressure,
                    &log,
                    mode)) {
                ownerFailure = true;
                break;
            }
        }

        /*
         * The duration belongs only to ACQUISITION. Logging continues in
         * every mode, but action time does not consume survey time.
         */
        if (twg_stage_acquisition_active(&stage)) {
            publication.acquiredNs +=
                TWG_SERVICE_PERIOD_NS;

            if (publication.acquiredNs >= acquisitionNs) {
                completed = true;
            }
        }

        publication.windowJustOpened = false;

        if (twg_log_write_failed(&log)) {
            ownerFailure = true;
            break;
        }

        if (!monotonic_ns(&bodyEndNs)) {
            ownerFailure = true;
            break;
        }

        if (bodyEndNs >= bodyStartNs &&
            bodyEndNs - bodyStartNs >
                maximumBodyNs) {
            maximumBodyNs =
                bodyEndNs - bodyStartNs;
        }

        sleepRc = RT_SleepUntil(
            TWG_SERVICE_PERIOD_SEC);

        if (sleepRc < 0) {
            if (sStop) {
                break;
            }

            ownerFailure = true;
            break;
        }

        publication.nextTurnTicks =
            UINT64_C(1) + (uint64_t)sleepRc;

        if (sleepRc > 0) {
            serviceOverruns +=
                (uint64_t)sleepRc;
        }
    }

cleanup:
    if (termStarted) {
        twg_term_request_stop(&term);

        if (!twg_term_join(&term)) {
            ownerFailure = true;
        }
    }

    if (logStarted && !twg_log_stop(&log)) {
        ownerFailure = true;
    }

    if (pressureOpened &&
        ms5837_driver_shutdown(
            &pressure.driver) < 0) {
        ownerFailure = true;
    }

    if (imuOpened) {
        imu_session_close();
    }

    if (termReady) {
        twg_term_destroy(&term);
    }

    if (ownerFailure) {
        exitStatus = 1;
    }

    printf(
        "shutdown status=%d signal=%d"
        " acquired_ns=%" PRIu64
        " requested_ns=%" PRIu64
        " service_ticks=%" PRIu64
        " overruns=%" PRIu64
        " max_body_ns=%" PRIu64
        " rows=%" PRIu64
        " enqueued=%" PRIu64
        " written=%" PRIu64
        " logger_drops=%" PRIu64
        " terminal_drops=%" PRIu64
        " csv=%s\n",

        exitStatus,
        (int)sSignal,
        publication.acquiredNs,
        acquisitionNs,
        serviceTicks,
        serviceOverruns,
        maximumBodyNs,
        publication.rowSequence,
        logReady ? twg_log_enqueued(&log) : 0u,
        logReady ? twg_log_written(&log) : 0u,
        logReady ? twg_log_dropped(&log) : 0u,
        termReady ? twg_term_dropped(&term) : 0u,
        logReady ? twg_log_filename(&log) : "none");

    return exitStatus;
}
