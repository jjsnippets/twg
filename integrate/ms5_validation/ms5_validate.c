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

#include "app/imu_cal.h"
#include "app/imu_cal_adapter.h"
#include "app/imu_cmd.h"
#include "app/imu_console.h"
#include "app/imu_publish.h"
#include "app/imu_session.h"
#include "app/imu_tare.h"
#include "app/imu_tare_adapter.h"

#include "ms5837_contract.h"
#include "ms5837_driver.h"
#include "ms5837_hal_rpi.h"
#include "ms5837_math.h"

#include "rt/realtime.h"
#include "rt/rt_policy.h"

#include "twg_log.h"
#include "twg_term.h"

#define VAL_RT_PRIORITY         90
#define VAL_SERVICE_PERIOD_SEC  0.001
#define VAL_SERVICE_PERIOD_NS   UINT64_C(1000000)

#define VAL_STARTUP_ATTEMPTS    3u
#define VAL_STARTUP_RETRY_NS    UINT64_C(200000000)
#define VAL_SETTLE_NS           UINT64_C(300000000)
#define VAL_STATUS_NS           UINT64_C(500000000)
#define VAL_WATCHDOG_NS         UINT64_C(1000000000)

#define VAL_CYCLE_COUNT         9u
#define VAL_CYCLE_DURATION_S    30u
#define VAL_TRIAL_DURATION_S    (VAL_CYCLE_COUNT * VAL_CYCLE_DURATION_S)
#define VAL_TRIAL_DURATION_NS   ((uint64_t)VAL_TRIAL_DURATION_S * UINT64_C(1000000000))
#define VAL_DENSITY_KGM3        998.2

typedef enum {
    VAL_MODE_READY = 0,
    VAL_MODE_RECORDING
} ValMode;

typedef enum {
    PROTO_SEG_REST_PRE = 0,
    PROTO_SEG_PUSH,
    PROTO_SEG_REST_POST,
    PROTO_SEG_RETURN
} ProtoSegment;

typedef struct {
    Ms5837Driver_t driver;
    Ms5837HalRpi_t halContext;
    Ms5837Hal_t hal;

    BaroDriverStats_t stats;
    BaroSample_t latest;
    bool haveLatest;

    uint64_t previousSequence;
    bool previousValid;
} PressureContext;

typedef struct {
    ImuPublisherState_t imuPublisher;
    ImuPublishGate_t gate;

    uint64_t rowSequence;
    uint64_t nextTurnTicks;
    uint64_t trialStartNs;
    bool active;
} TrialContext;

typedef struct {
    bool calibrationOk;
    bool tareOk;
    bool settleStarted;
    bool printedCalibration;
    bool printedTare;

    uint64_t settleDeadlineNs;
    uint64_t lastStatusNs;
    uint64_t flushTicket;

    ImuCmdIdentity_t inputStage;
    ImuCmdIdentity_t renderedStage;
} Preparation;

static volatile sig_atomic_t sStop;
static volatile sig_atomic_t sSignal;

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

    *out = (uint64_t)now.tv_sec * UINT64_C(1000000000) +
           (uint64_t)now.tv_nsec;
    return true;
}

static int sleep_relative(uint64_t durationNs)
{
    struct timespec pause;

    pause.tv_sec = (time_t)(durationNs / UINT64_C(1000000000));
    pause.tv_nsec = (long)(durationNs % UINT64_C(1000000000));

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

static bool sleep_turn(uint64_t *nextWakeNs)
{
    struct timespec deadline;
    uint64_t now;
    int rc;

    if (!monotonic_ns(&now)) {
        return false;
    }

    if (*nextWakeNs <= now) {
        *nextWakeNs = now + VAL_SERVICE_PERIOD_NS;
    }

    deadline.tv_sec = (time_t)(*nextWakeNs / UINT64_C(1000000000));
    deadline.tv_nsec = (long)(*nextWakeNs % UINT64_C(1000000000));

    do {
        rc = clock_nanosleep(
            CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL);
    } while (rc == EINTR && !sStop);

    *nextWakeNs += VAL_SERVICE_PERIOD_NS;

    if (rc != 0 && !(rc == EINTR && sStop)) {
        errno = rc;
        return false;
    }

    return true;
}

/* ---------------------------------------------------------------- pressure */
static bool pressure_recovering(const PressureContext *pressure)
{
    Ms5837DriverState_t state = ms5837_driver_state(&pressure->driver);

    return state >= MS5837_STATE_RECOVERY_BACKOFF &&
           state <= MS5837_STATE_RECOVERY_VALIDATE;
}

static bool pressure_initialize(PressureContext *pressure)
{
    unsigned attempt;
    int rc = MS5837_DRIVER_ERR_NOT_INITIALIZED;

    ms5837_hal_rpi_configure(
        &pressure->halContext,
        MS5837_DEFAULT_I2C_BUS,
        MS5837_I2C_ADDRESS);

    pressure->hal = ms5837_hal_rpi_make(&pressure->halContext);

    for (attempt = 1u; attempt <= VAL_STARTUP_ATTEMPTS; ++attempt) {
        rc = ms5837_driver_init(&pressure->driver, &pressure->hal);

        if (rc == MS5837_DRIVER_OK) {
            return true;
        }

        fprintf(
            stderr,
            "pressure init attempt %u/%u failed: %d\n",
            attempt,
            VAL_STARTUP_ATTEMPTS,
            rc);

        if (attempt < VAL_STARTUP_ATTEMPTS &&
            sleep_relative(VAL_STARTUP_RETRY_NS) < 0) {
            break;
        }
    }

    return false;
}

static bool pressure_service(PressureContext *pressure, uint64_t nowNs)
{
    BaroSample_t completed;
    int rc = ms5837_driver_service(
        &pressure->driver,
        nowNs,
        &pressure->stats,
        &completed);

    if (rc == MS5837_SERVICE_SAMPLE_READY) {
        pressure->latest = completed;
        pressure->haveLatest = true;
        return true;
    }

    if (rc < 0 &&
        !pressure_recovering(pressure) &&
        ms5837_driver_state(&pressure->driver) != MS5837_STATE_STOPPED) {
        return false;
    }

    return true;
}

static bool pressure_trigger_if_idle(PressureContext *pressure)
{
    int rc;

    if (ms5837_driver_state(&pressure->driver) != MS5837_STATE_IDLE) {
        return true;
    }

    rc = ms5837_driver_trigger(&pressure->driver, &pressure->stats);
    return rc >= 0 || pressure_recovering(pressure);
}

static bool pressure_sample_acceptable(const BaroSample_t *sample)
{
    uint32_t rejectMask =
        BARO_STATUS_NOT_READY |
        BARO_STATUS_RAW_INVALID |
        BARO_STATUS_PRESSURE_INVALID |
        BARO_STATUS_TEMPERATURE_EXTENDED;

    return sample != NULL &&
           sample->contract_version == BARO_SAMPLE_CONTRACT_VERSION &&
           (sample->status_flags & rejectMask) == 0u &&
           isfinite(sample->pressure_mbar);
}

static void classify_pressure(
    PressureContext *pressure,
    TwgLogRecord *record)
{
    if (!pressure->haveLatest ||
        !pressure_sample_acceptable(&pressure->latest)) {
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
    pressure->previousSequence = pressure->latest.measurement_sequence;

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

/* ----------------------------------------------------- startup preparation */
static bool post_event(ImuCmdEventType_t type, uint64_t nowNs)
{
    ImuCmdEvent_t event;

    memset(&event, 0, sizeof(event));
    event.type = type;
    event.monotonicNs = nowNs;
    return imu_cmd_post(&event);
}

static bool pump_preparation_adapter(ImuCmdIdentity_t active)
{
    ImuCmdProgress_t machine;

    if (active == IMU_CMD_ID_CALIBRATION) {
        if (!imu_cal_get_progress(&machine) ||
            machine.active != active ||
            imu_cal_complete()) {
            return true;
        }
        return imu_cal_adapter_pump();
    }

    if (active == IMU_CMD_ID_TARE) {
        if (!imu_tare_get_progress(&machine) ||
            machine.active != active ||
            imu_tare_complete()) {
            return true;
        }
        return imu_tare_adapter_pump();
    }

    return true;
}

static bool check_preparation_results(Preparation *prep)
{
    const ImuCmdIdentity_t identities[2] = {
        IMU_CMD_ID_CALIBRATION,
        IMU_CMD_ID_TARE
    };
    unsigned i;

    for (i = 0u; i < 2u; ++i) {
        ImuCmdResult_t result;
        bool *printed =
            i == 0u ? &prep->printedCalibration : &prep->printedTare;
        bool acceptable;
        char text[2048];

        if (*printed ||
            !imu_cmd_get_result(identities[i], &result) ||
            result.state == IMU_CMD_STATE_NOT_REQUESTED ||
            result.state == IMU_CMD_STATE_RUNNING) {
            continue;
        }

        if (!imu_console_render_result(
                &result, imu_cmd_do_not_acquire(),
                text, sizeof(text))) {
            return false;
        }

        fputs(text, stdout);
        *printed = true;

        acceptable =
            result.state == IMU_CMD_STATE_SUCCEEDED &&
            result.verified &&
            result.restoredProduction;

        if (i == 0u) {
            acceptable = acceptable && result.dcdSaved;
            if (result.state == IMU_CMD_STATE_RECOVERY_FAILED) {
                fprintf(stderr, "Startup stopped: calibration left IMU unrecoverable.\n");
                return false;
            }
            if (!acceptable) {
                fprintf(
                    stderr,
                    "WARNING: calibration did not pass; continuing to Z-axis tare.\n");
            }
            prep->calibrationOk = acceptable;
            continue;
        } else {
            acceptable =
                acceptable &&
                result.sub.tareNow == IMU_CMD_SUB_SUCCEEDED &&
                result.sub.persist == IMU_CMD_SUB_SUCCEEDED;
        }

        if (!acceptable) {
            fprintf(
                stderr,
                "Startup stopped: %s did not complete successfully.\n",
                i == 0u ? "calibration" : "Z tare");
            return false;
        }

        prep->tareOk = true;
    }

    return true;
}

static bool preparation_status(
    Preparation *prep,
    ImuConsole_t *console,
    const ImuCmdProgress_t *progress,
    uint64_t nowNs)
{
    char text[2048];
    bool due =
        prep->lastStatusNs == 0u ||
        nowNs - prep->lastStatusNs >= VAL_STATUS_NS;
    bool changed = prep->renderedStage != progress->active;

    if (!due && !changed) {
        return true;
    }

    if (progress->active == IMU_CMD_ID_CALIBRATION) {
        if (!imu_console_stage_flush_complete(console, prep->flushTicket)) {
            return true;
        }

        if (!imu_console_render_progress(progress, text, sizeof(text))) {
            return false;
        }

        fputs(text, stdout);
        printf(
            "CAL status: accel=%u gyro=%u mag=%u rv=%u "
            "gate=%u sustained_ms=%" PRIu64 "\n",
            (unsigned)progress->cal.accelAccuracy,
            (unsigned)progress->cal.gyroAccuracy,
            (unsigned)progress->cal.magAccuracy,
            (unsigned)progress->cal.rvAccuracy,
            progress->cal.gatePassingNow ? 1u : 0u,
            progress->cal.sustainedGoodNs / UINT64_C(1000000));
    } else if (progress->active == IMU_CMD_ID_TARE) {
        printf(
            "TARE-Z: phase=%d remaining_ms=%" PRIu64
            "; keep the cart aligned at rest.\n",
            (int)progress->tare.phase,
            progress->tare.remainingNs / UINT64_C(1000000));
    } else if (progress->active == IMU_CMD_ID_SETTLE) {
        puts("SETTLE: restoring production IMU reports.");
    } else {
        puts("STARTUP: preparing next stage.");
    }

    prep->lastStatusNs = nowNs;
    prep->renderedStage = progress->active;
    return fflush(stdout) == 0;
}

static bool prepare_imu(
    PressureContext *pressure,
    ImuConsole_t *console)
{
    Preparation prep;
    ImuCmdPlan_t plan;
    uint64_t nextWake = 0u;

    memset(&prep, 0, sizeof(prep));

    if (!imu_cmd_plan_acquire_default(&plan)) {
        return false;
    }

    plan.slot1 = IMU_CMD_ID_CALIBRATION;
    plan.slot2 = IMU_CMD_ID_TARE;
    plan.tareAxes = IMU_CMD_TARE_AXES_Z;
    plan.persistTare = true;
    plan.confirmTare = true;
    plan.flightCalMask = 0u;
    plan.acquisitionDurationS = VAL_TRIAL_DURATION_S;

    if (!imu_cmd_init(&plan)) {
        fprintf(
            stderr,
            "Coordinator initialization failed: reason=%d\n",
            (int)imu_cmd_init_reason());
        return false;
    }

    puts(
        "Startup: guided calibration, then automatic persisted Z-axis tare.\n"
        "Calibration retains its existing q and Enter inputs.\n"
        "Finish calibration with the cart aligned at rest.");

    while (!sStop) {
        ImuCmdProgress_t progress;
        ImuSampleSnapshot_t snapshot;
        ImuConsoleObservation_t obs;
        uint64_t nowNs;

        imu_session_service();

        if (!monotonic_ns(&nowNs) ||
            !pressure_service(pressure, nowNs) ||
            !imu_cmd_get_progress(&progress) ||
            !pump_preparation_adapter(progress.active)) {
            return false;
        }

        if (!imu_session_get_snapshot(&snapshot) ||
            snapshot.readerState == IMU_READER_STATE_CLOSED ||
            snapshot.readerState == IMU_READER_STATE_FAULTED ||
            snapshot.readerState == IMU_READER_STATE_RECOVERING) {
            fprintf(stderr, "Startup IMU session became unusable.\n");
            return false;
        }

        if (sStop) {
            break;
        }

        if (!check_preparation_results(&prep)) {
            return false;
        }

        if (progress.active != prep.inputStage) {
            prep.inputStage = progress.active;
            prep.flushTicket = imu_console_request_stage_flush(console);
            if (prep.flushTicket == 0u) {
                return false;
            }
        }

        if (!post_event(IMU_CMD_EVENT_TICK, nowNs) ||
            !imu_cmd_get_progress(&progress)) {
            return false;
        }

        if (!imu_console_observe(console, &obs) ||
            obs.inputError ||
            obs.eofObserved) {
            fprintf(stderr, "Startup terminal input unavailable.\n");
            return false;
        }

        if (progress.active == IMU_CMD_ID_CALIBRATION &&
            imu_console_stage_flush_complete(console, prep.flushTicket)) {
            ImuConsoleInput_t token;
            ImuCmdEvent_t event;

            if (imu_console_dequeue(console, &token) &&
                imu_console_translate(token, &progress, &event) &&
                !imu_cmd_post(&event)) {
                return false;
            }
        }

        if (progress.active == IMU_CMD_ID_TARE &&
            progress.requiredAction == IMU_CMD_ACTION_ALIGN_AND_CONFIRM &&
            progress.tare.confirmationPending) {
            if (!post_event(IMU_CMD_EVENT_OPERATOR_CONFIRM, nowNs)) {
                return false;
            }
        }

        if (progress.active == IMU_CMD_ID_SETTLE) {
            if (!prep.tareOk) {
                return false;
            }

            if (!prep.settleStarted) {
                if (!imu_session_begin_settle() ||
                    !monotonic_ns(&nowNs)) {
                    return false;
                }
                prep.settleDeadlineNs = nowNs + VAL_SETTLE_NS;
                prep.settleStarted = true;
            }

            if (nowNs >= prep.settleDeadlineNs) {
                if (!imu_session_mark_operational() ||
                    !post_event(IMU_CMD_EVENT_SETTLE_DONE, nowNs)) {
                    return false;
                }
            }
        }

        imu_cmd_service();

        if (!check_preparation_results(&prep) ||
            !imu_cmd_get_progress(&progress) ||
            !monotonic_ns(&nowNs)) {
            return false;
        }

        if (progress.active == IMU_CMD_ID_ACQUISITION) {
            return prep.tareOk;
        }

        if (imu_cmd_plan_complete()) {
            fprintf(stderr, "Startup plan ended before READY.\n");
            return false;
        }

        if (!preparation_status(&prep, console, &progress, nowNs) ||
            !sleep_turn(&nextWake)) {
            return false;
        }
    }

    {
        uint64_t nowNs = 0u;
        (void)monotonic_ns(&nowNs);
        (void)post_event(IMU_CMD_EVENT_PROCESS_STOP, nowNs);
    }

    return false;
}

/* ----------------------------------------------------- protocol guidance */
static const char *speed_label(unsigned cycleIndex)
{
    if (cycleIndex < 3u) {
        return "SLOW";
    }
    if (cycleIndex < 6u) {
        return "MODERATE";
    }
    return "FAST";
}

static ProtoSegment segment_for_offset(unsigned secInCycle)
{
    if (secInCycle < 5u) {
        return PROTO_SEG_REST_PRE;
    }
    if (secInCycle < 15u) {
        return PROTO_SEG_PUSH;
    }
    if (secInCycle < 20u) {
        return PROTO_SEG_REST_POST;
    }
    return PROTO_SEG_RETURN;
}

static const char *segment_name(ProtoSegment seg)
{
    switch (seg) {
    case PROTO_SEG_REST_PRE:  return "REST_PRE";
    case PROTO_SEG_PUSH:      return "PUSH";
    case PROTO_SEG_REST_POST: return "REST_POST";
    case PROTO_SEG_RETURN:
    default:                  return "RETURN";
    }
}

static const char *segment_instruction(ProtoSegment seg, const char *speed)
{
    switch (seg) {
    case PROTO_SEG_REST_PRE:
        return "0-5s   HOLD STILL (baseline, hands off)";
    case PROTO_SEG_PUSH:
        if (strcmp(speed, "SLOW") == 0) {
            return "5-15s  PUSH SLOW";
        }
        if (strcmp(speed, "MODERATE") == 0) {
            return "5-15s  PUSH MODERATE";
        }
        return "5-15s  PUSH FAST";
    case PROTO_SEG_REST_POST:
        return "15-20s HOLD STILL (post-push rest, hands off)";
    case PROTO_SEG_RETURN:
    default:
        return "20-30s RETURN SLOWLY TO START";
    }
}

static void protocol_step(
    uint64_t elapsedNs,
    unsigned *cycleOut,
    ProtoSegment *segOut)
{
    uint64_t totalSec = elapsedNs / UINT64_C(1000000000);
    unsigned cycle = (unsigned)(totalSec / VAL_CYCLE_DURATION_S);
    unsigned secInCycle = (unsigned)(totalSec % VAL_CYCLE_DURATION_S);

    if (cycle >= VAL_CYCLE_COUNT) {
        cycle = VAL_CYCLE_COUNT - 1u;
        secInCycle = VAL_CYCLE_DURATION_S - 1u;
    }

    *cycleOut = cycle;
    *segOut = segment_for_offset(secInCycle);
}

static bool runtime_healthy(
    const ImuSampleSnapshot_t *snapshot,
    uint32_t expectedEpoch,
    uint64_t readySinceNs,
    uint64_t nowNs)
{
    const ImuGroupMeta_t *groups[3] = {
        &snapshot->rotationMeta,
        &snapshot->accelMeta,
        &snapshot->gyroMeta
    };
    const unsigned bits[3] = {
        IMU_GROUP_BIT_ROTATION,
        IMU_GROUP_BIT_ACCEL,
        IMU_GROUP_BIT_GYRO
    };
    unsigned i;

    if (snapshot->readerState != IMU_READER_STATE_OPERATIONAL ||
        snapshot->configurationEpoch != expectedEpoch) {
        return false;
    }

    for (i = 0u; i < 3u; ++i) {
        bool present = (snapshot->validMask & bits[i]) != 0u;
        uint64_t hostNs = groups[i]->hostDecodeNs;

        if (!present || hostNs == 0u) {
            if (nowNs - readySinceNs > VAL_WATCHDOG_NS) {
                return false;
            }
            continue;
        }

        if (hostNs > nowNs || nowNs - hostNs > VAL_WATCHDOG_NS) {
            return false;
        }
    }

    return true;
}

static bool trial_begin(
    TrialContext *trial,
    TwgLog *log,
    uint64_t nowNs)
{
    ImuSampleSnapshot_t baseline;

    memset(trial, 0, sizeof(*trial));
    memset(log, 0, sizeof(*log));

    if (!twg_log_init(log)) {
        fprintf(stderr, "logger init failed\n");
        return false;
    }

    if (!twg_log_start(log)) {
        fprintf(stderr, "logger start failed\n");
        return false;
    }

    if (!imu_session_get_snapshot(&baseline) ||
        baseline.readerState != IMU_READER_STATE_OPERATIONAL ||
        !imu_publish_begin_window(&trial->imuPublisher, &baseline) ||
        !imu_publish_gate_begin(
            &trial->gate,
            nowNs + VAL_SERVICE_PERIOD_NS,
            true,
            VAL_TRIAL_DURATION_NS)) {
        (void)twg_log_stop(log);
        return false;
    }

    trial->trialStartNs = nowNs;
    trial->nextTurnTicks = 1u;
    trial->active = true;
    return true;
}

static bool trial_emit(
    TrialContext *trial,
    PressureContext *pressure,
    TwgLog *log,
    unsigned cycle,
    ProtoSegment seg,
    uint64_t elapsedNs,
    uint64_t trialNumber)
{
    ImuPublishGateDecision_t decision;
    ImuPublishGateStatus_t gateStatus;
    ImuSampleSnapshot_t snapshot;
    TwgLogRecord record;
    uint64_t actualNs = 0u;
    bool actualValid;

    gateStatus = imu_publish_gate_advance(
        &trial->gate,
        trial->nextTurnTicks,
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
    record.rowSequence = ++trial->rowSequence;
    record.phase = TWG_LOG_PHASE_RUN;
    record.stageActive = segment_name(seg);

    actualValid = monotonic_ns(&actualNs);

    if (!imu_publish_evaluate_snapshot(
            &trial->imuPublisher,
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
    record.gatesSkippedBefore = record.imu.gatesSkippedBefore;

    record.pressureDriverState =
        (int32_t)ms5837_driver_state(&pressure->driver);
    record.surfacePressureMbar = NAN;
    record.fluidDensityKgM3 = VAL_DENSITY_KGM3;

    classify_pressure(pressure, &record);
    record.depthM = NAN;
    record.loggerDropsBefore = twg_log_dropped(log);

    if (!twg_log_enqueue(log, &record)) {
        return false;
    }

    if (!pressure_trigger_if_idle(pressure)) {
        return false;
    }

    if ((record.rowSequence % UINT64_C(50)) == 0u) {
        double elapsedS = (double)elapsedNs / 1e9;
        double remainingS = (double)VAL_TRIAL_DURATION_S - elapsedS;
        if (remainingS < 0.0) {
            remainingS = 0.0;
        }

        printf(
            "trial=%" PRIu64 " t=%5.1f/%us (rem=%5.1fs)"
            " [C%u/%u %s %-9s]"
            " P=%.2f mbar"
            " a=(%+.3f,%+.3f,%+.3f)"
            " drops=%" PRIu64 "\n",
            trialNumber,
            elapsedS,
            VAL_TRIAL_DURATION_S,
            remainingS,
            cycle + 1u,
            VAL_CYCLE_COUNT,
            speed_label(cycle),
            segment_name(seg),
            record.pressure.pressure_mbar,
            (double)record.imu.ax,
            (double)record.imu.ay,
            (double)record.imu.az,
            record.loggerDropsBefore);
        (void)fflush(stdout);
    }

    return true;
}

int main(int argc, char **argv)
{
    PressureContext pressure;
    TrialContext trial;
    TwgTerm term;
    TwgLog log;
    ImuConsole_t console;
    ImuSampleSnapshot_t snapshot;

    RT_StartStatus_t rtStatus;
    AppRtPolicy_t rtPolicy;

    ValMode mode = VAL_MODE_READY;
    uint64_t trialNumber = 0u;
    uint64_t readySinceNs = 0u;
    uint64_t lastReadyStatusNs = 0u;
    uint64_t nowNs = 0u;
    uint32_t expectedEpoch = 0u;

    unsigned lastCycle = VAL_CYCLE_COUNT;
    ProtoSegment lastSeg = PROTO_SEG_RETURN;

    bool consoleInit = false;
    bool consoleStarted = false;
    bool termInit = false;
    bool termStarted = false;
    bool imuOpened = false;
    bool pressureOpened = false;
    bool logActive = false;
    bool ownerFailure = false;
    int exitStatus = 0;

    (void)argv;
    memset(&pressure, 0, sizeof(pressure));
    memset(&trial, 0, sizeof(trial));
    memset(&term, 0, sizeof(term));
    memset(&log, 0, sizeof(log));
    memset(&console, 0, sizeof(console));
    memset(&snapshot, 0, sizeof(snapshot));

    if (argc != 1) {
        fprintf(stderr, "Usage: ms5_validation\n");
        return 2;
    }

    if (!isatty(STDIN_FILENO)) {
        fprintf(stderr, "An interactive terminal is required.\n");
        return 1;
    }

    if (!install_signals()) {
        fprintf(stderr, "signal setup failed\n");
        return 1;
    }

    if (!imu_console_init(&console, STDIN_FILENO)) {
        fprintf(stderr, "console init failed\n");
        return 1;
    }
    consoleInit = true;

    if (!imu_console_start(&console)) {
        fprintf(stderr, "console start failed\n");
        exitStatus = 1;
        goto cleanup;
    }
    consoleStarted = true;

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

    if (!prepare_imu(&pressure, &console)) {
        exitStatus = sStop ? 0 : 1;
        goto cleanup;
    }

    imu_console_request_stop(&console);
    if (!imu_console_join(&console)) {
        fprintf(stderr, "console join failed\n");
        exitStatus = 1;
        goto cleanup;
    }
    consoleStarted = false;
    imu_console_destroy(&console);
    consoleInit = false;

    if (sStop) {
        goto cleanup;
    }

    if (!twg_term_init(&term, STDIN_FILENO)) {
        fprintf(stderr, "terminal init failed\n");
        exitStatus = 1;
        goto cleanup;
    }
    termInit = true;

    if (!twg_term_start(&term)) {
        fprintf(stderr, "terminal worker start failed\n");
        exitStatus = 1;
        goto cleanup;
    }
    termStarted = true;

    rtStatus = StartRT(VAL_RT_PRIORITY, VAL_SERVICE_PERIOD_SEC);
    rtPolicy = app_rt_policy_from_start(rtStatus);

    if (rtPolicy == APP_RT_POLICY_FATAL) {
        fprintf(stderr, "fatal StartRT status=%u\n", (unsigned)rtStatus);
        exitStatus = 1;
        goto cleanup;
    }
    if (rtPolicy == APP_RT_POLICY_WARN_AND_CONTINUE) {
        fprintf(
            stderr,
            "StartRT warning status=%u; continuing with fallback\n",
            (unsigned)rtStatus);
    }

    if (RT_Reset() < 0) {
        fprintf(stderr, "RT reset failed\n");
        exitStatus = 1;
        goto cleanup;
    }

    if (!imu_session_get_snapshot(&snapshot) || !monotonic_ns(&readySinceNs)) {
        exitStatus = 1;
        goto cleanup;
    }
    expectedEpoch = snapshot.configurationEpoch;

    puts(
        "\n============================================================\n"
        "READY: IMU calibrated and Z-tared; pressure sensor active.\n"
        "Press 's' to start a 270 s (4.5 min) dynamic trial.\n"
        "Press Ctrl+C to exit.\n"
        "============================================================");
    (void)fflush(stdout);

    while (!sStop) {
        TwgTermKey key;
        bool startKey = false;
        int sleepRc;

        imu_session_service();

        if (!monotonic_ns(&nowNs) ||
            !pressure_service(&pressure, nowNs) ||
            !imu_session_get_snapshot(&snapshot)) {
            ownerFailure = true;
            break;
        }

        if (!runtime_healthy(&snapshot, expectedEpoch, readySinceNs, nowNs)) {
            fprintf(stderr, "IMU watchdog/epoch failure; stopping.\n");
            ownerFailure = true;
            break;
        }

        while (twg_term_dequeue(&term, &key)) {
            if (mode == VAL_MODE_READY && (key.key == 's' || key.key == 'S')) {
                startKey = true;
            }
        }

        if (twg_term_eof_seen(&term) || twg_term_input_error(&term)) {
            fprintf(stderr, "terminal input error or EOF\n");
            ownerFailure = true;
            break;
        }

        if (mode == VAL_MODE_READY) {
            if (!pressure_trigger_if_idle(&pressure)) {
                ownerFailure = true;
                break;
            }

            if (startKey) {
                if (!trial_begin(&trial, &log, nowNs)) {
                    ownerFailure = true;
                    break;
                }
                logActive = true;
                ++trialNumber;
                mode = VAL_MODE_RECORDING;
                lastCycle = VAL_CYCLE_COUNT;
                lastSeg = PROTO_SEG_RETURN;

                printf(
                    "\n*** TRIAL %" PRIu64 " STARTED (270 s) | CSV: %s ***\n",
                    trialNumber,
                    twg_log_filename(&log));
                (void)fflush(stdout);
            } else if (lastReadyStatusNs == 0u ||
                       nowNs - lastReadyStatusNs >= UINT64_C(2000000000)) {
                printf(
                    "READY (trials_done=%" PRIu64 ") P=%.2f mbar"
                    " a=(%+.3f,%+.3f,%+.3f)  [s=start, Ctrl+C=exit]\n",
                    trialNumber,
                    pressure.haveLatest ? pressure.latest.pressure_mbar : NAN,
                    (double)snapshot.ax,
                    (double)snapshot.ay,
                    (double)snapshot.az);
                (void)fflush(stdout);
                lastReadyStatusNs = nowNs;
            }
        } else {
            uint64_t elapsedNs = nowNs >= trial.trialStartNs
                                     ? nowNs - trial.trialStartNs
                                     : 0u;
            unsigned cycle;
            ProtoSegment seg;

            if (elapsedNs >= VAL_TRIAL_DURATION_NS) {
                if (!twg_log_stop(&log)) {
                    ownerFailure = true;
                    break;
                }
                logActive = false;
                trial.active = false;
                mode = VAL_MODE_READY;
                lastReadyStatusNs = 0u;

                printf(
                    "\n*** TRIAL %" PRIu64 " COMPLETE ***"
                    " rows=%" PRIu64 " written=%" PRIu64 " drops=%" PRIu64
                    " csv=%s\n"
                    "READY: press 's' for next trial, or Ctrl+C to exit.\n\n",
                    trialNumber,
                    trial.rowSequence,
                    twg_log_written(&log),
                    twg_log_dropped(&log),
                    twg_log_filename(&log));
                (void)fflush(stdout);
            } else {
                protocol_step(elapsedNs, &cycle, &seg);

                if (cycle != lastCycle || seg != lastSeg) {
                    const char *spd = speed_label(cycle);
                    printf(
                        "\n>>> [Cycle %u/%u | %-8s] %s <<<\n",
                        cycle + 1u,
                        VAL_CYCLE_COUNT,
                        spd,
                        segment_instruction(seg, spd));
                    (void)fflush(stdout);
                    lastCycle = cycle;
                    lastSeg = seg;
                }

                if (!trial_emit(
                        &trial, &pressure, &log,
                        cycle, seg, elapsedNs, trialNumber)) {
                    ownerFailure = true;
                    break;
                }

                if (twg_log_write_failed(&log)) {
                    ownerFailure = true;
                    break;
                }
            }
        }

        sleepRc = RT_SleepUntil(VAL_SERVICE_PERIOD_SEC);
        if (sleepRc < 0) {
            if (sStop) {
                break;
            }
            ownerFailure = true;
            break;
        }

        if (mode == VAL_MODE_RECORDING) {
            trial.nextTurnTicks = UINT64_C(1) + (uint64_t)sleepRc;
        }
    }

cleanup:
    if (logActive && !twg_log_stop(&log)) {
        ownerFailure = true;
    }

    if (termStarted) {
        twg_term_request_stop(&term);
        if (!twg_term_join(&term)) {
            ownerFailure = true;
        }
    }
    if (termInit) {
        twg_term_destroy(&term);
    }

    if (consoleStarted) {
        imu_console_request_stop(&console);
        (void)imu_console_join(&console);
    }
    if (consoleInit) {
        imu_console_destroy(&console);
    }

    if (pressureOpened && ms5837_driver_shutdown(&pressure.driver) < 0) {
        ownerFailure = true;
    }

    if (imuOpened) {
        imu_session_close();
    }

    if (ownerFailure) {
        exitStatus = 1;
    }

    printf(
        "shutdown status=%d signal=%d trials=%" PRIu64 "\n",
        exitStatus,
        (int)sSignal,
        trialNumber);
    return exitStatus;
}
