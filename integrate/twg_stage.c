#include "twg_stage.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_cal.h"
#include "app/imu_cal_adapter.h"
#include "app/imu_check.h"
#include "app/imu_check_adapter.h"
#include "app/imu_session.h"
#include "app/imu_tare.h"
#include "app/imu_tare_adapter.h"

#define TWG_STATUS_PERIOD_NS UINT64_C(500000000)

static bool post_event(
    ImuCmdEventType_t type,
    uint64_t nowNs)
{
    ImuCmdEvent_t event;

    memset(&event, 0, sizeof(event));
    event.type = type;
    event.monotonicNs = nowNs;

    return imu_cmd_post(&event);
}

static ImuCmdIdentity_t active_command(void)
{
    ImuCmdProgress_t progress;

    if (!imu_cmd_get_progress(&progress)) {
        return IMU_CMD_ID_NONE;
    }

    return progress.active;
}

static const char *identity_name(ImuCmdIdentity_t id)
{
    switch (id) {
    case IMU_CMD_ID_CALIBRATION: return "CALIBRATION";
    case IMU_CMD_ID_DCD_CLEAR:   return "DCD_CLEAR";
    case IMU_CMD_ID_TARE:        return "TARE";
    case IMU_CMD_ID_TARE_CLEAR:  return "TARE_CLEAR";
    case IMU_CMD_ID_TARE_CHECK:  return "TARE_CHECK";
    case IMU_CMD_ID_CHECK:       return "CHECK";
    case IMU_CMD_ID_PROBE:       return "PROBE";
    case IMU_CMD_ID_SETTLE:      return "SETTLE";
    case IMU_CMD_ID_ACQUISITION: return "ACQUISITION";
    case IMU_CMD_ID_NONE:
    default:                     return "NONE";
    }
}

static const char *state_name(ImuCmdResultState_t state)
{
    switch (state) {
    case IMU_CMD_STATE_SUCCEEDED:       return "SUCCEEDED";
    case IMU_CMD_STATE_FAILED:          return "FAILED";
    case IMU_CMD_STATE_CANCELLED:       return "CANCELLED";
    case IMU_CMD_STATE_TIMED_OUT:       return "TIMED_OUT";
    case IMU_CMD_STATE_RECOVERY_FAILED: return "RECOVERY_FAILED";
    case IMU_CMD_STATE_ABANDONED:       return "ABANDONED";
    case IMU_CMD_STATE_RUNNING:         return "RUNNING";
    case IMU_CMD_STATE_NOT_REQUESTED:
    default:                            return "NOT_REQUESTED";
    }
}

static const char *pose_name(uint8_t pose)
{
    switch (pose) {
    case 1u: return "flat, label side up";
    case 2u: return "flat, label side down";
    case 3u: return "resting on its left edge";
    case 4u: return "resting on its right edge";
    case 5u: return "resting on its top edge";
    case 6u: return "resting on its bottom edge";
    default: return "in the indicated orientation";
    }
}

static bool cal_enter_ready(const ImuCmdProgress_t *p)
{
    return p->active == IMU_CMD_ID_CALIBRATION &&
           p->requiredAction == IMU_CMD_ACTION_PRESS_Q_TO_END &&
           p->cal.deadlineNs == 0u &&
           (p->cal.phase == IMU_CMD_CAL_PHASE_ACCEL ||
            p->cal.phase == IMU_CMD_CAL_PHASE_GYRO ||
            p->cal.phase == IMU_CMD_CAL_PHASE_MAG ||
            p->cal.phase == IMU_CMD_CAL_PHASE_VERIFY);
}

static bool clear_confirm_pending(const ImuCmdProgress_t *p)
{
    return (p->active == IMU_CMD_ID_DCD_CLEAR &&
            p->requiredAction == IMU_CMD_ACTION_CONFIRM) ||
           (p->active == IMU_CMD_ID_TARE_CLEAR &&
            p->requiredAction == IMU_CMD_ACTION_ALIGN_AND_CONFIRM &&
            p->tare.confirmationPending);
}

static bool build_plan(
    TwgStageMode mode,
    uint8_t flightCalMask,
    ImuCmdPlan_t *plan)
{
    if (plan == NULL ||
        !imu_cmd_plan_acquire_default(plan)) {
        return false;
    }

    plan->flightCalMask = flightCalMask;
    plan->acquisitionDurationS = 1u;

    switch (mode) {
    case TWG_STAGE_MODE_CALIBRATION:
        plan->slot1 = IMU_CMD_ID_CALIBRATION;
        return true;

    case TWG_STAGE_MODE_TARE_Z:
        plan->slot2 = IMU_CMD_ID_TARE;
        plan->tareAxes = IMU_CMD_TARE_AXES_Z;
        plan->persistTare = true;
        plan->confirmTare = true;
        return true;

    case TWG_STAGE_MODE_TARE_FULL:
        plan->slot2 = IMU_CMD_ID_TARE;
        plan->tareAxes = IMU_CMD_TARE_AXES_FULL;
        plan->persistTare = true;
        plan->confirmTare = true;
        return true;

    case TWG_STAGE_MODE_CLEAR_ALL:
        plan->slot1 = IMU_CMD_ID_DCD_CLEAR;
        plan->slot2 = IMU_CMD_ID_TARE_CLEAR;
        plan->confirmDcdClear = true;
        plan->confirmTareClear = true;
        return true;

    default:
        return false;
    }
}

static bool start_imu_plan(
    TwgStage *stage,
    TwgStageMode mode)
{
    ImuCmdPlan_t plan;
    ImuSampleSnapshot_t snapshot;

    if (!build_plan(mode, stage->flightCalMask, &plan) ||
        !imu_session_get_snapshot(&snapshot)) {
        return false;
    }

    if (snapshot.readerState == IMU_READER_STATE_OPERATIONAL) {
        if (!imu_session_return_to_configuring()) {
            return false;
        }
    } else if (snapshot.readerState != IMU_READER_STATE_CONFIGURING) {
        return false;
    }

    if (!imu_cmd_init(&plan)) {
        return false;
    }

    stage->mode = mode;
    stage->imuPlanActive = true;
    stage->settleStarted = false;
    stage->productionConfigured = false;
    stage->promptKey = 0u;
    stage->lastStatusNs = 0u;
    stage->lastActive = IMU_CMD_ID_NONE;
    memset(stage->resultPrinted, 0, sizeof(stage->resultPrinted));

    return true;
}

static bool pump_active_adapter(ImuCmdIdentity_t active)
{
    ImuCmdProgress_t machine;

    switch (active) {
    case IMU_CMD_ID_CALIBRATION:
    case IMU_CMD_ID_DCD_CLEAR:
        if (!imu_cal_get_progress(&machine) ||
            machine.active != active ||
            imu_cal_complete()) {
            return true;
        }
        return imu_cal_adapter_pump();

    case IMU_CMD_ID_TARE:
    case IMU_CMD_ID_TARE_CLEAR:
    case IMU_CMD_ID_TARE_CHECK:
        if (!imu_tare_get_progress(&machine) ||
            machine.active != active ||
            imu_tare_complete()) {
            return true;
        }
        return imu_tare_adapter_pump();

    case IMU_CMD_ID_CHECK:
    case IMU_CMD_ID_PROBE:
        if (!imu_check_get_progress(&machine) ||
            machine.active != active ||
            imu_check_complete()) {
            return true;
        }
        return imu_check_adapter_pump();

    default:
        return true;
    }
}

static bool plan_restored_production(void)
{
    static const ImuCmdIdentity_t ids[] = {
        IMU_CMD_ID_CALIBRATION,
        IMU_CMD_ID_DCD_CLEAR,
        IMU_CMD_ID_TARE,
        IMU_CMD_ID_TARE_CLEAR
    };
    size_t i;

    for (i = 0u; i < sizeof(ids) / sizeof(ids[0]); ++i) {
        ImuCmdResult_t result;

        if (imu_cmd_get_result(ids[i], &result) &&
            result.restoredProduction) {
            return true;
        }
    }

    return false;
}

static bool service_settle(
    TwgStage *stage,
    uint64_t nowNs)
{
    if (!stage->settleStarted) {
        if (!stage->productionConfigured &&
            !plan_restored_production()) {
            if (!imu_session_configure_production(
                    stage->flightCalMask)) {
                return false;
            }
        }

        stage->productionConfigured = true;

        if (!imu_session_begin_settle()) {
            return false;
        }

        stage->settleDeadlineNs = nowNs + TWG_STAGE_SETTLE_NS;
        stage->settleStarted = true;
    }

    if (nowNs < stage->settleDeadlineNs) {
        return true;
    }

    if (!imu_session_mark_operational() ||
        !post_event(IMU_CMD_EVENT_SETTLE_DONE, nowNs)) {
        return false;
    }

    stage->settleStarted = false;
    return true;
}

static uint32_t prompt_key(const ImuCmdProgress_t *p)
{
    return (uint32_t)p->active |
           ((uint32_t)p->requiredAction << 4) |
           ((uint32_t)p->cal.phase << 8) |
           ((uint32_t)p->cal.currentPoseIndex << 12) |
           ((uint32_t)(p->cal.deadlineNs == 0u) << 16) |
           ((uint32_t)p->tare.confirmationPending << 17) |
           ((uint32_t)p->tare.phase << 18);
}

static void print_calibration_prompt(const ImuCmdProgress_t *p)
{
    const ImuCmdCalProgress_t *c = &p->cal;
    bool prompt = cal_enter_ready(p);

    switch (c->phase) {
    case IMU_CMD_CAL_PHASE_STARTUP:
        puts("CAL: configuring calibration reports; wait."
             " (q cancels)");
        break;

    case IMU_CMD_CAL_PHASE_ACCEL:
        if (prompt) {
            printf("CAL: accel pose %u/%u: place the device %s,"
                   " then press ENTER to start this face."
                   " (q cancels)\n",
                   (unsigned)c->currentPoseIndex,
                   (unsigned)c->totalPoseCount,
                   pose_name(c->currentPoseIndex));
        } else {
            printf("CAL: accel pose %u/%u: keep the face still"
                   " while the accel window runs. Do not press"
                   " ENTER. (q cancels)\n",
                   (unsigned)c->currentPoseIndex,
                   (unsigned)c->totalPoseCount);
        }
        break;

    case IMU_CMD_CAL_PHASE_GYRO:
        puts(prompt
             ? "CAL: gyro: place the device stationary, then press"
               " ENTER to begin gyro rest. (q cancels)"
             : "CAL: gyro: keep stationary while the gyro converges."
               " Do not press ENTER. (q cancels)");
        break;

    case IMU_CMD_CAL_PHASE_MAG:
        puts(prompt
             ? "CAL: mag: prepare full ~180-degree swings about roll,"
               " pitch and yaw, then press ENTER to begin."
               " (q cancels)"
             : "CAL: mag: keep making roll/pitch/yaw swings until"
               " mag accuracy rises. Do not press ENTER."
               " (q cancels)");
        break;

    case IMU_CMD_CAL_PHASE_HOLD:
        puts("CAL: hold the device still for the DCD snapshot."
             " (q cancels)");
        break;

    case IMU_CMD_CAL_PHASE_SAVE:
        puts("CAL: saving DCD; wait.");
        break;

    case IMU_CMD_CAL_PHASE_RESET:
        puts("CAL: reopening the sensor for verification; wait.");
        break;

    case IMU_CMD_CAL_PHASE_VERIFY:
        puts(prompt
             ? "CAL: verify: prepare gentle yaw motion, then press"
               " ENTER to start verification. (q cancels)"
             : "CAL: verify: complete the timed yaw motion and rest."
               " Do not press ENTER. (q cancels)");
        break;

    case IMU_CMD_CAL_PHASE_RESTORE:
        puts("CAL: restoring production reports; wait.");
        break;

    case IMU_CMD_CAL_PHASE_NONE:
    default:
        break;
    }
}

static void print_prompt(const ImuCmdProgress_t *p)
{
    switch (p->active) {
    case IMU_CMD_ID_CALIBRATION:
        print_calibration_prompt(p);
        break;

    case IMU_CMD_ID_TARE:
        if (p->tare.confirmationPending &&
            p->requiredAction == IMU_CMD_ACTION_ALIGN_AND_CONFIRM) {
            printf("TARE: align the device with its reference"
                   " attitude, then press y to tare %s and persist."
                   " (q cancels)\n",
                   p->tare.requestedAxes == IMU_CMD_TARE_AXES_FULL
                       ? "all axes" : "the Z axis");
        } else if (p->tare.phase == IMU_CMD_TARE_PHASE_RESTORE) {
            puts("TARE: restoring production reports; wait.");
        } else {
            puts("TARE: working; keep the aligned device still.");
        }
        break;

    case IMU_CMD_ID_DCD_CLEAR:
        puts("CLEAR: erasing saved DCD calibration; wait.");
        break;

    case IMU_CMD_ID_TARE_CLEAR:
        puts("CLEAR: erasing active and saved tare; wait.");
        break;

    case IMU_CMD_ID_SETTLE:
        puts("restoring IMU production reports; wait.");
        break;

    default:
        break;
    }
}

static void report_progress(
    TwgStage *stage,
    const ImuCmdProgress_t *p,
    uint64_t nowNs)
{
    uint32_t key = prompt_key(p);

    if (key != stage->promptKey) {
        stage->promptKey = key;
        print_prompt(p);
    }

    if (p->active == IMU_CMD_ID_CALIBRATION &&
        nowNs - stage->lastStatusNs >= TWG_STATUS_PERIOD_NS) {
        stage->lastStatusNs = nowNs;
        printf("  cal status: accel=%u gyro=%u mag=%u rv=%u"
               " gate=%u sustained_ms=%" PRIu64 "\n",
               (unsigned)p->cal.accelAccuracy,
               (unsigned)p->cal.gyroAccuracy,
               (unsigned)p->cal.magAccuracy,
               (unsigned)p->cal.rvAccuracy,
               p->cal.gatePassingNow ? 1u : 0u,
               p->cal.sustainedGoodNs / UINT64_C(1000000));
    }
}

static void report_results(TwgStage *stage)
{
    unsigned id;

    for (id = (unsigned)IMU_CMD_ID_CALIBRATION;
         id < (unsigned)IMU_CMD_ID_COUNT;
         ++id) {
        ImuCmdResult_t result;

        if (stage->resultPrinted[id] ||
            !imu_cmd_get_result((ImuCmdIdentity_t)id, &result) ||
            result.state == IMU_CMD_STATE_NOT_REQUESTED ||
            result.state == IMU_CMD_STATE_RUNNING) {
            continue;
        }

        printf("result %s: %s reason=%d warning=%u restored=%u"
               " epoch %u->%u\n",
               identity_name(result.identity),
               state_name(result.state),
               (int)result.reason,
               result.warningRequired ? 1u : 0u,
               result.restoredProduction ? 1u : 0u,
               (unsigned)result.epochBefore,
               (unsigned)result.epochAfter);

        stage->resultPrinted[id] = true;
    }
}

static bool service_imu_plan(
    TwgStage *stage,
    uint64_t nowNs)
{
    ImuCmdIdentity_t active;
    ImuCmdProgress_t progress;

    imu_session_service();

    active = active_command();

    if (!pump_active_adapter(active) ||
        !post_event(IMU_CMD_EVENT_TICK, nowNs)) {
        return false;
    }

    if (active == IMU_CMD_ID_SETTLE &&
        !service_settle(stage, nowNs)) {
        return false;
    }

    if (!imu_cmd_get_progress(&progress)) {
        return false;
    }

    if (stage->mode == TWG_STAGE_MODE_CLEAR_ALL &&
        stage->clearConfirmed &&
        clear_confirm_pending(&progress) &&
        !post_event(IMU_CMD_EVENT_OPERATOR_CONFIRM, nowNs)) {
        return false;
    }

    imu_cmd_service();

    if (!imu_cmd_get_progress(&progress)) {
        return false;
    }

    report_progress(stage, &progress, nowNs);
    report_results(stage);

    if (progress.active == IMU_CMD_ID_ACQUISITION) {
        stage->imuPlanActive = false;
        stage->mode = TWG_STAGE_MODE_ACQUISITION;
        stage->clearConfirmed = false;
        stage->lastActive = IMU_CMD_ID_NONE;
        puts("acquisition resumed.");
        return true;
    }

    if (imu_cmd_plan_complete()) {
        fprintf(stderr,
                "IMU command plan ended without acquisition"
                " (do_not_acquire=%u)\n",
                imu_cmd_do_not_acquire() ? 1u : 0u);
        return false;
    }

    return true;
}

static void consume_acquisition_key(
    TwgStage *stage,
    char key)
{
    switch (key) {
    case 'c':
        stage->requestedMode = TWG_STAGE_MODE_CALIBRATION;
        break;

    case 'z':
        stage->requestedMode = TWG_STAGE_MODE_TARE_Z;
        break;

    case 'a':
        stage->requestedMode = TWG_STAGE_MODE_TARE_FULL;
        break;

    case 'w':
        stage->pressureZeroRequested = true;
        stage->mode = TWG_STAGE_MODE_PRESSURE_ZERO;
        puts("pressure zero started: hold still; q cancels.");
        break;

    case 'x':
        stage->mode = TWG_STAGE_MODE_CLEAR_CONFIRM;
        puts("WARNING: press y to clear DCD calibration, IMU tare"
             " and pressure reference; q cancels.");
        break;

    default:
        break;
    }
}

static bool consume_action_key(
    TwgStage *stage,
    char key,
    uint64_t nowNs)
{
    ImuCmdProgress_t progress;

    if (stage->mode == TWG_STAGE_MODE_CLEAR_CONFIRM) {
        if (key == 'q') {
            stage->mode = TWG_STAGE_MODE_ACQUISITION;
            puts("full clear cancelled.");
        } else if (key == 'y') {
            stage->clearConfirmed = true;
            stage->pressureClearRequested = true;
            stage->requestedMode = TWG_STAGE_MODE_CLEAR_ALL;
            puts("full clear confirmed.");
        }
        return true;
    }

    if (stage->mode == TWG_STAGE_MODE_PRESSURE_ZERO) {
        if (key == 'q') {
            stage->pressureZeroCancelled = true;
            stage->mode = TWG_STAGE_MODE_ACQUISITION;
            puts("pressure zero cancelled.");
        }
        return true;
    }

    if (!stage->imuPlanActive ||
        !imu_cmd_get_progress(&progress)) {
        return true;
    }

    if (key == 'q') {
        return post_event(IMU_CMD_EVENT_OPERATOR_Q, nowNs);
    }

    if ((key == '\n' || key == ' ') &&
        cal_enter_ready(&progress)) {
        return post_event(IMU_CMD_EVENT_OPERATOR_CONFIRM, nowNs);
    }

    if (key == 'y' &&
        progress.active == IMU_CMD_ID_TARE &&
        progress.requiredAction == IMU_CMD_ACTION_ALIGN_AND_CONFIRM) {
        return post_event(IMU_CMD_EVENT_OPERATOR_CONFIRM, nowNs);
    }

    return true;
}

bool twg_stage_init(
    TwgStage *stage,
    uint8_t flightCalMask)
{
    ImuSampleSnapshot_t snapshot;

    if (stage == NULL ||
        !imu_session_get_snapshot(&snapshot) ||
        snapshot.readerState != IMU_READER_STATE_CONFIGURING ||
        !imu_session_configure_production(flightCalMask) ||
        !imu_session_begin_settle()) {
        return false;
    }

    memset(stage, 0, sizeof(*stage));

    stage->flightCalMask = flightCalMask;
    stage->mode = TWG_STAGE_MODE_RESTORE;
    stage->settleStarted = true;
    stage->initialized = true;

    return true;
}

TwgStageStatus twg_stage_turn(
    TwgStage *stage,
    TwgTerm *term,
    uint64_t nowNs,
    bool processStop)
{
    TwgTermKey key;

    if (stage == NULL ||
        term == NULL ||
        !stage->initialized) {
        return TWG_STAGE_ERROR;
    }

    if (processStop) {
        stage->mode = TWG_STAGE_MODE_STOPPED;

        if (stage->imuPlanActive &&
            !stage->processStopPosted) {
            if (!post_event(IMU_CMD_EVENT_PROCESS_STOP, nowNs)) {
                return TWG_STAGE_ERROR;
            }
            stage->processStopPosted = true;
        }

        return TWG_STAGE_PROCESS_STOP;
    }

    imu_session_service();

    if (twg_term_input_error(term)) {
        return TWG_STAGE_ERROR;
    }

    if (stage->mode == TWG_STAGE_MODE_RESTORE) {
        if (nowNs >= stage->settleDeadlineNs) {
            if (!imu_session_mark_operational()) {
                return TWG_STAGE_ERROR;
            }

            stage->mode = TWG_STAGE_MODE_ACQUISITION;
            stage->settleStarted = false;
            puts("acquisition ready:"
                 " c=cal z=tare-z a=tare-full"
                 " w=pressure-zero x=clear");
        }

        return TWG_STAGE_OK;
    }

    if (twg_term_dequeue(term, &key)) {
        if (stage->mode == TWG_STAGE_MODE_ACQUISITION) {
            consume_acquisition_key(stage, key.key);
        } else if (!consume_action_key(stage, key.key, nowNs)) {
            return TWG_STAGE_ERROR;
        }
    }

    if (stage->requestedMode != TWG_STAGE_MODE_ACQUISITION) {
        TwgStageMode requested = stage->requestedMode;

        stage->requestedMode = TWG_STAGE_MODE_ACQUISITION;

        if (!start_imu_plan(stage, requested)) {
            return TWG_STAGE_ERROR;
        }
    }

    if (stage->imuPlanActive &&
        !service_imu_plan(stage, nowNs)) {
        return TWG_STAGE_ERROR;
    }

    return TWG_STAGE_OK;
}

TwgStageMode twg_stage_mode(const TwgStage *stage)
{
    return stage != NULL ? stage->mode : TWG_STAGE_MODE_ERROR;
}

bool twg_stage_take_pressure_zero_request(TwgStage *stage)
{
    bool requested;

    if (stage == NULL) {
        return false;
    }

    requested = stage->pressureZeroRequested;
    stage->pressureZeroRequested = false;

    return requested;
}

bool twg_stage_take_pressure_zero_cancel(TwgStage *stage)
{
    bool cancelled;

    if (stage == NULL) {
        return false;
    }

    cancelled = stage->pressureZeroCancelled;
    stage->pressureZeroCancelled = false;

    return cancelled;
}

bool twg_stage_take_pressure_clear(TwgStage *stage)
{
    bool requested;

    if (stage == NULL) {
        return false;
    }

    requested = stage->pressureClearRequested;
    stage->pressureClearRequested = false;

    return requested;
}

bool twg_stage_acquisition_active(const TwgStage *stage)
{
    return stage != NULL &&
           stage->mode == TWG_STAGE_MODE_ACQUISITION;
}

void twg_stage_pressure_zero_finished(TwgStage *stage)
{
    if (stage != NULL &&
        stage->mode == TWG_STAGE_MODE_PRESSURE_ZERO) {
        stage->mode = TWG_STAGE_MODE_ACQUISITION;
        puts("acquisition resumed.");
    }
}
