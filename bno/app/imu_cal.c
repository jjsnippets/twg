#include "app/imu_cal.h"

#include <string.h>

typedef enum {
    IMU_CAL_ST_IDLE = 0,
    IMU_CAL_ST_CLEAR_CONFIRM,
    IMU_CAL_ST_WAIT_CLEAR,
    IMU_CAL_ST_WAIT_CONFIG,
    IMU_CAL_ST_ACCEL_PROMPT,
    IMU_CAL_ST_ACCEL_WINDOW,
    IMU_CAL_ST_ACCEL_GATE,
    IMU_CAL_ST_GYRO_PROMPT,
    IMU_CAL_ST_GYRO_GATE,
    IMU_CAL_ST_MAG_PROMPT,
    IMU_CAL_ST_MAG_MOTION,
    IMU_CAL_ST_MAG_GATE,
    IMU_CAL_ST_HOLD_WINDOW,
    IMU_CAL_ST_HOLD_GATE,
    IMU_CAL_ST_WAIT_SAVE,
    IMU_CAL_ST_SAVE_RETRY_HOLD,
    IMU_CAL_ST_WAIT_REOPEN,
    IMU_CAL_ST_WAIT_VERIFY_CONFIG,
    IMU_CAL_ST_VERIFY_PROMPT,
    IMU_CAL_ST_VERIFY_MOTION,
    IMU_CAL_ST_VERIFY_GATE,
    IMU_CAL_ST_WAIT_RESTORE,
    IMU_CAL_ST_DONE
} ImuCalState_t;

static bool s_inited;
static bool s_clearMode;
static ImuCalState_t s_state;
static uint8_t s_flightMask;
static uint64_t s_nowNs;
static uint64_t s_stateEntryNs;
static uint64_t s_deadlineNs;
static uint64_t s_goodSinceNs;
static uint8_t s_poseIndex;
static uint8_t s_accelRound;
static uint8_t s_magRound;
static uint8_t s_saveAttempt;
static bool s_saveRetryUsed;
static bool s_dcdSaved;
static bool s_verified;
static ImuCalPendingRequest_t s_pending;
static ImuCmdResult_t s_result;
static ImuCmdProgress_t s_progress;
static ImuCmdResultState_t s_termState;
static ImuCmdReason_t s_termReason;
static bool s_termWarning;
static ImuCalFacts_t s_facts;

static void clear_pending(void)
{
    memset(&s_pending, 0, sizeof(s_pending));
    s_pending.type = IMU_CAL_REQ_NONE;
}

static void set_pending(ImuCalRequestType_t type, uint8_t mask)
{
    s_pending.type = type;
    s_pending.calMask = mask;
}

static uint64_t remaining_ns(void)
{
    if (s_deadlineNs == 0ull || s_nowNs >= s_deadlineNs) {
        return 0ull;
    }
    return s_deadlineNs - s_nowNs;
}

static void enter_state(ImuCalState_t state, ImuCmdCalPhase_t phase,
                        uint64_t durationNs, ImuCmdOperatorAction_t action)
{
    s_state = state;
    s_stateEntryNs = s_nowNs;
    s_deadlineNs = durationNs == 0ull ? 0ull : s_nowNs + durationNs;
    s_goodSinceNs = 0ull;
    s_progress.cal.phase = phase;
    s_progress.requiredAction = action;
    s_progress.cal.stateEntryNs = s_stateEntryNs;
    s_progress.cal.deadlineNs = s_deadlineNs;
    s_progress.cal.remainingNs = durationNs;
    s_progress.cal.sustainedGoodNs = 0ull;
}

static void reset_progress(void)
{
    memset(&s_progress, 0, sizeof(s_progress));
    s_progress.version = IMU_CMD_PROGRESS_VERSION;
    s_progress.active = IMU_CMD_ID_CALIBRATION;
    s_progress.requiredAction = IMU_CMD_ACTION_PRESS_Q_TO_END;
    s_progress.cal.phase = IMU_CMD_CAL_PHASE_STARTUP;
    s_progress.cal.totalPoseCount = IMU_CAL_ACCEL_POSE_COUNT;
}

static void reset_result(void)
{
    memset(&s_result, 0, sizeof(s_result));
    s_result.version = IMU_CMD_RESULT_VERSION;
    s_result.identity = IMU_CMD_ID_CALIBRATION;
    s_result.state = IMU_CMD_STATE_RUNNING;
    s_result.reason = IMU_CMD_REASON_NONE;
}

static void begin_restore(ImuCmdResultState_t state, ImuCmdReason_t reason, bool warning)
{
    s_termState = state;
    s_termReason = reason;
    s_termWarning = warning;
    enter_state(IMU_CAL_ST_WAIT_RESTORE, IMU_CMD_CAL_PHASE_RESTORE, 0ull,
                IMU_CMD_ACTION_NONE);
    set_pending(IMU_CAL_REQ_RESTORE_PRODUCTION, s_flightMask);
}

static void finish(ImuCmdResultState_t state, ImuCmdReason_t reason, bool warning, bool restored)
{
    s_result.state = state;
    s_result.reason = reason;
    s_result.warningRequired = warning;
    s_result.endedNs = s_nowNs;
    s_result.dcdSaved = s_dcdSaved;
    s_result.verified = s_verified;
    s_result.restoredProduction = restored;
    s_result.terminalProgress = s_progress;
    s_result.terminalProgress.version = IMU_CMD_PROGRESS_VERSION;
    s_progress.active = IMU_CMD_ID_NONE;
    s_progress.requiredAction = IMU_CMD_ACTION_NONE;
    s_progress.cal.phase = IMU_CMD_CAL_PHASE_NONE;
    clear_pending();
    s_state = IMU_CAL_ST_DONE;
}

static bool gate_accel(void)
{
    return s_facts.valid &&
           s_facts.accelAccuracy >= IMU_CAL_ACCURACY_GOAL;
}

static bool gate_gyro(void)
{
    return s_facts.valid &&
           s_facts.gyroAccuracy >= IMU_CAL_ACCURACY_GOAL;
}

static bool gate_mag(void)
{
    return s_facts.valid &&
           s_facts.magAccuracy >= IMU_CAL_ACCURACY_GOAL;
}

static bool gate_verify(void)
{
    return s_facts.valid &&
           s_facts.accelAccuracy >= IMU_CAL_ACCURACY_GOAL &&
           s_facts.magAccuracy >= IMU_CAL_ACCURACY_GOAL;
}

static bool gate_sustained(bool passing)
{
    if (!passing) {
        s_goodSinceNs = 0ull;
        s_progress.cal.sustainedGoodNs = 0ull;
        return false;
    }
    if (s_goodSinceNs == 0ull) {
        s_goodSinceNs = s_nowNs;
    }
    s_progress.cal.sustainedGoodNs = s_nowNs - s_goodSinceNs;
    return s_progress.cal.sustainedGoodNs >= IMU_CAL_SUSTAINED_GOOD_NS;
}

static void begin_mag_round(void)
{
    s_magRound += 1u;
    s_progress.cal.magRound = s_magRound;
    enter_state(IMU_CAL_ST_MAG_PROMPT, IMU_CMD_CAL_PHASE_MAG, 0ull,
                IMU_CMD_ACTION_PRESS_Q_TO_END);
}

static void begin_outer_attempt(void)
{
    s_saveAttempt += 1u;
    s_magRound = 0u;
    s_saveRetryUsed = false;
    s_progress.cal.saveAttempt = s_saveAttempt;
    begin_mag_round();
}

static void begin_hold(void)
{
    enter_state(IMU_CAL_ST_HOLD_WINDOW, IMU_CMD_CAL_PHASE_HOLD,
                IMU_CAL_HOLD_WINDOW_NS, IMU_CMD_ACTION_PRESS_Q_TO_END);
}

static void begin_save(void)
{
    s_progress.cal.saveAttempt = s_saveAttempt;
    enter_state(IMU_CAL_ST_WAIT_SAVE, IMU_CMD_CAL_PHASE_SAVE, 0ull,
                IMU_CMD_ACTION_PRESS_Q_TO_END);
    set_pending(IMU_CAL_REQ_SAVE_DCD, 0u);
}

static void begin_verify_reopen(void)
{
    enter_state(IMU_CAL_ST_WAIT_REOPEN, IMU_CMD_CAL_PHASE_RESET, 0ull,
                IMU_CMD_ACTION_PRESS_Q_TO_END);
    set_pending(IMU_CAL_REQ_VERIFY_REOPEN, 0u);
}

bool imu_cal_init(uint8_t flightCalMask, uint64_t nowNs)
{
    s_inited = true;
    s_clearMode = false;
    s_flightMask = flightCalMask;
    s_nowNs = nowNs;
    s_stateEntryNs = nowNs;
    s_deadlineNs = 0ull;
    s_goodSinceNs = 0ull;
    s_poseIndex = 0u;
    s_accelRound = 1u;
    s_magRound = 0u;
    s_saveAttempt = 0u;
    s_saveRetryUsed = false;
    s_dcdSaved = false;
    s_verified = false;
    s_termReason = IMU_CMD_REASON_NONE;
    s_termWarning = false;
    memset(&s_facts, 0, sizeof(s_facts));
    reset_result();
    reset_progress();
    s_result.startedNs = nowNs;
    enter_state(IMU_CAL_ST_WAIT_CONFIG, IMU_CMD_CAL_PHASE_STARTUP, 0ull,
                IMU_CMD_ACTION_PRESS_Q_TO_END);
    set_pending(IMU_CAL_REQ_CONFIGURE_CALIBRATION, IMU_CAL_ENABLE_MASK);
    return true;
}

bool imu_cal_init_dcd_clear(uint8_t flightCalMask, uint64_t nowNs)
{
    s_inited = true;
    s_clearMode = true;
    s_flightMask = flightCalMask;
    s_nowNs = nowNs;
    s_stateEntryNs = nowNs;
    s_deadlineNs = 0ull;
    s_goodSinceNs = 0ull;
    s_dcdSaved = false;
    s_verified = false;
    s_termReason = IMU_CMD_REASON_NONE;
    s_termWarning = false;
    memset(&s_facts, 0, sizeof(s_facts));
    clear_pending();
    reset_result();
    reset_progress();
    s_result.identity = IMU_CMD_ID_DCD_CLEAR;
    s_result.startedNs = nowNs;
    s_progress.active = IMU_CMD_ID_DCD_CLEAR;
    s_progress.cal.phase = IMU_CMD_CAL_PHASE_NONE;
    s_progress.requiredAction = IMU_CMD_ACTION_CONFIRM;
    s_state = IMU_CAL_ST_CLEAR_CONFIRM;
    return true;
}

static bool post_tick(uint64_t ns)
{
    s_nowNs = ns;
    return true;
}

static bool post_q(void)
{
    if (s_clearMode) {
        if (s_state == IMU_CAL_ST_CLEAR_CONFIRM ||
            s_state == IMU_CAL_ST_WAIT_CLEAR) {
            /*
            * A still-pending clear has not reached the adapter. Drop it;
             * do not claim a reset, restoration, or configuration epoch.
             */
            finish(IMU_CMD_STATE_CANCELLED, IMU_CMD_REASON_OPERATOR_Q,
                   true, false);
        }
        return true;
    }

    if (s_state == IMU_CAL_ST_IDLE || s_state == IMU_CAL_ST_DONE ||
        s_state == IMU_CAL_ST_WAIT_RESTORE) {
        return true;
    }

    begin_restore(IMU_CMD_STATE_CANCELLED, IMU_CMD_REASON_OPERATOR_Q, true);
    return true;
}

static bool post_confirm(void)
{
    switch (s_state) {
        case IMU_CAL_ST_CLEAR_CONFIRM:
            s_progress.requiredAction = IMU_CMD_ACTION_NONE;
            s_state = IMU_CAL_ST_WAIT_CLEAR;
            s_stateEntryNs = s_nowNs;
            set_pending(IMU_CAL_REQ_CLEAR_DCD, 0u);
            return true;
    case IMU_CAL_ST_ACCEL_PROMPT:
        enter_state(IMU_CAL_ST_ACCEL_WINDOW, IMU_CMD_CAL_PHASE_ACCEL,
                    IMU_CAL_ACCEL_FACE_WINDOW_NS,
                    IMU_CMD_ACTION_PRESS_Q_TO_END);
        return true;
    case IMU_CAL_ST_GYRO_PROMPT:
        enter_state(IMU_CAL_ST_GYRO_GATE, IMU_CMD_CAL_PHASE_GYRO,
                    IMU_CAL_GYRO_GATE_TIMEOUT_NS,
                    IMU_CMD_ACTION_PRESS_Q_TO_END);
        return true;
    case IMU_CAL_ST_MAG_PROMPT:
        enter_state(IMU_CAL_ST_MAG_MOTION, IMU_CMD_CAL_PHASE_MAG,
                    IMU_CAL_MAG_MOTION_NS,
                    IMU_CMD_ACTION_PRESS_Q_TO_END);
        return true;
    case IMU_CAL_ST_VERIFY_PROMPT:
       enter_state(IMU_CAL_ST_VERIFY_MOTION, IMU_CMD_CAL_PHASE_VERIFY,
                    IMU_CAL_VERIFY_MOTION_NS,
                    IMU_CMD_ACTION_PRESS_Q_TO_END);
        return true;
    default:
        return false;
    }
}

static bool post_session(const ImuCalSessionResult_t *sr)
{
    if (sr == NULL || s_pending.type == IMU_CAL_REQ_NONE ||
        sr->type != s_pending.type) {
        return false;
    }
    clear_pending();
    if (s_result.epochBefore == 0u) {
        s_result.epochBefore = sr->epochBefore;
    }
    s_result.epochAfter = sr->epochAfter;

    if (s_state == IMU_CAL_ST_WAIT_CLEAR) {
        if (!sr->sessionUsable) {
            finish(IMU_CMD_STATE_RECOVERY_FAILED,
                   IMU_CMD_REASON_SESSION_UNUSABLE, true, false);
            return true;
        }
        begin_restore(sr->success ? IMU_CMD_STATE_SUCCEEDED
                                  : IMU_CMD_STATE_FAILED,
                      sr->success ? IMU_CMD_REASON_OK
                                  : IMU_CMD_REASON_DCD_CLEAR_FAILED,
                      !sr->success);
        return true;
    }

    if (s_state == IMU_CAL_ST_WAIT_CONFIG) {
        if (!sr->success) {
            begin_restore(IMU_CMD_STATE_FAILED, IMU_CMD_REASON_CAL_CONFIG_FAILED, true);
            return true;
        }
        s_result.epochBefore = sr->epochBefore;
        s_result.epochAfter = sr->epochAfter;
        s_progress.cal.accelRound = s_accelRound;
        s_progress.cal.currentPoseIndex = 1u;
        enter_state(IMU_CAL_ST_ACCEL_PROMPT, IMU_CMD_CAL_PHASE_ACCEL, 0ull,
                    IMU_CMD_ACTION_PRESS_Q_TO_END);
        return true;
    }
    if (s_state == IMU_CAL_ST_WAIT_SAVE) {
        if (!sr->success) {
            if (s_saveRetryUsed) {
                s_saveRetryUsed = false;
                if (s_saveAttempt >= IMU_CAL_MAX_SAVE_ATTEMPTS) {
                    begin_restore(IMU_CMD_STATE_FAILED, IMU_CMD_REASON_CAL_SAVE_FAILED,
                                  true);
                } else {
                    begin_outer_attempt();
                }
                return true;
            }
            s_saveRetryUsed = true;
            enter_state(IMU_CAL_ST_SAVE_RETRY_HOLD, IMU_CMD_CAL_PHASE_HOLD,
                        IMU_CAL_SAVE_RETRY_HOLD_NS,
                        IMU_CMD_ACTION_PRESS_Q_TO_END);
            return true;
        }
        s_dcdSaved = true;
        begin_verify_reopen();
        return true;
    }
    if (s_state == IMU_CAL_ST_WAIT_REOPEN) {
        if (!sr->success) {
            finish(IMU_CMD_STATE_RECOVERY_FAILED, IMU_CMD_REASON_CAL_REOPEN_FAILED,
                   true, false);
            return true;
        }
        enter_state(IMU_CAL_ST_WAIT_VERIFY_CONFIG, IMU_CMD_CAL_PHASE_VERIFY, 0ull,
                    IMU_CMD_ACTION_PRESS_Q_TO_END);
        set_pending(IMU_CAL_REQ_CONFIGURE_CALIBRATION, 0u);
        return true;
    }
    if (s_state == IMU_CAL_ST_WAIT_VERIFY_CONFIG) {
        if (!sr->success) {
            begin_restore(IMU_CMD_STATE_FAILED,
                          IMU_CMD_REASON_CAL_VERIFY_CONFIG_FAILED, true);
            return true;
        }
        enter_state(IMU_CAL_ST_VERIFY_PROMPT, IMU_CMD_CAL_PHASE_VERIFY, 0ull,
                    IMU_CMD_ACTION_PRESS_Q_TO_END);
        return true;
    }
    if (s_state == IMU_CAL_ST_WAIT_RESTORE) {
        if (!sr->success) {
            if (s_clearMode) {
                finish(IMU_CMD_STATE_RECOVERY_FAILED,
                       IMU_CMD_REASON_SESSION_UNUSABLE, true, false);
                return true;
            }
            finish(IMU_CMD_STATE_RECOVERY_FAILED, IMU_CMD_REASON_CAL_RESTORE_FAILED,
                   true, false);
            return true;
        }
        finish(s_termState, s_termReason, s_termWarning, true);
        return true;
    }
    return false;
}

bool imu_cal_post(const ImuCalEvent_t *event)
{
    if (!s_inited || event == NULL || s_state == IMU_CAL_ST_IDLE) {
        return false;
    }
    switch (event->type) {
    case IMU_CAL_EVENT_TICK:
        return post_tick(event->monotonicNs);
    case IMU_CAL_EVENT_OPERATOR_Q:
        return post_q();
    case IMU_CAL_EVENT_OPERATOR_CONFIRM:
        return post_confirm();
    case IMU_CAL_EVENT_SESSION_RESULT:
        return post_session(&event->session);
    default:
        return false;
    }
}

void imu_cal_service(void)
{
    bool passed;

    if (!s_inited || s_state == IMU_CAL_ST_DONE || s_state == IMU_CAL_ST_IDLE) {
        return;
    }
    s_progress.cal.remainingNs = remaining_ns();
    s_progress.cal.stateEntryNs = s_stateEntryNs;
    s_progress.cal.deadlineNs = s_deadlineNs;

    switch (s_state) {
    case IMU_CAL_ST_ACCEL_WINDOW:
        if (s_deadlineNs != 0ull && s_nowNs >= s_deadlineNs) {
            if (s_poseIndex + 1u < IMU_CAL_ACCEL_POSE_COUNT) {
                s_poseIndex += 1u;
                s_progress.cal.currentPoseIndex = s_poseIndex + 1u;
                enter_state(IMU_CAL_ST_ACCEL_PROMPT, IMU_CMD_CAL_PHASE_ACCEL, 0ull,
                            IMU_CMD_ACTION_PRESS_Q_TO_END);
            } else {
                enter_state(IMU_CAL_ST_ACCEL_GATE, IMU_CMD_CAL_PHASE_ACCEL,
                            IMU_CAL_ACCEL_GATE_TIMEOUT_NS,
                            IMU_CMD_ACTION_PRESS_Q_TO_END);
            }
        }
        break;

    case IMU_CAL_ST_ACCEL_GATE:
        if (gate_sustained(gate_accel())) {
            enter_state(IMU_CAL_ST_GYRO_PROMPT, IMU_CMD_CAL_PHASE_GYRO, 0ull,
                        IMU_CMD_ACTION_PRESS_Q_TO_END);
        } else if (s_nowNs >= s_deadlineNs) {
            if (s_accelRound < IMU_CAL_ACCEL_MAX_ROUNDS) {
                s_accelRound += 1u;
                s_poseIndex = 0u;
                s_progress.cal.accelRound = s_accelRound;
                s_progress.cal.currentPoseIndex = 1u;
                enter_state(IMU_CAL_ST_ACCEL_PROMPT, IMU_CMD_CAL_PHASE_ACCEL, 0ull,
                            IMU_CMD_ACTION_PRESS_Q_TO_END);
            } else {
                enter_state(IMU_CAL_ST_GYRO_PROMPT, IMU_CMD_CAL_PHASE_GYRO, 0ull,
                            IMU_CMD_ACTION_PRESS_Q_TO_END);
            }
        }
        break;

    case IMU_CAL_ST_GYRO_GATE:
        if (gate_sustained(gate_gyro()) || s_nowNs >= s_deadlineNs) {
            begin_outer_attempt();
        }
        break;

    case IMU_CAL_ST_MAG_MOTION:
        if (s_nowNs >= s_deadlineNs) {
            enter_state(IMU_CAL_ST_MAG_GATE, IMU_CMD_CAL_PHASE_MAG,
                        IMU_CAL_MAG_GATE_TIMEOUT_NS,
                        IMU_CMD_ACTION_PRESS_Q_TO_END);
        }
        break;

    case IMU_CAL_ST_MAG_GATE:
        if (gate_sustained(gate_mag())) {
            begin_hold();
        } else if (s_nowNs >= s_deadlineNs) {
            if (s_magRound < IMU_CAL_MAG_MAX_ROUNDS) {
                begin_mag_round();
            } else {
                begin_restore(IMU_CMD_STATE_FAILED, IMU_CMD_REASON_CAL_MAG_EXHAUSTED,
                              true);
            }
        }
        break;

    case IMU_CAL_ST_HOLD_WINDOW:
        if (s_nowNs >= s_deadlineNs) {
            enter_state(IMU_CAL_ST_HOLD_GATE, IMU_CMD_CAL_PHASE_HOLD,
                        IMU_CAL_HOLD_GATE_TIMEOUT_NS,
                        IMU_CMD_ACTION_PRESS_Q_TO_END);
        }
        break;

    case IMU_CAL_ST_HOLD_GATE:
        passed = gate_sustained(gate_verify());
        if (passed) {
            begin_save();
        } else if (s_nowNs >= s_deadlineNs) {
            if (s_saveAttempt >= IMU_CAL_MAX_SAVE_ATTEMPTS) {
                begin_restore(IMU_CMD_STATE_FAILED,
                              IMU_CMD_REASON_CAL_HOLD_DEGRADED, true);
            } else {
                begin_outer_attempt();
            }
        }
        break;

    case IMU_CAL_ST_SAVE_RETRY_HOLD:
        if (s_nowNs >= s_deadlineNs) {
            begin_save();
        }
        break;

    case IMU_CAL_ST_VERIFY_MOTION:
        if (s_nowNs >= s_deadlineNs) {
            enter_state(IMU_CAL_ST_VERIFY_GATE, IMU_CMD_CAL_PHASE_VERIFY,
                        IMU_CAL_VERIFY_GATE_TIMEOUT_NS,
                        IMU_CMD_ACTION_PRESS_Q_TO_END);
        }
        break;

    case IMU_CAL_ST_VERIFY_GATE:
        if (gate_sustained(gate_verify())) {
            s_verified = true;
            begin_restore(IMU_CMD_STATE_SUCCEEDED, IMU_CMD_REASON_OK, false);
        } else if (s_nowNs >= s_deadlineNs) {
            begin_restore(IMU_CMD_STATE_FAILED,
                          IMU_CMD_REASON_CAL_VERIFY_GATE_FAILED, true);
        }
        break;

    default:
        break;
    }
}

bool imu_cal_feed_facts(const ImuCalFacts_t *facts)
{
    if (!s_inited || s_clearMode || facts == NULL ||
        s_state == IMU_CAL_ST_DONE) {
        return false;
    }
    s_facts = *facts;
    s_progress.cal.accelAccuracy = s_facts.accelAccuracy;
    s_progress.cal.gyroAccuracy = s_facts.gyroAccuracy;
    s_progress.cal.magAccuracy = s_facts.magAccuracy;
    s_progress.cal.rvAccuracy = s_facts.rvAccuracy;
    s_progress.cal.rvErrRad = s_facts.rvErrRad;
    s_progress.cal.magXuT = s_facts.magXuT;
    s_progress.cal.magYuT = s_facts.magYuT;
    s_progress.cal.magZuT = s_facts.magZuT;
    s_progress.cal.gatePassingNow = gate_accel() || gate_gyro() ||
                                    gate_mag() || gate_verify();
    return true;
}

bool imu_cal_get_progress(ImuCmdProgress_t *out)
{
    if (!s_inited || out == NULL) {
        return false;
    }
    *out = s_progress;
    out->version = IMU_CMD_PROGRESS_VERSION;
    return true;
}

bool imu_cal_get_result(ImuCmdResult_t *out)
{
    if (!s_inited || out == NULL) {
        return false;
    }
    *out = s_result;
    out->version = IMU_CMD_RESULT_VERSION;
    return true;
}

ImuCalPendingRequest_t imu_cal_pending_request(void)
{
    ImuCalPendingRequest_t none;

    memset(&none, 0, sizeof(none));
    none.type = IMU_CAL_REQ_NONE;
    if (!s_inited) {
        return none;
    }
    return s_pending;
}

bool imu_cal_complete(void)
{
    return s_inited && s_state == IMU_CAL_ST_DONE;
}
