#include <stdint.h>
#include <string.h>

#include "app/imu_check.h"

typedef enum {
    CHECK_IDLE = 0,
    CHECK_WAIT_CONFIG,
    CHECK_MONITOR,
    CHECK_WAIT_RESTORE,
    CHECK_DONE
} CheckState_t;

static bool s_inited;
static CheckState_t s_state;
static uint64_t s_nowNs;
static uint64_t s_goodSinceNs;
static bool s_goodArmed;
static uint32_t s_configEpoch;
static uint8_t s_flightMask;
static ImuCheckPendingRequest_t s_pending;
static ImuCmdProgress_t s_progress;
static ImuCmdResult_t s_result;
static ImuCmdResultState_t s_terminalState;
static ImuCmdReason_t s_terminalReason;

static uint64_t add_saturated(uint64_t a, uint64_t b)
{
    return UINT64_MAX - a < b ? UINT64_MAX : a + b;
}

static void clear_pending(void)
{
    memset(&s_pending, 0, sizeof(s_pending));
    s_pending.type = IMU_CHECK_REQ_NONE;
}

static void set_pending(ImuCheckRequestType_t type)
{
    clear_pending();
    s_pending.type = type;
    if (type == IMU_CHECK_REQ_CONFIGURE_CHECK) {
        s_pending.identity = s_result.identity;
        s_pending.effectiveMask = s_progress.check.effectiveMask;
    } else if (type == IMU_CHECK_REQ_RESTORE_PRODUCTION) {
        s_pending.flightCalMask = s_flightMask;
    }
}

static void enter_phase(CheckState_t state, ImuCmdCheckPhase_t phase)
{
    s_state = state;
    s_progress.check.phase = phase;
    s_progress.check.stateEntryNs = s_nowNs;
}

static void update_remaining(void)
{
    ImuCmdCheckProgress_t *p = &s_progress.check;

    if (!p->deadlineValid) {
        p->remainingNs = 0ull;
    } else {
        p->remainingNs = s_nowNs >= p->deadlineNs
                       ? 0ull : p->deadlineNs - s_nowNs;
    }
    s_progress.probeTimeValid = p->deadlineValid;
    s_progress.probeRemainingNs = p->remainingNs;
}

static void update_gate(void)
{
    ImuCmdCheckProgress_t *p = &s_progress.check;
    bool passing;

    if (s_state != CHECK_MONITOR) {
        return;
    }
    passing = p->factsEpochMatched && p->haveAccel && p->haveMag &&
              p->accelStatus >= IMU_CHECK_ACCURACY_GOAL &&
              p->magStatus >= IMU_CHECK_ACCURACY_GOAL;
    p->gatePassingNow = passing;
    if (!passing) {
        s_goodArmed = false;
        p->sustainedGoodNs = 0ull;
        return;
    }
    if (!s_goodArmed) {
        s_goodSinceNs = s_nowNs;
        s_goodArmed = true;
    }
    p->sustainedGoodNs = s_nowNs - s_goodSinceNs;
    if (p->sustainedGoodNs >= IMU_CHECK_GATE_NS) {
        p->gateReached = true;
    }
}

static void finish(ImuCmdResultState_t state, ImuCmdReason_t reason,
                   bool restored)
{
    enter_phase(CHECK_DONE, IMU_CMD_CHECK_PHASE_COMPLETE);
    update_remaining();
    s_result.state = state;
    s_result.reason = reason;
    s_result.warningRequired =
        state == IMU_CMD_STATE_FAILED ||
        state == IMU_CMD_STATE_RECOVERY_FAILED;
    s_result.endedNs = s_nowNs;
    s_result.restoredProduction = restored;
    s_result.sub.probeReachedGate = s_progress.check.gateReached;
    s_result.sub.probeTimedOut =
        s_result.identity == IMU_CMD_ID_PROBE &&
        reason == IMU_CMD_REASON_PROBE_DEADLINE;
    s_result.sub.probeOperatorEndedEarly =
        s_result.identity == IMU_CMD_ID_PROBE &&
        reason == IMU_CMD_REASON_OPERATOR_Q;
    s_result.terminalProgress = s_progress;
    s_result.terminalProgress.version = IMU_CMD_PROGRESS_VERSION;
    s_progress.active = IMU_CMD_ID_NONE;
    s_progress.requiredAction = IMU_CMD_ACTION_NONE;
    clear_pending();
}

static void begin_restore(ImuCmdResultState_t state, ImuCmdReason_t reason)
{
    s_terminalState = state;
    s_terminalReason = reason;
    enter_phase(CHECK_WAIT_RESTORE, IMU_CMD_CHECK_PHASE_RESTORE);
    s_progress.requiredAction = IMU_CMD_ACTION_NONE;
    set_pending(IMU_CHECK_REQ_RESTORE_PRODUCTION);
}

bool imu_check_init(ImuCmdIdentity_t identity, bool probeMaskPresent,
                    uint8_t probeMask, uint8_t flightCalMask,
                    uint64_t nowNs)
{
    if ((identity != IMU_CMD_ID_CHECK && identity != IMU_CMD_ID_PROBE) ||
        (identity == IMU_CMD_ID_CHECK && probeMaskPresent) ||
        (identity == IMU_CMD_ID_PROBE && !probeMaskPresent)) {
        s_inited = false;
        s_state = CHECK_IDLE;
        clear_pending();
        return false;
    }

    memset(&s_progress, 0, sizeof(s_progress));
    memset(&s_result, 0, sizeof(s_result));
    s_inited = true;
    s_nowNs = nowNs;
    s_goodSinceNs = 0ull;
    s_goodArmed = false;
    s_configEpoch = 0u;
    s_flightMask = flightCalMask;
    s_terminalState = IMU_CMD_STATE_RUNNING;
    s_terminalReason = IMU_CMD_REASON_NONE;
    s_progress.version = IMU_CMD_PROGRESS_VERSION;
    s_progress.active = identity;
    s_progress.requiredAction = IMU_CMD_ACTION_PRESS_Q_TO_END;
    s_progress.check.identity = identity;
    s_progress.check.effectiveMask =
        identity == IMU_CMD_ID_CHECK ? 0u : probeMask;
    s_progress.check.requestedMaskValid = probeMaskPresent;
    s_progress.check.requestedMask =
        probeMaskPresent ? probeMask : 0u;
    if (identity == IMU_CMD_ID_PROBE) {
        s_progress.check.deadlineValid = true;
        s_progress.check.deadlineNs =
            add_saturated(nowNs, IMU_CHECK_PROBE_NS);
    }
    update_remaining();
    s_result.version = IMU_CMD_RESULT_VERSION;
    s_result.identity = identity;
    s_result.state = IMU_CMD_STATE_RUNNING;
    s_result.reason = IMU_CMD_REASON_NONE;
    s_result.startedNs = nowNs;
    s_result.probeMaskRequestedValid = probeMaskPresent;
    s_result.probeMaskRequested = probeMaskPresent ? probeMask : 0u;
    enter_phase(CHECK_WAIT_CONFIG, IMU_CMD_CHECK_PHASE_CONFIGURE);
    set_pending(IMU_CHECK_REQ_CONFIGURE_CHECK);
    return true;
}

static bool post_session(const ImuCheckSessionResult_t *sr)
{
    ImuCheckRequestType_t type;

    if (s_pending.type == IMU_CHECK_REQ_NONE ||
        sr->type != s_pending.type) {
        return false;
    }
    type = s_pending.type;
    clear_pending();
    if (s_result.epochBefore == 0u) {
        s_result.epochBefore = sr->epochBefore;
    }
    s_result.epochAfter = sr->epochAfter;

    if (s_state == CHECK_WAIT_CONFIG &&
        type == IMU_CHECK_REQ_CONFIGURE_CHECK) {
        s_progress.check.actualMaskValid = sr->actualMaskValid;
        s_progress.check.actualMask =
            sr->actualMaskValid ? sr->actualMask : 0u;
        s_result.sub.probeMaskActualValid =
            s_result.identity == IMU_CMD_ID_PROBE && sr->actualMaskValid;
        s_result.sub.probeMaskActual =
            s_result.sub.probeMaskActualValid ? sr->actualMask : 0u;
        if (!sr->success || sr->epochAfter == 0u) {
            begin_restore(IMU_CMD_STATE_FAILED, IMU_CMD_REASON_CONFIG_FAILED);
            return true;
        }
        s_configEpoch = sr->epochAfter;
        enter_phase(CHECK_MONITOR, IMU_CMD_CHECK_PHASE_MONITOR);
        return true;
    }
    if (s_state == CHECK_WAIT_RESTORE &&
        type == IMU_CHECK_REQ_RESTORE_PRODUCTION) {
        if (!sr->success) {
            finish(IMU_CMD_STATE_RECOVERY_FAILED,
                   IMU_CMD_REASON_SESSION_UNUSABLE, false);
        } else {
            finish(s_terminalState, s_terminalReason, true);
        }
        return true;
    }
    return false;
}

bool imu_check_post(const ImuCheckEvent_t *event)
{
    if (!s_inited || event == NULL || s_state == CHECK_IDLE) {
        return false;
    }
    if (s_state == CHECK_DONE) {
        return event->type == IMU_CHECK_EVENT_TICK ||
               event->type == IMU_CHECK_EVENT_OPERATOR_Q;
    }
    switch (event->type) {
    case IMU_CHECK_EVENT_TICK:
        if (event->monotonicNs > s_nowNs) {
            s_nowNs = event->monotonicNs;
        }
        update_remaining();
        return true;
    case IMU_CHECK_EVENT_OPERATOR_Q:
        if (s_state == CHECK_WAIT_RESTORE) {
            return true;
        }
        if (s_state == CHECK_WAIT_CONFIG) {
            /* The first session action has not executed: no restore needed. */
            finish(IMU_CMD_STATE_CANCELLED, IMU_CMD_REASON_OPERATOR_Q,
                   false);
        } else {
            update_gate();
            begin_restore(IMU_CMD_STATE_CANCELLED,
                          IMU_CMD_REASON_OPERATOR_Q);
        }
        return true;
    case IMU_CHECK_EVENT_SESSION_RESULT:
        return post_session(&event->session);
    default:
        return false;
    }
}

bool imu_check_feed_facts(const ImuCheckFacts_t *facts)
{
    ImuCmdCheckProgress_t *p;

    if (!s_inited || facts == NULL || s_state == CHECK_DONE ||
        facts->version != IMU_CHECK_FACTS_VERSION) {
        return false;
    }
    if (s_state != CHECK_MONITOR || !facts->valid ||
        facts->configurationEpoch != s_configEpoch) {
        return true; /* Ignore absent, premature, or different-epoch facts. */
    }

    p = &s_progress.check;
    p->factsEpoch = facts->configurationEpoch;
    p->factsEpochMatched = true;
    p->haveAccel = facts->haveAccel;
    p->haveGyro = facts->haveGyro;
    p->haveMag = facts->haveMag;
    p->haveRv = facts->haveRv;
    p->accelHostDecodeNs = facts->accelHostDecodeNs;
    p->gyroHostDecodeNs = facts->gyroHostDecodeNs;
    p->magHostDecodeNs = facts->magHostDecodeNs;
    p->rvHostDecodeNs = facts->rvHostDecodeNs;
    p->accelStatus = facts->accelStatus;
    p->gyroStatus = facts->gyroStatus;
    p->magStatus = facts->magStatus;
    p->rvStatus = facts->rvStatus;
    p->rvErrRad = facts->rvErrRad;
    p->magXuT = facts->magXuT;
    p->magYuT = facts->magYuT;
    p->magZuT = facts->magZuT;
    update_gate();
    return true;
}

void imu_check_service(void)
{
    if (!s_inited || s_state != CHECK_MONITOR) {
        return;
    }
    update_gate();
    if (s_result.identity == IMU_CMD_ID_PROBE &&
        s_nowNs >= s_progress.check.deadlineNs) {
        begin_restore(IMU_CMD_STATE_TIMED_OUT,
                      IMU_CMD_REASON_PROBE_DEADLINE);
    }
}

ImuCheckPendingRequest_t imu_check_pending_request(void)
{
    ImuCheckPendingRequest_t none;

    memset(&none, 0, sizeof(none));
    return s_inited ? s_pending : none;
}

bool imu_check_get_progress(ImuCmdProgress_t *out)
{
    if (!s_inited || out == NULL) {
        return false;
    }
    *out = s_progress;
    return true;
}

bool imu_check_get_result(ImuCmdResult_t *out)
{
    if (!s_inited || out == NULL) {
        return false;
    }
    *out = s_result;
    return true;
}

bool imu_check_complete(void)
{
    return s_inited && s_state == CHECK_DONE;
}
