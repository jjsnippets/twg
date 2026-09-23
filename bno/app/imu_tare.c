#include "app/imu_tare.h"

#include <string.h>

typedef enum {
    IMU_TARE_ST_IDLE = 0,
    IMU_TARE_ST_WAIT_CONFIRM,
    IMU_TARE_ST_WAIT_CONFIG,
    IMU_TARE_ST_SETTLE,
    IMU_TARE_ST_WAIT_TARE_NOW,
    IMU_TARE_ST_WAIT_PERSIST,
    IMU_TARE_ST_VERIFY,
    IMU_TARE_ST_WAIT_CLEAR,
    IMU_TARE_ST_CHECK,
    IMU_TARE_ST_WAIT_RESTORE,
    IMU_TARE_ST_DONE
} ImuTareState_t;

static bool s_inited;
static ImuTareState_t s_state;
static ImuCmdIdentity_t s_identity;
static ImuCmdTareAxes_t s_axes;
static uint8_t s_flightMask;
static uint64_t s_nowNs;
static uint64_t s_stateEntryNs;
static uint64_t s_deadlineNs;
static uint32_t s_expectedEpoch;
static bool s_haveEvidence;
static bool s_verified;
static ImuTarePendingRequest_t s_pending;
static ImuCmdResult_t s_result;
static ImuCmdProgress_t s_progress;
static ImuCmdResultState_t s_termState;
static ImuCmdReason_t s_termReason;
static bool s_termWarning;
static ImuTareFacts_t s_facts;

static void clear_pending(void)
{
    memset(&s_pending, 0, sizeof(s_pending));
    s_pending.type = IMU_TARE_REQ_NONE;
}

static void set_pending(ImuTareRequestType_t type)
{
    memset(&s_pending, 0, sizeof(s_pending));
    s_pending.type = type;
    if (type == IMU_TARE_REQ_TARE_NOW) {
        s_pending.tareAxes = s_axes;
    }
    if (type == IMU_TARE_REQ_RESTORE_PRODUCTION) {
        s_pending.flightCalMask = s_flightMask;
    }
}

static uint64_t remaining_ns(void)
{
    if (s_deadlineNs == 0ull || s_nowNs >= s_deadlineNs) {
        return 0ull;
    }
    return s_deadlineNs - s_nowNs;
}

static void enter_state(ImuTareState_t state, ImuCmdTarePhase_t phase,
                        uint64_t durationNs, ImuCmdOperatorAction_t action)
{
    s_state = state;
    s_stateEntryNs = s_nowNs;
    s_deadlineNs = (durationNs == 0ull) ? 0ull : (s_nowNs + durationNs);
    s_progress.tare.phase = phase;
    s_progress.requiredAction = action;
    s_progress.tare.stateEntryNs = s_stateEntryNs;
    s_progress.tare.deadlineNs = s_deadlineNs;
    s_progress.tare.remainingNs = durationNs;
    s_progress.tare.confirmationPending = (state == IMU_TARE_ST_WAIT_CONFIRM);
}

static void apply_facts_to_progress(const ImuTareFacts_t *facts)
{
    bool match;

    s_progress.tare.attitudeValid = facts->valid;
    s_progress.tare.rotationEpoch = facts->configurationEpoch;
    s_progress.tare.rotationEventSequence = facts->rotationEventSequence;
    s_progress.tare.rotationHostDecodeNs = facts->hostDecodeNs;
    s_progress.tare.quatI = facts->quatI;
    s_progress.tare.quatJ = facts->quatJ;
    s_progress.tare.quatK = facts->quatK;
    s_progress.tare.quatReal = facts->quatReal;
    s_progress.tare.yawRad = facts->yawRad;
    s_progress.tare.pitchRad = facts->pitchRad;
    s_progress.tare.rollRad = facts->rollRad;
    s_progress.tare.oriErrRad = facts->oriErrRad;
    s_progress.tare.rotationStatus = facts->rotationStatus;
    match = facts->valid &&
            (s_expectedEpoch == 0u ||
             facts->configurationEpoch == s_expectedEpoch);
    s_progress.tare.attitudeEpochMatched = match;
}

static bool init_legal(ImuCmdIdentity_t identity, ImuCmdTareAxes_t axes)
{
    if (identity == IMU_CMD_ID_TARE) {
        return axes == IMU_CMD_TARE_AXES_Z || axes == IMU_CMD_TARE_AXES_FULL;
    }
    if (identity == IMU_CMD_ID_TARE_CLEAR ||
        identity == IMU_CMD_ID_TARE_CHECK) {
        return axes == IMU_CMD_TARE_AXES_NONE;
    }
    return false;
}

static void reset_progress(void)
{
    memset(&s_progress, 0, sizeof(s_progress));
    s_progress.version = IMU_CMD_PROGRESS_VERSION;
    s_progress.active = s_identity;
    s_progress.tare.identity = s_identity;
    s_progress.tare.requestedAxes = s_axes;
}

static void reset_result(void)
{
    memset(&s_result, 0, sizeof(s_result));
    s_result.version = IMU_CMD_RESULT_VERSION;
    s_result.identity = s_identity;
    s_result.state = IMU_CMD_STATE_RUNNING;
    s_result.reason = IMU_CMD_REASON_NONE;
    s_result.requestedTareAxes = s_axes;
}

static void finish(ImuCmdResultState_t state, ImuCmdReason_t reason,
                   bool warning, bool restored)
{
    s_result.state = state;
    s_result.reason = reason;
    s_result.warningRequired = warning;
    s_result.endedNs = s_nowNs;
    s_result.verified = s_verified;
    s_result.restoredProduction = restored;
    s_result.terminalProgress = s_progress;
    s_result.terminalProgress.version = IMU_CMD_PROGRESS_VERSION;
    s_progress.active = IMU_CMD_ID_NONE;
    s_progress.requiredAction = IMU_CMD_ACTION_NONE;
    s_progress.tare.phase = IMU_CMD_TARE_PHASE_NONE;
    s_progress.tare.confirmationPending = false;
    clear_pending();
    s_state = IMU_TARE_ST_DONE;
}

static void begin_restore(ImuCmdResultState_t state, ImuCmdReason_t reason,
                          bool warning)
{
    s_termState = state;
    s_termReason = reason;
    s_termWarning = warning;
    enter_state(IMU_TARE_ST_WAIT_RESTORE, IMU_CMD_TARE_PHASE_RESTORE, 0ull,
                IMU_CMD_ACTION_NONE);
    set_pending(IMU_TARE_REQ_RESTORE_PRODUCTION);
}

static bool q_sets_warning(void)
{
    return s_identity == IMU_CMD_ID_TARE ||
           s_identity == IMU_CMD_ID_TARE_CLEAR;
}

static bool mutation_pending_unexecuted(void)
{
    return s_state == IMU_TARE_ST_WAIT_CONFIRM ||
           s_state == IMU_TARE_ST_WAIT_CONFIG;
}

static void note_epochs(const ImuTareSessionResult_t *sr)
{
    if (s_result.epochBefore == 0u) {
        s_result.epochBefore = sr->epochBefore;
    }
    s_result.epochAfter = sr->epochAfter;
}

static ImuCmdSubResult_t mapped_clear_sub(ImuCmdSubResult_t sub, bool success)
{
    if (sub != IMU_CMD_SUB_NOT_ATTEMPTED) {
        return sub;
    }
    return success ? IMU_CMD_SUB_SUCCEEDED : IMU_CMD_SUB_FAILED;
}

static void apply_clear_outcome(const ImuTareSessionResult_t *sr)
{
    ImuCmdSubResult_t active;
    ImuCmdSubResult_t saved;

    active = mapped_clear_sub(sr->clearActive, sr->success);
    saved = mapped_clear_sub(sr->clearSaved, sr->success);
    s_result.sub.clearActive = active;
    s_result.sub.clearSaved = saved;

    if (active == IMU_CMD_SUB_SUCCEEDED && saved == IMU_CMD_SUB_SUCCEEDED) {
        begin_restore(IMU_CMD_STATE_SUCCEEDED, IMU_CMD_REASON_OK, false);
        return;
    }
    if (active == IMU_CMD_SUB_SUCCEEDED || saved == IMU_CMD_SUB_SUCCEEDED) {
        begin_restore(IMU_CMD_STATE_FAILED, IMU_CMD_REASON_TARE_CLEAR_PARTIAL,
                      true);
        return;
    }
    begin_restore(IMU_CMD_STATE_FAILED, IMU_CMD_REASON_TARE_CLEAR_FAILED, true);
}

bool imu_tare_init(ImuCmdIdentity_t identity, ImuCmdTareAxes_t axes,
                   uint8_t flightCalMask, uint64_t nowNs)
{
    if (!init_legal(identity, axes)) {
        s_inited = false;
        return false;
    }

    s_inited = true;
    s_identity = identity;
    s_axes = axes;
    s_flightMask = flightCalMask;
    s_nowNs = nowNs;
    s_expectedEpoch = 0u;
    s_haveEvidence = false;
    s_verified = false;
    s_termReason = IMU_CMD_REASON_NONE;
    s_termWarning = false;
    memset(&s_facts, 0, sizeof(s_facts));
    reset_result();
    reset_progress();
    s_result.startedNs = nowNs;
    clear_pending();

    if (identity == IMU_CMD_ID_TARE_CHECK) {
        enter_state(IMU_TARE_ST_WAIT_CONFIG, IMU_CMD_TARE_PHASE_CONFIGURE, 0ull,
                    IMU_CMD_ACTION_PRESS_Q_TO_END);
        set_pending(IMU_TARE_REQ_CONFIGURE);
        return true;
    }

    enter_state(IMU_TARE_ST_WAIT_CONFIRM, IMU_CMD_TARE_PHASE_WAIT_CONFIRM, 0ull,
                IMU_CMD_ACTION_ALIGN_AND_CONFIRM);
    return true;
}

static bool post_tick(uint64_t ns)
{
    s_nowNs = ns;
    s_progress.tare.remainingNs = remaining_ns();
    return true;
}

static bool post_confirm(void)
{
    if (s_state == IMU_TARE_ST_WAIT_CONFIRM) {
        enter_state(IMU_TARE_ST_WAIT_CONFIG, IMU_CMD_TARE_PHASE_CONFIGURE, 0ull,
                    IMU_CMD_ACTION_NONE);
        set_pending(IMU_TARE_REQ_CONFIGURE);
        return true;
    }
    return true;
}

static bool post_q(void)
{
    if (s_state == IMU_TARE_ST_IDLE || s_state == IMU_TARE_ST_DONE ||
        s_state == IMU_TARE_ST_WAIT_RESTORE) {
        return true;
    }
    if (mutation_pending_unexecuted()) {
        finish(IMU_CMD_STATE_CANCELLED, IMU_CMD_REASON_OPERATOR_Q,
               q_sets_warning(), false);
        return true;
    }
    begin_restore(IMU_CMD_STATE_CANCELLED, IMU_CMD_REASON_OPERATOR_Q,
                  q_sets_warning());
    return true;
}

static bool post_facts(const ImuTareFacts_t *facts)
{
    if (facts == NULL) {
        return false;
    }
    s_facts = *facts;
    apply_facts_to_progress(&s_facts);
    if (s_state == IMU_TARE_ST_VERIFY && s_facts.valid &&
        s_facts.configurationEpoch == s_expectedEpoch) {
        s_haveEvidence = true;
        s_progress.tare.verificationEvidence = true;
    }
    return true;
}

static bool post_session(const ImuTareSessionResult_t *sr)
{
    ImuTareRequestType_t type;

    if (sr == NULL || s_pending.type == IMU_TARE_REQ_NONE ||
        sr->type != s_pending.type) {
        return false;
    }

    type = s_pending.type;
    clear_pending();
    note_epochs(sr);

    if (s_state == IMU_TARE_ST_WAIT_CONFIG &&
        type == IMU_TARE_REQ_CONFIGURE) {
        if (!sr->success) {
            begin_restore(IMU_CMD_STATE_FAILED, IMU_CMD_REASON_CONFIG_FAILED,
                          true);
            return true;
        }
        s_expectedEpoch = sr->epochAfter;
        if (s_identity == IMU_CMD_ID_TARE) {
            enter_state(IMU_TARE_ST_SETTLE, IMU_CMD_TARE_PHASE_SETTLE,
                        IMU_TARE_SETTLE_NS, IMU_CMD_ACTION_PRESS_Q_TO_END);
            return true;
        }
        if (s_identity == IMU_CMD_ID_TARE_CLEAR) {
            enter_state(IMU_TARE_ST_WAIT_CLEAR, IMU_CMD_TARE_PHASE_CLEAR, 0ull,
                        IMU_CMD_ACTION_NONE);
            set_pending(IMU_TARE_REQ_CLEAR);
            return true;
        }
        enter_state(IMU_TARE_ST_CHECK, IMU_CMD_TARE_PHASE_CHECK, 0ull,
                    IMU_CMD_ACTION_PRESS_Q_TO_END);
        return true;
    }

    if (s_state == IMU_TARE_ST_WAIT_TARE_NOW &&
        type == IMU_TARE_REQ_TARE_NOW) {
        if (!sr->success) {
            s_result.sub.tareNow = IMU_CMD_SUB_FAILED;
            s_result.sub.persist = IMU_CMD_SUB_NOT_ATTEMPTED;
            begin_restore(IMU_CMD_STATE_FAILED, IMU_CMD_REASON_TARE_NOW_FAILED,
                          true);
            return true;
        }
        s_result.sub.tareNow = IMU_CMD_SUB_SUCCEEDED;
        s_expectedEpoch = sr->epochAfter;
        enter_state(IMU_TARE_ST_WAIT_PERSIST, IMU_CMD_TARE_PHASE_PERSIST, 0ull,
                    IMU_CMD_ACTION_NONE);
        set_pending(IMU_TARE_REQ_PERSIST);
        return true;
    }

    if (s_state == IMU_TARE_ST_WAIT_PERSIST &&
        type == IMU_TARE_REQ_PERSIST) {
        if (!sr->success) {
            s_result.sub.persist = IMU_CMD_SUB_FAILED;
            begin_restore(IMU_CMD_STATE_FAILED,
                          IMU_CMD_REASON_TARE_PERSIST_FAILED, true);
            return true;
        }
        s_result.sub.persist = IMU_CMD_SUB_SUCCEEDED;
        s_haveEvidence = false;
        s_progress.tare.verificationEvidence = false;
        enter_state(IMU_TARE_ST_VERIFY, IMU_CMD_TARE_PHASE_VERIFY,
                    IMU_TARE_VERIFY_NS, IMU_CMD_ACTION_PRESS_Q_TO_END);
        return true;
    }

    if (s_state == IMU_TARE_ST_WAIT_CLEAR && type == IMU_TARE_REQ_CLEAR) {
        apply_clear_outcome(sr);
        return true;
    }

    if (s_state == IMU_TARE_ST_WAIT_RESTORE &&
        type == IMU_TARE_REQ_RESTORE_PRODUCTION) {
        if (!sr->success) {
            finish(IMU_CMD_STATE_RECOVERY_FAILED,
                   IMU_CMD_REASON_SESSION_UNUSABLE, true, false);
            return true;
        }
        finish(s_termState, s_termReason, s_termWarning, true);
        return true;
    }

    return false;
}

bool imu_tare_post(const ImuTareEvent_t *event)
{
    if (!s_inited || event == NULL || s_state == IMU_TARE_ST_IDLE) {
        return false;
    }
    switch (event->type) {
    case IMU_TARE_EVENT_TICK:
        return post_tick(event->monotonicNs);
    case IMU_TARE_EVENT_OPERATOR_CONFIRM:
        return post_confirm();
    case IMU_TARE_EVENT_OPERATOR_Q:
        return post_q();
    case IMU_TARE_EVENT_FACTS:
        return post_facts(&event->facts);
    case IMU_TARE_EVENT_SESSION_RESULT:
        return post_session(&event->session);
    default:
        return false;
    }
}

void imu_tare_service(void)
{
    if (!s_inited || s_state == IMU_TARE_ST_DONE ||
        s_state == IMU_TARE_ST_IDLE) {
        return;
    }
    s_progress.tare.remainingNs = remaining_ns();
    s_progress.tare.stateEntryNs = s_stateEntryNs;
    s_progress.tare.deadlineNs = s_deadlineNs;

    if (s_state == IMU_TARE_ST_SETTLE) {
        if (s_deadlineNs != 0ull && s_nowNs >= s_deadlineNs) {
            enter_state(IMU_TARE_ST_WAIT_TARE_NOW, IMU_CMD_TARE_PHASE_TARE_NOW,
                        0ull, IMU_CMD_ACTION_NONE);
            set_pending(IMU_TARE_REQ_TARE_NOW);
        }
        return;
    }

    if (s_state == IMU_TARE_ST_VERIFY) {
        if (s_deadlineNs != 0ull && s_nowNs >= s_deadlineNs) {
            if (s_haveEvidence) {
                s_verified = true;
                s_progress.tare.verificationEvidence = true;
                begin_restore(IMU_CMD_STATE_SUCCEEDED, IMU_CMD_REASON_OK,
                              false);
            } else {
                begin_restore(IMU_CMD_STATE_FAILED,
                              IMU_CMD_REASON_TARE_VERIFY_FAILED, true);
            }
        }
    }
}

bool imu_tare_feed_facts(const ImuTareFacts_t *facts)
{
    if (!s_inited || facts == NULL || s_state == IMU_TARE_ST_DONE) {
        return false;
    }
    return post_facts(facts);
}

bool imu_tare_get_progress(ImuCmdProgress_t *out)
{
    if (!s_inited || out == NULL) {
        return false;
    }
    *out = s_progress;
    out->version = IMU_CMD_PROGRESS_VERSION;
    return true;
}

bool imu_tare_get_result(ImuCmdResult_t *out)
{
    if (!s_inited || out == NULL) {
        return false;
    }
    *out = s_result;
    out->version = IMU_CMD_RESULT_VERSION;
    return true;
}

ImuTarePendingRequest_t imu_tare_pending_request(void)
{
    ImuTarePendingRequest_t none;

    memset(&none, 0, sizeof(none));
    none.type = IMU_TARE_REQ_NONE;
    if (!s_inited) {
        return none;
    }
    return s_pending;
}

bool imu_tare_complete(void)
{
    return s_inited && s_state == IMU_TARE_ST_DONE;
}
