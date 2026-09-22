#include "imu_cmd.h"
#include "app/imu_cal.h"

#include <string.h>

static bool s_inited;
static ImuCmdPlan_t s_plan;
static ImuCmdResult_t s_results[IMU_CMD_ID_COUNT];
static ImuCmdIdentity_t s_active;
static ImuCmdOperatorAction_t s_action;
static ImuCmdRequest_t s_request;
static ImuCmdIdentity_t s_request_id;
static bool s_do_not_acquire;
static bool s_process_stop;
static bool s_warning;
static bool s_complete;
static bool s_unrestorable;
static bool s_confirmed;
static uint64_t s_last_ns;
static uint64_t s_stage_start_ns;
static bool s_cal_initialized;
static ImuCmdReason_t s_init_reason;
static unsigned s_slot_done;

static void zero_sub(ImuCmdSubResults_t *sub)
{
    memset(sub, 0, sizeof(*sub));
}

static void reset_result(ImuCmdIdentity_t id)
{
    ImuCmdResult_t *r = &s_results[id];

    memset(r, 0, sizeof(*r));
    r->version = IMU_CMD_RESULT_VERSION;
    r->identity = id;
    r->state = IMU_CMD_STATE_NOT_REQUESTED;
    r->reason = IMU_CMD_REASON_NONE;
}

static bool is_slot1(ImuCmdIdentity_t id)
{
    return id == IMU_CMD_ID_NONE ||
           id == IMU_CMD_ID_CALIBRATION ||
           id == IMU_CMD_ID_DCD_CLEAR;
}

static bool is_slot2(ImuCmdIdentity_t id)
{
    return id == IMU_CMD_ID_NONE ||
           id == IMU_CMD_ID_TARE ||
           id == IMU_CMD_ID_TARE_CLEAR ||
           id == IMU_CMD_ID_TARE_CHECK;
}

static bool is_slot3(ImuCmdIdentity_t id)
{
    return id == IMU_CMD_ID_NONE ||
           id == IMU_CMD_ID_CHECK ||
           id == IMU_CMD_ID_PROBE;
}

static bool needs_confirm(ImuCmdIdentity_t id)
{
    return (id == IMU_CMD_ID_DCD_CLEAR && s_plan.confirmDcdClear) ||
           (id == IMU_CMD_ID_TARE && s_plan.confirmTare) ||
           (id == IMU_CMD_ID_TARE_CLEAR && s_plan.confirmTareClear);
}

static ImuCmdOperatorAction_t action_for(ImuCmdIdentity_t id)
{
    if (id == IMU_CMD_ID_TARE && needs_confirm(id)) {
        return IMU_CMD_ACTION_ALIGN_AND_CONFIRM;
    }
    if (needs_confirm(id)) {
        return IMU_CMD_ACTION_CONFIRM;
    }
    if (id == IMU_CMD_ID_CALIBRATION ||
        id == IMU_CMD_ID_TARE_CHECK ||
        id == IMU_CMD_ID_CHECK ||
        id == IMU_CMD_ID_PROBE) {
        return IMU_CMD_ACTION_PRESS_Q_TO_END;
    }
    return IMU_CMD_ACTION_NONE;
}

static bool q_sets_warning(ImuCmdIdentity_t id)
{
    return id == IMU_CMD_ID_CALIBRATION ||
           id == IMU_CMD_ID_DCD_CLEAR ||
           id == IMU_CMD_ID_TARE ||
           id == IMU_CMD_ID_TARE_CLEAR;
}

static ImuCmdIdentity_t slot_id(unsigned slot)
{
    if (slot == 1u) {
        return s_plan.slot1;
    }
    if (slot == 2u) {
        return s_plan.slot2;
    }
    if (slot == 3u) {
        return s_plan.slot3;
    }
    return IMU_CMD_ID_NONE;
}

static unsigned slot_of(ImuCmdIdentity_t id)
{
    if (id == IMU_CMD_ID_NONE) {
        return 0u;
    }
    if (id == s_plan.slot1) {
        return 1u;
    }
    if (id == s_plan.slot2) {
        return 2u;
    }
    if (id == s_plan.slot3) {
        return 3u;
    }
    if (id == IMU_CMD_ID_SETTLE) {
        return 4u;
    }
    if (id == IMU_CMD_ID_ACQUISITION) {
        return 5u;
    }
    return 0u;
}

static bool plan_legal(const ImuCmdPlan_t *plan, ImuCmdReason_t *reason)
{
    if (plan->version != IMU_CMD_PLAN_VERSION) {
        *reason = IMU_CMD_REASON_ILLEGAL_PLAN;
        return false;
    }
    if (!is_slot1(plan->slot1) || !is_slot2(plan->slot2) || !is_slot3(plan->slot3)) {
        *reason = IMU_CMD_REASON_ILLEGAL_PLAN;
        return false;
    }
    if (plan->slot2 == IMU_CMD_ID_TARE &&
        plan->tareAxes != IMU_CMD_TARE_AXES_Z &&
        plan->tareAxes != IMU_CMD_TARE_AXES_FULL) {
        *reason = IMU_CMD_REASON_ILLEGAL_PLAN;
        return false;
    }
    if (plan->slot2 == IMU_CMD_ID_TARE && !plan->persistTare) {
        *reason = IMU_CMD_REASON_ILLEGAL_PLAN;
        return false;
    }
    if (plan->slot3 == IMU_CMD_ID_PROBE && !plan->probeMaskPresent) {
        *reason = IMU_CMD_REASON_ILLEGAL_PLAN;
        return false;
    }
    if (plan->slot3 == IMU_CMD_ID_CHECK && plan->probeMaskPresent) {
        *reason = IMU_CMD_REASON_ILLEGAL_PLAN;
        return false;
    }
    if (plan->slot3 == IMU_CMD_ID_PROBE &&
        plan->probeDeadlineS != 0u &&
        plan->probeDeadlineS != IMU_CMD_PROBE_DEADLINE_S) {
        *reason = IMU_CMD_REASON_ILLEGAL_PLAN;
        return false;
    }
    if (plan->acquisitionDurationS < IMU_CMD_ACQUIRE_MIN_S ||
        plan->acquisitionDurationS > IMU_CMD_ACQUIRE_MAX_S) {
        *reason = IMU_CMD_REASON_ILLEGAL_PLAN;
        return false;
    }
    *reason = IMU_CMD_REASON_NONE;
    return true;
}

static void idle_request(void)
{
    s_active = IMU_CMD_ID_NONE;
    s_action = IMU_CMD_ACTION_NONE;
    s_request = IMU_CMD_REQ_NONE;
    s_request_id = IMU_CMD_ID_NONE;
    s_confirmed = false;
    s_stage_start_ns = 0ull;
    s_cal_initialized = false;
}

static void complete_active_bookkeeping(void)
{
    ImuCmdResult_t *r;
    unsigned slot;
    ImuCmdIdentity_t id = s_active;

    if (id == IMU_CMD_ID_NONE) {
        return;
    }

    r = &s_results[id];
    if (r->warningRequired) {
        s_warning = true;
    }

    slot = slot_of(id);
    if (slot > s_slot_done) {
        s_slot_done = slot;
    }

    if (r->state == IMU_CMD_STATE_RECOVERY_FAILED || s_unrestorable) {
        s_do_not_acquire = true;
        s_complete = true;
        idle_request();
        s_request = IMU_CMD_REQ_STOP_PLAN;
        s_request_id = IMU_CMD_ID_NONE;
        return;
    }

    if (r->state == IMU_CMD_STATE_ABANDONED &&
        r->reason == IMU_CMD_REASON_PROCESS_STOP) {
        if (id != IMU_CMD_ID_ACQUISITION) {
            s_do_not_acquire = true;
        }
        s_complete = true;
        idle_request();
        s_request = IMU_CMD_REQ_STOP_PLAN;
        s_request_id = IMU_CMD_ID_NONE;
        return;
    }

    if (id == IMU_CMD_ID_ACQUISITION) {
        s_complete = true;
        idle_request();
        return;
    }

    idle_request();
}

static void start_stage(ImuCmdIdentity_t id)
{
    ImuCmdResult_t *r = &s_results[id];

    s_active = id;
    s_confirmed = !needs_confirm(id);
    s_cal_initialized = false;
    s_action = action_for(id);
    s_stage_start_ns = 0ull;    // Duration windows arm on the first TICK of this stage.
    s_request_id = id;
    if (id == IMU_CMD_ID_SETTLE) {
        s_request = IMU_CMD_REQ_SETTLE;
    } else if (id == IMU_CMD_ID_ACQUISITION) {
        s_request = IMU_CMD_REQ_ACQUIRE;
    } else {
        s_request = IMU_CMD_REQ_RUN_COMMAND;
    }

    r->version = IMU_CMD_RESULT_VERSION;
    r->identity = id;
    r->state = IMU_CMD_STATE_RUNNING;
    r->reason = IMU_CMD_REASON_NONE;
    r->warningRequired = false;
    r->startedNs = 0ull;
    r->endedNs = 0ull;
    r->epochBefore = 0u;
    r->epochAfter = 0u;
    r->requestedTareAxes = (id == IMU_CMD_ID_TARE) ? s_plan.tareAxes
                                                   : IMU_CMD_TARE_AXES_NONE;
    r->probeMaskRequestedValid = (id == IMU_CMD_ID_PROBE &&
                                  s_plan.probeMaskPresent);
    r->probeMaskRequested = r->probeMaskRequestedValid ? s_plan.probeMask : 0u;
    r->dcdSaved                = false;
    r->verified                = false;
    r->restoredProduction      = false;
    memset(&r->terminalProgress, 0, sizeof(r->terminalProgress));
    zero_sub(&r->sub);
}

static void finish_active(ImuCmdResultState_t state, ImuCmdReason_t reason,
                          bool warning, const ImuCmdSubResults_t *sub)
{
    ImuCmdResult_t *r;
    ImuCmdIdentity_t id = s_active;

    if (id == IMU_CMD_ID_NONE) {
        return;
    }

    r = &s_results[id];
    r->state = state;
    r->reason = reason;
    r->warningRequired = warning;
    r->endedNs = s_last_ns;
    if (sub != NULL) {
        r->sub = *sub;
    }
    complete_active_bookkeeping();
}

static bool is_terminal_state(ImuCmdResultState_t state)
{
    return state != IMU_CMD_STATE_NOT_REQUESTED &&
           state != IMU_CMD_STATE_RUNNING;
}

static void adopt_calibration_result(void)
{
    ImuCmdResult_t result;

    if (s_active != IMU_CMD_ID_CALIBRATION || !s_cal_initialized) {
        return;
    }

    if (!imu_cal_get_result(&result) ||
        result.version != IMU_CMD_RESULT_VERSION ||
        result.identity != IMU_CMD_ID_CALIBRATION ||
        !is_terminal_state(result.state)) {
        finish_active(IMU_CMD_STATE_RECOVERY_FAILED,
                      IMU_CMD_REASON_CAL_SESSION_UNUSABLE,
                      true,
                      &s_results[IMU_CMD_ID_CALIBRATION].sub);
        return;
    }

    s_results[IMU_CMD_ID_CALIBRATION] = result;
    complete_active_bookkeeping();
}

static void apply_tare_rules(ImuCmdStageTerminal_t *term)
{
    if (term->identity == IMU_CMD_ID_TARE) {
        if (term->sub.tareNow != IMU_CMD_SUB_SUCCEEDED) {
            term->state = IMU_CMD_STATE_FAILED;
            if (term->reason == IMU_CMD_REASON_NONE ||
                term->reason == IMU_CMD_REASON_OK) {
                term->reason = IMU_CMD_REASON_TARE_NOW_FAILED;
            }
            return;
        }
        if (term->sub.persist != IMU_CMD_SUB_SUCCEEDED) {
            term->state = IMU_CMD_STATE_FAILED;
            term->reason = IMU_CMD_REASON_TARE_PERSIST_FAILED;
            term->warningRequired = true;
            return;
        }
        if (term->state == IMU_CMD_STATE_SUCCEEDED) {
            term->reason = IMU_CMD_REASON_OK;
        }
        return;
    }

    if (term->identity == IMU_CMD_ID_TARE_CLEAR) {
        if (term->sub.clearActive != IMU_CMD_SUB_SUCCEEDED ||
            term->sub.clearSaved != IMU_CMD_SUB_SUCCEEDED) {
            term->state = IMU_CMD_STATE_FAILED;
            if (term->sub.clearActive == IMU_CMD_SUB_SUCCEEDED ||
                term->sub.clearSaved == IMU_CMD_SUB_SUCCEEDED) {
                term->reason = IMU_CMD_REASON_TARE_CLEAR_PARTIAL;
            } else if (term->reason == IMU_CMD_REASON_NONE ||
                       term->reason == IMU_CMD_REASON_OK) {
                term->reason = IMU_CMD_REASON_TARE_CLEAR_FAILED;
            }
            term->warningRequired = true;
        }
    }
}

static uint64_t duration_ns(uint32_t seconds)
{
    return (uint64_t)seconds * 1000000000ull;
}

void imu_cmd_plan_clear(ImuCmdPlan_t *plan)
{
    if (plan == NULL) {
        return;
    }
    memset(plan, 0, sizeof(*plan));
    plan->version = IMU_CMD_PLAN_VERSION;
    plan->acquisitionDurationS = IMU_CMD_ACQUIRE_DEFAULT_S;
    plan->settleDurationMs = IMU_CMD_SETTLE_DEFAULT_MS;
    plan->probeDeadlineS = IMU_CMD_PROBE_DEADLINE_S;
    plan->servicePeriodNs = IMU_CMD_SERVICE_PERIOD_NS;
    plan->publicationPeriodNs = IMU_CMD_PUB_PERIOD_NS;
}

bool imu_cmd_plan_acquire_default(ImuCmdPlan_t *plan)
{
    if (plan == NULL) {
        return false;
    }
    imu_cmd_plan_clear(plan);
    return true;
}

bool imu_cmd_init(const ImuCmdPlan_t *plan)
{
    unsigned i;

    s_inited = false;
    s_init_reason = IMU_CMD_REASON_NONE;
    memset(&s_plan, 0, sizeof(s_plan));
    s_active = IMU_CMD_ID_NONE;
    s_action = IMU_CMD_ACTION_NONE;
    s_request = IMU_CMD_REQ_NONE;
    s_request_id = IMU_CMD_ID_NONE;
    s_do_not_acquire = false;
    s_process_stop = false;
    s_warning = false;
    s_complete = false;
    s_unrestorable = false;
    s_confirmed = false;
    s_last_ns = 0ull;
    s_stage_start_ns = 0ull;
    s_cal_initialized = false;
    s_slot_done = 0u;

    for (i = 0u; i < (unsigned)IMU_CMD_ID_COUNT; i++) {
        reset_result((ImuCmdIdentity_t)i);
    }

    if (plan == NULL || !plan_legal(plan, &s_init_reason)) {
        if (s_init_reason == IMU_CMD_REASON_NONE) {
            s_init_reason = IMU_CMD_REASON_ILLEGAL_PLAN;
        }
        return false;
    }

    s_plan = *plan;
    if (s_plan.slot1 == IMU_CMD_ID_DCD_CLEAR) {
        s_plan.confirmDcdClear = true;
    }
    if (s_plan.slot2 == IMU_CMD_ID_TARE) {
        s_plan.persistTare = true;
        s_plan.confirmTare = true;
    }
    if (s_plan.slot2 == IMU_CMD_ID_TARE_CLEAR) {
        s_plan.confirmTareClear = true;
    }
    if (s_plan.slot3 == IMU_CMD_ID_PROBE && s_plan.probeDeadlineS == 0u) {
        s_plan.probeDeadlineS = IMU_CMD_PROBE_DEADLINE_S;
    }

    s_inited = true;
    return true;
}

static bool post_q(void)
{
    ImuCmdSubResults_t sub;

    if (s_active == IMU_CMD_ID_CALIBRATION) {
        ImuCalEvent_t event;

        if (!s_cal_initialized) {
            return false;
        }
        memset(&event, 0, sizeof(event));
        event.type = IMU_CAL_EVENT_OPERATOR_Q;
        return imu_cal_post(&event);
    }

    if (s_active == IMU_CMD_ID_NONE ||
        s_active == IMU_CMD_ID_SETTLE ||
        s_active == IMU_CMD_ID_ACQUISITION) {
        return true;
    }

    sub = s_results[s_active].sub;
    if (s_active == IMU_CMD_ID_PROBE) {
        sub.probeOperatorEndedEarly = true;
    }
    finish_active(IMU_CMD_STATE_CANCELLED, IMU_CMD_REASON_OPERATOR_Q,
                  q_sets_warning(s_active), &sub);
    return true;
}

static bool post_confirm(void)
{
    if (s_active == IMU_CMD_ID_CALIBRATION) {
        ImuCalEvent_t event;

        if (!s_cal_initialized) {
            return false;
        }
        memset(&event, 0, sizeof(event));
        event.type = IMU_CAL_EVENT_OPERATOR_CONFIRM;
        return imu_cal_post(&event);
    }

    if (s_active == IMU_CMD_ID_NONE || s_confirmed ||
        (s_action != IMU_CMD_ACTION_CONFIRM &&
         s_action != IMU_CMD_ACTION_ALIGN_AND_CONFIRM)) {
        return true;
    }
    s_confirmed = true;
    s_action = IMU_CMD_ACTION_NONE;
    return true;
}

static bool post_process_stop(void)
{
    s_process_stop = true;
    if (s_active != IMU_CMD_ID_NONE) {
        finish_active(IMU_CMD_STATE_ABANDONED, IMU_CMD_REASON_PROCESS_STOP,
                      false, &s_results[s_active].sub);
    } else {
        if (s_slot_done < 5u) {
            s_do_not_acquire = true;
        }
        s_complete = true;
        s_request = IMU_CMD_REQ_STOP_PLAN;
        s_request_id = IMU_CMD_ID_NONE;
    }
    return true;
}

static bool post_tick(uint64_t ns)
{
    uint64_t elapsed;
    uint64_t limit;

    s_last_ns = ns;

    if (s_active == IMU_CMD_ID_CALIBRATION) {
        ImuCalEvent_t event;

        if (!s_cal_initialized) {
            s_stage_start_ns = ns;
            s_results[s_active].startedNs = ns;
            s_cal_initialized = imu_cal_init(s_plan.flightCalMask, ns);
            if (!s_cal_initialized) {
                finish_active(IMU_CMD_STATE_RECOVERY_FAILED,
                              IMU_CMD_REASON_CAL_SESSION_UNUSABLE,
                              true,
                              &s_results[IMU_CMD_ID_CALIBRATION].sub);
            }
            return s_cal_initialized;
        }

        memset(&event, 0, sizeof(event));
        event.type = IMU_CAL_EVENT_TICK;
        event.monotonicNs = ns;
        return imu_cal_post(&event);
    }

    if (s_active == IMU_CMD_ID_NONE) {
        return true;
    }

    if (s_stage_start_ns == 0ull) {
        s_stage_start_ns = ns;
        s_results[s_active].startedNs = ns;
        return true;
    }
    if (ns < s_stage_start_ns) {
        return true;
    }
    elapsed = ns - s_stage_start_ns;

    if (s_active == IMU_CMD_ID_PROBE) {
        limit = duration_ns(s_plan.probeDeadlineS);
        if (elapsed >= limit) {
            ImuCmdSubResults_t sub = s_results[s_active].sub;

            sub.probeTimedOut = true;
            finish_active(IMU_CMD_STATE_TIMED_OUT, IMU_CMD_REASON_PROBE_DEADLINE,
                          false, &sub);
        }
        return true;
    }

    if (s_active == IMU_CMD_ID_ACQUISITION) {
        limit = duration_ns(s_plan.acquisitionDurationS);
        if (elapsed >= limit) {
            finish_active(IMU_CMD_STATE_SUCCEEDED, IMU_CMD_REASON_OK,
                          false, NULL);
        }
    }
    return true;
}

static bool post_terminal(const ImuCmdStageTerminal_t *term_in)
{
    ImuCmdStageTerminal_t term;

    if (s_active == IMU_CMD_ID_NONE || term_in->identity != s_active) {
        return false;
    }
    if (s_active == IMU_CMD_ID_CALIBRATION) {
        return false;
    }
    if (term_in->state == IMU_CMD_STATE_NOT_REQUESTED ||
        term_in->state == IMU_CMD_STATE_RUNNING) {
        return false;
    }

    term = *term_in;
    apply_tare_rules(&term);
    s_results[s_active].sub = term.sub;
    if (term.identity == IMU_CMD_ID_PROBE && term.sub.probeMaskActualValid) {
        /* actual mask is stored in sub; requested already on the result */
    }
    finish_active(term.state, term.reason, term.warningRequired, &term.sub);
    return true;
}

static bool post_settle_done(void)
{
    if (s_active != IMU_CMD_ID_SETTLE) {
        return false;
    }
    finish_active(IMU_CMD_STATE_SUCCEEDED, IMU_CMD_REASON_OK, false, NULL);
    return true;
}

bool imu_cmd_post(const ImuCmdEvent_t *event)
{
    if (!s_inited || event == NULL) {
        return false;
    }

    switch (event->type) {
    case IMU_CMD_EVENT_OPERATOR_Q:
        return post_q();
    case IMU_CMD_EVENT_OPERATOR_CONFIRM:
        return post_confirm();
    case IMU_CMD_EVENT_PROCESS_STOP:
        return post_process_stop();
    case IMU_CMD_EVENT_TICK:
        return post_tick(event->monotonicNs);
    case IMU_CMD_EVENT_STAGE_TERMINAL:
        return post_terminal(&event->terminal);
    case IMU_CMD_EVENT_SETTLE_DONE:
        return post_settle_done();
    case IMU_CMD_EVENT_SESSION_USABLE:
        return true;
    case IMU_CMD_EVENT_SESSION_UNRESTORABLE:
        s_unrestorable = true;
        s_do_not_acquire = true;
        if (s_active != IMU_CMD_ID_NONE) {
            finish_active(IMU_CMD_STATE_RECOVERY_FAILED,
                          IMU_CMD_REASON_SESSION_UNUSABLE, true,
                          &s_results[s_active].sub);
        } else {
            s_complete = true;
            s_request = IMU_CMD_REQ_STOP_PLAN;
            s_request_id = IMU_CMD_ID_NONE;
        }
        return true;
    default:
        return false;
    }
}

void imu_cmd_service(void)
{
    ImuCmdIdentity_t id;

    if (!s_inited || s_complete || s_process_stop) {
        return;
    }

    if (s_active == IMU_CMD_ID_CALIBRATION) {
        if (!s_cal_initialized) {
            return;
        }

        imu_cal_service();
        if (!imu_cal_complete()) {
            return;
        }

        adopt_calibration_result();
        return;
    }

    if (s_active != IMU_CMD_ID_NONE) {
        return;
    }

    while (s_slot_done < 3u) {
        id = slot_id(s_slot_done + 1u);
        if (id != IMU_CMD_ID_NONE) {
            start_stage(id);
            return;
        }
        s_slot_done++;
    }

    if (s_do_not_acquire) {
        s_complete = true;
        s_request = IMU_CMD_REQ_STOP_PLAN;
        s_request_id = IMU_CMD_ID_NONE;
        return;
    }

    if (s_slot_done == 3u) {
        start_stage(IMU_CMD_ID_SETTLE);
        return;
    }
    if (s_slot_done == 4u) {
        start_stage(IMU_CMD_ID_ACQUISITION);
        return;
    }

    s_complete = true;
    s_request = IMU_CMD_REQ_NONE;
    s_request_id = IMU_CMD_ID_NONE;
}

bool imu_cmd_get_result(ImuCmdIdentity_t id, ImuCmdResult_t *out)
{
    if (!s_inited || out == NULL || id <= IMU_CMD_ID_NONE ||
        id >= IMU_CMD_ID_COUNT) {
        return false;
    }
    *out = s_results[id];
    return true;
}

bool imu_cmd_get_progress(ImuCmdProgress_t *out)
{
    uint64_t limit;
    uint64_t elapsed;

    if (!s_inited || out == NULL) {
        return false;
    }

    if (s_active == IMU_CMD_ID_CALIBRATION && s_cal_initialized) {
        if (!imu_cal_get_progress(out)) {
            return false;
        }
        out->version = IMU_CMD_PROGRESS_VERSION;
        out->active = IMU_CMD_ID_CALIBRATION;
        return true;
    }

    memset(out, 0, sizeof(*out));
    out->version = IMU_CMD_PROGRESS_VERSION;
    out->active = s_active;
    out->requiredAction = s_action;
    if (s_active == IMU_CMD_ID_PROBE && s_stage_start_ns != 0ull) {
        limit = duration_ns(s_plan.probeDeadlineS);
        elapsed = (s_last_ns >= s_stage_start_ns) ? (s_last_ns - s_stage_start_ns)
                                                  : 0ull;
        out->probeTimeValid = true;
        out->probeRemainingNs = (elapsed >= limit) ? 0ull : (limit - elapsed);
    }
    return true;
}

ImuCmdRequest_t imu_cmd_request(void)
{
    return s_inited ? s_request : IMU_CMD_REQ_NONE;
}

ImuCmdIdentity_t imu_cmd_request_identity(void)
{
    return s_inited ? s_request_id : IMU_CMD_ID_NONE;
}

bool imu_cmd_do_not_acquire(void)
{
    return s_inited && s_do_not_acquire;
}

bool imu_cmd_process_stop_seen(void)
{
    return s_inited && s_process_stop;
}

bool imu_cmd_warning_required(void)
{
    return s_inited && s_warning;
}

bool imu_cmd_plan_complete(void)
{
    return s_inited && s_complete;
}

ImuCmdReason_t imu_cmd_init_reason(void)
{
    return s_init_reason;
}