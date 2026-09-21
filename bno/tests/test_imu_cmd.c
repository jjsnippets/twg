#define _POSIX_C_SOURCE 200809L

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_cmd.h"

/*
 * Phase 4 host-only oracles: 1I family K plus T sub-results.
 * No SPI, no session, no sudo.
 */

static int g_fail;

static const uint64_t T0 = 1000000000ull;
static const uint64_t NS_S = 1000000000ull;

static void check(bool ok, const char *id, const char *what)
{
    if (!ok) {
        fprintf(stderr, "FAIL %s: %s\n", id, what);
        g_fail++;
    }
}

static void check_state(ImuCmdIdentity_t id, ImuCmdResultState_t want,
                        const char *tid, const char *what)
{
    ImuCmdResult_t r;

    if (!imu_cmd_get_result(id, &r)) {
        fprintf(stderr, "FAIL %s: %s (get_result)\n", tid, what);
        g_fail++;
        return;
    }
    if (r.state != want) {
        fprintf(stderr, "FAIL %s: %s (state %d want %d)\n",
                tid, what, (int)r.state, (int)want);
        g_fail++;
    }
}

static void check_reason(ImuCmdIdentity_t id, ImuCmdReason_t want,
                         const char *tid, const char *what)
{
    ImuCmdResult_t r;

    if (!imu_cmd_get_result(id, &r)) {
        fprintf(stderr, "FAIL %s: %s (get_result)\n", tid, what);
        g_fail++;
        return;
    }
    if (r.reason != want) {
        fprintf(stderr, "FAIL %s: %s (reason %d want %d)\n",
                tid, what, (int)r.reason, (int)want);
        g_fail++;
    }
}

static ImuCmdEvent_t make_event(ImuCmdEventType_t type)
{
    ImuCmdEvent_t e;

    memset(&e, 0, sizeof(e));
    e.type = type;
    return e;
}

static ImuCmdEvent_t make_tick(uint64_t ns)
{
    ImuCmdEvent_t e = make_event(IMU_CMD_EVENT_TICK);

    e.monotonicNs = ns;
    return e;
}

static ImuCmdEvent_t make_terminal(ImuCmdIdentity_t id,
                                   ImuCmdResultState_t state,
                                   ImuCmdReason_t reason,
                                   bool warning)
{
    ImuCmdEvent_t e = make_event(IMU_CMD_EVENT_STAGE_TERMINAL);

    e.terminal.identity = id;
    e.terminal.state = state;
    e.terminal.reason = reason;
    e.terminal.warningRequired = warning;
    return e;
}

static ImuCmdIdentity_t active_id(void)
{
    ImuCmdProgress_t p;

    if (!imu_cmd_get_progress(&p)) {
        return IMU_CMD_ID_NONE;
    }
    return p.active;
}

static ImuCmdOperatorAction_t active_action(void)
{
    ImuCmdProgress_t p;

    if (!imu_cmd_get_progress(&p)) {
        return IMU_CMD_ACTION_NONE;
    }
    return p.requiredAction;
}

static ImuCmdResult_t result_of(ImuCmdIdentity_t id)
{
    ImuCmdResult_t r;

    memset(&r, 0, sizeof(r));
    (void)imu_cmd_get_result(id, &r);
    return r;
}

static bool finish_settle_acquire(const char *tid)
{
    ImuCmdEvent_t e;

    imu_cmd_service();
    if (active_id() != IMU_CMD_ID_SETTLE) {
        fprintf(stderr, "FAIL %s: expected settle\n", tid);
        g_fail++;
        return false;
    }
    e = make_event(IMU_CMD_EVENT_SETTLE_DONE);
    if (!imu_cmd_post(&e)) {
        fprintf(stderr, "FAIL %s: settle done\n", tid);
        g_fail++;
        return false;
    }
    imu_cmd_service();
    if (active_id() != IMU_CMD_ID_ACQUISITION) {
        fprintf(stderr, "FAIL %s: expected acquire\n", tid);
        g_fail++;
        return false;
    }
    e = make_tick(T0);
    (void)imu_cmd_post(&e);
    e = make_tick(T0 + 10ull * NS_S);
    (void)imu_cmd_post(&e);
    check_state(IMU_CMD_ID_ACQUISITION, IMU_CMD_STATE_SUCCEEDED, tid,
                "acquire succeeded");
    check(imu_cmd_plan_complete(), tid, "plan complete");
    return true;
}

static void test_k01(void)
{
    ImuCmdPlan_t plan;
    ImuCmdIdentity_t id;
    ImuCmdEvent_t e;

    check(imu_cmd_plan_acquire_default(&plan), "K01", "default plan");
    check(imu_cmd_init(&plan), "K01", "init");

    for (id = IMU_CMD_ID_CALIBRATION; id <= IMU_CMD_ID_PROBE; id++) {
        check_state(id, IMU_CMD_STATE_NOT_REQUESTED, "K01",
                    "command not requested");
    }

    imu_cmd_service();
    check(active_id() == IMU_CMD_ID_SETTLE, "K01", "settle active");
    check(imu_cmd_request() == IMU_CMD_REQ_SETTLE, "K01", "settle request");
    check(!imu_cmd_do_not_acquire(), "K01", "may acquire");

    e = make_event(IMU_CMD_EVENT_SETTLE_DONE);
    check(imu_cmd_post(&e), "K01", "settle done");
    imu_cmd_service();
    check(active_id() == IMU_CMD_ID_ACQUISITION, "K01", "acquire active");
    check(imu_cmd_request() == IMU_CMD_REQ_ACQUIRE, "K01", "acquire request");

    e = make_tick(T0);
    check(imu_cmd_post(&e), "K01", "arm acquire");
    e = make_tick(T0 + 10ull * NS_S);
    check(imu_cmd_post(&e), "K01", "acquire duration");
    check_state(IMU_CMD_ID_SETTLE, IMU_CMD_STATE_SUCCEEDED, "K01", "settle");
    check_state(IMU_CMD_ID_ACQUISITION, IMU_CMD_STATE_SUCCEEDED, "K01",
                "acquire");
    check(imu_cmd_plan_complete(), "K01", "complete");
    check(!imu_cmd_warning_required(), "K01", "no warning");
}

static void test_k02(void)
{
    ImuCmdPlan_t plan;
    ImuCmdEvent_t e;
    ImuCmdResult_t r;

    imu_cmd_plan_clear(&plan);
    plan.slot2 = IMU_CMD_ID_TARE;
    plan.tareAxes = IMU_CMD_TARE_AXES_Z;
    plan.persistTare = true;
    plan.confirmTare = true;
    plan.slot3 = IMU_CMD_ID_CHECK;
    check(imu_cmd_init(&plan), "K02", "init");

    imu_cmd_service();
    check(active_id() == IMU_CMD_ID_TARE, "K02", "tare running");
    check(active_action() == IMU_CMD_ACTION_ALIGN_AND_CONFIRM, "K02",
          "align confirm");

    e = make_event(IMU_CMD_EVENT_OPERATOR_Q);
    check(imu_cmd_post(&e), "K02", "q before tare-now");
    check_state(IMU_CMD_ID_TARE, IMU_CMD_STATE_CANCELLED, "K02", "tare cancelled");
    check_reason(IMU_CMD_ID_TARE, IMU_CMD_REASON_OPERATOR_Q, "K02", "operator_q");
    r = result_of(IMU_CMD_ID_TARE);
    check(r.sub.tareNow == IMU_CMD_SUB_NOT_ATTEMPTED, "K02",
          "no tare-now sub-result");
    check(r.sub.persist == IMU_CMD_SUB_NOT_ATTEMPTED, "K02",
          "no persist sub-result");

    imu_cmd_service();
    check(active_id() == IMU_CMD_ID_CHECK, "K02", "check still runs");
    check(imu_cmd_request() == IMU_CMD_REQ_RUN_COMMAND, "K02", "run check");
    check(imu_cmd_request_identity() == IMU_CMD_ID_CHECK, "K02", "check id");

    e = make_event(IMU_CMD_EVENT_OPERATOR_Q);
    check(imu_cmd_post(&e), "K02", "end check");
    check_state(IMU_CMD_ID_CHECK, IMU_CMD_STATE_CANCELLED, "K02", "check ended");
    (void)finish_settle_acquire("K02");
}

static void test_k03(void)
{
    ImuCmdPlan_t plan;
    ImuCmdEvent_t e;

    imu_cmd_plan_clear(&plan);
    plan.slot1 = IMU_CMD_ID_CALIBRATION;
    plan.slot2 = IMU_CMD_ID_TARE;
    plan.tareAxes = IMU_CMD_TARE_AXES_Z;
    plan.persistTare = true;
    check(imu_cmd_init(&plan), "K03", "init");

    imu_cmd_service();
    check(active_id() == IMU_CMD_ID_CALIBRATION, "K03", "cal running");
    e = make_terminal(IMU_CMD_ID_CALIBRATION, IMU_CMD_STATE_FAILED,
                      IMU_CMD_REASON_GATE_NOT_REACHED, true);
    check(imu_cmd_post(&e), "K03", "cal fail");
    check_state(IMU_CMD_ID_CALIBRATION, IMU_CMD_STATE_FAILED, "K03", "cal failed");
    check_reason(IMU_CMD_ID_CALIBRATION, IMU_CMD_REASON_GATE_NOT_REACHED,
                 "K03", "gate");
    check(imu_cmd_warning_required(), "K03", "warning");

    imu_cmd_service();
    check(active_id() == IMU_CMD_ID_TARE, "K03", "tare still runs");
    check(!imu_cmd_do_not_acquire(), "K03", "session usable");
}

static void test_k04(void)
{
    ImuCmdPlan_t plan;
    ImuCmdEvent_t e;

    imu_cmd_plan_clear(&plan);
    plan.slot1 = IMU_CMD_ID_CALIBRATION;
    plan.slot2 = IMU_CMD_ID_TARE;
    plan.tareAxes = IMU_CMD_TARE_AXES_Z;
    plan.persistTare = true;
    check(imu_cmd_init(&plan), "K04", "init");

    imu_cmd_service();
    e = make_terminal(IMU_CMD_ID_CALIBRATION, IMU_CMD_STATE_RECOVERY_FAILED,
                      IMU_CMD_REASON_SESSION_UNUSABLE, true);
    check(imu_cmd_post(&e), "K04", "recovery_failed");
    check_state(IMU_CMD_ID_CALIBRATION, IMU_CMD_STATE_RECOVERY_FAILED,
                "K04", "cal recovery_failed");
    check(imu_cmd_do_not_acquire(), "K04", "do not acquire");
    check(imu_cmd_plan_complete(), "K04", "stopped");

    imu_cmd_service();
    check(active_id() == IMU_CMD_ID_NONE, "K04", "no next stage");
    check_state(IMU_CMD_ID_TARE, IMU_CMD_STATE_NOT_REQUESTED, "K04",
                "tare not started");
    check_state(IMU_CMD_ID_SETTLE, IMU_CMD_STATE_NOT_REQUESTED, "K04",
                "no settle");
    check_state(IMU_CMD_ID_ACQUISITION, IMU_CMD_STATE_NOT_REQUESTED, "K04",
                "no acquire");
    check(imu_cmd_request() == IMU_CMD_REQ_STOP_PLAN, "K04", "stop plan");
}

static void test_k05a(void)
{
    ImuCmdPlan_t plan;
    ImuCmdEvent_t e;
    ImuCmdResult_t r;

    imu_cmd_plan_clear(&plan);
    plan.slot3 = IMU_CMD_ID_PROBE;
    plan.probeMaskPresent = true;
    plan.probeMask = 0x02;
    plan.probeDeadlineS = IMU_CMD_PROBE_DEADLINE_S;
    check(imu_cmd_init(&plan), "K05a", "init");

    imu_cmd_service();
    check(active_id() == IMU_CMD_ID_PROBE, "K05a", "probe running");
    e = make_tick(T0);
    check(imu_cmd_post(&e), "K05a", "arm");
    e = make_tick(T0 + 10ull * NS_S);
    check(imu_cmd_post(&e), "K05a", "10 s tick");
    check_state(IMU_CMD_ID_PROBE, IMU_CMD_STATE_TIMED_OUT, "K05a", "timed_out");
    check_reason(IMU_CMD_ID_PROBE, IMU_CMD_REASON_PROBE_DEADLINE, "K05a",
                 "probe_deadline");
    r = result_of(IMU_CMD_ID_PROBE);
    check(r.sub.probeTimedOut, "K05a", "sub timed out");
    check(!r.sub.probeOperatorEndedEarly, "K05a", "not operator q");
    check(!imu_cmd_do_not_acquire(), "K05a", "probe continues");
    (void)finish_settle_acquire("K05a");
}

static void test_k05b(void)
{
    ImuCmdPlan_t plan;
    ImuCmdEvent_t e;
    ImuCmdResult_t r;

    imu_cmd_plan_clear(&plan);
    plan.slot3 = IMU_CMD_ID_PROBE;
    plan.probeMaskPresent = true;
    plan.probeMask = 0x02;
    check(imu_cmd_init(&plan), "K05b", "init");

    imu_cmd_service();
    e = make_tick(T0);
    check(imu_cmd_post(&e), "K05b", "arm");
    e = make_tick(T0 + 3ull * NS_S);
    check(imu_cmd_post(&e), "K05b", "3 s");
    check(active_id() == IMU_CMD_ID_PROBE, "K05b", "still probe");
    e = make_event(IMU_CMD_EVENT_OPERATOR_Q);
    check(imu_cmd_post(&e), "K05b", "q");
    check_state(IMU_CMD_ID_PROBE, IMU_CMD_STATE_CANCELLED, "K05b", "cancelled");
    check_reason(IMU_CMD_ID_PROBE, IMU_CMD_REASON_OPERATOR_Q, "K05b",
                 "operator_q");
    r = result_of(IMU_CMD_ID_PROBE);
    check(r.sub.probeOperatorEndedEarly, "K05b", "ended early");
    check(!r.sub.probeTimedOut, "K05b", "not deadline");
    (void)finish_settle_acquire("K05b");
}

static void test_k06(void)
{
    ImuCmdPlan_t plan;
    ImuCmdEvent_t e;

    imu_cmd_plan_clear(&plan);
    plan.slot1 = IMU_CMD_ID_DCD_CLEAR;
    plan.confirmDcdClear = true;
    check(imu_cmd_init(&plan), "K06", "init");

    imu_cmd_service();
    check(active_id() == IMU_CMD_ID_DCD_CLEAR, "K06", "dcd clear");
    check(active_action() == IMU_CMD_ACTION_CONFIRM, "K06", "confirm");

    e = make_event(IMU_CMD_EVENT_PROCESS_STOP);
    check(imu_cmd_post(&e), "K06", "process stop");
    check_state(IMU_CMD_ID_DCD_CLEAR, IMU_CMD_STATE_ABANDONED, "K06",
                "abandoned");
    check_reason(IMU_CMD_ID_DCD_CLEAR, IMU_CMD_REASON_PROCESS_STOP, "K06",
                 "process_stop");
    check(result_of(IMU_CMD_ID_DCD_CLEAR).state != IMU_CMD_STATE_CANCELLED,
          "K06", "not operator_q cancel");
    check(imu_cmd_process_stop_seen(), "K06", "stop seen");
    check(imu_cmd_do_not_acquire(), "K06", "no window");
    imu_cmd_service();
    check(active_id() == IMU_CMD_ID_NONE, "K06", "no further stage");
}

static void test_k07(void)
{
    ImuCmdPlan_t plan;
    ImuCmdEvent_t e;

    check(imu_cmd_plan_acquire_default(&plan), "K07", "plan");
    check(imu_cmd_init(&plan), "K07", "init");
    imu_cmd_service();
    e = make_event(IMU_CMD_EVENT_SETTLE_DONE);
    check(imu_cmd_post(&e), "K07", "settle");
    imu_cmd_service();
    check(active_id() == IMU_CMD_ID_ACQUISITION, "K07", "acquire");

    e = make_event(IMU_CMD_EVENT_OPERATOR_Q);
    check(imu_cmd_post(&e), "K07", "q ignored");
    check(active_id() == IMU_CMD_ID_ACQUISITION, "K07", "still acquire");
    check_state(IMU_CMD_ID_ACQUISITION, IMU_CMD_STATE_RUNNING, "K07",
                "not cancelled");

    e = make_tick(T0);
    check(imu_cmd_post(&e), "K07", "arm");
    e = make_tick(T0 + 10ull * NS_S);
    check(imu_cmd_post(&e), "K07", "duration");
    check_state(IMU_CMD_ID_ACQUISITION, IMU_CMD_STATE_SUCCEEDED, "K07",
                "duration end");
}

static void test_t01(void)
{
    ImuCmdPlan_t plan;
    ImuCmdEvent_t e;
    ImuCmdResult_t r;

    imu_cmd_plan_clear(&plan);
    plan.slot2 = IMU_CMD_ID_TARE;
    plan.tareAxes = IMU_CMD_TARE_AXES_Z;
    plan.persistTare = true;
    check(imu_cmd_init(&plan), "T01", "init");
    imu_cmd_service();
    e = make_event(IMU_CMD_EVENT_OPERATOR_CONFIRM);
    check(imu_cmd_post(&e), "T01", "confirm");
    e = make_terminal(IMU_CMD_ID_TARE, IMU_CMD_STATE_SUCCEEDED,
                      IMU_CMD_REASON_OK, false);
    e.terminal.sub.tareNow = IMU_CMD_SUB_SUCCEEDED;
    e.terminal.sub.persist = IMU_CMD_SUB_SUCCEEDED;
    check(imu_cmd_post(&e), "T01", "terminal");
    check_state(IMU_CMD_ID_TARE, IMU_CMD_STATE_SUCCEEDED, "T01", "tare ok");
    r = result_of(IMU_CMD_ID_TARE);
    check(r.sub.tareNow == IMU_CMD_SUB_SUCCEEDED, "T01", "tare-now");
    check(r.sub.persist == IMU_CMD_SUB_SUCCEEDED, "T01", "persist");
    check(!imu_cmd_warning_required(), "T01", "no warning");
}

static void test_t02(void)
{
    ImuCmdPlan_t plan;
    ImuCmdEvent_t e;
    ImuCmdResult_t r;

    imu_cmd_plan_clear(&plan);
    plan.slot2 = IMU_CMD_ID_TARE;
    plan.tareAxes = IMU_CMD_TARE_AXES_FULL;
    plan.persistTare = true;
    check(imu_cmd_init(&plan), "T02", "init");
    imu_cmd_service();
    e = make_event(IMU_CMD_EVENT_OPERATOR_CONFIRM);
    check(imu_cmd_post(&e), "T02", "confirm");
    e = make_terminal(IMU_CMD_ID_TARE, IMU_CMD_STATE_FAILED,
                      IMU_CMD_REASON_TARE_NOW_FAILED, true);
    e.terminal.sub.tareNow = IMU_CMD_SUB_FAILED;
    e.terminal.sub.persist = IMU_CMD_SUB_NOT_ATTEMPTED;
    check(imu_cmd_post(&e), "T02", "tare-now fail");
    check_state(IMU_CMD_ID_TARE, IMU_CMD_STATE_FAILED, "T02", "failed");
    check_reason(IMU_CMD_ID_TARE, IMU_CMD_REASON_TARE_NOW_FAILED, "T02",
                 "tare_now_failed");
    r = result_of(IMU_CMD_ID_TARE);
    check(r.sub.persist != IMU_CMD_SUB_SUCCEEDED, "T02", "persist not ok");
}

static void test_t03(void)
{
    ImuCmdPlan_t plan;
    ImuCmdEvent_t e;
    ImuCmdResult_t r;

    imu_cmd_plan_clear(&plan);
    plan.slot2 = IMU_CMD_ID_TARE;
    plan.tareAxes = IMU_CMD_TARE_AXES_Z;
    plan.persistTare = true;
    check(imu_cmd_init(&plan), "T03", "init");
    imu_cmd_service();
    e = make_event(IMU_CMD_EVENT_OPERATOR_CONFIRM);
    check(imu_cmd_post(&e), "T03", "confirm");
    e = make_terminal(IMU_CMD_ID_TARE, IMU_CMD_STATE_SUCCEEDED,
                      IMU_CMD_REASON_OK, false);
    e.terminal.sub.tareNow = IMU_CMD_SUB_SUCCEEDED;
    e.terminal.sub.persist = IMU_CMD_SUB_FAILED;
    check(imu_cmd_post(&e), "T03", "persist fail");
    check_state(IMU_CMD_ID_TARE, IMU_CMD_STATE_FAILED, "T03", "overall fail");
    check_reason(IMU_CMD_ID_TARE, IMU_CMD_REASON_TARE_PERSIST_FAILED, "T03",
                 "tare_persist_failed");
    r = result_of(IMU_CMD_ID_TARE);
    check(r.sub.tareNow == IMU_CMD_SUB_SUCCEEDED, "T03", "tare-now ok");
    check(r.sub.persist == IMU_CMD_SUB_FAILED, "T03", "persist failed");
    check(imu_cmd_warning_required(), "T03", "warning");
    check(!imu_cmd_do_not_acquire(), "T03", "continue");
    (void)finish_settle_acquire("T03");
}

static void test_t04(void)
{
    ImuCmdPlan_t plan;
    ImuCmdEvent_t e;
    ImuCmdResult_t r;

    imu_cmd_plan_clear(&plan);
    plan.slot2 = IMU_CMD_ID_TARE_CLEAR;
    plan.confirmTareClear = true;
    check(imu_cmd_init(&plan), "T04", "init");
    imu_cmd_service();
    e = make_event(IMU_CMD_EVENT_OPERATOR_CONFIRM);
    check(imu_cmd_post(&e), "T04", "confirm");
    e = make_terminal(IMU_CMD_ID_TARE_CLEAR, IMU_CMD_STATE_SUCCEEDED,
                      IMU_CMD_REASON_OK, false);
    e.terminal.sub.clearActive = IMU_CMD_SUB_SUCCEEDED;
    e.terminal.sub.clearSaved = IMU_CMD_SUB_SUCCEEDED;
    check(imu_cmd_post(&e), "T04", "both clears");
    check_state(IMU_CMD_ID_TARE_CLEAR, IMU_CMD_STATE_SUCCEEDED, "T04", "ok");
    r = result_of(IMU_CMD_ID_TARE_CLEAR);
    check(r.sub.clearActive == IMU_CMD_SUB_SUCCEEDED, "T04", "active");
    check(r.sub.clearSaved == IMU_CMD_SUB_SUCCEEDED, "T04", "saved");
}

static void test_t05(void)
{
    ImuCmdPlan_t plan;
    ImuCmdEvent_t e;
    ImuCmdResult_t r;

    imu_cmd_plan_clear(&plan);
    plan.slot2 = IMU_CMD_ID_TARE_CLEAR;
    check(imu_cmd_init(&plan), "T05", "init");
    imu_cmd_service();
    e = make_event(IMU_CMD_EVENT_OPERATOR_CONFIRM);
    check(imu_cmd_post(&e), "T05", "confirm");
    e = make_terminal(IMU_CMD_ID_TARE_CLEAR, IMU_CMD_STATE_SUCCEEDED,
                      IMU_CMD_REASON_OK, false);
    e.terminal.sub.clearActive = IMU_CMD_SUB_SUCCEEDED;
    e.terminal.sub.clearSaved = IMU_CMD_SUB_FAILED;
    check(imu_cmd_post(&e), "T05", "partial");
    check_state(IMU_CMD_ID_TARE_CLEAR, IMU_CMD_STATE_FAILED, "T05", "failed");
    check_reason(IMU_CMD_ID_TARE_CLEAR, IMU_CMD_REASON_TARE_CLEAR_PARTIAL,
                 "T05", "partial reason");
    r = result_of(IMU_CMD_ID_TARE_CLEAR);
    check(r.sub.clearActive == IMU_CMD_SUB_SUCCEEDED, "T05", "active ok");
    check(r.sub.clearSaved == IMU_CMD_SUB_FAILED, "T05", "saved fail");
    check(imu_cmd_warning_required(), "T05", "warning");
    check(!imu_cmd_do_not_acquire(), "T05", "continue");
    (void)finish_settle_acquire("T05");
}

static void test_g_illegal(void)
{
    ImuCmdPlan_t plan;

    imu_cmd_plan_clear(&plan);
    plan.slot2 = IMU_CMD_ID_TARE;
    plan.persistTare = true;
    check(!imu_cmd_init(&plan), "G01", "tare without axes");
    check(imu_cmd_init_reason() == IMU_CMD_REASON_ILLEGAL_PLAN, "G01",
          "illegal plan");

    imu_cmd_plan_clear(&plan);
    plan.slot3 = IMU_CMD_ID_CHECK;
    plan.probeMaskPresent = true;
    check(!imu_cmd_init(&plan), "G02", "check with mask");

    imu_cmd_plan_clear(&plan);
    plan.slot3 = IMU_CMD_ID_PROBE;
    check(!imu_cmd_init(&plan), "G03", "probe without mask");

    imu_cmd_plan_acquire_default(&plan);
    plan.acquisitionDurationS = 0u;
    check(!imu_cmd_init(&plan), "G04", "duration 0");
}

static void test_g_stop_preserves_prior(void)
{
    ImuCmdPlan_t plan;
    ImuCmdEvent_t e;

    imu_cmd_plan_clear(&plan);
    plan.slot1 = IMU_CMD_ID_CALIBRATION;
    check(imu_cmd_init(&plan), "G05", "init");
    imu_cmd_service();
    e = make_terminal(IMU_CMD_ID_CALIBRATION, IMU_CMD_STATE_SUCCEEDED,
                      IMU_CMD_REASON_OK, false);
    check(imu_cmd_post(&e), "G05", "cal ok");
    imu_cmd_service();
    e = make_event(IMU_CMD_EVENT_SETTLE_DONE);
    check(imu_cmd_post(&e), "G05", "settle");
    imu_cmd_service();
    check(active_id() == IMU_CMD_ID_ACQUISITION, "G05", "acquire");
    e = make_event(IMU_CMD_EVENT_PROCESS_STOP);
    check(imu_cmd_post(&e), "G05", "stop");
    check_state(IMU_CMD_ID_CALIBRATION, IMU_CMD_STATE_SUCCEEDED, "G05",
                "cal not rewritten");
    check_reason(IMU_CMD_ID_CALIBRATION, IMU_CMD_REASON_OK, "G05", "still ok");
    check_state(IMU_CMD_ID_ACQUISITION, IMU_CMD_STATE_ABANDONED, "G05",
                "acquire abandoned");
    check(imu_cmd_process_stop_seen(), "G05", "stop seen");
}

int main(void)
{
    test_k01();
    test_k02();
    test_k03();
    test_k04();
    test_k05a();
    test_k05b();
    test_k06();
    test_k07();
    test_t01();
    test_t02();
    test_t03();
    test_t04();
    test_t05();
    test_g_illegal();
    test_g_stop_preserves_prior();

    if (g_fail != 0) {
        fprintf(stderr, "test_imu_cmd: %d failure(s)\n", g_fail);
        return 1;
    }

    printf("test_imu_cmd: pass\n");
    return 0;
}