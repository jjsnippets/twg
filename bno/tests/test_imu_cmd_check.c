#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_cal.h"
#include "app/imu_cal_adapter.h"
#include "app/imu_check.h"
#include "app/imu_check_adapter.h"
#include "app/imu_cmd.h"
#include "app/imu_session.h"
#include "app/imu_tare.h"
#include "app/imu_tare_adapter.h"

static int failures;
static bool calInitialized;
static bool tareInitialized;
static bool checkInitialized;
static const uint64_t T0 = 1000000000ull;

static void check(bool ok, const char *id, const char *what)
{
    if (!ok) {
        fprintf(stderr, "FAIL %s: %s\n", id, what);
        failures++;
    }
}

static bool tare_family(ImuCmdIdentity_t id)
{
    return id == IMU_CMD_ID_TARE ||
           id == IMU_CMD_ID_TARE_CLEAR ||
           id == IMU_CMD_ID_TARE_CHECK;
}

static bool check_family(ImuCmdIdentity_t id)
{
    return id == IMU_CMD_ID_CHECK || id == IMU_CMD_ID_PROBE;
}

static ImuCmdEvent_t event_of(ImuCmdEventType_t type)
{
    ImuCmdEvent_t event;
    memset(&event, 0, sizeof(event));
    event.type = type;
    return event;
}

static ImuCmdProgress_t progress(const char *id)
{
    ImuCmdProgress_t p;
    memset(&p, 0, sizeof(p));
    check(imu_cmd_get_progress(&p), id, "progress");
    return p;
}

static ImuCmdResult_t result(ImuCmdIdentity_t identity, const char *id)
{
    ImuCmdResult_t r;
    memset(&r, 0, sizeof(r));
    check(imu_cmd_get_result(identity, &r), id, "result");
    return r;
}

static ImuSampleSnapshot_t snapshot(const char *id)
{
    ImuSampleSnapshot_t s;
    memset(&s, 0, sizeof(s));
    check(imu_session_get_snapshot(&s), id, "snapshot");
    return s;
}

static ImuCmdPlan_t plan_for(ImuCmdIdentity_t id, uint8_t mask)
{
    ImuCmdPlan_t plan;
    imu_cmd_plan_clear(&plan);
    plan.slot3 = id;
    plan.probeMaskPresent = id == IMU_CMD_ID_PROBE;
    plan.probeMask = mask;
    plan.flightCalMask = 0x04u;
    return plan;
}

static void start(const ImuCmdPlan_t *plan, const char *id)
{
    calInitialized = false;
    tareInitialized = false;
    checkInitialized = false;
    imu_session_test_reset();
    check(imu_session_test_open(true), id, "session test open");
    check(imu_cmd_init(plan), id, "coordinator init");
    imu_cmd_service(); /* Select the first requested stage, not an owner turn. */
}

static bool turn(uint64_t nowNs, ImuCmdEventType_t optional,
                 const char *id)
{
    ImuCmdEvent_t event;
    ImuCmdIdentity_t active = progress(id).active;
    bool adapterOk = true;

    imu_session_service();
    if (active == IMU_CMD_ID_CALIBRATION && calInitialized) {
        adapterOk = imu_cal_adapter_pump();
    } else if (tare_family(active) && tareInitialized) {
        adapterOk = imu_tare_adapter_pump();
    } else if (check_family(active) && checkInitialized) {
        adapterOk = imu_check_adapter_pump();
    }
    if (!adapterOk) {
        event = event_of(IMU_CMD_EVENT_SESSION_UNRESTORABLE);
        check(imu_cmd_post(&event), id, "adapter invariant failure routed");
        imu_cmd_service();
        return false;
    }

    event = event_of(IMU_CMD_EVENT_TICK);
    event.monotonicNs = nowNs;
    check(imu_cmd_post(&event), id, "tick");
    if (optional != IMU_CMD_EVENT_TICK) {
        event = event_of(optional);
        check(imu_cmd_post(&event), id, "operator event");
    }
    imu_cmd_service();

    active = progress(id).active;
    if (active == IMU_CMD_ID_CALIBRATION) {
        calInitialized = true;
    } else if (tare_family(active)) {
        tareInitialized = true;
    } else if (check_family(active)) {
        checkInitialized = true;
    }
    return true;
}

static void begin_configured(ImuCmdIdentity_t identity, uint8_t mask,
                             const char *id)
{
    ImuCmdPlan_t plan = plan_for(identity, mask);
    start(&plan, id);
    check(turn(T0, IMU_CMD_EVENT_TICK, id), id, "first turn");
    check(turn(T0 + 1ull, IMU_CMD_EVENT_TICK, id),
          id, "configure turn");
}

static void inject_good(void)
{
    imu_session_test_inject_check_report(IMU_SESSION_TEST_REPORT_ACCEL,
                                          3u, 101ull, 0, 0, 0, 0);
    imu_session_test_inject_check_report(IMU_SESSION_TEST_REPORT_MAG,
                                          2u, 102ull, 0,
                                          12.0f, -4.0f, 40.0f);
    imu_session_test_inject_check_report(IMU_SESSION_TEST_REPORT_GYRO,
                                          0u, 103ull, 0, 0, 0, 0);
    imu_session_test_inject_check_report(IMU_SESSION_TEST_REPORT_RV,
                                          0u, 104ull, 0.25f, 0, 0, 0);
}

static void test_nc01_first_check_turn_only_initializes(void)
{
    ImuCmdPlan_t plan = plan_for(IMU_CMD_ID_CHECK, 0u);
    ImuSampleSnapshot_t before;
    ImuCmdResult_t r;

    start(&plan, "NC01");
    before = snapshot("NC01");
    check(progress("NC01").check.phase == IMU_CMD_CHECK_PHASE_NONE,
          "NC01", "not initialized at stage selection");
    check(turn(T0, IMU_CMD_EVENT_TICK, "NC01"),
          "NC01", "first owner turn");
    r = result(IMU_CMD_ID_CHECK, "NC01");
    check(r.startedNs == T0 &&
          progress("NC01").check.phase == IMU_CMD_CHECK_PHASE_CONFIGURE &&
          imu_check_pending_request().type ==
              IMU_CHECK_REQ_CONFIGURE_CHECK,
          "NC01", "first tick arms pure machine");
    check(snapshot("NC01").readerState == IMU_READER_STATE_CONFIGURING &&
          snapshot("NC01").configurationEpoch ==
              before.configurationEpoch,
          "NC01", "no adapter action on first turn");
    check(turn(T0 + 1ull, IMU_CMD_EVENT_TICK, "NC01"),
          "NC01", "next owner turn");
    check(snapshot("NC01").readerState == IMU_READER_STATE_CHECK &&
          progress("NC01").check.phase == IMU_CMD_CHECK_PHASE_MONITOR,
          "NC01", "configuration pumped on next turn");
}

static void test_nc02_probe_first_tick_preserves_mask(void)
{
    ImuCmdPlan_t plan = plan_for(IMU_CMD_ID_PROBE, 0x80u);
    start(&plan, "NC02");
    check(turn(T0, IMU_CMD_EVENT_TICK, "NC02"),
          "NC02", "first turn");
    check(imu_check_pending_request().identity == IMU_CMD_ID_PROBE &&
          imu_check_pending_request().effectiveMask == 0x80u &&
          progress("NC02").check.deadlineValid &&
          progress("NC02").check.deadlineNs ==
              T0 + IMU_CHECK_PROBE_NS &&
          result(IMU_CMD_ID_PROBE, "NC02").probeMaskRequested ==
              0x80u,
          "NC02", "mask, identity, ten-second clock");
}

static void test_nc03_live_progress_mirrors_check_machine(void)
{
    ImuCmdProgress_t p;
    begin_configured(IMU_CMD_ID_CHECK, 0u, "NC03");
    inject_good();
    check(turn(T0 + 2ull, IMU_CMD_EVENT_TICK, "NC03"),
          "NC03", "facts");
    p = progress("NC03");
    check(p.version == IMU_CMD_PROGRESS_VERSION &&
          p.active == IMU_CMD_ID_CHECK &&
          p.check.identity == IMU_CMD_ID_CHECK &&
          p.check.haveAccel && p.check.haveMag &&
          p.check.accelStatus == 3u && p.check.magStatus == 2u &&
          p.check.gyroStatus == 0u && p.check.rvErrRad == 0.25f &&
          p.check.magXuT == 12.0f && p.check.gatePassingNow,
          "NC03", "authoritative machine facts and verdict");
    check(turn(T0 + 2ull + 1500000000ull,
               IMU_CMD_EVENT_TICK, "NC03"),
          "NC03", "good duration");
    check(progress("NC03").check.sustainedGoodNs >=
              1500000000ull,
          "NC03", "sustained-good time mirrored");
}

static void test_nc04_check_q_restores_then_continues(void)
{
    ImuCmdResult_t r;
    begin_configured(IMU_CMD_ID_CHECK, 0u, "NC04");
    inject_good();
    check(turn(T0 + 2ull, IMU_CMD_EVENT_TICK, "NC04"),
          "NC04", "forward facts");
    check(turn(T0 + 3ull, IMU_CMD_EVENT_OPERATOR_Q, "NC04"),
          "NC04", "q turn");
    check(result(IMU_CMD_ID_CHECK, "NC04").state ==
              IMU_CMD_STATE_RUNNING &&
          imu_check_pending_request().type ==
              IMU_CHECK_REQ_RESTORE_PRODUCTION,
          "NC04", "terminal withheld pending restore");
    check(turn(T0 + 4ull, IMU_CMD_EVENT_TICK, "NC04"),
          "NC04", "restore turn");
    r = result(IMU_CMD_ID_CHECK, "NC04");
    check(r.state == IMU_CMD_STATE_CANCELLED &&
          r.reason == IMU_CMD_REASON_OPERATOR_Q &&
          r.restoredProduction && !r.warningRequired &&
          r.terminalProgress.check.gatePassingNow,
          "NC04", "full cancelled result adopted");
    check(!imu_cmd_do_not_acquire(), "NC04", "acquisition allowed");
    imu_cmd_service();
    check(progress("NC04").active == IMU_CMD_ID_SETTLE,
          "NC04", "settle follows");
    {
        ImuCmdEvent_t e = event_of(IMU_CMD_EVENT_SETTLE_DONE);
        check(imu_cmd_post(&e), "NC04", "settle complete");
        imu_cmd_service();
        check(progress("NC04").active == IMU_CMD_ID_ACQUISITION,
              "NC04", "acquisition follows settle");
    }
}

static void test_nc05_gate_does_not_complete_probe(void)
{
    ImuCmdResult_t r;
    begin_configured(IMU_CMD_ID_PROBE, 0x02u, "NC05");
    inject_good();
    check(turn(T0 + 2ull, IMU_CMD_EVENT_TICK, "NC05"),
          "NC05", "start good");
    check(turn(T0 + 2ull + IMU_CHECK_GATE_NS,
               IMU_CMD_EVENT_TICK, "NC05"),
          "NC05", "reach gate");
    r = result(IMU_CMD_ID_PROBE, "NC05");
    check(progress("NC05").check.gateReached &&
          progress("NC05").active == IMU_CMD_ID_PROBE &&
          r.state == IMU_CMD_STATE_RUNNING &&
          imu_check_pending_request().type == IMU_CHECK_REQ_NONE,
          "NC05", "no early termination or restore");
}

static void test_nc06_deadline_preserves_terminal_verdict(void)
{
    ImuCmdResult_t r;
    begin_configured(IMU_CMD_ID_PROBE, 0x02u, "NC06");
    inject_good();
    check(turn(T0 + 2ull, IMU_CMD_EVENT_TICK, "NC06"),
          "NC06", "facts");
    check(turn(T0 + 2ull + IMU_CHECK_GATE_NS,
               IMU_CMD_EVENT_TICK, "NC06"),
          "NC06", "gate");
    check(turn(T0 + IMU_CHECK_PROBE_NS,
               IMU_CMD_EVENT_TICK, "NC06"),
          "NC06", "deadline");
    check(result(IMU_CMD_ID_PROBE, "NC06").state ==
              IMU_CMD_STATE_RUNNING &&
          imu_check_pending_request().type ==
              IMU_CHECK_REQ_RESTORE_PRODUCTION,
          "NC06", "deadline still awaits restore");
    check(turn(T0 + IMU_CHECK_PROBE_NS + 1ull,
               IMU_CMD_EVENT_TICK, "NC06"),
          "NC06", "restore");
    r = result(IMU_CMD_ID_PROBE, "NC06");
    check(r.state == IMU_CMD_STATE_TIMED_OUT &&
          r.reason == IMU_CMD_REASON_PROBE_DEADLINE &&
          r.sub.probeTimedOut && r.sub.probeReachedGate &&
          r.restoredProduction &&
          r.terminalProgress.check.gatePassingNow &&
          r.terminalProgress.check.gateReached &&
          r.terminalProgress.check.remainingNs == 0ull,
          "NC06", "R7 timeout distinct from good final verdict");
    imu_cmd_service();
    check(progress("NC06").active == IMU_CMD_ID_SETTLE,
          "NC06", "ordinary timeout continues");
}

static void test_nc07_probe_q_is_early_cancel(void)
{
    ImuCmdResult_t r;
    begin_configured(IMU_CMD_ID_PROBE, 0u, "NC07");
    check(turn(T0 + 1000000000ull,
               IMU_CMD_EVENT_OPERATOR_Q, "NC07"),
          "NC07", "q");
    check(turn(T0 + 1000000001ull,
               IMU_CMD_EVENT_TICK, "NC07"),
          "NC07", "restore");
    r = result(IMU_CMD_ID_PROBE, "NC07");
    check(r.state == IMU_CMD_STATE_CANCELLED &&
          r.reason == IMU_CMD_REASON_OPERATOR_Q &&
          r.sub.probeOperatorEndedEarly &&
          !r.sub.probeTimedOut && r.restoredProduction,
          "NC07", "early operator cancellation");
    imu_cmd_service();
    check(progress("NC07").active == IMU_CMD_ID_SETTLE,
          "NC07", "continues");
}

static void test_nc08_config_mismatch_restores_and_continues(void)
{
    ImuCmdPlan_t plan = plan_for(IMU_CMD_ID_PROBE, 0x80u);
    ImuCmdResult_t r;
    start(&plan, "NC08");
    check(turn(T0, IMU_CMD_EVENT_TICK, "NC08"),
          "NC08", "init");
    imu_session_test_set_check_readback(true, 0x01u);
    check(turn(T0 + 1ull, IMU_CMD_EVENT_TICK, "NC08"),
          "NC08", "mismatched configure");
    check(imu_check_pending_request().type ==
              IMU_CHECK_REQ_RESTORE_PRODUCTION &&
          result(IMU_CMD_ID_PROBE, "NC08").state ==
              IMU_CMD_STATE_RUNNING,
          "NC08", "failed config waits for restore");
    check(turn(T0 + 2ull, IMU_CMD_EVENT_TICK, "NC08"),
          "NC08", "restore");
    r = result(IMU_CMD_ID_PROBE, "NC08");
    check(r.state == IMU_CMD_STATE_FAILED &&
          r.reason == IMU_CMD_REASON_CONFIG_FAILED &&
          r.warningRequired && r.restoredProduction &&
          r.sub.probeMaskActualValid &&
          r.sub.probeMaskActual == 0x01u,
          "NC08", "ordinary failure keeps actual-mask evidence");
    check(!imu_cmd_do_not_acquire(), "NC08", "may continue");
    imu_cmd_service();
    check(progress("NC08").active == IMU_CMD_ID_SETTLE,
          "NC08", "settle follows");
}

static void test_nc09_restore_failure_stops_plan(void)
{
    ImuCmdResult_t r;
    begin_configured(IMU_CMD_ID_CHECK, 0u, "NC09");
    check(turn(T0 + 2ull, IMU_CMD_EVENT_OPERATOR_Q, "NC09"),
          "NC09", "q");
    imu_session_test_set_production_result(false);
    check(turn(T0 + 3ull, IMU_CMD_EVENT_TICK, "NC09"),
          "NC09", "failed restore delivered");
    r = result(IMU_CMD_ID_CHECK, "NC09");
    check(r.state == IMU_CMD_STATE_RECOVERY_FAILED &&
          r.reason == IMU_CMD_REASON_SESSION_UNUSABLE &&
          !r.restoredProduction &&
          imu_cmd_do_not_acquire() &&
          imu_cmd_plan_complete() &&
          imu_cmd_request() == IMU_CMD_REQ_STOP_PLAN,
          "NC09", "stop instead of false acquisition");
    imu_cmd_service();
    check(result(IMU_CMD_ID_SETTLE, "NC09").state ==
              IMU_CMD_STATE_NOT_REQUESTED &&
          result(IMU_CMD_ID_ACQUISITION, "NC09").state ==
              IMU_CMD_STATE_NOT_REQUESTED,
          "NC09", "no settle or acquisition");
}

static void test_nc10_external_terminal_rejected(void)
{
    ImuCmdIdentity_t ids[2] = { IMU_CMD_ID_CHECK, IMU_CMD_ID_PROBE };
    unsigned i;

    for (i = 0u; i < 2u; ++i) {
        ImuCmdPlan_t plan = plan_for(ids[i], 0x02u);
        ImuCmdEvent_t e = event_of(IMU_CMD_EVENT_STAGE_TERMINAL);
        start(&plan, "NC10");
        e.terminal.identity = ids[i];
        e.terminal.state = IMU_CMD_STATE_SUCCEEDED;
        e.terminal.reason = IMU_CMD_REASON_OK;
        check(!imu_cmd_post(&e), "NC10", "rejected before init");
        check(turn(T0, IMU_CMD_EVENT_TICK, "NC10"),
              "NC10", "first turn");
        check(!imu_cmd_post(&e), "NC10", "rejected after init");
    }
}

static void test_nc11_process_stop_abandons_immediately(void)
{
    ImuCmdIdentity_t ids[2] = { IMU_CMD_ID_CHECK, IMU_CMD_ID_PROBE };
    unsigned i;

    for (i = 0u; i < 2u; ++i) {
        ImuCmdPlan_t plan = plan_for(ids[i], 0x02u);
        ImuCmdEvent_t e = event_of(IMU_CMD_EVENT_PROCESS_STOP);
        start(&plan, "NC11");
        check(turn(T0, IMU_CMD_EVENT_TICK, "NC11"),
              "NC11", "first tick");
        check(imu_check_pending_request().type ==
              IMU_CHECK_REQ_CONFIGURE_CHECK,
              "NC11", "configure still pending");
        check(imu_cmd_post(&e), "NC11", "stop");
        imu_cmd_service();
        check(result(ids[i], "NC11").state ==
                  IMU_CMD_STATE_ABANDONED &&
              result(ids[i], "NC11").reason ==
                  IMU_CMD_REASON_PROCESS_STOP &&
              imu_cmd_plan_complete() && imu_cmd_do_not_acquire() &&
              imu_cmd_request() == IMU_CMD_REQ_STOP_PLAN,
              "NC11", "immediate coordinator-owned abandonment");
        check(imu_check_pending_request().type ==
              IMU_CHECK_REQ_CONFIGURE_CHECK,
              "NC11", "owner did not pump after stop");
        check(snapshot("NC11").readerState ==
              IMU_READER_STATE_CONFIGURING,
              "NC11", "session was not mutated");
    }
}

static void test_nc12_adapter_invariant_stops_acquisition(void)
{
    ImuCmdPlan_t plan = plan_for(IMU_CMD_ID_CHECK, 0u);
    ImuCmdResult_t r;
    start(&plan, "NC12");
    check(turn(T0, IMU_CMD_EVENT_TICK, "NC12"),
          "NC12", "first tick");
    imu_session_test_reset(); /* No epoch for the pending adapter action. */
    check(!turn(T0 + 1ull, IMU_CMD_EVENT_TICK, "NC12"),
          "NC12", "adapter reports invariant failure");
    r = result(IMU_CMD_ID_CHECK, "NC12");
    check(r.state == IMU_CMD_STATE_RECOVERY_FAILED &&
          r.reason == IMU_CMD_REASON_SESSION_UNUSABLE &&
          imu_cmd_do_not_acquire() && imu_cmd_plan_complete(),
          "NC12", "coordinator stops on adapter invariant");
}

static void test_nc13_prior_tare_result_and_slot_order(void)
{
    ImuCmdPlan_t plan = plan_for(IMU_CMD_ID_CHECK, 0u);
    ImuCmdResult_t prior;
    uint64_t now = T0;

    plan.slot2 = IMU_CMD_ID_TARE_CHECK;
    start(&plan, "NC13");
    check(progress("NC13").active == IMU_CMD_ID_TARE_CHECK,
          "NC13", "slot 2 precedes slot 3");
    check(turn(now, IMU_CMD_EVENT_TICK, "NC13"),
          "NC13", "tare first tick");
    check(turn(++now, IMU_CMD_EVENT_TICK, "NC13"),
          "NC13", "tare configuration");
    check(turn(++now, IMU_CMD_EVENT_OPERATOR_Q, "NC13"),
          "NC13", "end tare check");
    check(turn(++now, IMU_CMD_EVENT_TICK, "NC13"),
          "NC13", "tare restore");
    prior = result(IMU_CMD_ID_TARE_CHECK, "NC13");
    check(prior.state == IMU_CMD_STATE_CANCELLED &&
          prior.restoredProduction, "NC13", "prior full result");
    imu_cmd_service();
    check(progress("NC13").active == IMU_CMD_ID_CHECK,
          "NC13", "slot 3 follows");
    check(turn(++now, IMU_CMD_EVENT_TICK, "NC13"),
          "NC13", "check first tick");
    check(result(IMU_CMD_ID_TARE_CHECK, "NC13").state ==
              prior.state &&
          result(IMU_CMD_ID_TARE_CHECK, "NC13").epochAfter ==
              prior.epochAfter,
          "NC13", "prior result not overwritten");
}

static void test_nc14_r7_requires_no_cached_r8(void)
{
    ImuCmdResult_t r;
    begin_configured(IMU_CMD_ID_PROBE, 0x02u, "NC14");
    inject_good();
    check(turn(T0 + 2ull, IMU_CMD_EVENT_TICK, "NC14"),
          "NC14", "good facts");
    check(turn(T0 + 2ull + IMU_CHECK_GATE_NS,
               IMU_CMD_EVENT_TICK, "NC14"),
          "NC14", "gate");
    /* Last verdict is bad; the earlier gate remains sticky. */
    imu_session_test_inject_check_report(IMU_SESSION_TEST_REPORT_MAG,
                                          0u, 105ull, 0, 0, 0, 0);
    check(turn(T0 + IMU_CHECK_PROBE_NS,
               IMU_CMD_EVENT_TICK, "NC14"),
          "NC14", "deadline");
    check(turn(T0 + IMU_CHECK_PROBE_NS + 1ull,
               IMU_CMD_EVENT_TICK, "NC14"),
          "NC14", "restore");
    r = result(IMU_CMD_ID_PROBE, "NC14");
    check(r.version == IMU_CMD_RESULT_VERSION &&
          r.state == IMU_CMD_STATE_TIMED_OUT &&
          r.reason == IMU_CMD_REASON_PROBE_DEADLINE &&
          r.sub.probeTimedOut && r.sub.probeReachedGate &&
          r.terminalProgress.version ==
              IMU_CMD_PROGRESS_VERSION &&
          r.terminalProgress.check.gateReached &&
          !r.terminalProgress.check.gatePassingNow,
          "NC14", "R7 separates reason, history, and final verdict");
}

int main(void)
{
    test_nc01_first_check_turn_only_initializes();
    test_nc02_probe_first_tick_preserves_mask();
    test_nc03_live_progress_mirrors_check_machine();
    test_nc04_check_q_restores_then_continues();
    test_nc05_gate_does_not_complete_probe();
    test_nc06_deadline_preserves_terminal_verdict();
    test_nc07_probe_q_is_early_cancel();
    test_nc08_config_mismatch_restores_and_continues();
    test_nc09_restore_failure_stops_plan();
    test_nc10_external_terminal_rejected();
    test_nc11_process_stop_abandons_immediately();
    test_nc12_adapter_invariant_stops_acquisition();
    test_nc13_prior_tare_result_and_slot_order();
    test_nc14_r7_requires_no_cached_r8();
    if (failures != 0) {
        fprintf(stderr, "test_imu_cmd_check: %d failure(s)\n",
                failures);
        return 1;
    }
    puts("test_imu_cmd_check: pass");
    return 0;
}
