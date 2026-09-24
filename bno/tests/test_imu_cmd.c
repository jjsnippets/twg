#define _POSIX_C_SOURCE 200809L

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_cmd.h"

/*
 * Phase 4 / 5.2 / 6.1 generic coordinator oracles: family K plus T
 * sub-results and G plan/stop/version checks. Calibration and the tare
 * family and DCD-clear are real stages; STAGE_TERMINAL cannot finish
 * them. DCD-clear failures/recovery live in test_imu_cmd_cal.c.
 * Tare T01-T05 and K02 live in test_imu_tare.c /
 * test_imu_cmd_tare.c; check/probe live in test_imu_cmd_check.c.
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
    if (r.version != IMU_CMD_RESULT_VERSION) {
        fprintf(stderr, "FAIL %s %s: result version %u != %u\n",
                tid, what, r.version, IMU_CMD_RESULT_VERSION);
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

static void test_k01_default_plan_skips_commands_and_acquires(void)
{
    ImuCmdPlan_t plan;
    ImuCmdIdentity_t id;
    ImuCmdEvent_t e;
    ImuCmdProgress_t prog;

    check(imu_cmd_plan_acquire_default(&plan), "K01", "default plan");
    check(plan.version == IMU_CMD_PLAN_VERSION, "K01", "plan version macro");
    check(IMU_CMD_PLAN_VERSION == 2u, "K01", "plan version remains 2");
    check(IMU_CMD_RESULT_VERSION == 4u, "K01", "result version 4");
    check(IMU_CMD_PROGRESS_VERSION == 4u, "K01", "progress version 4");
    check(plan.flightCalMask == 0u, "K01", "default flight mask 0");
    check(imu_cmd_init(&plan), "K01", "init");

    for (id = IMU_CMD_ID_CALIBRATION; id <= IMU_CMD_ID_PROBE; id++) {
        check_state(id, IMU_CMD_STATE_NOT_REQUESTED, "K01",
                    "command not requested");
    }

    imu_cmd_service();
    check(imu_cmd_get_progress(&prog), "K01", "progress read");
    check(prog.version == IMU_CMD_PROGRESS_VERSION, "K01", "progress version 4");
    check(prog.cal.phase == IMU_CMD_CAL_PHASE_NONE, "K01",
          "cal progress zero on settle");
    check(prog.tare.identity == IMU_CMD_ID_NONE, "K01",
          "tare identity zero on settle");
    check(prog.tare.requestedAxes == IMU_CMD_TARE_AXES_NONE, "K01",
          "tare axes zero on settle");
    check(prog.tare.phase == IMU_CMD_TARE_PHASE_NONE, "K01",
          "tare phase zero on settle");
    check(!prog.tare.confirmationPending, "K01", "tare confirm not pending");
    check(!prog.tare.attitudeValid, "K01", "tare attitude not valid");
    check(!prog.tare.attitudeEpochMatched, "K01", "tare epoch match clear");
    check(prog.tare.rotationEpoch == 0u, "K01", "tare rotation epoch zero");
    check(prog.tare.rotationEventSequence == 0ull, "K01",
          "tare rotation sequence zero");
    check(!prog.tare.verificationEvidence, "K01", "tare verify evidence clear");
    check(prog.check.identity == IMU_CMD_ID_NONE, "K01",
          "check identity zero on settle");
    check(prog.check.phase == IMU_CMD_CHECK_PHASE_NONE, "K01",
          "check phase zero on settle");
    check(!prog.check.deadlineValid && !prog.check.gatePassingNow &&
          !prog.check.gateReached, "K01", "check progress zero on settle");
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

static void test_k06_process_stop_abandons_dcd_clear(void)
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

static void test_k07_acquisition_ignores_operator_q(void)
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

static void test_g01_illegal_plans_rejected(void)
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

static void test_g05_process_stop_preserves_prior(void)
{
    ImuCmdPlan_t plan;
    ImuCmdEvent_t e;

    imu_cmd_plan_clear(&plan);
    check(imu_cmd_init(&plan), "G05", "init");
    imu_cmd_service();
    check(active_id() == IMU_CMD_ID_SETTLE, "G05", "settle active");
    e = make_event(IMU_CMD_EVENT_SETTLE_DONE);
    check(imu_cmd_post(&e), "G05", "settle");
    imu_cmd_service();
    check(active_id() == IMU_CMD_ID_ACQUISITION, "G05", "acquire");
    e = make_event(IMU_CMD_EVENT_PROCESS_STOP);
    check(imu_cmd_post(&e), "G05", "stop");
    check_state(IMU_CMD_ID_SETTLE, IMU_CMD_STATE_SUCCEEDED, "G05",
                "prior settle not rewritten");
    check_reason(IMU_CMD_ID_SETTLE, IMU_CMD_REASON_OK, "G05", "still ok");
    check_state(IMU_CMD_ID_ACQUISITION, IMU_CMD_STATE_ABANDONED, "G05",
                "acquire abandoned");
    check(imu_cmd_process_stop_seen(), "G05", "stop seen");
}

static void test_g06_stale_plan_version_rejected(void)
{
    ImuCmdPlan_t plan;

    imu_cmd_plan_clear(&plan);
    check(plan.version == IMU_CMD_PLAN_VERSION, "G06",
          "clear uses current version");
    check(IMU_CMD_PLAN_VERSION != 1u, "G06", "current plan is not version 1");
    plan.version = 1u;
    check(!imu_cmd_init(&plan), "G06", "stale version-1 plan");
    check(imu_cmd_init_reason() == IMU_CMD_REASON_ILLEGAL_PLAN, "G06",
          "illegal plan");
}

static void test_g07_stale_result_and_progress_versions_are_not_current(void)
{
    ImuCmdPlan_t plan;
    ImuCmdResult_t result;
    ImuCmdProgress_t progress;

    check(IMU_CMD_RESULT_VERSION == 4u, "G07", "result is version 4");
    check(IMU_CMD_PROGRESS_VERSION == 4u, "G07", "progress is version 4");
    check(imu_cmd_plan_acquire_default(&plan), "G07", "default plan");
    check(plan.version == 2u, "G07", "plan stays version 2");
    check(imu_cmd_init(&plan), "G07", "init");
    check(imu_cmd_get_result(IMU_CMD_ID_SETTLE, &result), "G07",
          "settle result");
   check(result.version == IMU_CMD_RESULT_VERSION, "G07",
          "fresh result version");
    check(result.version != 2u, "G07",
          "stale result version 2 is not current");
    check(imu_cmd_get_progress(&progress), "G07", "progress");
    check(progress.version == IMU_CMD_PROGRESS_VERSION, "G07",
          "fresh progress version");
    check(progress.version != 2u, "G07",
          "stale progress version 2 is not current");
    check(progress.tare.phase == IMU_CMD_TARE_PHASE_NONE, "G07",
          "tare progress remains zero-initialized");
    check(progress.check.phase == IMU_CMD_CHECK_PHASE_NONE, "G07",
          "check progress remains zero-initialized");
}

int main(void)
{
    test_k01_default_plan_skips_commands_and_acquires();
    test_k06_process_stop_abandons_dcd_clear();
    test_k07_acquisition_ignores_operator_q();
    test_g01_illegal_plans_rejected();
    test_g05_process_stop_preserves_prior();
    test_g06_stale_plan_version_rejected();
    test_g07_stale_result_and_progress_versions_are_not_current();

    if (g_fail != 0) {
        fprintf(stderr, "test_imu_cmd: %d failure(s)\n", g_fail);
        return 1;
    }

    printf("test_imu_cmd: pass\n");
    return 0;
}
