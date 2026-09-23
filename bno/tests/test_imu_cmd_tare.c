#define _POSIX_C_SOURCE 200809L

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_cal.h"
#include "app/imu_cal_adapter.h"
#include "app/imu_cmd.h"
#include "app/imu_session.h"
#include "app/imu_tare.h"
#include "app/imu_tare_adapter.h"

/*
 * Phase 6.5 host composition. Frozen owner turn:
 *   1. imu_session_service()
 *   2. pump the adapter for the one active initialized composed command
 *   3. post TICK
 *   4. at most one operator event
 *   5. imu_cmd_service() once
 *
 * K02/E08: N03. E07: N01. K04: N11/N12.
 */

static int g_fail;
static bool g_cal_inited;
static bool g_tare_inited;
static const uint64_t T0 = 1000000000ull;

static void check(bool ok, const char *id, const char *what)
{
    if (!ok) {
        fprintf(stderr, "FAIL %s: %s\n", id, what);
        g_fail++;
    }
}

static bool is_tare_family(ImuCmdIdentity_t id)
{
    return id == IMU_CMD_ID_TARE ||
           id == IMU_CMD_ID_TARE_CLEAR ||
           id == IMU_CMD_ID_TARE_CHECK;
}

static ImuCmdEvent_t make_event(ImuCmdEventType_t type)
{
    ImuCmdEvent_t event;

    memset(&event, 0, sizeof(event));
    event.type = type;
    return event;
}

static ImuCmdEvent_t make_tick(uint64_t now_ns)
{
    ImuCmdEvent_t event = make_event(IMU_CMD_EVENT_TICK);

    event.monotonicNs = now_ns;
    return event;
}

static ImuCmdProgress_t progress_of(const char *id)
{
    ImuCmdProgress_t progress;

    memset(&progress, 0, sizeof(progress));
    check(imu_cmd_get_progress(&progress), id, "get progress");
    return progress;
}

static ImuCmdResult_t result_of(ImuCmdIdentity_t identity, const char *id)
{
    ImuCmdResult_t result;

    memset(&result, 0, sizeof(result));
    check(imu_cmd_get_result(identity, &result), id, "get result");
    return result;
}

static ImuSampleSnapshot_t snapshot_of(const char *id)
{
    ImuSampleSnapshot_t snapshot;

    memset(&snapshot, 0, sizeof(snapshot));
    check(imu_session_get_snapshot(&snapshot), id, "snapshot");
    return snapshot;
}

static void reset_plan(const ImuCmdPlan_t *plan, const char *id)
{
    g_cal_inited = false;
    g_tare_inited = false;
    imu_session_test_reset();
    check(imu_session_test_open(true), id, "open");
    check(imu_cmd_init(plan), id, "cmd init");
    imu_cmd_service();
}

static bool owner_turn(uint64_t now_ns, ImuCmdEventType_t optional_event,
                       const char *id)
{
    ImuCmdEvent_t event;
    ImuCmdProgress_t progress;
    ImuCmdIdentity_t active;

    progress = progress_of(id);
    active = progress.active;

    imu_session_service();

    if (g_cal_inited && active == IMU_CMD_ID_CALIBRATION) {
        if (!imu_cal_adapter_pump()) {
            event = make_event(IMU_CMD_EVENT_SESSION_UNRESTORABLE);
            check(imu_cmd_post(&event), id, "cal unrestorable");
            imu_cmd_service();
            return false;
        }
    } else if (g_tare_inited && is_tare_family(active)) {
        if (!imu_tare_adapter_pump()) {
            event = make_event(IMU_CMD_EVENT_SESSION_UNRESTORABLE);
            check(imu_cmd_post(&event), id, "tare unrestorable");
            imu_cmd_service();
            return false;
        }
    }

    event = make_tick(now_ns);
    check(imu_cmd_post(&event), id, "tick");
    if (optional_event != IMU_CMD_EVENT_TICK) {
        event = make_event(optional_event);
        check(imu_cmd_post(&event), id, "operator event");
    }
    imu_cmd_service();

    progress = progress_of(id);
    if (progress.active == IMU_CMD_ID_CALIBRATION) {
        g_cal_inited = true;
    }
    if (is_tare_family(progress.active)) {
        g_tare_inited = true;
    }
    return true;
}

static ImuCmdPlan_t tare_plan(ImuCmdIdentity_t tare_id, ImuCmdTareAxes_t axes,
                             ImuCmdIdentity_t slot3)
{
    ImuCmdPlan_t plan;

    imu_cmd_plan_clear(&plan);
    plan.slot2 = tare_id;
    plan.tareAxes = axes;
    plan.persistTare = (tare_id == IMU_CMD_ID_TARE);
    plan.slot3 = slot3;
    if (slot3 == IMU_CMD_ID_PROBE) {
        plan.probeMaskPresent = true;
        plan.probeMask = 0x02u;
    }
    return plan;
}

static void inject_verify_facts(const char *id)
{
    ImuTareFacts_t facts;
    ImuSampleSnapshot_t snap;

    memset(&facts, 0, sizeof(facts));
    snap = snapshot_of(id);
    facts.version = IMU_TARE_FACTS_VERSION;
    facts.valid = true;
    facts.configurationEpoch = snap.configurationEpoch;
    facts.quatReal = 1.0f;
    facts.yawRad = 0.05f;
    imu_session_test_inject_tare_facts(&facts);
}

static uint64_t drive_success_to_restore(uint64_t now_ns, const char *id)
{
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, id), id, "first tick");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_OPERATOR_CONFIRM, id), id,
          "confirm");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, id), id, "configure pump");
    check(progress_of(id).tare.phase == IMU_CMD_TARE_PHASE_SETTLE, id,
          "settle");
    now_ns += IMU_TARE_SETTLE_NS;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, id), id, "settle deadline");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_TARE_NOW, id,
          "tare-now pending");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, id), id, "tare-now pump");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_PERSIST, id,
          "persist pending");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, id), id, "persist pump");
    inject_verify_facts(id);
    now_ns += IMU_TARE_VERIFY_NS;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, id), id, "verify deadline");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_RESTORE_PRODUCTION,
          id, "restore pending");
    return now_ns;
}

static void test_n01_z_tare_success(void)
{
    ImuCmdPlan_t plan = tare_plan(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z,
                                 IMU_CMD_ID_NONE);
    ImuCmdResult_t result;
    ImuSampleSnapshot_t before_now;
    uint64_t now_ns = T0;

    reset_plan(&plan, "N01");
    check(progress_of("N01").active == IMU_CMD_ID_TARE, "N01", "tare selected");

    now_ns = drive_success_to_restore(now_ns, "N01");
    before_now = snapshot_of("N01");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N01"), "N01", "restore");
    result = result_of(IMU_CMD_ID_TARE, "N01");
    check(result.state == IMU_CMD_STATE_SUCCEEDED, "N01", "succeeded");
    check(result.sub.tareNow == IMU_CMD_SUB_SUCCEEDED, "N01", "tare-now");
    check(result.sub.persist == IMU_CMD_SUB_SUCCEEDED, "N01", "persist");
    check(result.verified, "N01", "verified");
    check(result.restoredProduction, "N01", "restored");
    check(result.terminalProgress.tare.phase == IMU_CMD_TARE_PHASE_RESTORE,
          "N01", "adopted full result");
    check(imu_session_test_last_tare_axes() == 0x04u, "N01", "E07 Z mapping");
    check(snapshot_of("N01").configurationEpoch >=
              before_now.configurationEpoch,
          "N01", "restore epoch factual");

    imu_cmd_service();
    check(progress_of("N01").active == IMU_CMD_ID_SETTLE, "N01",
          "N18 settle follows");
}

static void test_n02_full_axis_request(void)
{
    ImuCmdPlan_t plan = tare_plan(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_FULL,
                                 IMU_CMD_ID_NONE);
    uint64_t now_ns = T0;

    reset_plan(&plan, "N02");
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N02"), "N02", "init");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_OPERATOR_CONFIRM, "N02"), "N02",
          "confirm");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N02"), "N02", "configure");
    now_ns += IMU_TARE_SETTLE_NS;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N02"), "N02", "deadline");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N02"), "N02", "tare-now");
    check(imu_session_test_last_tare_axes() == 0x07u, "N02", "full axes");
}

static void test_n03_q_before_mutation(void)
{
    /* K02 / E08 */
    ImuCmdPlan_t plan = tare_plan(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z,
                                 IMU_CMD_ID_CHECK);
    ImuCmdResult_t result;
    ImuSampleSnapshot_t before;
    uint64_t now_ns = T0;

    reset_plan(&plan, "N03");
    before = snapshot_of("N03");
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N03"), "N03", "init");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_NONE, "N03",
          "no configure yet");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_OPERATOR_Q, "N03"), "N03", "q");
    result = result_of(IMU_CMD_ID_TARE, "N03");
    check(result.state == IMU_CMD_STATE_CANCELLED, "N03", "cancelled");
    check(result.reason == IMU_CMD_REASON_OPERATOR_Q, "N03", "q");
    check(result.sub.tareNow == IMU_CMD_SUB_NOT_ATTEMPTED, "N03",
          "no tare-now");
    check(!result.restoredProduction, "N03", "no restore");
    check(snapshot_of("N03").configurationEpoch == before.configurationEpoch,
          "N03", "E08 epoch unchanged");
    check(!imu_session_test_have_last_tare_now(), "N03", "no adapter tare-now");

    imu_cmd_service();
    check(progress_of("N03").active == IMU_CMD_ID_CHECK, "N03",
          "K02 continues to check");
}

static void test_n04_tare_now_fail_continues(void)
{
    ImuCmdPlan_t plan = tare_plan(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z,
                                 IMU_CMD_ID_CHECK);
    ImuCmdResult_t result;
    uint64_t now_ns = T0;

    reset_plan(&plan, "N04");
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N04"), "N04", "init");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_OPERATOR_CONFIRM, "N04"), "N04",
          "confirm");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N04"), "N04", "configure");
    now_ns += IMU_TARE_SETTLE_NS;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N04"), "N04", "deadline");
    imu_session_test_set_tare_now_result(false);
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N04"), "N04", "tare-now fail");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N04"), "N04", "restore");
    result = result_of(IMU_CMD_ID_TARE, "N04");
    check(result.state == IMU_CMD_STATE_FAILED, "N04", "failed");
    check(result.reason == IMU_CMD_REASON_TARE_NOW_FAILED, "N04", "reason");
    check(result.warningRequired, "N04", "warning");
    check(result.restoredProduction, "N04", "restored");
    imu_cmd_service();
    check(progress_of("N04").active == IMU_CMD_ID_CHECK, "N04",
          "N17 continues");
}

static void test_n05_persist_fail_preserves_tare_now(void)
{
    ImuCmdPlan_t plan = tare_plan(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z,
                                 IMU_CMD_ID_NONE);
    ImuCmdResult_t result;
    uint64_t now_ns = T0;

    reset_plan(&plan, "N05");
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N05"), "N05", "init");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_OPERATOR_CONFIRM, "N05"), "N05",
          "confirm");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N05"), "N05", "configure");
    now_ns += IMU_TARE_SETTLE_NS;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N05"), "N05", "deadline");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N05"), "N05", "tare-now");
    imu_session_test_set_persist_tare_result(false);
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N05"), "N05", "persist fail");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N05"), "N05", "restore");
    result = result_of(IMU_CMD_ID_TARE, "N05");
    check(result.sub.tareNow == IMU_CMD_SUB_SUCCEEDED, "N05", "tare-now kept");
    check(result.sub.persist == IMU_CMD_SUB_FAILED, "N05", "persist failed");
    check(result.state == IMU_CMD_STATE_FAILED, "N05", "overall fail");
    check(result.warningRequired, "N05", "warning");
    check(result.restoredProduction, "N05", "restored");
}

static void test_n06_verify_fail_continues(void)
{
    ImuCmdPlan_t plan = tare_plan(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z,
                                 IMU_CMD_ID_NONE);
    ImuCmdResult_t result;
    uint64_t now_ns = T0;

    reset_plan(&plan, "N06");
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N06"), "N06", "init");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_OPERATOR_CONFIRM, "N06"), "N06",
          "confirm");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N06"), "N06", "configure");
    now_ns += IMU_TARE_SETTLE_NS;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N06"), "N06", "deadline");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N06"), "N06", "tare-now");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N06"), "N06", "persist");
    now_ns += IMU_TARE_VERIFY_NS;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N06"), "N06", "no evidence");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N06"), "N06", "restore");
    result = result_of(IMU_CMD_ID_TARE, "N06");
    check(result.reason == IMU_CMD_REASON_TARE_VERIFY_FAILED, "N06", "verify");
    check(result.restoredProduction, "N06", "restored");
}

static void test_n07_full_clear_success(void)
{
    ImuCmdPlan_t plan = tare_plan(IMU_CMD_ID_TARE_CLEAR, IMU_CMD_TARE_AXES_NONE,
                                 IMU_CMD_ID_NONE);
    ImuCmdResult_t result;
    uint64_t now_ns = T0;

    reset_plan(&plan, "N07");
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N07"), "N07", "init");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_OPERATOR_CONFIRM, "N07"), "N07",
          "confirm");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N07"), "N07", "configure");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N07"), "N07", "clear");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N07"), "N07", "restore");
    result = result_of(IMU_CMD_ID_TARE_CLEAR, "N07");
    check(result.state == IMU_CMD_STATE_SUCCEEDED, "N07", "ok");
    check(result.sub.clearActive == IMU_CMD_SUB_SUCCEEDED, "N07", "active");
    check(result.sub.clearSaved == IMU_CMD_SUB_SUCCEEDED, "N07", "saved");
}

static void test_n08_clear_failure_continues(void)
{
    /* Session backend cannot observe T05 partial; that stays in test_imu_tare.
     * N08 proves ordinary clear failure still restores and continues. */
    ImuCmdPlan_t plan = tare_plan(IMU_CMD_ID_TARE_CLEAR, IMU_CMD_TARE_AXES_NONE,
                                 IMU_CMD_ID_CHECK);
    ImuCmdResult_t result;
    uint64_t now_ns = T0;

    reset_plan(&plan, "N08");
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N08"), "N08", "init");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_OPERATOR_CONFIRM, "N08"), "N08",
          "confirm");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N08"), "N08", "configure");
    imu_session_test_set_clear_tare_result(false);
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N08"), "N08", "clear fail");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N08"), "N08", "restore");
    result = result_of(IMU_CMD_ID_TARE_CLEAR, "N08");
    check(result.state == IMU_CMD_STATE_FAILED, "N08", "failed");
    check(result.reason == IMU_CMD_REASON_TARE_CLEAR_FAILED, "N08", "reason");
    check(result.warningRequired, "N08", "warning");
    check(result.restoredProduction, "N08", "restored");
    imu_cmd_service();
    check(progress_of("N08").active == IMU_CMD_ID_CHECK, "N08", "continues");
}

static void test_n09_n10_tare_check(void)
{
    ImuCmdPlan_t plan = tare_plan(IMU_CMD_ID_TARE_CHECK, IMU_CMD_TARE_AXES_NONE,
                                 IMU_CMD_ID_NONE);
    ImuTareFacts_t facts;
    ImuCmdProgress_t progress;
    ImuCmdResult_t result;
    uint64_t now_ns = T0;

    reset_plan(&plan, "N09");
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N09"), "N09", "init");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_CONFIGURE, "N09",
          "N15 configure waits for next pump");
    check(snapshot_of("N09").readerState == IMU_READER_STATE_CONFIGURING,
          "N09", "N15 first turn did not pump");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N09"), "N09", "configure");
    check(progress_of("N09").tare.phase == IMU_CMD_TARE_PHASE_CHECK, "N09",
          "check");
    memset(&facts, 0, sizeof(facts));
    facts.version = IMU_TARE_FACTS_VERSION;
    facts.valid = true;
    facts.yawRad = 0.20f;
    imu_session_test_inject_tare_facts(&facts);
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N09"), "N09", "facts");
    progress = progress_of("N09");
    check(progress.tare.yawRad == 0.20f, "N09", "mirrored");
    check(progress.active == IMU_CMD_ID_TARE_CHECK, "N09", "still active");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_OPERATOR_Q, "N10"), "N10", "q");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N10"), "N10", "restore");
    result = result_of(IMU_CMD_ID_TARE_CHECK, "N10");
    check(result.state == IMU_CMD_STATE_CANCELLED, "N10", "cancelled");
    check(!result.warningRequired, "N10", "check q has no warning");
    imu_cmd_service();
    check(progress_of("N10").active == IMU_CMD_ID_SETTLE, "N10", "continues");
}

static void test_n11_restore_failure_stops(void)
{
    /* K04 analogue for tare */
    ImuCmdPlan_t plan = tare_plan(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z,
                                 IMU_CMD_ID_CHECK);
    ImuCmdResult_t result;
    uint64_t now_ns = T0;

    reset_plan(&plan, "N11");
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N11"), "N11", "init");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_OPERATOR_CONFIRM, "N11"), "N11",
          "confirm");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N11"), "N11", "configure");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_OPERATOR_Q, "N11"), "N11", "q");
    imu_session_test_set_production_result(false);
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N11"), "N11", "restore fail");
    result = result_of(IMU_CMD_ID_TARE, "N11");
    check(result.state == IMU_CMD_STATE_RECOVERY_FAILED, "N11", "recovery");
    check(imu_cmd_do_not_acquire(), "N11", "do not acquire");
    check(imu_cmd_plan_complete(), "N11", "stopped");
    imu_cmd_service();
    check(progress_of("N11").active == IMU_CMD_ID_NONE, "N11", "no later stage");
}

static void test_n12_adapter_hard_failure(void)
{
    ImuCmdPlan_t plan = tare_plan(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z,
                                 IMU_CMD_ID_NONE);
    ImuCmdResult_t result;
    uint64_t now_ns = T0;

    reset_plan(&plan, "N12");
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N12"), "N12", "init");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_OPERATOR_CONFIRM, "N12"), "N12",
          "confirm");
    imu_session_test_reset();
    now_ns += 1ull;
    (void)owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N12");
    result = result_of(IMU_CMD_ID_TARE, "N12");
    check(result.state == IMU_CMD_STATE_RECOVERY_FAILED, "N12", "unrestorable");
    check(imu_cmd_do_not_acquire(), "N12", "do not acquire");
}

static void test_n13_terminal_rejected(void)
{
    ImuCmdPlan_t plan;
    ImuCmdEvent_t terminal;
    ImuCmdIdentity_t ids[3];
    unsigned i;

    ids[0] = IMU_CMD_ID_TARE;
    ids[1] = IMU_CMD_ID_TARE_CLEAR;
    ids[2] = IMU_CMD_ID_TARE_CHECK;
    for (i = 0u; i < 3u; i++) {
        plan = tare_plan(ids[i],
                         (ids[i] == IMU_CMD_ID_TARE) ? IMU_CMD_TARE_AXES_Z
                                                     : IMU_CMD_TARE_AXES_NONE,
                         IMU_CMD_ID_NONE);
        reset_plan(&plan, "N13");
        memset(&terminal, 0, sizeof(terminal));
        terminal.type = IMU_CMD_EVENT_STAGE_TERMINAL;
        terminal.terminal.identity = ids[i];
        terminal.terminal.state = IMU_CMD_STATE_SUCCEEDED;
        terminal.terminal.reason = IMU_CMD_REASON_OK;
        check(!imu_cmd_post(&terminal), "N13", "rejected before tick");
        check(owner_turn(T0, IMU_CMD_EVENT_TICK, "N13"), "N13", "init");
        check(!imu_cmd_post(&terminal), "N13", "rejected after init");
    }
}

static void test_n14_process_stop(void)
{
    ImuCmdPlan_t plan = tare_plan(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z,
                                 IMU_CMD_ID_NONE);
    ImuCmdEvent_t event;
    ImuCmdResult_t result;
    ImuTarePendingRequest_t request;

    reset_plan(&plan, "N14");
    check(owner_turn(T0, IMU_CMD_EVENT_TICK, "N14"), "N14", "init");
    check(owner_turn(T0 + 1ull, IMU_CMD_EVENT_OPERATOR_CONFIRM, "N14"),
          "N14", "confirm");
    request = imu_tare_pending_request();
    check(request.type == IMU_TARE_REQ_CONFIGURE, "N14", "configure pending");
    event = make_event(IMU_CMD_EVENT_PROCESS_STOP);
    check(imu_cmd_post(&event), "N14", "stop");
    imu_cmd_service();
    result = result_of(IMU_CMD_ID_TARE, "N14");
    check(result.state == IMU_CMD_STATE_ABANDONED, "N14", "abandoned");
    check(result.reason == IMU_CMD_REASON_PROCESS_STOP, "N14", "process stop");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_CONFIGURE, "N14",
          "owner did not pump after stop");
    check(imu_cmd_plan_complete(), "N14", "stopped");
}

static void test_n16_cal_then_tare_slot_order(void)
{
    ImuCmdPlan_t plan;
    uint64_t now_ns = T0;

    imu_cmd_plan_clear(&plan);
    plan.slot1 = IMU_CMD_ID_CALIBRATION;
    plan.slot2 = IMU_CMD_ID_TARE;
    plan.tareAxes = IMU_CMD_TARE_AXES_Z;
    plan.persistTare = true;
    plan.flightCalMask = 0x00u;
    reset_plan(&plan, "N16");
    check(progress_of("N16").active == IMU_CMD_ID_CALIBRATION, "N16",
          "slot 1 first");
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N16"), "N16", "cal init");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N16"), "N16", "cal configure");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_OPERATOR_Q, "N16"), "N16", "cal q");
    now_ns += 1ull;
    check(owner_turn(now_ns, IMU_CMD_EVENT_TICK, "N16"), "N16", "cal restore");
    imu_cmd_service();
    check(progress_of("N16").active == IMU_CMD_ID_TARE, "N16", "slot 2 tare");
}

int main(void)
{
    test_n01_z_tare_success();
    test_n02_full_axis_request();
    test_n03_q_before_mutation();
    test_n04_tare_now_fail_continues();
    test_n05_persist_fail_preserves_tare_now();
    test_n06_verify_fail_continues();
    test_n07_full_clear_success();
    test_n08_clear_failure_continues();
    test_n09_n10_tare_check();
    test_n11_restore_failure_stops();
    test_n12_adapter_hard_failure();
    test_n13_terminal_rejected();
    test_n14_process_stop();
    test_n16_cal_then_tare_slot_order();

    if (g_fail != 0) {
        fprintf(stderr, "test_imu_cmd_tare: %d failure(s)\n", g_fail);
        return 1;
    }
    printf("test_imu_cmd_tare: pass\n");
    return 0;
}
