#define _POSIX_C_SOURCE 200809L
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_tare.h"

static int g_fail;
static const uint64_t T0 = 1000000000ull;

static void check(bool ok, const char *id, const char *what)
{
    if (!ok) {
        fprintf(stderr, "FAIL %s: %s\n", id, what);
        g_fail++;
    }
}

static ImuTareEvent_t make_event(ImuTareEventType_t type)
{
    ImuTareEvent_t e;

    memset(&e, 0, sizeof(e));
    e.type = type;
    return e;
}

static void tick_and_service(uint64_t nowNs)
{
    ImuTareEvent_t e = make_event(IMU_TARE_EVENT_TICK);

    e.monotonicNs = nowNs;
    check(imu_tare_post(&e), "HELPER", "tick");
    imu_tare_service();
}

static void confirm(void)
{
    ImuTareEvent_t e = make_event(IMU_TARE_EVENT_OPERATOR_CONFIRM);

    check(imu_tare_post(&e), "HELPER", "confirm");
    imu_tare_service();
}

static ImuTareEvent_t make_session(ImuTareRequestType_t type, bool success,
                                  uint32_t epochBefore, uint32_t epochAfter)
{
    ImuTareEvent_t e = make_event(IMU_TARE_EVENT_SESSION_RESULT);

    e.session.type = type;
    e.session.success = success;
    e.session.epochBefore = epochBefore;
    e.session.epochAfter = epochAfter;
    return e;
}

static ImuCmdProgress_t progress(void)
{
    ImuCmdProgress_t p;

    memset(&p, 0, sizeof(p));
    check(imu_tare_get_progress(&p), "HELPER", "get progress");
    return p;
}

static ImuCmdResult_t result_of(void)
{
    ImuCmdResult_t r;

    memset(&r, 0, sizeof(r));
    check(imu_tare_get_result(&r), "HELPER", "get result");
    return r;
}

static ImuTareFacts_t good_facts(uint32_t epoch)
{
    ImuTareFacts_t facts;

    memset(&facts, 0, sizeof(facts));
    facts.version = IMU_TARE_FACTS_VERSION;
    facts.configurationEpoch = epoch;
    facts.valid = true;
    facts.quatReal = 1.0f;
    facts.rotationEventSequence = 4ull;
    facts.hostDecodeNs = T0 + 1ull;
    facts.yawRad = 0.10f;
    return facts;
}

static bool configure_ok(ImuCmdIdentity_t id, ImuCmdTareAxes_t axes,
                         const char *tid)
{
    ImuTareEvent_t e;

    check(imu_tare_init(id, axes, 0x00u, T0), tid, "init");
    if (id != IMU_CMD_ID_TARE_CHECK) {
        confirm();
    }
    check(imu_tare_pending_request().type == IMU_TARE_REQ_CONFIGURE, tid,
          "configure pending");
    e = make_session(IMU_TARE_REQ_CONFIGURE, true, 1u, 2u);
    check(imu_tare_post(&e), tid, "configure ok");
    return true;
}

static void restore_ok(const char *tid)
{
    ImuTareEvent_t e;

    check(imu_tare_pending_request().type == IMU_TARE_REQ_RESTORE_PRODUCTION,
          tid, "restore pending");
    e = make_session(IMU_TARE_REQ_RESTORE_PRODUCTION, true, 3u, 4u);
    check(imu_tare_post(&e), tid, "restore ok");
}

static void drive_to_persist_pending(ImuCmdTareAxes_t axes, const char *tid)
{
    ImuTareEvent_t e;

    configure_ok(IMU_CMD_ID_TARE, axes, tid);
    check(progress().tare.phase == IMU_CMD_TARE_PHASE_SETTLE, tid, "settle");
    tick_and_service(T0 + IMU_TARE_SETTLE_NS);
    check(imu_tare_pending_request().type == IMU_TARE_REQ_TARE_NOW, tid,
          "tare-now pending");
    check(imu_tare_pending_request().tareAxes == axes, tid, "axes on request");
    e = make_session(IMU_TARE_REQ_TARE_NOW, true, 2u, 3u);
    check(imu_tare_post(&e), tid, "tare-now ok");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_PERSIST, tid,
          "persist pending");
}

static void test_t01_tare_now_and_persist_succeed(void)
{
    ImuTareEvent_t e;
    ImuCmdResult_t r;
    ImuTareFacts_t facts;


    drive_to_persist_pending(IMU_CMD_TARE_AXES_Z, "T01");
    e = make_session(IMU_TARE_REQ_PERSIST, true, 3u, 3u);
    check(imu_tare_post(&e), "T01", "persist ok");
    facts = good_facts(3u);
    check(imu_tare_feed_facts(&facts), "T01", "post-tare facts");
    tick_and_service(T0 + IMU_TARE_SETTLE_NS + IMU_TARE_VERIFY_NS);
    restore_ok("T01");
    check(imu_tare_complete(), "T01", "complete");
    r = result_of();
    check(r.state == IMU_CMD_STATE_SUCCEEDED, "T01", "succeeded");
    check(r.reason == IMU_CMD_REASON_OK, "T01", "ok");
    check(r.sub.tareNow == IMU_CMD_SUB_SUCCEEDED, "T01", "tare-now");
    check(r.sub.persist == IMU_CMD_SUB_SUCCEEDED, "T01", "persist");
    check(r.verified, "T01", "verified");
    check(r.restoredProduction, "T01", "restored");
    check(!r.warningRequired, "T01", "no warning");
    check(r.terminalProgress.tare.phase == IMU_CMD_TARE_PHASE_RESTORE, "T01",
          "terminal progress is restore");
}

static void test_t02_tare_now_failed_skips_persist(void)
{
    ImuTareEvent_t e;
    ImuCmdResult_t r;

    configure_ok(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z, "T02");
    tick_and_service(T0 + IMU_TARE_SETTLE_NS);
    e = make_session(IMU_TARE_REQ_TARE_NOW, false, 2u, 2u);
    check(imu_tare_post(&e), "T02", "tare-now fail");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_RESTORE_PRODUCTION,
          "T02", "restore after tare-now fail");
    restore_ok("T02");
    r = result_of();
    check(r.state == IMU_CMD_STATE_FAILED, "T02", "failed");
    check(r.reason == IMU_CMD_REASON_TARE_NOW_FAILED, "T02", "tare_now_failed");
    check(r.sub.tareNow == IMU_CMD_SUB_FAILED, "T02", "tare-now failed");
    check(r.sub.persist == IMU_CMD_SUB_NOT_ATTEMPTED, "T02", "persist skipped");
}

static void test_t03_persist_failed_continues(void)
{
    ImuTareEvent_t e;
    ImuCmdResult_t r;

    drive_to_persist_pending(IMU_CMD_TARE_AXES_Z, "T03");
    e = make_session(IMU_TARE_REQ_PERSIST, false, 3u, 3u);
    check(imu_tare_post(&e), "T03", "persist fail");
    restore_ok("T03");
    r = result_of();
    check(r.state == IMU_CMD_STATE_FAILED, "T03", "overall fail");
    check(r.reason == IMU_CMD_REASON_TARE_PERSIST_FAILED, "T03",
          "tare_persist_failed");
    check(r.sub.tareNow == IMU_CMD_SUB_SUCCEEDED, "T03", "tare-now ok");
    check(r.sub.persist == IMU_CMD_SUB_FAILED, "T03", "persist failed");
    check(r.warningRequired, "T03", "warning");
    check(!r.verified, "T03", "not verified");
}

static void test_t04_clear_active_and_saved_succeed(void)
{
    ImuTareEvent_t e;
    ImuCmdResult_t r;

    configure_ok(IMU_CMD_ID_TARE_CLEAR, IMU_CMD_TARE_AXES_NONE, "T04");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_CLEAR, "T04",
          "clear pending");
    e = make_session(IMU_TARE_REQ_CLEAR, true, 2u, 3u);
    e.session.clearActive = IMU_CMD_SUB_SUCCEEDED;
    e.session.clearSaved = IMU_CMD_SUB_SUCCEEDED;
    check(imu_tare_post(&e), "T04", "clear ok");
    restore_ok("T04");
    r = result_of();
    check(r.state == IMU_CMD_STATE_SUCCEEDED, "T04", "ok");
    check(r.sub.clearActive == IMU_CMD_SUB_SUCCEEDED, "T04", "active");
    check(r.sub.clearSaved == IMU_CMD_SUB_SUCCEEDED, "T04", "saved");
}

static void test_t05_partial_clear_is_failed(void)
{
    ImuTareEvent_t e;
    ImuCmdResult_t r;

    configure_ok(IMU_CMD_ID_TARE_CLEAR, IMU_CMD_TARE_AXES_NONE, "T05");
    e = make_session(IMU_TARE_REQ_CLEAR, true, 2u, 3u);
    e.session.clearActive = IMU_CMD_SUB_SUCCEEDED;
    e.session.clearSaved = IMU_CMD_SUB_FAILED;
    check(imu_tare_post(&e), "T05", "partial");
    restore_ok("T05");
    r = result_of();
    check(r.state == IMU_CMD_STATE_FAILED, "T05", "failed");
    check(r.reason == IMU_CMD_REASON_TARE_CLEAR_PARTIAL, "T05", "partial");
    check(r.sub.clearActive == IMU_CMD_SUB_SUCCEEDED, "T05", "active ok");
    check(r.sub.clearSaved == IMU_CMD_SUB_FAILED, "T05", "saved fail");
    check(r.warningRequired, "T05", "warning");
    check(r.restoredProduction, "T05", "restored");
}

static void test_h01_z_and_full_axes(void)
{
    configure_ok(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z, "H01z");
    tick_and_service(T0 + IMU_TARE_SETTLE_NS);
    check(imu_tare_pending_request().type == IMU_TARE_REQ_TARE_NOW, "H01z",
          "tare-now pending");
    check(imu_tare_pending_request().tareAxes == IMU_CMD_TARE_AXES_Z, "H01z",
          "Z request");

    configure_ok(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_FULL, "H01f");
    tick_and_service(T0 + IMU_TARE_SETTLE_NS);
    check(imu_tare_pending_request().type == IMU_TARE_REQ_TARE_NOW, "H01f",
          "tare-now pending");
    check(imu_tare_pending_request().tareAxes == IMU_CMD_TARE_AXES_FULL,
          "H01f", "full request");
}

static void test_h02_q_before_confirm(void)
{
    ImuTareEvent_t e;
    ImuCmdResult_t r;

    check(imu_tare_init(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z, 0x00u, T0),
          "H02", "init");
    e = make_event(IMU_TARE_EVENT_OPERATOR_Q);
    check(imu_tare_post(&e), "H02", "q");
    check(imu_tare_complete(), "H02", "complete");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_NONE, "H02",
          "no configure");
    r = result_of();
    check(r.state == IMU_CMD_STATE_CANCELLED, "H02", "cancelled");
    check(r.reason == IMU_CMD_REASON_OPERATOR_Q, "H02", "q");
    check(r.sub.tareNow == IMU_CMD_SUB_NOT_ATTEMPTED, "H02", "no tare-now");
    check(r.sub.persist == IMU_CMD_SUB_NOT_ATTEMPTED, "H02", "no persist");
    check(!r.restoredProduction, "H02", "no restore");
    check(r.epochBefore == 0u, "H02", "epoch unchanged");
}

static void test_h03_q_during_settle(void)
{
    ImuTareEvent_t e;
    ImuCmdResult_t r;

    configure_ok(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z, "H03");
    e = make_event(IMU_TARE_EVENT_OPERATOR_Q);
    check(imu_tare_post(&e), "H03", "q in settle");
    check(progress().tare.phase == IMU_CMD_TARE_PHASE_RESTORE, "H03",
          "restore after mutation");
    restore_ok("H03");
    r = result_of();
    check(r.state == IMU_CMD_STATE_CANCELLED, "H03", "cancelled");
    check(r.warningRequired, "H03", "warning");
    check(r.restoredProduction, "H03", "restored");
}

static void test_h04_config_failure(void)
{
    ImuTareEvent_t e;
    ImuCmdResult_t r;

    check(imu_tare_init(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z, 0x00u, T0),
          "H04", "init");
    confirm();
    e = make_session(IMU_TARE_REQ_CONFIGURE, false, 1u, 1u);
    check(imu_tare_post(&e), "H04", "configure fail");
    restore_ok("H04");
    r = result_of();
    check(r.state == IMU_CMD_STATE_FAILED, "H04", "failed");
    check(r.reason == IMU_CMD_REASON_CONFIG_FAILED, "H04", "config failed");
}

static void test_h05_verify_current_epoch_sample(void)
{
    ImuCmdProgress_t p;
    ImuTareEvent_t e;
    ImuTareFacts_t facts;

    drive_to_persist_pending(IMU_CMD_TARE_AXES_Z, "H05");
    e = make_session(IMU_TARE_REQ_PERSIST, true, 3u, 3u);
    check(imu_tare_post(&e), "H05", "persist");
    facts = good_facts(3u);
    check(imu_tare_feed_facts(&facts), "H05", "current epoch");
    p = progress();
    check(p.tare.attitudeEpochMatched, "H05", "epoch matched");
    check(p.tare.verificationEvidence, "H05", "evidence");
}

static void test_h06_verify_rejects_old_epoch(void)
{
    ImuCmdResult_t r;
    ImuTareEvent_t e;
    ImuTareFacts_t facts;

    drive_to_persist_pending(IMU_CMD_TARE_AXES_Z, "H06");
    e = make_session(IMU_TARE_REQ_PERSIST, true, 3u, 3u);
    check(imu_tare_post(&e), "H06", "persist");
    facts = good_facts(2u);
    check(imu_tare_feed_facts(&facts), "H06", "old epoch");
    check(!progress().tare.attitudeEpochMatched, "H06", "not matched");
    check(!progress().tare.verificationEvidence, "H06", "no evidence");
    tick_and_service(T0 + IMU_TARE_SETTLE_NS + IMU_TARE_VERIFY_NS);
    restore_ok("H06");
    r = result_of();
    check(r.state == IMU_CMD_STATE_FAILED, "H06", "failed");
    check(r.reason == IMU_CMD_REASON_TARE_VERIFY_FAILED, "H06", "verify");
}

static void test_h07_verify_timeout_no_evidence(void)
{
    ImuCmdResult_t r;
    ImuTareEvent_t e;

    drive_to_persist_pending(IMU_CMD_TARE_AXES_Z, "H07");
    e = make_session(IMU_TARE_REQ_PERSIST, true, 3u, 3u);
    check(imu_tare_post(&e), "H07", "persist");
    tick_and_service(T0 + IMU_TARE_SETTLE_NS + IMU_TARE_VERIFY_NS);
    restore_ok("H07");
    r = result_of();
    check(r.reason == IMU_CMD_REASON_TARE_VERIFY_FAILED, "H07", "no evidence");
    check(!r.verified, "H07", "not verified");
}

static void test_h08_restore_failure(void)
{
    ImuTareEvent_t e;
    ImuCmdResult_t r;

    check(imu_tare_init(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z, 0x00u, T0),
          "H08", "init");
    confirm();
    e = make_event(IMU_TARE_EVENT_OPERATOR_Q);
    check(imu_tare_post(&e), "H08", "q before config result");
    check(imu_tare_complete(), "H08", "pre-mutation q completes");

    configure_ok(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z, "H08b");
    e = make_event(IMU_TARE_EVENT_OPERATOR_Q);
    check(imu_tare_post(&e), "H08b", "q after config");
    e = make_session(IMU_TARE_REQ_RESTORE_PRODUCTION, false, 2u, 2u);
    check(imu_tare_post(&e), "H08b", "restore fail");
    r = result_of();
    check(r.state == IMU_CMD_STATE_RECOVERY_FAILED, "H08b", "recovery failed");
    check(r.reason == IMU_CMD_REASON_SESSION_UNUSABLE, "H08b",
          "session unusable");
    check(!r.restoredProduction, "H08b", "not restored");
}

static void test_h09_tare_check_mirrors_until_q(void)
{
    ImuTareEvent_t e;
    ImuCmdProgress_t p;
    ImuCmdResult_t r;
    ImuTareFacts_t facts;

    configure_ok(IMU_CMD_ID_TARE_CHECK, IMU_CMD_TARE_AXES_NONE, "H09");
    check(progress().tare.phase == IMU_CMD_TARE_PHASE_CHECK, "H09", "check");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_NONE, "H09",
          "no mutation");
    facts = good_facts(2u);
    check(imu_tare_feed_facts(&facts), "H09", "facts");
    p = progress();
    check(p.tare.attitudeValid, "H09", "mirrored");
    check(p.tare.yawRad == 0.10f, "H09", "yaw");
    check(p.requiredAction == IMU_CMD_ACTION_PRESS_Q_TO_END, "H09", "q action");
    e = make_event(IMU_TARE_EVENT_OPERATOR_Q);
    check(imu_tare_post(&e), "H09", "q");
    restore_ok("H09");
    r = result_of();
    check(r.state == IMU_CMD_STATE_CANCELLED, "H09", "cancelled");
    check(!r.warningRequired, "H09", "check q has no warning");
    check(r.sub.tareNow == IMU_CMD_SUB_NOT_ATTEMPTED, "H09", "no tare-now");
}

static void test_h10_mismatched_session_rejected(void)
{
    ImuTareEvent_t e;

    check(imu_tare_init(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z, 0x00u, T0),
          "H10", "init");
    confirm();
    e = make_session(IMU_TARE_REQ_PERSIST, true, 1u, 1u);
    check(!imu_tare_post(&e), "H10", "mismatch");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_CONFIGURE, "H10",
          "configure still pending");
}

static void test_h11_duplicate_confirm_and_deadlines(void)
{
    ImuTareEvent_t e;
    ImuTareFacts_t facts;

    check(imu_tare_init(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z, 0x00u, T0),
          "H11", "init");
    confirm();
    confirm();
    check(imu_tare_pending_request().type == IMU_TARE_REQ_CONFIGURE, "H11",
          "still one configure");
    e = make_session(IMU_TARE_REQ_CONFIGURE, true, 1u, 2u);
    check(imu_tare_post(&e), "H11", "configure");
    tick_and_service(T0 + IMU_TARE_SETTLE_NS - 1ull);
    check(imu_tare_pending_request().type == IMU_TARE_REQ_NONE, "H11",
          "no early tare-now");
    tick_and_service(T0 + IMU_TARE_SETTLE_NS);
    check(imu_tare_pending_request().type == IMU_TARE_REQ_TARE_NOW, "H11",
          "exact 2 s boundary");
    e = make_session(IMU_TARE_REQ_TARE_NOW, true, 2u, 3u);
    check(imu_tare_post(&e), "H11", "tare-now");
    e = make_session(IMU_TARE_REQ_PERSIST, true, 3u, 3u);
    check(imu_tare_post(&e), "H11", "persist");
    tick_and_service(T0 + IMU_TARE_SETTLE_NS + IMU_TARE_VERIFY_NS - 1ull);
    check(imu_tare_pending_request().type == IMU_TARE_REQ_NONE, "H11",
          "no early restore");
    facts = good_facts(3u);
    check(imu_tare_feed_facts(&facts), "H11", "evidence");
    tick_and_service(T0 + IMU_TARE_SETTLE_NS + IMU_TARE_VERIFY_NS);
    check(imu_tare_pending_request().type == IMU_TARE_REQ_RESTORE_PRODUCTION,
          "H11", "exact 500 ms boundary");
}

static void test_h12_no_request_after_complete(void)
{
    ImuTareEvent_t e;

    check(imu_tare_init(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z, 0x00u, T0),
          "H12", "init");
    e = make_event(IMU_TARE_EVENT_OPERATOR_Q);
    check(imu_tare_post(&e), "H12", "q");
    check(imu_tare_complete(), "H12", "complete");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_NONE, "H12",
          "none after terminal");
    e = make_session(IMU_TARE_REQ_CONFIGURE, true, 1u, 2u);
    check(!imu_tare_post(&e), "H12", "session after complete rejected");
}

static void test_h13_illegal_init(void)
{
    check(!imu_tare_init(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_NONE, 0x00u, T0),
          "H13", "tare needs axes");
    check(!imu_tare_init(IMU_CMD_ID_TARE_CLEAR, IMU_CMD_TARE_AXES_Z, 0x00u, T0),
          "H13", "clear forbids axes");
    check(!imu_tare_init(IMU_CMD_ID_TARE_CHECK, IMU_CMD_TARE_AXES_FULL, 0x00u,
                         T0),
          "H13", "check forbids axes");
    check(!imu_tare_init(IMU_CMD_ID_CALIBRATION, IMU_CMD_TARE_AXES_Z, 0x00u,
                         T0),
          "H13", "not a tare identity");
}

int main(void)
{
    test_t01_tare_now_and_persist_succeed();
    test_t02_tare_now_failed_skips_persist();
    test_t03_persist_failed_continues();
    test_t04_clear_active_and_saved_succeed();
    test_t05_partial_clear_is_failed();
    test_h01_z_and_full_axes();
    test_h02_q_before_confirm();
    test_h03_q_during_settle();
    test_h04_config_failure();
    test_h05_verify_current_epoch_sample();
    test_h06_verify_rejects_old_epoch();
    test_h07_verify_timeout_no_evidence();
    test_h08_restore_failure();
    test_h09_tare_check_mirrors_until_q();
    test_h10_mismatched_session_rejected();
    test_h11_duplicate_confirm_and_deadlines();
    test_h12_no_request_after_complete();
    test_h13_illegal_init();

    if (g_fail != 0) {
        fprintf(stderr, "test_imu_tare: %d failure(s)\n", g_fail);
        return 1;
    }
    printf("test_imu_tare: pass\n");
    return 0;
}
