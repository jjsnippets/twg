#define _POSIX_C_SOURCE 200809L
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_cal.h"

static int g_fail;
static const uint64_t T0 = 1000000000ull;

static void check(bool ok, const char *id, const char *what)
{
    if (!ok) {
        fprintf(stderr, "FAIL %s: %s\n", id, what);
        g_fail++;
    }
}

static ImuCalEvent_t make_event(ImuCalEventType_t type)
{
    ImuCalEvent_t e;

    memset(&e, 0, sizeof(e));
    e.type = type;
    return e;
}

static ImuCalEvent_t make_session(ImuCalRequestType_t type, bool success,
                                 uint32_t epochBefore, uint32_t epochAfter)
{
    ImuCalEvent_t e = make_event(IMU_CAL_EVENT_SESSION_RESULT);

    e.session.type = type;
    e.session.success = success;
    e.session.epochBefore = epochBefore;
    e.session.epochAfter = epochAfter;
    return e;
}

static void tick_and_service(uint64_t nowNs)
{
    ImuCalEvent_t e = make_event(IMU_CAL_EVENT_TICK);
    e.monotonicNs = nowNs;
    check(imu_cal_post(&e), "HELPER", "tick");
    imu_cal_service();
}

static void confirm_and_service(void)
{
    ImuCalEvent_t e = make_event(IMU_CAL_EVENT_OPERATOR_CONFIRM);
    check(imu_cal_post(&e), "HELPER", "confirm");
    imu_cal_service();
}

static void run_accel_faces(uint64_t *nowNs)
{
    unsigned i;

    for (i = 0u; i < IMU_CAL_ACCEL_POSE_COUNT; ++i) {
        confirm_and_service();
        *nowNs += IMU_CAL_ACCEL_FACE_WINDOW_NS;
        tick_and_service(*nowNs);
    }
}

static void feed_accurate(void)
{
    ImuCalFacts_t facts;

    memset(&facts, 0, sizeof(facts));
    facts.version = IMU_CAL_FACTS_VERSION;
    facts.valid = true;
    facts.accelAccuracy = 3u;
    facts.gyroAccuracy = 3u;
    facts.magAccuracy = 3u;
    facts.rvAccuracy = 3u;
    facts.rvErrRad = 0.10f;
    facts.haveMag = true;
    check(imu_cal_feed_facts(&facts), "HELPER", "feed accurate");
}


static void sustain_good(uint64_t *nowNs)
{
    feed_accurate();
    tick_and_service(*nowNs + 1ull);
    *nowNs += IMU_CAL_SUSTAINED_GOOD_NS + 2ull;
    tick_and_service(*nowNs);
}

static void feed_bad_verify(void)
{
    ImuCalFacts_t bad;

    memset(&bad, 0, sizeof(bad));
    bad.version = IMU_CAL_FACTS_VERSION;
    bad.valid = true;
    bad.accelAccuracy = 1u;
    bad.magAccuracy = 1u;
    check(imu_cal_feed_facts(&bad), "HELPER", "feed bad verify");
}

static void configure_ok(uint64_t nowNs)
{
    ImuCalEvent_t e;

    check(imu_cal_init(0u, nowNs), "HELPER", "init");
    e = make_session(IMU_CAL_REQ_CONFIGURE_CALIBRATION, true, 1u, 2u);
    check(imu_cal_post(&e), "HELPER", "configure ok");
}

static void drive_to_save(uint64_t *nowNs)
{
    configure_ok(*nowNs);
    run_accel_faces(nowNs);
    sustain_good(nowNs);
    confirm_and_service();
    sustain_good(nowNs);
    confirm_and_service();
    *nowNs += IMU_CAL_MAG_MOTION_NS;
    tick_and_service(*nowNs);
    sustain_good(nowNs);
    *nowNs += IMU_CAL_HOLD_WINDOW_NS;
    tick_and_service(*nowNs);
    sustain_good(nowNs);
    check(imu_cal_pending_request().type == IMU_CAL_REQ_SAVE_DCD,
          "HELPER", "at save");
}

static ImuCmdProgress_t progress(void)
{
    ImuCmdProgress_t p;
    memset(&p, 0, sizeof(p));
    check(imu_cal_get_progress(&p), "HELPER", "get progress");
    return p;
}

static void test_c01_init_emits_configure(void)
{
    ImuCalPendingRequest_t req;
    ImuCmdProgress_t p;
    ImuCmdResult_t r;

    check(imu_cal_init(0x00u, T0), "C01", "init");
    req = imu_cal_pending_request();
    check(req.type == IMU_CAL_REQ_CONFIGURE_CALIBRATION, "C01", "configure req");
    check(req.calMask == IMU_CAL_ENABLE_MASK, "C01", "mask 0x07");
    check(imu_cal_get_progress(&p), "C01", "progress");
    check(p.version == IMU_CMD_PROGRESS_VERSION, "C01", "progress v");
    check(p.active == IMU_CMD_ID_CALIBRATION, "C01", "active cal");
    check(p.cal.phase == IMU_CMD_CAL_PHASE_STARTUP, "C01", "startup");
    check(imu_cal_get_result(&r), "C01", "result");
    check(r.version == IMU_CMD_RESULT_VERSION, "C01", "result v");
    check(r.state == IMU_CMD_STATE_RUNNING, "C01", "running");
    check(!imu_cal_complete(), "C01", "not complete");
}

static void test_c02_config_success_stays_running(void)
{
    ImuCalEvent_t e;
    ImuCmdProgress_t p;

    check(imu_cal_init(0x00u, T0), "C02", "init");
    e = make_session(IMU_CAL_REQ_CONFIGURE_CALIBRATION, true, 1u, 2u);
    check(imu_cal_post(&e), "C02", "config ok");
    check(imu_cal_pending_request().type == IMU_CAL_REQ_NONE, "C02", "no pending");
    check(imu_cal_get_progress(&p), "C02", "progress");
    check(p.cal.phase == IMU_CMD_CAL_PHASE_ACCEL, "C02", "accel after config");
    check(p.cal.currentPoseIndex == 1u, "C02", "first face");
    check(!imu_cal_complete(), "C02", "not complete");
}

static void test_c03_q_after_config_restores(void)
{
    ImuCalEvent_t e;
    ImuCalPendingRequest_t req;
    ImuCmdResult_t r;

    check(imu_cal_init(0x00u, T0), "C03", "init");
    e = make_session(IMU_CAL_REQ_CONFIGURE_CALIBRATION, true, 1u, 2u);
    check(imu_cal_post(&e), "C03", "config ok");
    e = make_event(IMU_CAL_EVENT_OPERATOR_Q);
    check(imu_cal_post(&e), "C03", "q");
    req = imu_cal_pending_request();
    check(req.type == IMU_CAL_REQ_RESTORE_PRODUCTION, "C03", "restore req");
    check(req.calMask == 0x00u, "C03", "flight mask");
    e = make_session(IMU_CAL_REQ_RESTORE_PRODUCTION, true, 2u, 3u);
    check(imu_cal_post(&e), "C03", "restore ok");
    check(imu_cal_complete(), "C03", "complete");
    check(imu_cal_get_result(&r), "C03", "result");
    check(r.state == IMU_CMD_STATE_CANCELLED, "C03", "cancelled");
    check(r.reason == IMU_CMD_REASON_OPERATOR_Q, "C03", "operator q");
    check(r.warningRequired, "C03", "warning");
    check(r.restoredProduction, "C03", "restored");
    check(r.epochBefore == 1u, "C03", "epochBefore");
    check(r.epochAfter == 3u, "C03", "epochAfter");
}

static void test_c04_q_before_config_drops_pending(void)
{
    ImuCalEvent_t e;
    ImuCalPendingRequest_t req;

    check(imu_cal_init(0x00u, T0), "C04", "init");
    e = make_event(IMU_CAL_EVENT_OPERATOR_Q);
    check(imu_cal_post(&e), "C04", "q");
    req = imu_cal_pending_request();
    check(req.type == IMU_CAL_REQ_RESTORE_PRODUCTION, "C04", "restore replaces configure");
}

static void test_c05_config_fail_then_restore(void)
{
    ImuCalEvent_t e;
    ImuCmdResult_t r;

    check(imu_cal_init(0x00u, T0), "C05", "init");
    e = make_session(IMU_CAL_REQ_CONFIGURE_CALIBRATION, false, 1u, 1u);
    check(imu_cal_post(&e), "C05", "config fail");
    check(imu_cal_pending_request().type == IMU_CAL_REQ_RESTORE_PRODUCTION,
          "C05", "restore");
    e = make_session(IMU_CAL_REQ_RESTORE_PRODUCTION, true, 1u, 2u);
    check(imu_cal_post(&e), "C05", "restore ok");
    check(imu_cal_get_result(&r), "C05", "result");
    check(r.state == IMU_CMD_STATE_FAILED, "C05", "failed");
    check(r.reason == IMU_CMD_REASON_CAL_CONFIG_FAILED, "C05", "config failed");
    check(r.restoredProduction, "C05", "restored");
    check(r.state != IMU_CMD_STATE_RECOVERY_FAILED, "C05", "not recoveryfailed");
}

static void test_c06_restore_fail_is_recovery(void)
{
    ImuCalEvent_t e;
    ImuCmdResult_t r;

    check(imu_cal_init(0x00u, T0), "C06", "init");
    e = make_session(IMU_CAL_REQ_CONFIGURE_CALIBRATION, true, 1u, 2u);
    check(imu_cal_post(&e), "C06", "config ok");
    e = make_event(IMU_CAL_EVENT_OPERATOR_Q);
    check(imu_cal_post(&e), "C06", "q");
    e = make_session(IMU_CAL_REQ_RESTORE_PRODUCTION, false, 2u, 2u);
    check(imu_cal_post(&e), "C06", "restore fail");
    check(imu_cal_get_result(&r), "C06", "result");
    check(r.state == IMU_CMD_STATE_RECOVERY_FAILED, "C06", "recoveryfailed");
    check(r.reason == IMU_CMD_REASON_CAL_RESTORE_FAILED, "C06", "restore failed");
    check(!r.restoredProduction, "C06", "not restored");
}

static void test_c07_facts_in_progress(void)
{
    ImuCalFacts_t facts;
    ImuCmdProgress_t p;

    check(imu_cal_init(0x00u, T0), "C07", "init");
    memset(&facts, 0, sizeof(facts));
    facts.version = IMU_CAL_FACTS_VERSION;
    facts.accelAccuracy = 3u;
    facts.gyroAccuracy = 2u;
    facts.magAccuracy = 1u;
    facts.rvAccuracy = 2u;
    facts.rvErrRad = 0.25f;
    facts.magXuT = 11.0f;
    check(imu_cal_feed_facts(&facts), "C07", "feed");
    check(imu_cal_get_progress(&p), "C07", "progress");
    check(p.cal.accelAccuracy == 3u, "C07", "accel");
    check(p.cal.magAccuracy == 1u, "C07", "mag");
    check(p.cal.magXuT == 11.0f, "C07", "mag x");
}

static void test_c08_mismatched_session_rejected(void)
{
    ImuCalEvent_t e;

    check(imu_cal_init(0x00u, T0), "C08", "init");
    e = make_session(IMU_CAL_REQ_SAVE_DCD, true, 1u, 1u);
    check(!imu_cal_post(&e), "C08", "mismatch");
    check(imu_cal_pending_request().type == IMU_CAL_REQ_CONFIGURE_CALIBRATION,
          "C08", "configure still pending");
}

static void test_c09_tick_does_not_complete(void)
{
    ImuCalEvent_t e;

    check(imu_cal_init(0x00u, T0), "C09", "init");
    e = make_event(IMU_CAL_EVENT_TICK);
    e.monotonicNs = T0 + 5000000000ull;
    check(imu_cal_post(&e), "C09", "tick");
    imu_cal_service();
    check(!imu_cal_complete(), "C09", "still running");
}

static void test_c10_accel_six_faces(void)
{
    uint64_t now = T0;
    unsigned i;
    ImuCmdProgress_t p;

    configure_ok(now);
    p = progress();
    check(p.cal.phase == IMU_CMD_CAL_PHASE_ACCEL, "C10", "accel phase");
    check(p.cal.currentPoseIndex == 1u, "C10", "first face");

    for (i = 0u; i < IMU_CAL_ACCEL_POSE_COUNT; ++i) {
        confirm_and_service();
        now += IMU_CAL_ACCEL_FACE_WINDOW_NS;
        tick_and_service(now);
    }
    p = progress();
    check(p.cal.phase == IMU_CMD_CAL_PHASE_ACCEL, "C10", "accel gate phase");
    check(p.cal.currentPoseIndex == IMU_CAL_ACCEL_POSE_COUNT,
          "C10", "sixth face retained");
}

static void test_c11_accel_gate_to_gyro(void)
{
    uint64_t now = T0;
    unsigned i;
    ImuCmdProgress_t p;

    configure_ok(now);
    for (i = 0u; i < IMU_CAL_ACCEL_POSE_COUNT; ++i) {
        confirm_and_service();
        now += IMU_CAL_ACCEL_FACE_WINDOW_NS;
        tick_and_service(now);
    }
    feed_accurate();
    tick_and_service(now + 1u);
    tick_and_service(now + IMU_CAL_SUSTAINED_GOOD_NS + 1u);
    p = progress();
    check(p.cal.phase == IMU_CMD_CAL_PHASE_GYRO, "C11", "gyro prompt");
}

static void test_c12_accel_exhaustion_continues(void)
{
    uint64_t now = T0;
    unsigned round;
    unsigned pose;
    ImuCmdProgress_t p;

    configure_ok(now);
    for (round = 0u; round < IMU_CAL_ACCEL_MAX_ROUNDS; ++round) {
        for (pose = 0u; pose < IMU_CAL_ACCEL_POSE_COUNT; ++pose) {
            confirm_and_service();
            now += IMU_CAL_ACCEL_FACE_WINDOW_NS;
            tick_and_service(now);
        }
        now += IMU_CAL_ACCEL_GATE_TIMEOUT_NS;
        tick_and_service(now);
    }
    p = progress();
    check(p.cal.phase == IMU_CMD_CAL_PHASE_GYRO, "C12",
          "accel exhaustion is informational");
}

static void test_c13_gyro_timeout_continues_to_mag(void)
{
    uint64_t now = T0;
    ImuCmdProgress_t p;
    
    configure_ok(now);
    run_accel_faces(&now);
    sustain_good(&now);
    confirm_and_service();
    now += IMU_CAL_GYRO_GATE_TIMEOUT_NS;
    tick_and_service(now);
    p = progress();
    check(p.cal.phase == IMU_CMD_CAL_PHASE_MAG, "C13", "mag prompt");
    check(p.cal.magRound == 1u, "C13", "first mag round");
}

static void test_c14_mag_exhaustion_fails_and_restores(void)
{
    uint64_t now = T0;
    unsigned i;
    ImuCalEvent_t e;
    ImuCmdResult_t r;
    ImuCalFacts_t bad;

    configure_ok(now);
    run_accel_faces(&now);
    sustain_good(&now);
    confirm_and_service();
    sustain_good(&now);
    memset(&bad, 0, sizeof(bad));
    bad.version = IMU_CAL_FACTS_VERSION;
    bad.valid = true;
    check(imu_cal_feed_facts(&bad), "C14", "feed bad mag");
    for (i = 0u; i < IMU_CAL_MAG_MAX_ROUNDS; ++i) {
        confirm_and_service();
        now += IMU_CAL_MAG_MOTION_NS;
        tick_and_service(now);
        now += IMU_CAL_MAG_GATE_TIMEOUT_NS;
        tick_and_service(now);
    }
    check(imu_cal_pending_request().type == IMU_CAL_REQ_RESTORE_PRODUCTION,
          "C14", "restore after mag exhaustion");
    e = make_session(IMU_CAL_REQ_RESTORE_PRODUCTION, true, 2u, 3u);
    check(imu_cal_post(&e), "C14", "restore ok");
    check(imu_cal_get_result(&r), "C14", "result");
    check(r.state == IMU_CMD_STATE_FAILED, "C14", "failed");
    check(r.reason == IMU_CMD_REASON_CAL_MAG_EXHAUSTED, "C14", "reason");
}

static void test_c15_save_reopen_verify_success(void)
{
    uint64_t now = T0;
    ImuCalEvent_t e;
    ImuCmdResult_t r;

    configure_ok(now);
    run_accel_faces(&now);
    sustain_good(&now);
    confirm_and_service();
    sustain_good(&now);
    confirm_and_service();
    now += IMU_CAL_MAG_MOTION_NS;
    tick_and_service(now);
    sustain_good(&now);
    now += IMU_CAL_HOLD_WINDOW_NS;
    tick_and_service(now);
    sustain_good(&now);
    check(imu_cal_pending_request().type == IMU_CAL_REQ_SAVE_DCD,
          "C15", "save request");

    e = make_session(IMU_CAL_REQ_SAVE_DCD, true, 2u, 2u);
    check(imu_cal_post(&e), "C15", "save ok");
    check(imu_cal_pending_request().type == IMU_CAL_REQ_VERIFY_REOPEN,
          "C15", "reopen request");
    e = make_session(IMU_CAL_REQ_VERIFY_REOPEN, true, 2u, 3u);
    check(imu_cal_post(&e), "C15", "reopen ok");
    check(imu_cal_pending_request().type == IMU_CAL_REQ_CONFIGURE_CALIBRATION,
          "C15", "verify config request");
    check(imu_cal_pending_request().calMask == 0u, "C15", "verify mask zero");
    e = make_session(IMU_CAL_REQ_CONFIGURE_CALIBRATION, true, 3u, 4u);
    check(imu_cal_post(&e), "C15", "verify config ok");
    confirm_and_service();
    now += IMU_CAL_VERIFY_MOTION_NS;
    tick_and_service(now);
    sustain_good(&now);
    check(imu_cal_pending_request().type == IMU_CAL_REQ_RESTORE_PRODUCTION,
          "C15", "restore request");
    e = make_session(IMU_CAL_REQ_RESTORE_PRODUCTION, true, 4u, 5u);
    check(imu_cal_post(&e), "C15", "restore ok");
    check(imu_cal_get_result(&r), "C15", "result");
    check(r.state == IMU_CMD_STATE_SUCCEEDED, "C15", "succeeded");
    check(r.dcdSaved, "C15", "saved");
    check(r.verified, "C15", "verified");
    check(r.restoredProduction, "C15", "restored");
}

static void test_c16_q_during_motion_restores(void)
{
    ImuCalEvent_t e;
    ImuCmdResult_t r;

    configure_ok(T0);
    confirm_and_service();
    e = make_event(IMU_CAL_EVENT_OPERATOR_Q);
    check(imu_cal_post(&e), "C16", "q while accel window");
    check(imu_cal_pending_request().type == IMU_CAL_REQ_RESTORE_PRODUCTION,
          "C16", "restore request");
    e = make_session(IMU_CAL_REQ_RESTORE_PRODUCTION, true, 2u, 3u);
    check(imu_cal_post(&e), "C16", "restore ok");
    check(imu_cal_get_result(&r), "C16", "result");
    check(r.state == IMU_CMD_STATE_CANCELLED, "C16", "cancelled");
    check(r.reason == IMU_CMD_REASON_OPERATOR_Q, "C16", "q reason");
}

static void test_c17_save_retry_then_success(void)
{
    uint64_t now = T0;
    ImuCalEvent_t e;
    ImuCmdResult_t r;

    drive_to_save(&now);
    e = make_session(IMU_CAL_REQ_SAVE_DCD, false, 2u, 2u);
    check(imu_cal_post(&e), "C17", "save fail 1");
    check(progress().cal.phase == IMU_CMD_CAL_PHASE_HOLD, "C17", "retry hold");
    now += IMU_CAL_SAVE_RETRY_HOLD_NS;
    tick_and_service(now);
    check(imu_cal_pending_request().type == IMU_CAL_REQ_SAVE_DCD,
          "C17", "save retry");
    e = make_session(IMU_CAL_REQ_SAVE_DCD, true, 2u, 2u);
    check(imu_cal_post(&e), "C17", "save ok");
    check(imu_cal_pending_request().type == IMU_CAL_REQ_VERIFY_REOPEN,
          "C17", "reopen after retry");
    e = make_session(IMU_CAL_REQ_VERIFY_REOPEN, true, 2u, 3u);
    check(imu_cal_post(&e), "C17", "reopen");
    e = make_session(IMU_CAL_REQ_CONFIGURE_CALIBRATION, true, 3u, 4u);
    check(imu_cal_post(&e), "C17", "verify config");
    confirm_and_service();
    now += IMU_CAL_VERIFY_MOTION_NS;
    tick_and_service(now);
    sustain_good(&now);
    e = make_session(IMU_CAL_REQ_RESTORE_PRODUCTION, true, 4u, 5u);
    check(imu_cal_post(&e), "C17", "restore");
    check(imu_cal_get_result(&r), "C17", "result");
    check(r.state == IMU_CMD_STATE_SUCCEEDED, "C17", "succeeded");
    check(r.dcdSaved, "C17", "saved");
}

static void test_c18_double_save_fail_starts_next_attempt(void)
{
    uint64_t now = T0;
    ImuCalEvent_t e;
    ImuCmdProgress_t p;

    drive_to_save(&now);
    e = make_session(IMU_CAL_REQ_SAVE_DCD, false, 2u, 2u);
    check(imu_cal_post(&e), "C18", "save fail 1");
    now += IMU_CAL_SAVE_RETRY_HOLD_NS;
    tick_and_service(now);
    e = make_session(IMU_CAL_REQ_SAVE_DCD, false, 2u, 2u);
    check(imu_cal_post(&e), "C18", "save fail 2");
    p = progress();
    check(p.cal.phase == IMU_CMD_CAL_PHASE_MAG, "C18", "next mag attempt");
    check(p.cal.saveAttempt == 2u, "C18", "outer attempt 2");
    check(p.cal.magRound == 1u, "C18", "inner mag reset");
}

static void test_c19_hold_degrade_then_exhaust(void)
{
    uint64_t now = T0;
    unsigned attempt;
    ImuCalEvent_t e;
    ImuCmdResult_t r;

    configure_ok(now);
    run_accel_faces(&now);
    sustain_good(&now);
    confirm_and_service();
    sustain_good(&now);

    for (attempt = 0u; attempt < IMU_CAL_MAX_SAVE_ATTEMPTS; ++attempt) {
        confirm_and_service();
        now += IMU_CAL_MAG_MOTION_NS;
        tick_and_service(now);
        sustain_good(&now);
        now += IMU_CAL_HOLD_WINDOW_NS;
        tick_and_service(now);
        feed_bad_verify();
        tick_and_service(now + 1ull);
        now += IMU_CAL_HOLD_GATE_TIMEOUT_NS;
        tick_and_service(now);
    }
    check(imu_cal_pending_request().type == IMU_CAL_REQ_RESTORE_PRODUCTION,
          "C19", "restore after hold degrade");
    e = make_session(IMU_CAL_REQ_RESTORE_PRODUCTION, true, 2u, 3u);
    check(imu_cal_post(&e), "C19", "restore");
    check(imu_cal_get_result(&r), "C19", "result");
    check(r.state == IMU_CMD_STATE_FAILED, "C19", "failed");
    check(r.reason == IMU_CMD_REASON_CAL_HOLD_DEGRADED, "C19", "hold degraded");
    check(!r.dcdSaved, "C19", "not saved");
}

static void test_c20_reopen_fail_is_recovery(void)
{
    uint64_t now = T0;
    ImuCalEvent_t e;
    ImuCmdResult_t r;

    drive_to_save(&now);
    e = make_session(IMU_CAL_REQ_SAVE_DCD, true, 2u, 2u);
    check(imu_cal_post(&e), "C20", "save ok");
    e = make_session(IMU_CAL_REQ_VERIFY_REOPEN, false, 2u, 2u);
    check(imu_cal_post(&e), "C20", "reopen fail");
    check(imu_cal_pending_request().type == IMU_CAL_REQ_NONE,
          "C20", "no restore after unusable session");
    check(imu_cal_get_result(&r), "C20", "result");
    check(r.state == IMU_CMD_STATE_RECOVERY_FAILED, "C20", "recoveryfailed");
    check(r.reason == IMU_CMD_REASON_CAL_REOPEN_FAILED, "C20", "reopen failed");
    check(r.dcdSaved, "C20", "save already happened");
    check(!r.verified, "C20", "not verified");
    check(!r.restoredProduction, "C20", "not restored");
}

static void test_c21_verify_gate_fail_restores(void)
{
    uint64_t now = T0;
    ImuCalEvent_t e;
    ImuCmdResult_t r;

    drive_to_save(&now);
    e = make_session(IMU_CAL_REQ_SAVE_DCD, true, 2u, 2u);
    check(imu_cal_post(&e), "C21", "save");
    e = make_session(IMU_CAL_REQ_VERIFY_REOPEN, true, 2u, 3u);
    check(imu_cal_post(&e), "C21", "reopen");
    e = make_session(IMU_CAL_REQ_CONFIGURE_CALIBRATION, true, 3u, 4u);
    check(imu_cal_post(&e), "C21", "verify config");
    confirm_and_service();
    now += IMU_CAL_VERIFY_MOTION_NS;
    tick_and_service(now);
    feed_bad_verify();
    tick_and_service(now + 1ull);
    now += IMU_CAL_VERIFY_GATE_TIMEOUT_NS;
    tick_and_service(now);
    check(imu_cal_pending_request().type == IMU_CAL_REQ_RESTORE_PRODUCTION,
          "C21", "restore");
    e = make_session(IMU_CAL_REQ_RESTORE_PRODUCTION, true, 4u, 5u);
    check(imu_cal_post(&e), "C21", "restore ok");
    check(imu_cal_get_result(&r), "C21", "result");
    check(r.state == IMU_CMD_STATE_FAILED, "C21", "failed");
    check(r.reason == IMU_CMD_REASON_CAL_VERIFY_GATE_FAILED, "C21", "verify gate");
    check(r.dcdSaved, "C21", "saved");
    check(!r.verified, "C21", "not verified");
    check(r.restoredProduction, "C21", "restored");
}

int main(void)
{
    test_c01_init_emits_configure();
    test_c02_config_success_stays_running();
    test_c03_q_after_config_restores();
    test_c04_q_before_config_drops_pending();
    test_c05_config_fail_then_restore();
    test_c06_restore_fail_is_recovery();
    test_c07_facts_in_progress();
    test_c08_mismatched_session_rejected();
    test_c09_tick_does_not_complete();
    test_c10_accel_six_faces();
    test_c11_accel_gate_to_gyro();
    test_c12_accel_exhaustion_continues();
    test_c13_gyro_timeout_continues_to_mag();
    test_c14_mag_exhaustion_fails_and_restores();
    test_c15_save_reopen_verify_success();
    test_c16_q_during_motion_restores();
    test_c17_save_retry_then_success();
    test_c18_double_save_fail_starts_next_attempt();
    test_c19_hold_degrade_then_exhaust();
    test_c20_reopen_fail_is_recovery();
    test_c21_verify_gate_fail_restores();
    if (g_fail != 0) {
        fprintf(stderr, "test_imu_cal: %d failures\n", g_fail);
        return 1;
    }
    printf("test_imu_cal: pass\n");
    return 0;
}