#define _POSIX_C_SOURCE 200809L

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_cal.h"
#include "app/imu_cal_adapter.h"
#include "app/imu_session.h"

static int gfail;

static const uint64_t T0 = 1000000000ull;

static void check(bool ok, const char *id, const char *what)
{
    if (!ok) {
        fprintf(stderr, "FAIL %s: %s\n", id, what);
        gfail++;
    }
}

static ImuSampleSnapshot_t snapshot(void)
{
    ImuSampleSnapshot_t out;

    memset(&out, 0, sizeof(out));
    (void)imu_session_get_snapshot(&out);
    return out;
}

static ImuCalFacts_t good_facts(void)
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
    facts.magXuT = 20.0f;
    facts.magYuT = -5.0f;
    facts.magZuT = 40.0f;
    return facts;
}

static void post_confirm(const char *id, const char *where)
{
    ImuCalEvent_t event;
    ImuCmdProgress_t progress;

    memset(&event, 0, sizeof(event));
    event.type = IMU_CAL_EVENT_OPERATOR_CONFIRM;
    if (!imu_cal_post(&event)) {
      memset(&progress, 0, sizeof(progress));
      (void)imu_cal_get_progress(&progress);
      fprintf(stderr,
              "FAIL %s: confirm rejected at %s "
              "(phase=%d action=%d pending=%d)\n",
              id,
              where,
              (int)progress.cal.phase,
              (int)progress.requiredAction,
              (int)imu_cal_pending_request().type);
      gfail++;
      return;
    }
    imu_cal_service();
}

static void post_q(const char *id)
{
    ImuCalEvent_t event;

    memset(&event, 0, sizeof(event));
    event.type = IMU_CAL_EVENT_OPERATOR_Q;
    check(imu_cal_post(&event), id, "post q");
    imu_cal_service();
}

static void tick_and_service(uint64_t nowNs, const char *id)
{
    ImuCalEvent_t event;

    memset(&event, 0, sizeof(event));
    event.type = IMU_CAL_EVENT_TICK;
    event.monotonicNs = nowNs;
    check(imu_cal_post(&event), id, "post tick");
    imu_cal_service();
}

static void feed_good_facts(const char *id)
{
    ImuCalFacts_t facts = good_facts();

    check(imu_cal_feed_facts(&facts), id, "feed good facts");
}

static void sustain_good(uint64_t nowNs, const char *id)
{
    feed_good_facts(id);
    tick_and_service(nowNs + 1ull, id);
    tick_and_service(nowNs + IMU_CAL_SUSTAINED_GOOD_NS + 2ull, id);
}

static void open_and_init(uint8_t flightCalMask, const char *id)
{
    imu_session_test_reset();
    check(imu_session_test_open(true), id, "test open");
    check(imu_cal_init(flightCalMask, T0), id, "imu cal init");
}

static void configure_through_adapter(const char *id)
{
    ImuCalPendingRequest_t request;
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;
    uint8_t policyMask = 0u;

    request = imu_cal_pending_request();
    check(request.type == IMU_CAL_REQ_CONFIGURE_CALIBRATION,
          id,
          "configure request pending");
    check(request.calMask == IMU_CAL_ENABLE_MASK,
          id,
          "configure mask is enable mask");

    before = snapshot();
    check(imu_cal_adapter_pump(), id, "adapter configure pump");
    after = snapshot();

    check(after.readerState == IMU_READER_STATE_CALIBRATION,
          id,
          "session is calibration");
    check(after.configurationEpoch == before.configurationEpoch + 1u,
          id,
          "configure takes one epoch");
    check(imu_session_get_cal_policy(&policyMask), id, "read calibration policy");
    check(policyMask == IMU_CAL_ENABLE_MASK,
          id,
          "calibration policy applied");

    request = imu_cal_pending_request();
    check(request.type == IMU_CAL_REQ_NONE,
          id,
          "configure result consumed");
}

static void drive_to_save_request(uint64_t nowNs, const char *id)
{
    unsigned pose;

    configure_through_adapter(id);

    for (pose = 0u; pose < IMU_CAL_ACCEL_POSE_COUNT; pose++) {
        post_confirm(id, "accel-face");
        nowNs += IMU_CAL_ACCEL_FACE_WINDOW_NS;
        tick_and_service(nowNs, id);
    }

    sustain_good(nowNs, id); // Accel gate.
    nowNs += IMU_CAL_SUSTAINED_GOOD_NS + 2ull;
    
    {
      ImuCmdProgress_t progress;
      check(imu_cal_get_progress(&progress), id, "get accel-gate progress");
      check(progress.cal.phase == IMU_CMD_CAL_PHASE_GYRO, id,
            "accel gate must reach gyro prompt");
    }

    post_confirm(id, "gyro"); // Gyro prompt -> gyro gate.
    sustain_good(nowNs, id);
    nowNs += IMU_CAL_SUSTAINED_GOOD_NS + 2ull;

    post_confirm(id, "mag"); // Mag prompt -> mag motion -> mag gate.
    nowNs += IMU_CAL_MAG_MOTION_NS;
    tick_and_service(nowNs, id);
    sustain_good(nowNs, id);
    nowNs += IMU_CAL_SUSTAINED_GOOD_NS + 2ull;

    nowNs += IMU_CAL_HOLD_WINDOW_NS; // Hold window -> hold gate.
    tick_and_service(nowNs, id);
    sustain_good(nowNs, id); // Hold gate -> SAVE_DCD.
    nowNs += IMU_CAL_SUSTAINED_GOOD_NS + 2ull;

    check(imu_cal_pending_request().type == IMU_CAL_REQ_SAVE_DCD,
          id,
          "save request pending");
}

static void inject_good_session_facts(void)
{
    ImuCalFacts_t facts = good_facts();
    imu_session_test_inject_cal_facts(&facts);
}

static void owner_turn(uint64_t now_ns, const char *id)
{
    imu_session_service();
    /* Facts must be in imu_cal before this tick's gate evaluation.
     * A pending request created by this service tick stays visible
     * until the caller pumps again. */
    check(imu_cal_adapter_pump(), id, "owner adapter pump");
    tick_and_service(now_ns, id);
}

static void sustain_good_via_adapter(uint64_t *now_ns, const char *id)
{
    inject_good_session_facts();
    *now_ns += 1ull;
    owner_turn(*now_ns, id);
    *now_ns += IMU_CAL_SUSTAINED_GOOD_NS + 2ull;
    owner_turn(*now_ns, id);
}

static void drive_e2e_to_save(uint64_t *now_ns, const char *id)
{
    unsigned pose;

    configure_through_adapter(id);
    for (pose = 0u; pose < IMU_CAL_ACCEL_POSE_COUNT; pose++) {
        post_confirm(id, "accel-face");
        *now_ns += IMU_CAL_ACCEL_FACE_WINDOW_NS;
        owner_turn(*now_ns, id);
    }

    /* Accel gate. */
    sustain_good_via_adapter(now_ns, id);
    /* Gyro prompt -> gyro gate. */
    post_confirm(id, "gyro");
    sustain_good_via_adapter(now_ns, id);
    /* Mag prompt -> mag motion -> mag gate. */
    post_confirm(id, "mag");
    *now_ns += IMU_CAL_MAG_MOTION_NS;
    owner_turn(*now_ns, id);
    sustain_good_via_adapter(now_ns, id);
    /* Hold window -> hold gate -> SAVE_DCD. */
    *now_ns += IMU_CAL_HOLD_WINDOW_NS;
    owner_turn(*now_ns, id);
    sustain_good_via_adapter(now_ns, id);
    check(imu_cal_pending_request().type == IMU_CAL_REQ_SAVE_DCD,
          id, "save request pending");
}

static void test_a01_configure_calibration_maps_request(void)
{
    ImuCalPendingRequest_t request;
    ImuCmdProgress_t progress;

    open_and_init(0x00u, "A01");
    configure_through_adapter("A01");

    check(imu_cal_get_progress(&progress), "A01", "get progress");
    check(progress.cal.phase == IMU_CMD_CAL_PHASE_ACCEL,
          "A01",
          "configure result advances to accel");
    check(progress.cal.currentPoseIndex == 1u,
          "A01",
          "first accel pose selected");

    request = imu_cal_pending_request();
    check(request.type == IMU_CAL_REQ_NONE,
          "A01",
          "no duplicate configure request");
    check(imu_cal_adapter_pump(), "A01", "none request is no-op");
}

static void test_a02_save_dcd_maps_request(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;
    ImuCalPendingRequest_t request;

    open_and_init(0x00u, "A02");
    drive_to_save_request(T0, "A02");

    before = snapshot();
    imu_session_test_set_save_dcd_result(true);
    check(imu_cal_adapter_pump(), "A02", "adapter save pump");
    after = snapshot();

    check(after.configurationEpoch == before.configurationEpoch,
          "A02",
          "save does not increment epoch");
    check(after.readerState == IMU_READER_STATE_CALIBRATION,
          "A02",
          "save remains in calibration");

    request = imu_cal_pending_request();
    check(request.type == IMU_CAL_REQ_VERIFY_REOPEN,
          "A02",
          "save success requests verify reopen");
}

static void test_a03_verify_reopen_maps_request(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;
    ImuCalPendingRequest_t request;

    open_and_init(0x00u, "A03");
    drive_to_save_request(T0, "A03");

    imu_session_test_set_save_dcd_result(true);
    check(imu_cal_adapter_pump(), "A03", "save pump");

    before = snapshot();
    imu_session_test_set_reopen_result(true);
    check(imu_cal_adapter_pump(), "A03", "verify reopen pump");
    after = snapshot();

    check(after.readerState == IMU_READER_STATE_CONFIGURING,
          "A03",
          "reopen returns configuring");
    check(after.configurationEpoch == before.configurationEpoch + 1u,
          "A03",
          "reopen takes one epoch");
    check(!imu_session_test_recovery_observed(),
          "A03",
          "planned reopen is not recovery");

    request = imu_cal_pending_request();
    check(request.type == IMU_CAL_REQ_CONFIGURE_CALIBRATION,
          "A03",
          "verify configuration requested");
    check(request.calMask == 0u,
          "A03",
          "verify configuration mask is zero");

    check(imu_cal_adapter_pump(), "A03", "verify configure pump");
    check(imu_cal_pending_request().type == IMU_CAL_REQ_NONE,
          "A03",
          "verify configure result consumed");
}

static void test_a04_restore_production_maps_request(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;
    ImuCmdResult_t result;
    ImuCalPendingRequest_t request;
    uint8_t policyMask = 0xffu;

    open_and_init(0x00u, "A04");
    configure_through_adapter("A04");

    post_q("A04");
    request = imu_cal_pending_request();

    check(request.type == IMU_CAL_REQ_RESTORE_PRODUCTION,
          "A04",
          "q requests production restore");
    check(request.calMask == 0x00u,
          "A04",
          "restore uses flight mask");

    before = snapshot();
    check(imu_cal_adapter_pump(), "A04", "restore pump");
    after = snapshot();

    check(after.readerState == IMU_READER_STATE_CONFIGURING,
          "A04",
          "restore returns configuring");
    check(after.configurationEpoch == before.configurationEpoch + 1u,
          "A04",
          "restore takes one epoch");
    check(imu_session_get_cal_policy(&policyMask), "A04", "read production policy");
    check(policyMask == 0x00u,
          "A04",
          "production policy restored");

    check(imu_cal_complete(), "A04", "calibration completed after restore");
    check(imu_cal_get_result(&result), "A04", "get result");
    check(result.state == IMU_CMD_STATE_CANCELLED,
          "A04",
          "q result is cancelled");
    check(result.reason == IMU_CMD_REASON_OPERATOR_Q,
          "A04",
          "q reason retained");
    check(result.restoredProduction,
          "A04",
          "restore recorded");
    check(result.epochBefore == 1u,
          "A04",
          "result retains command starting epoch");
    check(result.epochAfter == after.configurationEpoch,
          "A04",
          "result epoch after restore");
    check(result.epochAfter == after.configurationEpoch,
          "A04",
          "restore result epoch after");
}

static void test_a05_session_failure_is_posted_to_imu_cal(void)
{
    ImuCalPendingRequest_t request;
    ImuCmdResult_t result;

    open_and_init(0x00u, "A05");
    imu_session_test_set_production_result(false);

    configure_through_adapter("A05");
    post_q("A05");

    request = imu_cal_pending_request();
    check(request.type == IMU_CAL_REQ_RESTORE_PRODUCTION,
          "A05",
          "restore request pending");

    check(imu_cal_adapter_pump(),
          "A05",
          "failed session action is posted, not adapter failure");
    check(imu_cal_complete(), "A05", "failed restore completes imu cal");
    check(imu_cal_get_result(&result), "A05", "get failure result");
    check(result.state == IMU_CMD_STATE_RECOVERY_FAILED,
          "A05",
          "restore failure is recovery failed");
    check(result.reason == IMU_CMD_REASON_CAL_RESTORE_FAILED,
          "A05",
          "restore failure reason retained");
    check(!result.restoredProduction,
          "A05",
          "failed restore is not claimed");
}

static void test_a06_full_success_host_flow(void)
{
    uint64_t now_ns = T0;
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;
    ImuCalPendingRequest_t request;
    ImuCmdResult_t result;
    uint8_t policy_mask = 0xffu;

    open_and_init(0x00u, "A06");
    drive_e2e_to_save(&now_ns, "A06");

    imu_session_test_set_save_dcd_result(true);
    check(imu_cal_adapter_pump(), "A06", "save pump");
    request = imu_cal_pending_request();
    check(request.type == IMU_CAL_REQ_VERIFY_REOPEN, "A06", "reopen requested");

    before = snapshot();
    imu_session_test_set_reopen_result(true);
    check(imu_cal_adapter_pump(), "A06", "verify reopen pump");
    after = snapshot();
    check(after.readerState == IMU_READER_STATE_CONFIGURING, "A06",
          "planned reopen returns CONFIGURING");
    check(after.configurationEpoch == before.configurationEpoch + 1u, "A06",
          "planned reopen takes one epoch");
    check(!imu_session_test_recovery_observed(), "A06",
          "planned reopen is not recovery");
    check(imu_session_test_recovery_attempt_count() == 0u, "A06",
          "planned reopen does not count a recovery attempt");

    request = imu_cal_pending_request();
    check(request.type == IMU_CAL_REQ_CONFIGURE_CALIBRATION, "A06",
          "verify configure requested");
    check(request.calMask == 0u, "A06", "verify configure mask is zero");
    check(imu_cal_adapter_pump(), "A06", "verify configure pump");
    check(imu_cal_pending_request().type == IMU_CAL_REQ_NONE, "A06",
          "verify configure consumed");

    post_confirm("A06", "verify");
    now_ns += IMU_CAL_VERIFY_MOTION_NS;
    owner_turn(now_ns, "A06");
    sustain_good_via_adapter(&now_ns, "A06");
    request = imu_cal_pending_request();
    check(request.type == IMU_CAL_REQ_RESTORE_PRODUCTION, "A06",
          "restore requested after verify");
    check(request.calMask == 0x00u, "A06", "restore uses flight mask");

    before = snapshot();
    check(imu_cal_adapter_pump(), "A06", "restore pump");
    after = snapshot();
    check(imu_cal_complete(), "A06", "complete");
    check(imu_cal_get_result(&result), "A06", "get result");
    check(result.state == IMU_CMD_STATE_SUCCEEDED, "A06", "succeeded");
    check(result.reason == IMU_CMD_REASON_OK, "A06", "ok");
    check(result.dcdSaved, "A06", "saved");
    check(result.verified, "A06", "verified");
    check(result.restoredProduction, "A06", "restored");
    check(result.epochBefore == 1u, "A06", "epochBefore from first configure");
    check(result.epochAfter == after.configurationEpoch, "A06",
          "epochAfter tracks restore");
    check(after.readerState == IMU_READER_STATE_CONFIGURING, "A06",
          "restore returns CONFIGURING");
    check(after.configurationEpoch == before.configurationEpoch + 1u, "A06",
          "restore takes one epoch");
    check(imu_session_get_cal_policy(&policy_mask), "A06", "read policy");
    check(policy_mask == 0x00u, "A06", "production policy restored");
    check(!imu_session_test_recovery_observed(), "A06",
          "success path never set recovery");
}

static void test_a07_save_retry_host_flow(void)
{
    uint64_t now_ns = T0;
    ImuCalPendingRequest_t request;

    open_and_init(0x00u, "A07");
    drive_e2e_to_save(&now_ns, "A07");

    imu_session_test_set_save_dcd_result(false);
    check(imu_cal_adapter_pump(), "A07", "failed save is posted");
    check(!imu_cal_complete(), "A07", "retry remains running");
    check(imu_cal_pending_request().type == IMU_CAL_REQ_NONE, "A07",
          "retry hold has no session request");

    now_ns += IMU_CAL_SAVE_RETRY_HOLD_NS;
    owner_turn(now_ns, "A07");
    request = imu_cal_pending_request();
    check(request.type == IMU_CAL_REQ_SAVE_DCD, "A07", "save retried");

    imu_session_test_set_save_dcd_result(true);
    check(imu_cal_adapter_pump(), "A07", "retry save succeeds");
    request = imu_cal_pending_request();
    check(request.type == IMU_CAL_REQ_VERIFY_REOPEN, "A07",
          "reopen follows successful retry");
}

static void test_a08_reopen_failure_host_flow(void)
{
    uint64_t now_ns = T0;
    ImuCmdResult_t result;

    open_and_init(0x00u, "A08");
    drive_e2e_to_save(&now_ns, "A08");

    imu_session_test_set_save_dcd_result(true);
    check(imu_cal_adapter_pump(), "A08", "save pump");
    check(imu_cal_pending_request().type == IMU_CAL_REQ_VERIFY_REOPEN, "A08",
          "reopen requested");

    imu_session_test_set_reopen_result(false);
    check(imu_cal_adapter_pump(), "A08", "failed reopen is posted");
    check(imu_cal_complete(), "A08", "unusable session completes cal");
    check(imu_cal_pending_request().type == IMU_CAL_REQ_NONE, "A08",
          "no restore after unusable session");
    check(imu_cal_get_result(&result), "A08", "get result");
    check(result.state == IMU_CMD_STATE_RECOVERY_FAILED, "A08",
          "recovery failed");
    check(result.reason == IMU_CMD_REASON_CAL_REOPEN_FAILED, "A08",
          "reopen failed");
    check(result.dcdSaved, "A08", "save already happened");
    check(!result.verified, "A08", "not verified");
    check(!result.restoredProduction, "A08", "not restored");
    check(imu_session_test_recovery_attempt_count() == 1u, "A08",
          "failed planned reopen counts recovery");
}

static void test_a09_facts_forwarded_without_request(void)
{
    ImuCalFacts_t facts;
    ImuCmdProgress_t progress;

    open_and_init(0x00u, "A09");
    configure_through_adapter("A09");
    check(imu_cal_pending_request().type == IMU_CAL_REQ_NONE, "A09",
          "no request while waiting on accel prompt");

    memset(&facts, 0, sizeof(facts));
    facts.version = IMU_CAL_FACTS_VERSION;
    facts.valid = true;
    facts.accelAccuracy = 3u;
    facts.gyroAccuracy = 2u;
    facts.magAccuracy = 1u;
    facts.rvAccuracy = 2u;
    facts.rvErrRad = 0.25f;
    facts.haveMag = true;
    facts.magXuT = 11.0f;
    imu_session_test_inject_cal_facts(&facts);

    check(imu_cal_adapter_pump(), "A09", "NONE pump still forwards facts");
    check(imu_cal_pending_request().type == IMU_CAL_REQ_NONE, "A09",
          "NONE pump does not invent a request");
    check(imu_cal_get_progress(&progress), "A09", "get progress");
    check(progress.cal.accelAccuracy == 3u, "A09", "accel forwarded");
    check(progress.cal.gyroAccuracy == 2u, "A09", "gyro forwarded");
    check(progress.cal.magAccuracy == 1u, "A09", "mag forwarded");
    check(progress.cal.magXuT == 11.0f, "A09", "mag X forwarded");
}

int main(void)
{
    test_a01_configure_calibration_maps_request();
    test_a02_save_dcd_maps_request();
    test_a03_verify_reopen_maps_request();
    test_a04_restore_production_maps_request();
    test_a05_session_failure_is_posted_to_imu_cal();
    test_a06_full_success_host_flow();
    test_a07_save_retry_host_flow();
    test_a08_reopen_failure_host_flow();
    test_a09_facts_forwarded_without_request();

    if (gfail != 0) {
        fprintf(stderr, "test_imu_cal_adapter: %d failures\n", gfail);
        return 1;
    }

    printf("test_imu_cal_adapter: pass\n");
    return 0;
}
