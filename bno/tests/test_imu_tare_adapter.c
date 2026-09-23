#define _POSIX_C_SOURCE 200809L

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_session.h"
#include "app/imu_tare.h"
#include "app/imu_tare_adapter.h"

static int g_fail;
static const uint64_t T0 = 1000000000ull;

static void check(bool ok, const char *id, const char *what)
{
    if (!ok) {
        fprintf(stderr, "FAIL %s: %s\n", id, what);
        g_fail++;
    }
}

static ImuSampleSnapshot_t snapshot(void)
{
    ImuSampleSnapshot_t out;

    memset(&out, 0, sizeof(out));
    (void)imu_session_get_snapshot(&out);
    return out;
}

static void tick_and_service(uint64_t nowNs, const char *id)
{
    ImuTareEvent_t event;

    memset(&event, 0, sizeof(event));
    event.type = IMU_TARE_EVENT_TICK;
    event.monotonicNs = nowNs;
    check(imu_tare_post(&event), id, "tick");
    imu_tare_service();
}

static void confirm(const char *id)
{
    ImuTareEvent_t event;

    memset(&event, 0, sizeof(event));
    event.type = IMU_TARE_EVENT_OPERATOR_CONFIRM;
    check(imu_tare_post(&event), id, "confirm");
    imu_tare_service();
}

static void post_q(const char *id)
{
    ImuTareEvent_t event;

    memset(&event, 0, sizeof(event));
    event.type = IMU_TARE_EVENT_OPERATOR_Q;
    check(imu_tare_post(&event), id, "q");
    imu_tare_service();
}

static void open_init(ImuCmdIdentity_t identity, ImuCmdTareAxes_t axes,
                      const char *id)
{
    imu_session_test_reset();
    check(imu_session_test_open(true), id, "test open");
    check(imu_tare_init(identity, axes, 0x00u, T0), id, "tare init");
}

static void configure_through_adapter(const char *id)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;
    ImuTarePendingRequest_t request;

    confirm(id);
    check(imu_tare_pending_request().type == IMU_TARE_REQ_CONFIGURE, id,
          "configure pending");
    before = snapshot();
    check(imu_tare_adapter_pump(), id, "configure pump");
    after = snapshot();
    check(after.readerState == IMU_READER_STATE_TARE, id, "session is TARE");
    check(after.configurationEpoch == before.configurationEpoch + 1u, id,
          "configure takes one epoch");
    request = imu_tare_pending_request();
    if (request.type == IMU_TARE_REQ_CLEAR) {
        check(true, id, "tare-clear arms CLEAR on the same result");
    } else {
        check(request.type == IMU_TARE_REQ_NONE, id, "configure consumed");
    }
}

static void drive_to_tare_now(ImuCmdTareAxes_t axes, const char *id)
{
    open_init(IMU_CMD_ID_TARE, axes, id);
    configure_through_adapter(id);
    tick_and_service(T0 + IMU_TARE_SETTLE_NS, id);
    check(imu_tare_pending_request().type == IMU_TARE_REQ_TARE_NOW, id,
          "tare-now pending");
    check(imu_tare_pending_request().tareAxes == axes, id, "axes on request");
}

static void test_u01_configure_maps_request(void)
{
    ImuCmdProgress_t progress;

    open_init(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z, "U01");
    configure_through_adapter("U01");
    check(imu_tare_get_progress(&progress), "U01", "progress");
    check(progress.tare.phase == IMU_CMD_TARE_PHASE_SETTLE, "U01",
          "configure advances to settle");
}

static void test_u02_z_axis_tare_now(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;

    drive_to_tare_now(IMU_CMD_TARE_AXES_Z, "U02");
    before = snapshot();
    check(imu_tare_adapter_pump(), "U02", "tare-now pump");
    after = snapshot();
    check(imu_session_test_last_tare_axes() == 0x04u, "U02", "SH2_TARE_Z");
    check(imu_session_test_last_tare_basis() == 0u, "U02", "RV basis");
    check(after.configurationEpoch == before.configurationEpoch + 1u, "U02",
          "tare-now epoch +1");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_PERSIST, "U02",
          "persist requested");
}

static void test_u03_full_axis_tare_now(void)
{
    drive_to_tare_now(IMU_CMD_TARE_AXES_FULL, "U03");
    check(imu_tare_adapter_pump(), "U03", "tare-now pump");
    check(imu_session_test_last_tare_axes() == 0x07u, "U03", "SH2_TARE_X|Y|Z");
}

static void test_u04_persist_maps_request(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;

    drive_to_tare_now(IMU_CMD_TARE_AXES_Z, "U04");
    check(imu_tare_adapter_pump(), "U04", "tare-now pump");
    before = snapshot();
    check(imu_tare_pending_request().type == IMU_TARE_REQ_PERSIST, "U04",
          "persist pending");
    check(imu_tare_adapter_pump(), "U04", "persist pump");
    after = snapshot();
    check(after.configurationEpoch == before.configurationEpoch, "U04",
          "persist does not increment epoch");
    check(after.readerState == IMU_READER_STATE_TARE, "U04", "still TARE");
}

static void test_u05_clear_result_mapping(void)
{
    ImuCmdResult_t result;

    open_init(IMU_CMD_ID_TARE_CLEAR, IMU_CMD_TARE_AXES_NONE, "U05");
    configure_through_adapter("U05");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_CLEAR, "U05",
          "clear pending");
    check(imu_tare_adapter_pump(), "U05", "clear pump");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_RESTORE_PRODUCTION,
          "U05", "restore after clear");
    check(imu_tare_adapter_pump(), "U05", "restore pump");
    check(imu_tare_complete(), "U05", "complete");
    check(imu_tare_get_result(&result), "U05", "result");
    check(result.state == IMU_CMD_STATE_SUCCEEDED, "U05", "succeeded");
    check(result.sub.clearActive == IMU_CMD_SUB_SUCCEEDED, "U05", "active");
    check(result.sub.clearSaved == IMU_CMD_SUB_SUCCEEDED, "U05", "saved");
}

static void test_u06_production_restore(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;
    ImuCmdResult_t result;
    uint8_t mask = 0xffu;

    open_init(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z, "U06");
    configure_through_adapter("U06");
    post_q("U06");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_RESTORE_PRODUCTION,
          "U06", "restore pending");
    check(imu_tare_pending_request().flightCalMask == 0x00u, "U06",
          "flight mask");
    before = snapshot();
    check(imu_tare_adapter_pump(), "U06", "restore pump");
    after = snapshot();
    check(after.readerState == IMU_READER_STATE_CONFIGURING, "U06",
          "CONFIGURING");
    check(after.configurationEpoch == before.configurationEpoch + 1u, "U06",
          "restore epoch +1");
    check(imu_session_get_cal_policy(&mask), "U06", "policy");
    check(mask == 0x00u, "U06", "mask restored");
    check(imu_tare_complete(), "U06", "complete");
    check(imu_tare_get_result(&result), "U06", "result");
    check(result.state == IMU_CMD_STATE_CANCELLED, "U06", "cancelled");
    check(result.restoredProduction, "U06", "restored");
    check(!imu_session_test_recovery_observed(), "U06", "not recovery");
}

static void test_u07_facts_forwarded(void)
{
    ImuTareFacts_t facts;
    ImuCmdProgress_t progress;

    open_init(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z, "U07");
    configure_through_adapter("U07");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_NONE, "U07",
          "none during settle");
    memset(&facts, 0, sizeof(facts));
    facts.version = IMU_TARE_FACTS_VERSION;
    facts.valid = true;
    facts.yawRad = 0.25f;
    facts.quatReal = 1.0f;
    imu_session_test_inject_tare_facts(&facts);
    check(imu_tare_adapter_pump(), "U07", "NONE pump forwards facts");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_NONE, "U07",
          "NONE does not invent a request");
    check(imu_tare_get_progress(&progress), "U07", "progress");
    check(progress.tare.attitudeValid, "U07", "valid");
    check(progress.tare.yawRad == 0.25f, "U07", "yaw forwarded");
}

static void test_u08_none_is_noop(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;

    open_init(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z, "U08");
    configure_through_adapter("U08");
    before = snapshot();
    check(imu_tare_adapter_pump(), "U08", "NONE pump");
    after = snapshot();
    check(after.configurationEpoch == before.configurationEpoch, "U08",
          "NONE does not change epoch");
    check(after.readerState == IMU_READER_STATE_TARE, "U08", "still TARE");
}

static void test_u09_session_failure_posted(void)
{
    ImuCmdProgress_t progress;

    open_init(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z, "U09");
    confirm("U09");
    imu_session_test_set_configure_tare_result(false);
    check(imu_tare_adapter_pump(), "U09",
          "failed session action is posted, not adapter failure");
    check(imu_tare_get_progress(&progress), "U09", "progress");
    check(progress.tare.phase == IMU_CMD_TARE_PHASE_RESTORE, "U09",
          "machine owns configure failure");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_RESTORE_PRODUCTION,
          "U09", "restore requested");
}

static void test_u10_missing_epoch_is_adapter_failure(void)
{
    imu_session_test_reset();
    check(imu_tare_init(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z, 0x00u, T0),
          "U10", "init");
    confirm("U10");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_CONFIGURE, "U10",
          "configure pending");
    check(!imu_tare_adapter_pump(), "U10",
          "no snapshot/epoch is adapter failure");
    check(imu_tare_pending_request().type == IMU_TARE_REQ_CONFIGURE, "U10",
          "request not consumed");
}

static void test_u11_epoch_captured_around_action(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;
    ImuCmdResult_t result;

    open_init(IMU_CMD_ID_TARE, IMU_CMD_TARE_AXES_Z, "U11");
    confirm("U11");
    before = snapshot();
    check(imu_tare_adapter_pump(), "U11", "configure pump");
    after = snapshot();
    check(after.configurationEpoch == before.configurationEpoch + 1u, "U11",
          "action epoch after");
    check(imu_tare_get_result(&result), "U11", "result");
    check(result.epochBefore == before.configurationEpoch, "U11",
          "epochBefore captured before action");
    check(result.epochAfter == after.configurationEpoch, "U11",
          "epochAfter captured after action");
}

static void test_u12_one_pump_one_request(void)
{
    ImuSampleSnapshot_t after_now;
    ImuTarePendingRequest_t request;

    drive_to_tare_now(IMU_CMD_TARE_AXES_Z, "U12");
    check(imu_tare_adapter_pump(), "U12", "first pump");
    after_now = snapshot();
    request = imu_tare_pending_request();
    check(request.type == IMU_TARE_REQ_PERSIST, "U12",
          "persist remains for next turn");
    check(imu_session_test_have_last_tare_now(), "U12", "tare-now executed");
    check(imu_tare_adapter_pump(), "U12", "second pump");
    check(snapshot().configurationEpoch == after_now.configurationEpoch,
          "U12", "second pump did not tare-now again");
}

int main(void)
{
    test_u01_configure_maps_request();
    test_u02_z_axis_tare_now();
    test_u03_full_axis_tare_now();
    test_u04_persist_maps_request();
    test_u05_clear_result_mapping();
    test_u06_production_restore();
    test_u07_facts_forwarded();
    test_u08_none_is_noop();
    test_u09_session_failure_posted();
    test_u10_missing_epoch_is_adapter_failure();
    test_u11_epoch_captured_around_action();
    test_u12_one_pump_one_request();

    if (g_fail != 0) {
        fprintf(stderr, "test_imu_tare_adapter: %d failure(s)\n", g_fail);
        return 1;
    }
    printf("test_imu_tare_adapter: pass\n");
    return 0;
}
