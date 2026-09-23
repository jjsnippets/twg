#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_check.h"
#include "app/imu_check_adapter.h"
#include "app/imu_session.h"

static int failures;
static const uint64_t T0 = 1000000000ull;

static void check(bool ok, const char *id, const char *what)
{
    if (!ok) {
        fprintf(stderr, "FAIL %s: %s\n", id, what);
        failures++;
    }
}

static ImuSampleSnapshot_t snapshot(void)
{
    ImuSampleSnapshot_t out;
    memset(&out, 0, sizeof(out));
    check(imu_session_get_snapshot(&out), "HELPER", "session snapshot");
    return out;
}

static ImuCmdProgress_t progress(void)
{
    ImuCmdProgress_t out;
    memset(&out, 0, sizeof(out));
    check(imu_check_get_progress(&out), "HELPER", "machine progress");
    return out;
}

static ImuCmdResult_t result(void)
{
    ImuCmdResult_t out;
    memset(&out, 0, sizeof(out));
    check(imu_check_get_result(&out), "HELPER", "machine result");
    return out;
}

static void open_init(ImuCmdIdentity_t identity, uint8_t mask,
                      const char *id)
{
    imu_session_test_reset();
    check(imu_session_test_open(true), id, "test open");
    check(imu_check_init(identity, identity == IMU_CMD_ID_PROBE,
                         mask, 0x04u, T0), id, "check machine init");
}

static void configure(ImuCmdIdentity_t identity, uint8_t mask,
                      const char *id)
{
    open_init(identity, mask, id);
    check(imu_check_adapter_pump(), id, "configure pump");
    check(imu_check_pending_request().type == IMU_CHECK_REQ_NONE,
          id, "configuration request consumed");
}

static void post_q(const char *id)
{
    ImuCheckEvent_t event;
    memset(&event, 0, sizeof(event));
    event.type = IMU_CHECK_EVENT_OPERATOR_Q;
    check(imu_check_post(&event), id, "q");
}

static void tick_and_service(uint64_t nowNs, const char *id)
{
    ImuCheckEvent_t event;
    memset(&event, 0, sizeof(event));
    event.type = IMU_CHECK_EVENT_TICK;
    event.monotonicNs = nowNs;
    check(imu_check_post(&event), id, "tick");
    imu_check_service();
}

static void test_ca01_none_is_successful_noop(void)
{
    ImuSampleSnapshot_t before;
    configure(IMU_CMD_ID_CHECK, 0u, "CA01");
    before = snapshot();
    check(imu_check_adapter_pump(), "CA01", "NONE pump succeeds");
    check(snapshot().configurationEpoch == before.configurationEpoch &&
          snapshot().readerState == IMU_READER_STATE_CHECK &&
          imu_check_pending_request().type == IMU_CHECK_REQ_NONE,
          "CA01", "no session mutation or invented request");
}

static void test_ca02_forwards_latest_facts_snapshot(void)
{
    ImuCmdProgress_t p;
    configure(IMU_CMD_ID_CHECK, 0u, "CA02");
    imu_session_test_inject_check_report(IMU_SESSION_TEST_REPORT_MAG,
                                          2u, 101ull, 0.0f,
                                          12.0f, -4.0f, 40.0f);
    imu_session_test_inject_check_report(IMU_SESSION_TEST_REPORT_ACCEL,
                                          3u, 102ull, 0.0f, 0, 0, 0);
    check(imu_check_adapter_pump(), "CA02", "forward one mailbox copy");
    p = progress();
    check(p.check.factsEpochMatched && p.check.haveAccel &&
          p.check.haveMag && p.check.accelHostDecodeNs == 102ull &&
          p.check.magHostDecodeNs == 101ull &&
          p.check.accelStatus == 3u && p.check.magStatus == 2u &&
          p.check.magXuT == 12.0f && p.check.gatePassingNow,
          "CA02", "latest facts preserved without session request");
    check(imu_check_pending_request().type == IMU_CHECK_REQ_NONE,
          "CA02", "facts did not invent an action");
}

static void test_ca03_check_configure_maps_identity_mask_epochs(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;
    ImuCmdResult_t r;

    open_init(IMU_CMD_ID_CHECK, 0xffu, "CA03");
    before = snapshot();
    check(imu_check_pending_request().identity == IMU_CMD_ID_CHECK &&
          imu_check_pending_request().effectiveMask == 0u,
          "CA03", "check request uses effective zero");
    check(imu_check_adapter_pump(), "CA03", "configure");
    after = snapshot();
    r = result();
    check(after.readerState == IMU_READER_STATE_CHECK &&
          after.configurationEpoch == before.configurationEpoch + 1u,
          "CA03", "CHECK session action");
    check(r.epochBefore == before.configurationEpoch &&
          r.epochAfter == after.configurationEpoch &&
          progress().check.actualMaskValid &&
          progress().check.actualMask == 0u,
          "CA03", "before/after epochs and actual readback");
}

static void test_ca04_probe_mismatch_keeps_actual_mask(void)
{
    ImuCmdProgress_t p;
    ImuCmdResult_t r;

    open_init(IMU_CMD_ID_PROBE, 0x80u, "CA04");
    imu_session_test_set_check_readback(true, 0x01u);
    check(imu_check_adapter_pump(), "CA04",
          "failed session action still posted");
    p = progress();
    r = result();
    check(snapshot().readerState == IMU_READER_STATE_PROBE,
          "CA04", "mismatch remains actionable");
    check(p.check.phase == IMU_CMD_CHECK_PHASE_RESTORE &&
          p.check.requestedMaskValid && p.check.requestedMask == 0x80u &&
          p.check.actualMaskValid && p.check.actualMask == 0x01u &&
          r.sub.probeMaskActualValid && r.sub.probeMaskActual == 0x01u,
          "CA04", "requested and actual evidence distinct");
    check(imu_check_pending_request().type ==
          IMU_CHECK_REQ_RESTORE_PRODUCTION,
          "CA04", "configuration mismatch requests restore");
}

static void test_ca05_unavailable_readback_stays_invalid(void)
{
    configure(IMU_CMD_ID_PROBE, 0x02u, "CA05");
    /* Repeat with controlled unavailable readback from a fresh session. */
    open_init(IMU_CMD_ID_PROBE, 0x02u, "CA05");
    imu_session_test_set_check_readback(false, 0xffu);
    check(imu_check_adapter_pump(), "CA05", "configuration succeeds");
    check(progress().check.phase == IMU_CMD_CHECK_PHASE_MONITOR &&
          !progress().check.actualMaskValid &&
          !result().sub.probeMaskActualValid,
          "CA05", "no fabricated actual mask");
}

static void test_ca06_config_failure_is_machine_result(void)
{
    ImuCmdProgress_t p;

    open_init(IMU_CMD_ID_PROBE, 0x02u, "CA06");
    imu_session_test_set_configure_check_result(false);
    check(imu_check_adapter_pump(), "CA06",
          "hard configuration failure is posted");
    p = progress();
    check(snapshot().readerState == IMU_READER_STATE_FAULTED &&
          p.check.phase == IMU_CMD_CHECK_PHASE_RESTORE &&
          imu_check_pending_request().type ==
              IMU_CHECK_REQ_RESTORE_PRODUCTION,
          "CA06", "machine owns recovery decision");
}

static void test_ca07_restore_uses_flight_mask_and_epochs(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;
    ImuCmdResult_t r;
    uint8_t policy = 0u;

    configure(IMU_CMD_ID_CHECK, 0u, "CA07");
    post_q("CA07");
    check(imu_check_pending_request().type ==
          IMU_CHECK_REQ_RESTORE_PRODUCTION &&
          imu_check_pending_request().flightCalMask == 0x04u,
          "CA07", "flight mask request");
    before = snapshot();
    check(imu_check_adapter_pump(), "CA07", "restore pump");
    after = snapshot();
    r = result();
    check(after.readerState == IMU_READER_STATE_CONFIGURING &&
          after.configurationEpoch == before.configurationEpoch + 1u &&
          imu_session_get_cal_policy(&policy) && policy == 0x04u,
          "CA07", "restored production and policy");
    check(imu_check_complete() && r.restoredProduction &&
          r.state == IMU_CMD_STATE_CANCELLED &&
          r.epochBefore == 1u &&
          r.epochAfter == after.configurationEpoch,
          "CA07", "machine received restore epoch evidence");
}

static void test_ca08_restore_failure_is_posted(void)
{
    ImuCmdResult_t r;
    configure(IMU_CMD_ID_CHECK, 0u, "CA08");
    post_q("CA08");
    imu_session_test_set_production_result(false);
    check(imu_check_adapter_pump(), "CA08",
          "failed restore is a delivered machine result");
    r = result();
    check(imu_check_complete() &&
          r.state == IMU_CMD_STATE_RECOVERY_FAILED &&
          r.reason == IMU_CMD_REASON_SESSION_UNUSABLE &&
          !r.restoredProduction &&
          imu_check_pending_request().type == IMU_CHECK_REQ_NONE,
          "CA08", "machine records unusable session");
}

static void test_ca09_bad_facts_version_rejected(void)
{
    ImuCheckFacts_t bad;
    ImuCmdProgress_t p;

    configure(IMU_CMD_ID_CHECK, 0u, "CA09");
    memset(&bad, 0, sizeof(bad));
    bad.version = IMU_CHECK_FACTS_VERSION + 1u;
    bad.configurationEpoch = snapshot().configurationEpoch;
    bad.valid = true;
    bad.haveAccel = true;
    bad.haveMag = true;
    bad.accelStatus = 3u;
    bad.magStatus = 3u;
    imu_session_test_inject_check_facts(&bad);
    check(!imu_check_adapter_pump(), "CA09", "invalid facts rejected");
    p = progress();
    check(!p.check.gatePassingNow && !p.check.gateReached &&
          p.check.phase == IMU_CMD_CHECK_PHASE_MONITOR,
          "CA09", "no invalid facts forwarded");
}

static void test_ca10_one_pump_consumes_one_request(void)
{
    ImuSampleSnapshot_t afterConfigure;

    open_init(IMU_CMD_ID_PROBE, 0x02u, "CA10");
    imu_session_test_set_check_readback(true, 0x01u);
    check(imu_check_adapter_pump(), "CA10", "first pump");
    afterConfigure = snapshot();
    check(imu_check_pending_request().type ==
          IMU_CHECK_REQ_RESTORE_PRODUCTION,
          "CA10", "restore left for a later pump");
    check(afterConfigure.readerState == IMU_READER_STATE_PROBE,
          "CA10", "not restored in configure pump");
    check(imu_check_adapter_pump(), "CA10", "second pump");
    check(snapshot().readerState == IMU_READER_STATE_CONFIGURING &&
          snapshot().configurationEpoch ==
              afterConfigure.configurationEpoch + 1u &&
          imu_check_complete(),
          "CA10", "second pump performed restore");
}

static void test_ca11_missing_epoch_rejects_before_action(void)
{
    imu_session_test_reset();
    check(imu_check_init(IMU_CMD_ID_CHECK, false, 0u, 0u, T0),
          "CA11", "machine init");
    check(!imu_check_adapter_pump(), "CA11", "no session epoch");
    check(imu_check_pending_request().type ==
          IMU_CHECK_REQ_CONFIGURE_CHECK,
          "CA11", "request not consumed without evidence");
}

static void test_ca12_facts_forwarded_before_tick_gate(void)
{
    configure(IMU_CMD_ID_PROBE, 0x02u, "CA12");
    imu_session_test_inject_check_report(IMU_SESSION_TEST_REPORT_ACCEL,
                                          3u, 101ull, 0, 0, 0, 0);
    imu_session_test_inject_check_report(IMU_SESSION_TEST_REPORT_MAG,
                                          3u, 102ull, 0, 0, 0, 0);
    check(imu_check_adapter_pump(), "CA12", "facts pump");
    tick_and_service(T0 + IMU_CHECK_GATE_NS, "CA12");
    check(progress().check.gatePassingNow &&
          progress().check.gateReached &&
          !imu_check_complete(),
          "CA12", "facts available to machine before tick");
}

int main(void)
{
    test_ca01_none_is_successful_noop();
    test_ca02_forwards_latest_facts_snapshot();
    test_ca03_check_configure_maps_identity_mask_epochs();
    test_ca04_probe_mismatch_keeps_actual_mask();
    test_ca05_unavailable_readback_stays_invalid();
    test_ca06_config_failure_is_machine_result();
    test_ca07_restore_uses_flight_mask_and_epochs();
    test_ca08_restore_failure_is_posted();
    test_ca09_bad_facts_version_rejected();
    test_ca10_one_pump_consumes_one_request();
    test_ca11_missing_epoch_rejects_before_action();
    test_ca12_facts_forwarded_before_tick_gate();
    if (failures != 0) {
        fprintf(stderr, "test_imu_check_adapter: %d failure(s)\n",
                failures);
        return 1;
    }
    puts("test_imu_check_adapter: pass");
    return 0;
}
