#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_check.h"

static int failures;
static const uint64_t T0 = 1000000000ull;

static void check(bool condition, const char *id, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL %s: %s\n", id, message);
        failures++;
    }
}

static ImuCheckEvent_t event_of(ImuCheckEventType_t type)
{
    ImuCheckEvent_t e;
    memset(&e, 0, sizeof(e));
    e.type = type;
    return e;
}

static void tick(uint64_t ns, const char *id)
{
    ImuCheckEvent_t e = event_of(IMU_CHECK_EVENT_TICK);
    e.monotonicNs = ns;
    check(imu_check_post(&e), id, "tick accepted");
    imu_check_service();
}

static void session(ImuCheckRequestType_t type, bool success,
                    uint32_t before, uint32_t after,
                    bool maskValid, uint8_t mask, const char *id)
{
    ImuCheckEvent_t e = event_of(IMU_CHECK_EVENT_SESSION_RESULT);
    e.session.type = type;
    e.session.success = success;
    e.session.epochBefore = before;
    e.session.epochAfter = after;
    e.session.actualMaskValid = maskValid;
    e.session.actualMask = mask;
    check(imu_check_post(&e), id, "session result accepted");
}

static ImuCmdProgress_t progress(void)
{
    ImuCmdProgress_t p;
    memset(&p, 0, sizeof(p));
    check(imu_check_get_progress(&p), "HELPER", "progress");
    return p;
}

static ImuCmdResult_t result(void)
{
    ImuCmdResult_t r;
    memset(&r, 0, sizeof(r));
    check(imu_check_get_result(&r), "HELPER", "result");
    return r;
}

static void configured(ImuCmdIdentity_t identity, uint8_t mask,
                       const char *id)
{
    check(imu_check_init(identity, identity == IMU_CMD_ID_PROBE,
                         mask, 0x04u, T0), id, "init");
    session(IMU_CHECK_REQ_CONFIGURE_CHECK, true, 1u, 2u,
            true, identity == IMU_CMD_ID_CHECK ? 0u : mask, id);
}

static ImuCheckFacts_t facts(uint8_t accel, uint8_t mag)
{
    ImuCheckFacts_t f;
    memset(&f, 0, sizeof(f));
    f.version = IMU_CHECK_FACTS_VERSION;
    f.configurationEpoch = 2u;
    f.valid = true;
    f.haveAccel = true;
    f.haveMag = true;
    f.accelStatus = accel;
    f.magStatus = mag;
    f.accelHostDecodeNs = T0 + 1ull;
    f.magHostDecodeNs = T0 + 2ull;
    return f;
}

static void q(const char *id)
{
    ImuCheckEvent_t e = event_of(IMU_CHECK_EVENT_OPERATOR_Q);
    check(imu_check_post(&e), id, "q accepted");
}

static void restored(const char *id)
{
    check(imu_check_pending_request().type ==
          IMU_CHECK_REQ_RESTORE_PRODUCTION, id, "restore pending");
    check(imu_check_pending_request().flightCalMask == 0x04u,
          id, "flight mask");
    session(IMU_CHECK_REQ_RESTORE_PRODUCTION, true, 2u, 3u,
            false, 0u, id);
}

static void test_ck01_check_configures_zero(void)
{
    ImuCheckPendingRequest_t req;
    check(imu_check_init(IMU_CMD_ID_CHECK, false, 0xffu, 0x04u, T0),
          "CK01", "init check");
    req = imu_check_pending_request();
    check(req.type == IMU_CHECK_REQ_CONFIGURE_CHECK, "CK01", "configure");
    check(req.identity == IMU_CMD_ID_CHECK, "CK01", "identity");
    check(req.effectiveMask == 0u, "CK01", "effective mask zero");
    check(progress().check.phase == IMU_CMD_CHECK_PHASE_CONFIGURE,
          "CK01", "configuration phase");
    check(!progress().check.deadlineValid, "CK01", "no deadline");
}

static void test_ck02_probe_preserves_mask_and_deadline(void)
{
    ImuCheckPendingRequest_t req;
    check(imu_check_init(IMU_CMD_ID_PROBE, true, 0x80u, 0x04u, T0),
          "CK02", "init probe");
    req = imu_check_pending_request();
    check(req.identity == IMU_CMD_ID_PROBE &&
          req.effectiveMask == 0x80u, "CK02", "full mask and identity");
    check(progress().check.requestedMaskValid &&
          progress().check.requestedMask == 0x80u,
          "CK02", "requested mask retained");
    check(progress().check.deadlineValid &&
          progress().check.deadlineNs == T0 + IMU_CHECK_PROBE_NS &&
          progress().check.remainingNs == IMU_CHECK_PROBE_NS,
          "CK02", "ten-second deadline");
    check(result().probeMaskRequestedValid &&
          result().probeMaskRequested == 0x80u,
          "CK02", "R7 requested mask");
    check(imu_check_init(IMU_CMD_ID_PROBE, true, 0u, 0u, T0),
          "CK02", "zero mask remains probe");
    check(imu_check_pending_request().effectiveMask == 0u,
          "CK02", "zero mask preserved");
}

static void test_ck03_rejects_wrong_version_and_epoch(void)
{
    ImuCheckFacts_t f;
    configured(IMU_CMD_ID_CHECK, 0u, "CK03");
    f = facts(3u, 3u);
    f.version++;
    check(!imu_check_feed_facts(&f), "CK03", "wrong version rejected");
    f.version = IMU_CHECK_FACTS_VERSION;
    f.configurationEpoch = 1u;
    check(imu_check_feed_facts(&f), "CK03", "old epoch ignored");
    tick(T0 + IMU_CHECK_GATE_NS, "CK03");
    check(!progress().check.gatePassingNow &&
          !progress().check.gateReached &&
          !progress().check.factsEpochMatched,
          "CK03", "neither fact counts");
}

static void test_ck04_only_accel_and_mag_gate(void)
{
    ImuCheckFacts_t f;
    configured(IMU_CMD_ID_CHECK, 0u, "CK04");
    f = facts(1u, 3u);
    f.haveGyro = true;
    f.haveRv = true;
    f.gyroStatus = 0u;
    f.rvStatus = 0u;
    f.rvErrRad = 0.75f;
    f.magXuT = 11.0f;
    check(imu_check_feed_facts(&f), "CK04", "low accel");
    check(!progress().check.gatePassingNow, "CK04", "accel threshold");
    f.accelStatus = 3u;
    f.magStatus = 1u;
    check(imu_check_feed_facts(&f), "CK04", "low mag");
    check(!progress().check.gatePassingNow, "CK04", "mag threshold");
    f.magStatus = 2u;
    check(imu_check_feed_facts(&f), "CK04", "good accel/mag");
    check(progress().check.gatePassingNow &&
          progress().check.gyroStatus == 0u &&
          progress().check.rvStatus == 0u &&
          progress().check.rvErrRad == 0.75f &&
          progress().check.magXuT == 11.0f,
          "CK04", "gyro and RV diagnostic only");
}

static void test_ck05_three_seconds_sets_sticky_gate(void)
{
    ImuCheckFacts_t f;
    configured(IMU_CMD_ID_CHECK, 0u, "CK05");
    f = facts(2u, 2u);
    check(imu_check_feed_facts(&f), "CK05", "good facts");
    tick(T0 + IMU_CHECK_GATE_NS - 1ull, "CK05");
    check(!progress().check.gateReached, "CK05", "not early");
    tick(T0 + IMU_CHECK_GATE_NS, "CK05");
    check(progress().check.gateReached &&
          progress().check.sustainedGoodNs == IMU_CHECK_GATE_NS,
          "CK05", "exact gate");
    f.magStatus = 0u;
    check(imu_check_feed_facts(&f), "CK05", "bad verdict");
    check(!progress().check.gatePassingNow &&
          progress().check.gateReached &&
          progress().check.sustainedGoodNs == 0ull,
          "CK05", "sticky history, reset timer");
}

static void test_ck06_bad_interval_restarts_timer(void)
{
    ImuCheckFacts_t f;
    configured(IMU_CMD_ID_CHECK, 0u, "CK06");
    f = facts(3u, 3u);
    check(imu_check_feed_facts(&f), "CK06", "first good");
    tick(T0 + 2000000000ull, "CK06");
    f.magStatus = 1u;
    check(imu_check_feed_facts(&f), "CK06", "bad");
    tick(T0 + 3000000000ull, "CK06");
    f.magStatus = 3u;
    check(imu_check_feed_facts(&f), "CK06", "recovered");
    tick(T0 + 5999999999ull, "CK06");
    check(!progress().check.gateReached, "CK06", "not cumulative");
    tick(T0 + 6000000000ull, "CK06");
    check(progress().check.gateReached, "CK06", "new full interval");
}

static void test_ck07_check_has_no_auto_deadline(void)
{
    configured(IMU_CMD_ID_CHECK, 0u, "CK07");
    tick(T0 + 60000000000ull, "CK07");
    check(!imu_check_complete() &&
          progress().check.phase == IMU_CMD_CHECK_PHASE_MONITOR &&
          !progress().check.deadlineValid,
          "CK07", "waits for q");
}

static void test_ck08_check_q_restores_and_keeps_verdict(void)
{
    ImuCheckFacts_t f;
    ImuCmdResult_t r;
    configured(IMU_CMD_ID_CHECK, 0u, "CK08");
    f = facts(3u, 3u);
    check(imu_check_feed_facts(&f), "CK08", "facts");
    tick(T0 + IMU_CHECK_GATE_NS, "CK08");
    q("CK08");
    check(!imu_check_complete(), "CK08", "not terminal before restore");
    restored("CK08");
    r = result();
    check(r.state == IMU_CMD_STATE_CANCELLED &&
          r.reason == IMU_CMD_REASON_OPERATOR_Q &&
          r.restoredProduction && !r.warningRequired,
          "CK08", "cancelled after restore");
    check(r.terminalProgress.check.gatePassingNow &&
          r.terminalProgress.check.gateReached &&
          r.terminalProgress.check.phase == IMU_CMD_CHECK_PHASE_COMPLETE,
          "CK08", "terminal verdict retained");
}

static void test_ck09_gate_does_not_end_probe(void)
{
    ImuCheckFacts_t f;
    configured(IMU_CMD_ID_PROBE, 0x02u, "CK09");
    f = facts(3u, 3u);
    check(imu_check_feed_facts(&f), "CK09", "facts");
    tick(T0 + IMU_CHECK_GATE_NS, "CK09");
    check(progress().check.gateReached &&
          !imu_check_complete() &&
          imu_check_pending_request().type == IMU_CHECK_REQ_NONE,
          "CK09", "still monitoring after gate");
}

static void test_ck10_deadline_restores_then_times_out(void)
{
    ImuCheckFacts_t f;
    ImuCmdResult_t r;
    configured(IMU_CMD_ID_PROBE, 0x02u, "CK10");
    f = facts(3u, 3u);
    check(imu_check_feed_facts(&f), "CK10", "facts");
    tick(T0 + IMU_CHECK_GATE_NS, "CK10");
    tick(T0 + IMU_CHECK_PROBE_NS, "CK10");
    check(!imu_check_complete() &&
          progress().check.remainingNs == 0ull &&
          progress().check.phase == IMU_CMD_CHECK_PHASE_RESTORE,
          "CK10", "deadline waits for restore");
    restored("CK10");
    r = result();
    check(r.state == IMU_CMD_STATE_TIMED_OUT &&
          r.reason == IMU_CMD_REASON_PROBE_DEADLINE &&
          r.sub.probeTimedOut && r.sub.probeReachedGate &&
          r.sub.probeMaskActualValid && r.sub.probeMaskActual == 0x02u,
          "CK10", "timeout and mask evidence");
    check(r.terminalProgress.check.gatePassingNow &&
          r.terminalProgress.check.gateReached,
          "CK10", "terminal verdict and history");
}

static void test_ck11_probe_q_ends_early(void)
{
    ImuCmdResult_t r;
    configured(IMU_CMD_ID_PROBE, 0u, "CK11");
    tick(T0 + 1000000000ull, "CK11");
    q("CK11");
    restored("CK11");
    r = result();
    check(r.state == IMU_CMD_STATE_CANCELLED &&
          r.reason == IMU_CMD_REASON_OPERATOR_Q &&
          r.sub.probeOperatorEndedEarly && !r.sub.probeTimedOut,
          "CK11", "early operator end");
}

static void test_ck12_config_failure_restores(void)
{
    ImuCmdResult_t r;
    check(imu_check_init(IMU_CMD_ID_PROBE, true, 0x80u,
                         0x04u, T0), "CK12", "init");
    session(IMU_CHECK_REQ_CONFIGURE_CHECK, false, 1u, 1u,
            true, 0x01u, "CK12");
    check(!imu_check_complete(), "CK12", "failure awaits restoration");
    restored("CK12");
    r = result();
    check(r.state == IMU_CMD_STATE_FAILED &&
          r.reason == IMU_CMD_REASON_CONFIG_FAILED &&
          r.warningRequired && r.restoredProduction &&
          r.sub.probeMaskActualValid && r.sub.probeMaskActual == 0x01u,
          "CK12", "failed with readable actual mask");
}

static void test_ck13_restore_failure_is_recovery_failure(void)
{
    ImuCmdResult_t r;
    configured(IMU_CMD_ID_CHECK, 0u, "CK13");
    q("CK13");
    session(IMU_CHECK_REQ_RESTORE_PRODUCTION, false, 2u, 2u,
            false, 0u, "CK13");
    r = result();
    check(imu_check_complete() &&
          r.state == IMU_CMD_STATE_RECOVERY_FAILED &&
          r.reason == IMU_CMD_REASON_SESSION_UNUSABLE &&
          !r.restoredProduction,
          "CK13", "unusable session");
}

static void test_ck14_live_and_terminal_progress(void)
{
    ImuCheckFacts_t f;
    ImuCmdResult_t r;
    configured(IMU_CMD_ID_CHECK, 0u, "CK14");
    f = facts(3u, 3u);
    check(imu_check_feed_facts(&f), "CK14", "good");
    tick(T0 + IMU_CHECK_GATE_NS, "CK14");
    f.magStatus = 0u;
    check(imu_check_feed_facts(&f), "CK14", "latest bad");
    check(!progress().check.gatePassingNow &&
          progress().check.gateReached, "CK14", "latest R8");
    q("CK14");
    restored("CK14");
    r = result();
    check(!r.terminalProgress.check.gatePassingNow &&
          r.terminalProgress.check.gateReached &&
          r.terminalProgress.version == IMU_CMD_PROGRESS_VERSION,
          "CK14", "R7 explains final verdict without cached R8");
}

static void test_ck15_backward_and_repeated_events(void)
{
    ImuCheckEvent_t e;
    configured(IMU_CMD_ID_PROBE, 0x02u, "CK15");
    tick(T0 + 5000000000ull, "CK15");
    tick(T0 + 1000000000ull, "CK15");
    check(progress().check.remainingNs == 5000000000ull,
          "CK15", "backward tick cannot increase remaining");
    e = event_of(IMU_CHECK_EVENT_SESSION_RESULT);
    e.session.type = IMU_CHECK_REQ_CONFIGURE_CHECK;
    check(!imu_check_post(&e), "CK15", "duplicate result rejected");
    q("CK15");
    q("CK15");
    check(imu_check_pending_request().type ==
          IMU_CHECK_REQ_RESTORE_PRODUCTION, "CK15", "one restore request");
    restored("CK15");
    check(imu_check_pending_request().type == IMU_CHECK_REQ_NONE,
          "CK15", "no request after completion");
    check(!imu_check_post(&e), "CK15", "late session result rejected");
    check(!imu_check_init(IMU_CMD_ID_CHECK, true, 0u, 0u, T0),
          "CK15", "check with mask illegal");
    check(!imu_check_init(IMU_CMD_ID_PROBE, false, 0u, 0u, T0),
          "CK15", "probe without mask illegal");
}

int main(void)
{
    test_ck01_check_configures_zero();
    test_ck02_probe_preserves_mask_and_deadline();
    test_ck03_rejects_wrong_version_and_epoch();
    test_ck04_only_accel_and_mag_gate();
    test_ck05_three_seconds_sets_sticky_gate();
    test_ck06_bad_interval_restarts_timer();
    test_ck07_check_has_no_auto_deadline();
    test_ck08_check_q_restores_and_keeps_verdict();
    test_ck09_gate_does_not_end_probe();
    test_ck10_deadline_restores_then_times_out();
    test_ck11_probe_q_ends_early();
    test_ck12_config_failure_restores();
    test_ck13_restore_failure_is_recovery_failure();
    test_ck14_live_and_terminal_progress();
    test_ck15_backward_and_repeated_events();
    if (failures != 0) {
        fprintf(stderr, "test_imu_check: %d failure(s)\n", failures);
        return 1;
    }
    puts("test_imu_check: pass");
    return 0;
}
