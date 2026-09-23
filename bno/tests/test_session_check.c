#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_session.h"

static int failures;

static void check(bool ok, const char *id, const char *what)
{
    if (!ok) {
        fprintf(stderr, "FAIL %s: %s\n", id, what);
        failures++;
    }
}

static ImuSampleSnapshot_t snap(void)
{
    ImuSampleSnapshot_t s;
    memset(&s, 0, sizeof(s));
    check(imu_session_get_snapshot(&s), "HELPER", "snapshot");
    return s;
}

static ImuCheckFacts_t facts(void)
{
    ImuCheckFacts_t f;
    memset(&f, 0, sizeof(f));
    check(imu_session_get_check_facts(&f), "HELPER", "check facts");
    return f;
}

static void open_configuring(const char *id)
{
    imu_session_test_reset();
    check(imu_session_test_open(true), id, "open");
}

static void enter(ImuSessionCheckMode_t mode, uint8_t mask, const char *id)
{
    ImuSessionCheckConfigResult_t r;
    open_configuring(id);
    memset(&r, 0, sizeof(r));
    check(imu_session_configure_check(mode, mask, &r), id, "configure");
}

static void report_is(ImuSessionTestReportId_t id, uint32_t interval,
                      const char *caseId)
{
    ImuSessionTestReportConfig_t c;
    memset(&c, 0, sizeof(c));
    check(imu_session_test_get_report_config(id, &c),
          caseId, "report trace");
    check(c.attempted && c.reportIntervalUs == interval &&
          c.batchIntervalUs == 0u, caseId, "interval and zero batch");
}

static void test_sc01_exact_report_configuration(void)
{
    check(IMU_CHECK_MAG_RATE_HZ == 50u &&
          IMU_CHECK_SLOW_RATE_HZ == 10u &&
          IMU_CHECK_MAG_INTERVAL_US == 20000u &&
          IMU_CHECK_SLOW_INTERVAL_US == 100000u,
          "SC01", "published rates and intervals");
    enter(IMU_SESSION_CHECK_MODE_CHECK, 0u, "SC01");
    report_is(IMU_SESSION_TEST_REPORT_MAG, 20000u, "SC01");
    report_is(IMU_SESSION_TEST_REPORT_ACCEL, 100000u, "SC01");
    report_is(IMU_SESSION_TEST_REPORT_GYRO, 100000u, "SC01");
    report_is(IMU_SESSION_TEST_REPORT_RV, 100000u, "SC01");
    report_is(IMU_SESSION_TEST_REPORT_LINEAR, 0u, "SC01");
}

static void test_sc02_check_enters_check_with_zero_mask(void)
{
    ImuSessionCheckConfigResult_t r;
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;
    uint8_t mask = 0xffu;

    open_configuring("SC02");
    before = snap();
    memset(&r, 0, sizeof(r));
    check(imu_session_configure_check(IMU_SESSION_CHECK_MODE_CHECK,
                                      0u, &r), "SC02", "configure check");
    after = snap();
    check(after.readerState == IMU_READER_STATE_CHECK &&
          after.configurationEpoch == before.configurationEpoch + 1u &&
          after.validMask == 0u, "SC02", "CHECK and one new epoch");
    check(imu_session_get_cal_policy(&mask) && mask == 0u,
          "SC02", "zero check policy");
}

static void test_sc03_probe_preserves_full_mask_including_zero(void)
{
    ImuSessionCheckConfigResult_t r;
    uint8_t mask = 0u;

    enter(IMU_SESSION_CHECK_MODE_PROBE, 0x80u, "SC03");
    check(snap().readerState == IMU_READER_STATE_PROBE,
          "SC03", "distinct PROBE state");
    check(imu_session_get_cal_policy(&mask) && mask == 0x80u,
          "SC03", "unnamed upper bit preserved");

    open_configuring("SC03");
    memset(&r, 0, sizeof(r));
    check(imu_session_configure_check(IMU_SESSION_CHECK_MODE_PROBE,
                                      0u, &r), "SC03", "probe mask zero");
    check(snap().readerState == IMU_READER_STATE_PROBE,
          "SC03", "zero mask is still PROBE");
}

static void test_sc04_readable_match_is_evidence(void)
{
    ImuSessionCheckConfigResult_t r;
    open_configuring("SC04");
    memset(&r, 0, sizeof(r));
    check(imu_session_configure_check(IMU_SESSION_CHECK_MODE_PROBE,
                                      0x05u, &r), "SC04", "configure");
    check(r.actualMaskValid && r.actualMask == 0x05u,
          "SC04", "actual match observed");
}

static void test_sc05_unavailable_readback_is_not_fabricated(void)
{
    ImuSessionCheckConfigResult_t r;
    open_configuring("SC05");
    imu_session_test_set_check_readback(false, 0xffu);
    memset(&r, 0, sizeof(r));
    check(imu_session_configure_check(IMU_SESSION_CHECK_MODE_PROBE,
                                      0x02u, &r), "SC05", "configure succeeds");
    check(!r.actualMaskValid && r.actualMask == 0u,
          "SC05", "no invented actual mask");
}

static void test_sc06_mismatch_fails_but_can_restore(void)
{
    ImuSessionCheckConfigResult_t r;
    ImuSampleSnapshot_t before;
    uint8_t mask = 0xffu;

    open_configuring("SC06");
    before = snap();
    imu_session_test_set_check_readback(true, 0x01u);
    memset(&r, 0, sizeof(r));
    check(!imu_session_configure_check(IMU_SESSION_CHECK_MODE_PROBE,
                                       0x80u, &r), "SC06", "mismatch fails");
    check(r.actualMaskValid && r.actualMask == 0x01u,
          "SC06", "observed mismatch retained");
    check(snap().readerState == IMU_READER_STATE_PROBE &&
          snap().configurationEpoch == before.configurationEpoch + 1u,
          "SC06", "actionable entered mode and epoch");
    check(imu_session_restore_production(0x04u),
          "SC06", "restoration allowed");
    check(imu_session_get_cal_policy(&mask) && mask == 0x04u,
          "SC06", "flight policy restored");
}

static void test_sc07_independent_report_evidence(void)
{
    ImuCheckFacts_t f;
    enter(IMU_SESSION_CHECK_MODE_CHECK, 0u, "SC07");
    imu_session_test_inject_check_report(IMU_SESSION_TEST_REPORT_MAG,
                                          2u, 101ull, 0.0f,
                                          12.0f, -3.0f, 40.0f);
    f = facts();
    check(f.valid && f.haveMag && !f.haveAccel &&
          f.magHostDecodeNs == 101ull && f.magStatus == 2u &&
          f.magXuT == 12.0f && f.magYuT == -3.0f &&
          f.magZuT == 40.0f, "SC07", "mag only");

    imu_session_test_inject_check_report(IMU_SESSION_TEST_REPORT_ACCEL,
                                          3u, 102ull, 0.0f, 0, 0, 0);
    imu_session_test_inject_check_report(IMU_SESSION_TEST_REPORT_GYRO,
                                          0u, 103ull, 0.0f, 0, 0, 0);
    imu_session_test_inject_check_report(IMU_SESSION_TEST_REPORT_RV,
                                          1u, 104ull, 0.25f, 0, 0, 0);
    f = facts();
    check(f.haveAccel && f.haveGyro && f.haveMag && f.haveRv &&
          f.accelHostDecodeNs == 102ull &&
          f.gyroHostDecodeNs == 103ull &&
          f.magHostDecodeNs == 101ull &&
          f.rvHostDecodeNs == 104ull &&
          f.accelStatus == 3u && f.gyroStatus == 0u &&
          f.magStatus == 2u && f.rvStatus == 1u &&
          f.rvErrRad == 0.25f, "SC07", "independent evidence");
}

static void test_sc08_epoch_transition_clears_check_only(void)
{
    ImuCheckFacts_t f;
    uint32_t before;
    enter(IMU_SESSION_CHECK_MODE_CHECK, 0u, "SC08");
    imu_session_test_inject_check_report(IMU_SESSION_TEST_REPORT_MAG,
                                          3u, 100ull, 0, 1, 2, 3);
    before = snap().configurationEpoch;
    check(imu_session_restore_production(0u), "SC08", "restore");
    f = facts();
    check(f.version == IMU_CHECK_FACTS_VERSION && !f.valid &&
          !f.haveMag && f.configurationEpoch == before + 1u,
          "SC08", "prior facts cleared at new epoch");
}

static void test_sc09_restore_check_exact_production_set(void)
{
    ImuSampleSnapshot_t before;
    enter(IMU_SESSION_CHECK_MODE_CHECK, 0u, "SC09");
    before = snap();
    check(imu_session_restore_production(0x04u), "SC09", "restore");
    check(snap().readerState == IMU_READER_STATE_CONFIGURING &&
          snap().configurationEpoch == before.configurationEpoch + 1u &&
          snap().validMask == 0u, "SC09", "one restore epoch");
    report_is(IMU_SESSION_TEST_REPORT_MAG, 0u, "SC09");
    report_is(IMU_SESSION_TEST_REPORT_ACCEL, 0u, "SC09");
    report_is(IMU_SESSION_TEST_REPORT_RV, 10000u, "SC09");
    report_is(IMU_SESSION_TEST_REPORT_LINEAR, 10000u, "SC09");
    report_is(IMU_SESSION_TEST_REPORT_GYRO, 10000u, "SC09");
}

static void test_sc10_restore_probe_matches_check(void)
{
    ImuSampleSnapshot_t before;
    enter(IMU_SESSION_CHECK_MODE_PROBE, 0x80u, "SC10");
    before = snap();
    check(imu_session_restore_production(0u), "SC10", "restore");
    check(snap().readerState == IMU_READER_STATE_CONFIGURING &&
          snap().configurationEpoch == before.configurationEpoch + 1u &&
          !facts().valid, "SC10", "probe restored, facts cleared");
}

static void test_sc11_illegal_states_and_inputs(void)
{
    ImuSessionCheckConfigResult_t r;
    ImuSampleSnapshot_t before;
    imu_session_test_reset();
    memset(&r, 0, sizeof(r));
    check(!imu_session_configure_check(IMU_SESSION_CHECK_MODE_CHECK,
                                       0u, &r), "SC11", "closed illegal");
    open_configuring("SC11");
    before = snap();
    check(!imu_session_configure_check(IMU_SESSION_CHECK_MODE_CHECK,
                                       1u, &r), "SC11", "check mask illegal");
    check(!imu_session_configure_check((ImuSessionCheckMode_t)99,
                                       0u, &r), "SC11", "mode illegal");
    check(snap().configurationEpoch == before.configurationEpoch &&
          snap().readerState == IMU_READER_STATE_CONFIGURING,
          "SC11", "no invented transition");
    check(imu_session_configure_production(0u), "SC11", "production");
    check(imu_session_begin_settle(), "SC11", "settle");
    check(!imu_session_configure_check(IMU_SESSION_CHECK_MODE_PROBE,
                                       0u, &r), "SC11", "settling illegal");
    check(imu_session_mark_operational(), "SC11", "operational");
    check(!imu_session_restore_production(0u),
          "SC11", "restore from OPERATIONAL illegal");
}

static void test_sc12_hard_configuration_failure_is_truthful(void)
{
    ImuSessionCheckConfigResult_t r;
    ImuSampleSnapshot_t before;
    open_configuring("SC12");
    before = snap();
    imu_session_test_set_configure_check_result(false);
    memset(&r, 0, sizeof(r));
    check(!imu_session_configure_check(IMU_SESSION_CHECK_MODE_PROBE,
                                       0x02u, &r), "SC12", "hard failure");
    check(snap().readerState == IMU_READER_STATE_FAULTED &&
          snap().configurationEpoch == before.configurationEpoch &&
          !r.actualMaskValid, "SC12", "no false entry or readback");
    check(!imu_session_restore_production(0u),
          "SC12", "faulted session cannot claim restore");
}

int main(void)
{
    test_sc01_exact_report_configuration();
    test_sc02_check_enters_check_with_zero_mask();
    test_sc03_probe_preserves_full_mask_including_zero();
    test_sc04_readable_match_is_evidence();
    test_sc05_unavailable_readback_is_not_fabricated();
    test_sc06_mismatch_fails_but_can_restore();
    test_sc07_independent_report_evidence();
    test_sc08_epoch_transition_clears_check_only();
    test_sc09_restore_check_exact_production_set();
    test_sc10_restore_probe_matches_check();
    test_sc11_illegal_states_and_inputs();
    test_sc12_hard_configuration_failure_is_truthful();
    if (failures != 0) {
        fprintf(stderr, "test_session_check: %d failure(s)\n", failures);
        return 1;
    }
    puts("test_session_check: pass");
    return 0;
}
