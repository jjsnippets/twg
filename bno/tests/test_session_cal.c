#define _POSIX_C_SOURCE 200809L
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_session.h"

static int g_fail;

static void check(bool ok, const char *id, const char *what)
{
    if (!ok) {
        fprintf(stderr, "FAIL %s: %s\n", id, what);
        g_fail++;
    }
}

static void open_configuring(const char *id)
{
    imu_session_test_reset();
    check(imu_session_test_open(true), id, "open");
}

static ImuSampleSnapshot_t snap(void)
{
    ImuSampleSnapshot_t s;
    memset(&s, 0, sizeof(s));
    (void)imu_session_get_snapshot(&s);
    return s;
}

static void test_s01_configure_cal_increments_epoch(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;

    open_configuring("S01");
    before = snap();
    check(before.readerState == IMU_READER_STATE_CONFIGURING, "S01", "configuring");
    check(imu_session_configure_calibration(0x07u), "S01", "configure cal");
    after = snap();
    check(after.readerState == IMU_READER_STATE_CALIBRATION, "S01", "state CALIBRATION");
    check(after.configurationEpoch == before.configurationEpoch + 1u, "S01", "epoch +1");
    check(after.validMask == 0u, "S01", "validMask cleared");
}

static void test_s02_cal_facts_not_in_r2(void)
{
    ImuCalFacts_t facts;
    ImuCalFacts_t got;
    ImuSampleSnapshot_t s;

    open_configuring("S02");
    check(imu_session_configure_calibration(0x07u), "S02", "configure");

    memset(&facts, 0, sizeof(facts));
    facts.version = IMU_CAL_FACTS_VERSION;
    facts.accelAccuracy = 2u;
    facts.gyroAccuracy = 3u;
    facts.magAccuracy = 2u;
    facts.rvAccuracy = 1u;
    facts.rvErrRad = 0.20f;
    facts.haveMag = true;
    facts.magXuT = 12.5f;
    facts.magYuT = -3.0f;
    facts.magZuT = 40.0f;
    facts.valid = true;
    imu_session_test_inject_cal_facts(&facts);

    check(imu_session_get_cal_facts(&got), "S02", "get cal facts");
    check(got.magAccuracy == 2u, "S02", "mag accuracy");
    check(got.haveMag, "S02", "have mag");
    check(got.magXuT == 12.5f, "S02", "mag x");

    s = snap();
    check(s.validMask == 0u, "S02", "R2 unchanged by cal mailbox");
}

static void test_s03_save_dcd_no_epoch(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;

    open_configuring("S03");
    check(imu_session_configure_calibration(0x07u), "S03", "configure");
    before = snap();
    imu_session_test_set_save_dcd_result(true);
    check(imu_session_save_dcd(), "S03", "save ok");
    after = snap();
    check(after.configurationEpoch == before.configurationEpoch, "S03", "save does not increment epoch");
    check(after.readerState == IMU_READER_STATE_CALIBRATION, "S03", "still CALIBRATION");

    imu_session_test_set_save_dcd_result(false);
    check(!imu_session_save_dcd(), "S03", "save fail");
}

static void test_s04_verify_reopen_not_recovery(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;

    open_configuring("S04");
    check(imu_session_configure_calibration(0x07u), "S04", "configure");
    before = snap();
    imu_session_test_set_reopen_result(true);
    check(imu_session_begin_verification_reopen(), "S04", "verify reopen");
    after = snap();
    check(after.readerState == IMU_READER_STATE_CONFIGURING, "S04", "back to CONFIGURING");
    check(after.configurationEpoch == before.configurationEpoch + 1u, "S04", "epoch +1 once");
    check(after.validMask == 0u, "S04", "validity cleared");
    check(!imu_session_test_recovery_observed(), "S04", "not recovery");
    check(imu_session_test_recovery_attempt_count() == 0u, "S04", "no recovery attempt");
}

static void test_s05_verify_reopen_failure_is_recovery(void)
{
    open_configuring("S05");
    check(imu_session_configure_calibration(0x07u), "S05", "configure");
    imu_session_test_set_reopen_result(false);
    check(!imu_session_begin_verification_reopen(), "S05", "reopen fail");
    check(imu_session_test_recovery_attempt_count() >= 1u, "S05", "recovery attempt counted");
}

static void test_s06_policy_roundtrip(void)
{
    uint8_t mask = 0u;

    open_configuring("S06");
    check(imu_session_configure_calibration(0x07u), "S06", "configure 0x07");
    check(imu_session_get_cal_policy(&mask), "S06", "get policy");
    check(mask == 0x07u, "S06", "mask 0x07");
}

static void test_s07_illegal_from_closed(void)
{
    imu_session_test_reset();
    check(!imu_session_configure_calibration(0x07u), "S07", "cal from CLOSED");
    check(!imu_session_save_dcd(), "S07", "save from CLOSED");
    check(!imu_session_begin_verification_reopen(), "S07", "reopen from CLOSED");
}

static void test_s08_restore_from_calibration(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;
    uint8_t mask = 0xffu;

    open_configuring("S08");
    check(imu_session_configure_calibration(0x07u), "S08", "configure cal");
    before = snap();
    check(imu_session_restore_production(0x00u), "S08", "restore production");
    after = snap();
    check(after.readerState == IMU_READER_STATE_CONFIGURING,
          "S08", "back to CONFIGURING");
    check(after.configurationEpoch == before.configurationEpoch + 1u,
          "S08", "epoch +1 once");
    check(after.validMask == 0u, "S08", "validity cleared");
    check(imu_session_get_cal_policy(&mask), "S08", "read production policy");
    check(mask == 0x00u, "S08", "production policy applied");
}

static void test_s09_restore_clears_cal_facts(void)
{
    ImuCalFacts_t facts;
    ImuCalFacts_t got;

    open_configuring("S09");
    check(imu_session_configure_calibration(0x07u), "S09", "configure cal");

    memset(&facts, 0, sizeof(facts));
    facts.version = IMU_CAL_FACTS_VERSION;
    facts.valid = true;
    facts.magAccuracy = 2u;
    imu_session_test_inject_cal_facts(&facts);

    check(imu_session_restore_production(0x00u), "S09", "restore production");
    check(imu_session_get_cal_facts(&got), "S09", "get cleared facts");
    check(got.version == IMU_CAL_FACTS_VERSION,
          "S09", "facts version retained");
    check(!got.valid, "S09", "facts invalidated");
    check(got.configurationEpoch == snap().configurationEpoch,
          "S09", "facts epoch updated");
}

static void test_s10_restore_after_verify_reopen(void)
{
    ImuSampleSnapshot_t afterReopen;
    ImuSampleSnapshot_t afterRestore;

    open_configuring("S10");
    check(imu_session_configure_calibration(0x07u), "S10", "configure cal");
    check(imu_session_begin_verification_reopen(), "S10", "verify reopen");
    afterReopen = snap();

    check(imu_session_restore_production(0x00u), "S10", "restore production");
    afterRestore = snap();
    check(afterRestore.configurationEpoch == afterReopen.configurationEpoch + 1u,
          "S10", "restore takes one epoch");
    check(!imu_session_test_recovery_observed(),
          "S10", "restore is not recovery");
}

static void test_s11_restore_failure_faults_without_epoch(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;

    open_configuring("S11");
    check(imu_session_configure_calibration(0x07u), "S11", "configure cal");
    before = snap();

    imu_session_test_set_production_result(false);
    check(!imu_session_restore_production(0x00u), "S11", "restore fails");
    after = snap();
    check(after.readerState == IMU_READER_STATE_FAULTED, "S11", "faulted");
    check(after.configurationEpoch == before.configurationEpoch,
          "S11", "no claimed epoch change");
    check(!imu_session_test_recovery_observed(),
          "S11", "no synthetic recovery");
}

static void test_s12_restore_illegal_states(void)
{
    imu_session_test_reset();
    check(!imu_session_restore_production(0x00u),
          "S12", "restore from CLOSED");

    open_configuring("S12");
    check(imu_session_configure_production(0x00u),
          "S12", "initial production configure");
    check(imu_session_begin_settle(), "S12", "begin settle");
    check(imu_session_mark_operational(), "S12", "mark operational");
    check(!imu_session_restore_production(0x00u),
          "S12", "restore from OPERATIONAL");
}

static void test_s13_clear_rejects_illegal_states(void)
{
    imu_session_test_reset();
    check(!imu_session_clear_dcd(), "S13", "closed rejected");
    check(imu_session_test_dcd_flash_delete_attempts() == 0u,
          "S13", "closed did not touch flash");

    open_configuring("S13");
    check(imu_session_configure_production(0u),
          "S13", "configure production");
    check(imu_session_begin_settle(), "S13", "settle");
    check(imu_session_mark_operational(), "S13", "operational");
    check(!imu_session_clear_dcd(), "S13", "operational rejected");
    check(imu_session_test_dcd_flash_delete_attempts() == 0u,
          "S13", "operational did not touch flash");
}

static void test_s14_clear_uses_one_owner_and_one_epoch(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;

    open_configuring("S14");
    before = snap();
    check(imu_session_clear_dcd(), "S14", "delete/reset/reopen");
    after = snap();
    check(imu_session_test_dcd_flash_delete_attempts() == 1u,
          "S14", "delete attempted once");
    check(imu_session_test_dcd_clear_reset_attempts() == 1u,
          "S14", "reset attempted once");
    check(after.readerState == IMU_READER_STATE_CONFIGURING,
          "S14", "same session owner returns configuring");
    check(after.configurationEpoch == before.configurationEpoch + 1u,
          "S14", "one real recovery epoch");
    check(after.validMask == 0u, "S14", "validity cleared");
}

static void test_s15_clear_failure_does_not_claim_reset(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;

    open_configuring("S15");
    before = snap();
    imu_session_test_set_clear_dcd_results(false, true);
    check(!imu_session_clear_dcd(), "S15", "flash delete fails");
    after = snap();
    check(imu_session_test_dcd_flash_delete_attempts() == 1u,
          "S15", "delete attempted");
    check(imu_session_test_dcd_clear_reset_attempts() == 0u,
          "S15", "reset not issued");
    check(after.configurationEpoch == before.configurationEpoch,
          "S15", "no reset epoch");
    check(after.readerState == IMU_READER_STATE_CONFIGURING,
          "S15", "still actionable");

    imu_session_test_reset();
    open_configuring("S15");
    before = snap();
    imu_session_test_set_clear_dcd_results(true, false);
    check(!imu_session_clear_dcd(), "S15", "reset command fails");
    after = snap();
    check(imu_session_test_dcd_flash_delete_attempts() == 1u &&
          imu_session_test_dcd_clear_reset_attempts() == 1u,
          "S15", "delete then reset attempted in order");
    check(after.configurationEpoch == before.configurationEpoch,
          "S15", "no accepted reset/reopen epoch");
    check(after.readerState == IMU_READER_STATE_CONFIGURING,
          "S15", "restoration can be attempted");
}

static void test_s16_clear_reopen_failure_is_unusable(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;

    open_configuring("S16");
    before = snap();
    imu_session_test_set_reopen_result(false);
    check(!imu_session_clear_dcd(), "S16", "reopen fails");
    after = snap();
    check(imu_session_test_dcd_flash_delete_attempts() == 1u &&
          imu_session_test_dcd_clear_reset_attempts() == 1u,
          "S16", "both clear calls preceded failed reopen");
    check(after.readerState == IMU_READER_STATE_FAULTED,
          "S16", "session faulted");
    check(after.configurationEpoch == before.configurationEpoch + 1u,
          "S16", "one recovery epoch, not zero or two");
    check(!imu_session_restore_production(0u),
          "S16", "cannot claim production restoration");
}

int main(void)
{
    test_s01_configure_cal_increments_epoch();
    test_s02_cal_facts_not_in_r2();
    test_s03_save_dcd_no_epoch();
    test_s04_verify_reopen_not_recovery();
    test_s05_verify_reopen_failure_is_recovery();
    test_s06_policy_roundtrip();
    test_s07_illegal_from_closed();
    test_s08_restore_from_calibration();
    test_s09_restore_clears_cal_facts();
    test_s10_restore_after_verify_reopen();
    test_s11_restore_failure_faults_without_epoch();
    test_s12_restore_illegal_states();
    test_s13_clear_rejects_illegal_states();
    test_s14_clear_uses_one_owner_and_one_epoch();
    test_s15_clear_failure_does_not_claim_reset();
    test_s16_clear_reopen_failure_is_unusable();

    if (g_fail != 0) {
        fprintf(stderr, "test_session_cal: %d failures\n", g_fail);
        return 1;
    }
    printf("test_session_cal: pass\n");
    return 0;
}