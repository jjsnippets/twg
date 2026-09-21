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

static void test_configure_cal_increments_epoch(void)
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

static void test_cal_facts_not_in_r2(void)
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

static void test_save_dcd_no_epoch(void)
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

static void test_verify_reopen_not_recovery(void)
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

static void test_verify_reopen_failure_is_recovery(void)
{
    open_configuring("S05");
    check(imu_session_configure_calibration(0x07u), "S05", "configure");
    imu_session_test_set_reopen_result(false);
    check(!imu_session_begin_verification_reopen(), "S05", "reopen fail");
    check(imu_session_test_recovery_attempt_count() >= 1u, "S05", "recovery attempt counted");
}

static void test_policy_roundtrip(void)
{
    uint8_t mask = 0u;

    open_configuring("S06");
    check(imu_session_configure_calibration(0x07u), "S06", "configure 0x07");
    check(imu_session_get_cal_policy(&mask), "S06", "get policy");
    check(mask == 0x07u, "S06", "mask 0x07");
}

static void test_illegal_from_closed(void)
{
    imu_session_test_reset();
    check(!imu_session_configure_calibration(0x07u), "S07", "cal from CLOSED");
    check(!imu_session_save_dcd(), "S07", "save from CLOSED");
    check(!imu_session_begin_verification_reopen(), "S07", "reopen from CLOSED");
}

int main(void)
{
    test_configure_cal_increments_epoch();
    test_cal_facts_not_in_r2();
    test_save_dcd_no_epoch();
    test_verify_reopen_not_recovery();
    test_verify_reopen_failure_is_recovery();
    test_policy_roundtrip();
    test_illegal_from_closed();
    if (g_fail != 0) {
        fprintf(stderr, "test_session_cal: %d failures\n", g_fail);
        return 1;
    }
    printf("test_session_cal: pass\n");
    return 0;
}