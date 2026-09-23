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

static bool enter_tare(const char *id)
{
    open_configuring(id);
    if (!imu_session_configure_tare()) {
        check(false, id, "configure tare");
        return false;
    }
    return true;
}

static void test_e07_tare_now_increments_epoch(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after_now;
    ImuSampleSnapshot_t after_persist;

    check(enter_tare("E07"), "E07", "enter tare");
    before = snap();
    check(before.readerState == IMU_READER_STATE_TARE, "E07", "in TARE");
    check(imu_session_tare_now(IMU_SESSION_TARE_AXES_Z), "E07", "tare-now");
    after_now = snap();
    check(after_now.readerState == IMU_READER_STATE_TARE, "E07",
          "remains TARE");
    check(after_now.configurationEpoch == before.configurationEpoch + 1u,
          "E07", "tare-now epoch +1");
    check(after_now.validMask == 0u, "E07", "validity cleared");
    check(imu_session_persist_tare(), "E07", "persist");
    after_persist = snap();
    check(after_persist.configurationEpoch == after_now.configurationEpoch,
          "E07", "persist does not increment");
    check(after_persist.readerState == IMU_READER_STATE_TARE, "E07",
          "persist remains TARE");
}

static void test_e08_no_mutation_leaves_epoch(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;

    open_configuring("E08");
    before = snap();
    after = snap();
    check(after.configurationEpoch == before.configurationEpoch, "E08",
          "no configure/tare leaves epoch");
    check(after.readerState == IMU_READER_STATE_CONFIGURING, "E08",
          "still CONFIGURING");
    check(!imu_session_test_have_last_tare_now(), "E08", "no tare-now");
}

static void test_st01_z_axis_mapping(void)
{
    check(enter_tare("ST01"), "ST01", "enter tare");
    check(imu_session_tare_now(IMU_SESSION_TARE_AXES_Z), "ST01", "Z tare-now");
    check(imu_session_test_have_last_tare_now(), "ST01", "captured mapping");
    check(imu_session_test_last_tare_axes() == 0x04u, "ST01", "SH2_TARE_Z");
}

static void test_st02_full_axis_mapping(void)
{
    check(enter_tare("ST02"), "ST02", "enter tare");
    check(imu_session_tare_now(IMU_SESSION_TARE_AXES_FULL), "ST02",
          "full tare-now");
    check(imu_session_test_last_tare_axes() == 0x07u, "ST02",
          "SH2_TARE_X|Y|Z");
}

static void test_st03_rotation_vector_basis(void)
{
    check(enter_tare("ST03"), "ST03", "enter tare");
    check(imu_session_tare_now(IMU_SESSION_TARE_AXES_Z), "ST03", "tare-now");
    check(imu_session_test_last_tare_basis() == 0u, "ST03",
          "SH2_TARE_BASIS_ROTATION_VECTOR");
}

static void test_st04_configure_failure(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;

    open_configuring("ST04");
    before = snap();
    imu_session_test_set_configure_tare_result(false);
    check(!imu_session_configure_tare(), "ST04", "configure fails");
    after = snap();
    check(after.readerState == IMU_READER_STATE_FAULTED, "ST04", "faulted");
    check(after.configurationEpoch == before.configurationEpoch, "ST04",
          "no epoch on configure fail");
}

static void test_st05_tare_now_failure_no_epoch(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;

    check(enter_tare("ST05"), "ST05", "enter tare");
    before = snap();
    imu_session_test_set_tare_now_result(false);
    check(!imu_session_tare_now(IMU_SESSION_TARE_AXES_Z), "ST05",
          "tare-now fails");
    after = snap();
    check(after.readerState == IMU_READER_STATE_TARE, "ST05", "still TARE");
    check(after.configurationEpoch == before.configurationEpoch, "ST05",
          "no epoch on tare-now fail");
}

static void test_st06_persist_failure_no_epoch(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;

    check(enter_tare("ST06"), "ST06", "enter tare");
    check(imu_session_tare_now(IMU_SESSION_TARE_AXES_Z), "ST06", "tare-now");
    before = snap();
    imu_session_test_set_persist_tare_result(false);
    check(!imu_session_persist_tare(), "ST06", "persist fails");
    after = snap();
    check(after.configurationEpoch == before.configurationEpoch, "ST06",
          "persist fail does not increment");
    check(after.readerState == IMU_READER_STATE_TARE, "ST06", "still TARE");
}

static void test_st07_clear_success_one_epoch(void)
{
    ImuSessionClearTareResult_t clear;
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;

    check(enter_tare("ST07"), "ST07", "enter tare");
    before = snap();
    check(imu_session_clear_tare(&clear), "ST07", "clear");
    after = snap();
    check(clear.success, "ST07", "clear success");
    check(clear.clearActive == IMU_SESSION_TARE_SUB_SUCCEEDED, "ST07",
          "active succeeded");
    check(clear.clearSaved == IMU_SESSION_TARE_SUB_SUCCEEDED, "ST07",
          "saved succeeded");
    check(after.configurationEpoch == before.configurationEpoch + 1u, "ST07",
          "clear epoch +1");
    check(after.readerState == IMU_READER_STATE_TARE, "ST07", "still TARE");
}

static void test_st08_clear_failure_no_epoch(void)
{
    ImuSessionClearTareResult_t clear;
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;

    check(enter_tare("ST08"), "ST08", "enter tare");
    before = snap();
    imu_session_test_set_clear_tare_result(false);
    check(!imu_session_clear_tare(&clear), "ST08", "clear fails");
    after = snap();
    check(!clear.success, "ST08", "clear unsuccessful");
    check(clear.clearActive == IMU_SESSION_TARE_SUB_FAILED, "ST08",
          "active failed");
    check(clear.clearSaved == IMU_SESSION_TARE_SUB_FAILED, "ST08",
          "saved failed");
    check(after.configurationEpoch == before.configurationEpoch, "ST08",
          "no epoch on clear fail");
}

static void test_st09_restore_from_tare(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;
    uint8_t mask = 0xffu;

    check(enter_tare("ST09"), "ST09", "enter tare");
    before = snap();
    check(imu_session_restore_production(0x00u), "ST09", "restore");
    after = snap();
    check(after.readerState == IMU_READER_STATE_CONFIGURING, "ST09",
          "CONFIGURING");
    check(after.configurationEpoch == before.configurationEpoch + 1u, "ST09",
          "restore epoch +1");
    check(after.validMask == 0u, "ST09", "validity cleared");
    check(imu_session_get_cal_policy(&mask), "ST09", "read policy");
    check(mask == 0x00u, "ST09", "flight mask restored");
    check(!imu_session_test_recovery_observed(), "ST09", "not recovery");
    check(imu_session_test_recovery_attempt_count() == 0u, "ST09",
          "no recovery attempt");
}

static void test_st10_restore_failure(void)
{
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;

    check(enter_tare("ST10"), "ST10", "enter tare");
    before = snap();
    imu_session_test_set_production_result(false);
    check(!imu_session_restore_production(0x00u), "ST10", "restore fails");
    after = snap();
    check(after.readerState == IMU_READER_STATE_FAULTED, "ST10", "faulted");
    check(after.configurationEpoch == before.configurationEpoch, "ST10",
          "no epoch on restore fail");
    check(!imu_session_test_recovery_observed(), "ST10",
          "no synthetic recovery");
}

static void test_st11_illegal_states(void)
{
    imu_session_test_reset();
    check(!imu_session_configure_tare(), "ST11", "configure from CLOSED");
    check(!imu_session_tare_now(IMU_SESSION_TARE_AXES_Z), "ST11",
          "tare-now from CLOSED");
    check(!imu_session_persist_tare(), "ST11", "persist from CLOSED");
    {
        ImuSessionClearTareResult_t clear;
        check(!imu_session_clear_tare(&clear), "ST11", "clear from CLOSED");
    }

    open_configuring("ST11");
    check(imu_session_test_force_state(IMU_READER_STATE_OPENING), "ST11",
          "force OPENING");
    check(!imu_session_configure_tare(), "ST11", "configure from OPENING");

    open_configuring("ST11");
    check(imu_session_configure_production(0x00u), "ST11", "production");
    check(imu_session_begin_settle(), "ST11", "settle");
    check(!imu_session_configure_tare(), "ST11", "configure from SETTLING");
    check(!imu_session_tare_now(IMU_SESSION_TARE_AXES_Z), "ST11",
          "tare-now from SETTLING");
    check(imu_session_mark_operational(), "ST11", "operational");
    check(!imu_session_configure_tare(), "ST11",
          "configure from OPERATIONAL");
    check(!imu_session_restore_production(0x00u), "ST11",
          "restore from OPERATIONAL");
}

static void test_st12_facts_current_epoch_only(void)
{
    ImuTareFacts_t in;
    ImuTareFacts_t got;
    ImuSampleSnapshot_t s;

    check(enter_tare("ST12"), "ST12", "enter tare");
    s = snap();
    memset(&in, 0, sizeof(in));
    in.version = IMU_TARE_FACTS_VERSION;
    in.valid = true;
    in.quatReal = 1.0f;
    in.yawRad = 0.25f;
    imu_session_test_inject_tare_facts(&in);
    check(imu_session_get_tare_facts(&got), "ST12", "get facts");
    check(got.valid, "ST12", "valid");
    check(got.configurationEpoch == s.configurationEpoch, "ST12",
          "facts stamped with current epoch");
    check(got.yawRad == 0.25f, "ST12", "yaw retained");
    check(s.validMask == 0u, "ST12", "R2 unchanged by tare mailbox");
}

static void test_st13_stale_facts_rejected_after_tare_now(void)
{
    ImuTareFacts_t in;
    ImuTareFacts_t got;
    uint32_t epoch_before;

    check(enter_tare("ST13"), "ST13", "enter tare");
    memset(&in, 0, sizeof(in));
    in.version = IMU_TARE_FACTS_VERSION;
    in.valid = true;
    in.quatReal = 1.0f;
    imu_session_test_inject_tare_facts(&in);
    check(imu_session_get_tare_facts(&got), "ST13", "pre facts");
    check(got.valid, "ST13", "pre-tare facts valid");
    epoch_before = snap().configurationEpoch;
    check(imu_session_tare_now(IMU_SESSION_TARE_AXES_Z), "ST13", "tare-now");
    check(snap().configurationEpoch == epoch_before + 1u, "ST13", "epoch +1");
    check(imu_session_get_tare_facts(&got), "ST13", "post facts");
    check(!got.valid, "ST13", "pre-tare facts invalidated");
    check(got.configurationEpoch == snap().configurationEpoch, "ST13",
          "mailbox epoch follows tare-now");
}

int main(void)
{
    test_e07_tare_now_increments_epoch();
    test_e08_no_mutation_leaves_epoch();
    test_st01_z_axis_mapping();
    test_st02_full_axis_mapping();
    test_st03_rotation_vector_basis();
    test_st04_configure_failure();
    test_st05_tare_now_failure_no_epoch();
    test_st06_persist_failure_no_epoch();
    test_st07_clear_success_one_epoch();
    test_st08_clear_failure_no_epoch();
    test_st09_restore_from_tare();
    test_st10_restore_failure();
    test_st11_illegal_states();
    test_st12_facts_current_epoch_only();
    test_st13_stale_facts_rejected_after_tare_now();

    if (g_fail != 0) {
        fprintf(stderr, "test_session_tare: %d failure(s)\n", g_fail);
        return 1;
    }
    printf("test_session_tare: pass\n");
    return 0;
}
