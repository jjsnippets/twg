#define _POSIX_C_SOURCE 200809L

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_session.h"

/*
 * Phase 2 host-only oracles for 1I families C/E/X, as far as R1/R2/R3
 * can observe them. No SPI. No publisher masks.
 *
 * Deferred (not implemented here):
 *   C01-C04 Then-clauses for fresh/stale/multi (Phase 9 publisher)
 *   E04/E11 publisher baseline
 *   E05-E08 probe/tare/q (Phases 5-7)
 *   E09/E10 R4 flags
 *   X03 R9 / integration prev-copy
 *   Family D device-sequence gap flags
 */

static int g_fail;

static void check(bool ok, const char *id, const char *what)
{
    if (!ok) {
        fprintf(stderr, "FAIL %s: %s\n", id, what);
        g_fail++;
    }
}

static void check_u32(uint32_t got, uint32_t want, const char *id, const char *what)
{
    if (got != want) {
        fprintf(stderr, "FAIL %s: %s (got %" PRIu32 " want %" PRIu32 ")\n",
                id, what, got, want);
        g_fail++;
    }
}

static void check_u64(uint64_t got, uint64_t want, const char *id, const char *what)
{
    if (got != want) {
        fprintf(stderr, "FAIL %s: %s (got %" PRIu64 " want %" PRIu64 ")\n",
                id, what, got, want);
        g_fail++;
    }
}

#include <inttypes.h>

static bool snap(ImuSampleSnapshot_t *out)
{
    return imu_session_get_snapshot(out) &&
           out->version == IMU_SAMPLE_CONTRACT_VERSION;
}

static ImuSessionTestGroupEvent_t make_rot(uint64_t seq, uint64_t hostNs,
                                           float yaw)
{
    ImuSessionTestGroupEvent_t e;

    memset(&e, 0, sizeof(e));
    e.groupBit = IMU_GROUP_BIT_ROTATION;
    e.groupEventSeq = seq;
    e.hostDecodeNs = hostNs;
    e.sensorTimeUs = 1000;
    e.qw = 1.0f;
    e.yaw = yaw;
    return e;
}

static ImuSessionTestGroupEvent_t make_acc(uint64_t seq, uint64_t hostNs)
{
    ImuSessionTestGroupEvent_t e;

    memset(&e, 0, sizeof(e));
    e.groupBit = IMU_GROUP_BIT_ACCEL;
    e.groupEventSeq = seq;
    e.hostDecodeNs = hostNs;
    e.sensorTimeUs = 2000;
    e.ax = 0.1f;
    return e;
}

static bool bring_up_operational(void)
{
    imu_session_test_reset();
    if (!imu_session_test_open(true)) {
        return false;
    }
    if (!imu_session_configure_production(0)) {
        return false;
    }
    if (!imu_session_begin_settle()) {
        return false;
    }
    return imu_session_mark_operational();
}

static void test_e01(void)
{
    ImuSampleSnapshot_t s;

    imu_session_test_reset();
    check(imu_session_test_open(true), "E01", "open success");
    check(snap(&s), "E01", "snapshot");
    check(s.readerState == IMU_READER_STATE_CONFIGURING, "E01", "CONFIGURING");
    check_u32(s.configurationEpoch, 1u, "E01", "epoch");
    check_u32(s.validMask, 0u, "E01", "validMask");
}

static void test_e02(void)
{
    ImuSampleSnapshot_t s;

    imu_session_test_reset();
    check(!imu_session_test_open(false), "E02", "open fails");
    check(snap(&s), "E02", "snapshot");
    check(s.readerState == IMU_READER_STATE_FAULTED, "E02", "FAULTED");
    check_u32(s.configurationEpoch, IMU_EPOCH_NONE, "E02", "epoch 0");
}

static void test_e03_e04(void)
{
    ImuSampleSnapshot_t s;

    imu_session_test_reset();
    check(imu_session_test_open(true), "E03", "open");
    check(imu_session_configure_production(0), "E03", "configure");
    check(snap(&s), "E03", "snapshot after configure");
    check_u32(s.configurationEpoch, 1u, "E03", "epoch stays 1");
    check(imu_session_begin_settle(), "E03", "begin_settle");
    check(snap(&s), "E03", "snapshot settle");
    check(s.readerState == IMU_READER_STATE_SETTLING, "E03", "SETTLING");
    check_u32(s.configurationEpoch, 1u, "E03", "epoch still 1");

    check(imu_session_mark_operational(), "E04", "mark_operational");
    check(snap(&s), "E04", "snapshot operational");
    check(s.readerState == IMU_READER_STATE_OPERATIONAL, "E04", "OPERATIONAL");
    check_u32(s.configurationEpoch, 1u, "E04", "epoch unchanged");
}

static void test_c01_c04(void)
{
    ImuSampleSnapshot_t s;
    ImuSessionTestGroupEvent_t e;

    check(bring_up_operational(), "C01", "bring-up");

    e = make_rot(10, 1000000ull, 1.0f);
    imu_session_test_inject_group(&e);
    check(snap(&s), "C01", "snapshot");
    check_u64(s.rotationMeta.groupEventSeq, 10ull, "C01", "seq 10");
    check(snap(&s), "C01", "second copy");
    check_u64(s.rotationMeta.groupEventSeq, 10ull, "C01", "seq unchanged");

    e = make_rot(11, 2000000ull, 1.0f);
    imu_session_test_inject_group(&e);
    check(snap(&s), "C02", "snapshot");
    check_u64(s.rotationMeta.groupEventSeq, 11ull, "C02", "seq +1");

    e = make_rot(14, 3000000ull, 1.0f);
    imu_session_test_inject_group(&e);
    check(snap(&s), "C03", "snapshot");
    check_u64(s.rotationMeta.groupEventSeq, 14ull, "C03", "seq +3 from 11");

    imu_session_test_reset();
    check(bring_up_operational(), "C04", "bring-up");
    e = make_rot(UINT64_MAX - 4ull, 4000000ull, 0.0f);
    imu_session_test_inject_group(&e);
    e = make_rot(UINT64_MAX - 3ull, 5000000ull, 0.0f);
    imu_session_test_inject_group(&e);
    check(snap(&s), "C04", "snapshot");
    check_u64(s.rotationMeta.groupEventSeq, UINT64_MAX - 3ull,
              "C04", "large 64-bit seq comparable");
}

static void test_c05(void)
{
    ImuSampleSnapshot_t s;
    ImuSessionTestGroupEvent_t e;
    uint64_t count;

    check(bring_up_operational(), "C05", "bring-up");
    e = make_rot(50, 1000000ull, 0.2f);
    imu_session_test_inject_group(&e);
    check(snap(&s), "C05", "after 50");
    count = s.processDecodeCount;
    check_u64(s.rotationMeta.groupEventSeq, 50ull, "C05", "adopted 50");

    e = make_rot(7, 2000000ull, 0.3f);
    imu_session_test_inject_group(&e);
    check(snap(&s), "C05", "after backward 7");
    check_u64(s.rotationMeta.groupEventSeq, 50ull, "C05", "seq not adopted");
    check_u64(s.processDecodeCount, count, "C05", "decode count unchanged");
}

static void test_c06(void)
{
    ImuSampleSnapshot_t s;
    ImuSessionTestGroupEvent_t e;

    check(bring_up_operational(), "C06", "bring-up");
    e = make_rot(IMU_GROUP_EVENT_SEQ_NONE, 1000000ull, 0.0f);
    imu_session_test_inject_group(&e);
    check(snap(&s), "C06", "snapshot");
    check((s.validMask & IMU_GROUP_BIT_ROTATION) == 0, "C06", "valid not set");
    check_u64(s.rotationMeta.groupEventSeq, IMU_GROUP_EVENT_SEQ_NONE,
              "C06", "seq 0 not a valid identity");
}

static void test_c07(void)
{
    ImuSampleSnapshot_t s;
    ImuSessionTestGroupEvent_t e;

    check(bring_up_operational(), "C07", "bring-up");
    e = make_rot(7, 1000000ull, 0.0f);
    imu_session_test_inject_group(&e);
    e = make_acc(1, 2000000ull);
    imu_session_test_inject_group(&e);
    check(snap(&s), "C07", "snapshot");
    check_u64(s.rotationMeta.groupEventSeq, 7ull, "C07", "RV seq unchanged");
    check_u64(s.accelMeta.groupEventSeq, 1ull, "C07", "accel seq");
    check_u64(s.processDecodeCount, 2ull, "C07", "process decode count");
}

static void test_c08(void)
{
    ImuSampleSnapshot_t s;
    ImuSessionTestGroupEvent_t e;

    check(bring_up_operational(), "C08", "bring-up");
    e = make_rot(1, 1000000ull, 1.25f);
    imu_session_test_inject_group(&e);
    e = make_rot(2, 2000000ull, 1.25f);
    imu_session_test_inject_group(&e);
    check(snap(&s), "C08", "snapshot");
    check_u64(s.rotationMeta.groupEventSeq, 2ull, "C08", "seq +1 despite equal yaw");
}

static void test_e09_e13(void)
{
    ImuSampleSnapshot_t s;
    ImuSessionTestGroupEvent_t e;

    check(bring_up_operational(), "E09", "bring-up");
    e = make_rot(80, 1000000ull, 0.0f);
    imu_session_test_inject_group(&e);
    check(snap(&s), "E13", "before reset");
    check_u64(s.processDecodeCount, 1ull, "E13", "count before");
    check_u64(s.rotationMeta.groupEventSeq, 80ull, "E13", "seq before");

    imu_session_test_inject_reset();
    check(snap(&s), "E09", "after reset");
    check(s.readerState == IMU_READER_STATE_RECOVERING, "E09", "RECOVERING");
    check_u32(s.configurationEpoch, 2u, "E09", "epoch +1");
    check_u32(s.validMask, 0u, "E09", "valid cleared");
    check_u64(s.rotationMeta.groupEventSeq, 80ull, "E13", "seq retained");
    check_u64(s.processDecodeCount, 1ull, "E13", "count retained");

    check(imu_session_begin_recovery(), "E10", "reopen");
    check(snap(&s), "E10", "after reopen");
    check(s.readerState == IMU_READER_STATE_CONFIGURING, "E10", "CONFIGURING");
    check_u32(s.configurationEpoch, 2u, "E12", "one increment only");

    check(imu_session_configure_production(0), "E11", "reconfigure");
    check(imu_session_begin_settle(), "E11", "re-settle");
    check(imu_session_mark_operational(), "E11", "operational");
    check(snap(&s), "E11", "snapshot");
    check(s.readerState == IMU_READER_STATE_OPERATIONAL, "E11", "OPERATIONAL");
    check_u32(s.configurationEpoch, 2u, "E11", "epoch unchanged by settle");
    check_u64(s.rotationMeta.groupEventSeq, 80ull, "E11", "group seq not reset");
    check_u32(s.validMask, 0u, "E11", "residue not valid");
}

static void test_e14(void)
{
    ImuSampleSnapshot_t s;

    imu_session_test_reset();
    check(imu_session_test_open(true), "E14", "open");
    check(imu_session_test_force_state(IMU_READER_STATE_CALIBRATION),
          "E14", "force CALIBRATION");
    check(!imu_session_mark_operational(), "E14", "mark_operational rejected");
    check(snap(&s), "E14", "snapshot");
    check(s.readerState == IMU_READER_STATE_CALIBRATION, "E14", "state unchanged");
}

static void test_x01_x03(void)
{
    ImuSampleSnapshot_t s;

    /*
     * X01: ImuSampleSnapshot_t has no freshMask/staleMask/missingMask.
     * X02: ImuSampleSnapshot_t has no R10 counters.
     * X03: imu_session_get_snapshot() takes only the mailbox pointer;
     *      there is no publisher/integration prev-copy argument.
     */
    check(bring_up_operational(), "X01", "bring-up");
    check(snap(&s), "X01", "snapshot");
    check_u32(s.validMask, 0u, "X01", "validMask present; no freshness field used");
    check_u32(s.readerStatusFlags, 0u, "X02", "flags stay 0; no R10 in snapshot");
    (void)imu_session_get_snapshot;
}

int main(void)
{
    test_e01();
    test_e02();
    test_e03_e04();
    test_c01_c04();
    test_c05();
    test_c06();
    test_c07();
    test_c08();
    test_e09_e13();
    test_e14();
    test_x01_x03();

    if (g_fail != 0) {
        fprintf(stderr, "test_session_r1_epoch: %d failure(s)\n", g_fail);
        return 1;
    }

    printf("test_session_r1_epoch: pass\n");
    return 0;
}