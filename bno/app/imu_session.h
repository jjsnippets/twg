#ifndef IMU_SESSION_H
#define IMU_SESSION_H

#include <stdbool.h>
#include <stdint.h>

#include "app/imu_contract.h"
#include "app/imu_check.h"

#define IMU_CAL_FACTS_VERSION 1u
#define IMU_TARE_FACTS_VERSION 1u

#define IMU_CAL_MAG_RATE_HZ   50u
#define IMU_CAL_SLOW_RATE_HZ  10u
#define IMU_CAL_MAG_INTERVAL_US   (1000000u / IMU_CAL_MAG_RATE_HZ)
#define IMU_CAL_SLOW_INTERVAL_US  (1000000u / IMU_CAL_SLOW_RATE_HZ)

#define IMU_CHECK_MAG_RATE_HZ      50u
#define IMU_CHECK_SLOW_RATE_HZ     10u
#define IMU_CHECK_MAG_INTERVAL_US  (1000000u / IMU_CHECK_MAG_RATE_HZ)
#define IMU_CHECK_SLOW_INTERVAL_US (1000000u / IMU_CHECK_SLOW_RATE_HZ)

/*
 * Calibration-only mailbox. Not part of production R2.
 * Accuracies are report STATUS bits 0-3. rvErrRad is the RV payload
 * heading-error estimate in radians, not the 0-3 status.
 */
typedef struct {
    uint8_t  version;
    uint32_t configurationEpoch;
    uint64_t hostDecodeNs;
    uint8_t  accelAccuracy;
    uint8_t  gyroAccuracy;
    uint8_t  magAccuracy;
    uint8_t  rvAccuracy;
    float    rvErrRad;
    bool     haveMag;
    float    magXuT;
    float    magYuT;
    float    magZuT;
    bool     valid;
} ImuCalFacts_t;

/*
 * CHECK and PROBE are separate reader modes, not a boolean overlay on
 * CALIBRATION or OPERATIONAL.
 */
typedef enum {
    IMU_SESSION_CHECK_MODE_CHECK = 1,
    IMU_SESSION_CHECK_MODE_PROBE
} ImuSessionCheckMode_t;

/*
 * The call's return value is success. A readable-but-different mask makes
 * success false while retaining actualMaskValid and actualMask here.
 * An unavailable readback is not invented and is not by itself a failure.
 */
typedef struct {
    bool actualMaskValid;
    uint8_t actualMask;
} ImuSessionCheckConfigResult_t;

/* Host-only report-configuration observation IDs. Not SH-2 sensor IDs. */
typedef enum {
    IMU_SESSION_TEST_REPORT_MAG = 0,
    IMU_SESSION_TEST_REPORT_ACCEL,
    IMU_SESSION_TEST_REPORT_GYRO,
    IMU_SESSION_TEST_REPORT_RV,
    IMU_SESSION_TEST_REPORT_LINEAR,
    IMU_SESSION_TEST_REPORT_COUNT
} ImuSessionTestReportId_t;

typedef struct {
    bool attempted;
    uint32_t reportIntervalUs;
    uint32_t batchIntervalUs;
} ImuSessionTestReportConfig_t;

typedef enum {
    IMU_SESSION_TARE_AXES_Z = 1,
    IMU_SESSION_TARE_AXES_FULL
} ImuSessionTareAxes_t;

typedef enum {
    IMU_SESSION_TARE_SUB_NOT_ATTEMPTED = 0,
    IMU_SESSION_TARE_SUB_SUCCEEDED,
    IMU_SESSION_TARE_SUB_FAILED
} ImuSessionTareSubResult_t;

typedef struct {
    bool success;
    ImuSessionTareSubResult_t clearActive;
    ImuSessionTareSubResult_t clearSaved;
} ImuSessionClearTareResult_t;

/*
 * Tare-only mailbox. Not part of production R2.
 * rotationEventSequence is process-lifetime; validity is epoch-scoped.
 */
typedef struct {
    uint8_t version;
    uint32_t configurationEpoch;
    uint64_t hostDecodeNs;
    uint64_t rotationEventSequence;
    uint8_t rotationStatus;
    float quatI;
    float quatJ;
    float quatK;
    float quatReal;
    float yawRad;
    float pitchRad;
    float rollRad;
    float oriErrRad;
    bool valid;
} ImuTareFacts_t;

typedef enum {
    IMU_SESSION_REOPEN_RECOVERY = 0,
    IMU_SESSION_REOPEN_VERIFY
} ImuSessionReopenKind_t;

/*
 * Sole HAL / SH-2 session owner for production BNO085 use.
 *
 * Thread-safety: start/configure/stop run on the owner thread. service()
 * runs on the caller's thread; SH-2 callbacks fire inside that call.
 * get_snapshot() copies the latest mailbox. In the current single-threaded
 * design, service() and get_snapshot() share a thread.
 *
 * This module must not call StartRT, RT_SleepUntil, usleep, nanosleep,
 * printf, or exit. CLOSED is terminal. Recovery must not pass through
 * CLOSED. A second in-process open is not part of this contract generation.
 */

/*
 * Brings up the Raspberry Pi HAL and opens the SH-2 session.
 *
 * Success: OPENING -> CONFIGURING, configurationEpoch 0 -> 1, validMask 0.
 * Failure: FAULTED, epoch remains 0. No production CSV/session is implied.
 *
 * Returns false if the session is not CLOSED, HAL/SH-2 open fails, or a
 * second in-process open is attempted.
 */
bool imu_session_open(void);

/*
 * Enables rotation vector, linear acceleration, and calibrated gyroscope
 * at 100 Hz (reportInterval_us 10000, batchInterval_us 0) and applies
 * sh2_setCalConfig(flightCalMask).
 *
 * First successful apply after open does not extra-increment epoch (E03).
 * A later apply that changes report set or flight policy increments epoch,
 * clears validMask, and zeroes epochUpdateCount.
 *
 * Legal from CONFIGURING (and from CONFIGURING after recovery). Returns
 * false on SH-2 failure or illegal state; does not call exit.
 */
bool imu_session_configure_production(uint8_t flightCalMask);

/*
 * Restores the normal 100 Hz production report set and flight calibration
 * policy after calibration, tare, check, or probe. Legal from CALIBRATION,
 * TARE, CHECK, PROBE, and CONFIGURING only.
 * A restore after an already configured calibration/verification path
 * increments epoch once, clears validity and calibration-only facts, and
 * leaves the session in CONFIGURING. It never closes or recovers the session.
 * An initial CONFIGURING call before any report set was configured is allowed
 * and behaves as the initial production apply (no extra epoch increment).
 */
bool imu_session_restore_production(uint8_t flightCalMask);

/*
 * Legal from CONFIGURING only. On successful report/mask configuration,
 * starts exactly one new epoch, clears production R2 validity and old check
 * facts, then enters CHECK or PROBE. CHECK requires mask 0x00; PROBE accepts
 * the complete requested uint8_t, including 0x00.
 *
 * A readable mask mismatch returns false but retains the new, actionable
 * CHECK/PROBE state and epoch so the caller can request production restore.
 * A hard configuration failure faults rather than claiming entry or epoch.
 */
bool imu_session_configure_check(ImuSessionCheckMode_t mode, uint8_t mask,
                                 ImuSessionCheckConfigResult_t *out);

/* Copy the separate check mailbox; never reinterpret production R2 as it. */
bool imu_session_get_check_facts(ImuCheckFacts_t *out);

/*
 * Enables the calibration report set and applies sh2_setCalConfig(calMask).
 * Legal from CONFIGURING. Increments epoch (report set + policy change),
 * clears validMask, enters CALIBRATION. Does not open a second session.
 */
bool imu_session_configure_calibration(uint8_t calMask);

/* Reads back the active ME cal mask. False if no open session. */
bool imu_session_get_cal_policy(uint8_t *outMask);

/*
 * sh2_saveDcdNow through the session owner.
 * Does not increment epoch. Increment happens on the later verify reopen.
 */
bool imu_session_save_dcd(void);

/*
 * Planned calibration verification reopen.
 * Must not pass through CLOSED. Increments epoch once, clears validity,
 * returns CONFIGURING. A successful planned reopen is not recovery and
 * must not set recovery-observed / recovery-success counters.
 * If reopen fails, normal recovery/fault accounting begins.
 */
bool imu_session_begin_verification_reopen(void);

/* Copy the calibration mailbox. False if out is NULL or never constructed. */
bool imu_session_get_cal_facts(ImuCalFacts_t *out);

/*
 * Enables rotation vector at 100 Hz, batching off, cal mask 0x00.
 * Legal from CONFIGURING only. Increments epoch once, clears validity
 * and tare facts, enters TARE. Failure faults without claiming epoch.
 */
bool imu_session_configure_tare(void);

/*
 * sh2_setTareNow through the session owner.
 * Z -> SH2_TARE_Z (0x04). Full -> X|Y|Z (0x07).
 * Basis SH2_TARE_BASIS_ROTATION_VECTOR (0).
 * Legal in TARE. Success increments epoch once and clears tare facts.
+ * Failure leaves epoch and TARE unchanged.
 */
bool imu_session_tare_now(ImuSessionTareAxes_t axes);

/*
 * sh2_persistTare through the session owner. Legal in TARE.
 * Does not increment epoch.
 */
bool imu_session_persist_tare(void);

/*
 * sh2_clearTare through the session owner. Legal in TARE.
 * One observed success maps both clearActive and clearSaved to SUCCEEDED.
 * One failure maps both to FAILED. This backend cannot observe a partial
 * clear. Success increments epoch once and clears tare facts.
 */
bool imu_session_clear_tare(ImuSessionClearTareResult_t *out);

/* Copy the tare mailbox. False if out is NULL or never constructed. */
bool imu_session_get_tare_facts(ImuTareFacts_t *out);


/*
 * CONFIGURING -> SETTLING. Epoch unchanged. Samples may decode but are
 * not acquisition-eligible. Returns false if not CONFIGURING.
 */
bool imu_session_begin_settle(void);

/*
 * SETTLING -> OPERATIONAL only. Epoch unchanged. Does not capture a
 * publisher baseline and does not reset identities.
 *
 * Returns false from any other state, including CALIBRATION/TARE/CHECK/PROBE
 * (E14).
 */
bool imu_session_mark_operational(void);

/*
 * Drives sh2_service() on the caller thread. Callbacks update only the
 * group represented by each decoded event.
 */
void imu_session_service(void);

/*
 * Copies the latest snapshot into out.
 *
 * Returns true if a mailbox has been constructed (including empty-but-open
 * snapshots after successful open). Returns false if out is NULL or the
 * session has never left CLOSED with a constructed mailbox.
 */
bool imu_session_get_snapshot(ImuSampleSnapshot_t *out);

/*
 * Observes reset / explicit reopen without entering CLOSED. Increments
 * epoch once for the event (E12), clears validMask, enters RECOVERING,
 * and on successful reopen returns to CONFIGURING with no second increment.
 *
 * Returns false if there is no open session or reopen cannot proceed.
 */
bool imu_session_begin_recovery(void);

/*
 * Closes SH-2/HAL, sets CLOSED, live epoch 0, clears validity.
 * Idempotent if already CLOSED.
 */
void imu_session_close(void);

/*
 * Host-test seam. Production main must not call these. They exist so
 * Phase 1I families C/E/X can run without hardware.
 *
 * imu_session_test_reset() restores CLOSED, epoch 0, zero mailbox.
 * imu_session_test_open() performs the open state change without HAL.
 * imu_session_test_inject_group() applies one decoded group using the
 * session's current epoch, the same validity rules as the production
 * callback. Injected residue does not set validMask on epoch mismatch,
 * seq 0, missing host time, or backward seq; those sequences are not
 * adopted (C05, C06).
 * imu_session_test_inject_reset() is the SH-2 reset observation path.
 * imu_session_test_force_state() is only for proving illegal transitions.
 */
void imu_session_test_reset(void);
bool imu_session_test_open(bool success);
void imu_session_test_inject_reset(void);
bool imu_session_test_force_state(ImuReaderState_t state);
void imu_session_test_inject_cal_facts(const ImuCalFacts_t *facts);
void imu_session_test_set_save_dcd_result(bool success);
void imu_session_test_set_reopen_result(bool success);
void imu_session_test_inject_tare_facts(const ImuTareFacts_t *facts);
void imu_session_test_set_configure_tare_result(bool success);
void imu_session_test_set_tare_now_result(bool success);
void imu_session_test_set_persist_tare_result(bool success);
void imu_session_test_set_clear_tare_result(bool success);

/*
 * Host-only check/probe seams. Report injection acts like one decoded
 * diagnostic report; the bulk injection seam permits negative adapter tests
 * with a deliberately wrong version or epoch. Neither modifies R2.
*/
void imu_session_test_set_configure_check_result(bool success);
void imu_session_test_set_check_readback(bool available, uint8_t actualMask);
void imu_session_test_inject_check_report(ImuSessionTestReportId_t report,
                                          uint8_t status,
                                          uint64_t hostDecodeNs,
                                          float rvErrRad,
                                          float magX, float magY, float magZ);
void imu_session_test_inject_check_facts(const ImuCheckFacts_t *facts);
bool imu_session_test_get_report_config(ImuSessionTestReportId_t report,
                                        ImuSessionTestReportConfig_t *out);

uint8_t imu_session_test_last_tare_axes(void);
uint8_t imu_session_test_last_tare_basis(void);
bool imu_session_test_have_last_tare_now(void);
void imu_session_test_set_production_result(bool success);
bool imu_session_test_recovery_observed(void);
uint32_t imu_session_test_recovery_attempt_count(void);

typedef struct {
    uint8_t  groupBit;
    uint64_t groupEventSeq;
    uint64_t hostDecodeNs;
    uint64_t sensorTimeUs;
    uint8_t  deviceReportSeq;
    uint8_t  rawStatus;
    float    qw;
    float    qi;
    float    qj;
    float    qk;
    float    yaw;
    float    pitch;
    float    roll;
    float    orientationErrRad;
    float    ax;
    float    ay;
    float    az;
    float    gx;
    float    gy;
    float    gz;
} ImuSessionTestGroupEvent_t;

void imu_session_test_inject_group(const ImuSessionTestGroupEvent_t *event);

#endif /* IMU_SESSION_H */