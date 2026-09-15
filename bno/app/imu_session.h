#ifndef IMU_SESSION_H
#define IMU_SESSION_H

#include <stdbool.h>
#include <stdint.h>

#include "app/imu_contract.h"

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