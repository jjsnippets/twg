#ifndef CAL_CONTRACT_H
#define CAL_CONTRACT_H

#include <stdbool.h>
#include <stdint.h>

#define CAL_SAMPLE_STRUCT_VERSION 2

/*
 * cal_contract.h — data contract for the calibration tool.
 *
 * Counterpart of app/app_contract.h with one deliberate divergence:
 * it carries NO fused orientation or motion data at all. Calibration
 * only needs the per-sensor accuracy bits (0-3), taken from each
 * report's status byte, and the calibrated magnetic field vector the
 * operator swings through during the magnetometer phase.
 *
 * This contract is intentionally separate from app/app_contract.h:
 * the calibration tool runs outside the acquisition phase and never
 * shares samples with bno_app. The only thing that crosses from
 * bno_cal to bno_app is the DCD saved in the BNO085's flash.
 */

typedef struct {
    uint8_t version;            /* CAL_SAMPLE_STRUCT_VERSION */

    /*
     * Monotonically increasing per decoded sensor event, incremented
     * in cal_sensor.c's sensorCallback(). Starts at 0. If the
     * consumer polls faster than events arrive it sees the same seq
     * repeated — a genuine duplicate, not a new event.
     */
    uint32_t seq;

    uint64_t tHost_uS;          /* host CLOCK_MONOTONIC at decode time */
    uint64_t tDevice_uS;        /* BNO085 event timestamp */

    /*
     * Accuracy bits, 0-3 (0 unreliable, 1 low, 2 medium, 3 high).
     * All four come from each report's STATUS BYTE, including the
     * rotation vector, whose payload accuracy field is something else
     * entirely (see rvErrRad below).
     *
     * Note on the gyro bit: it stays 0 whenever the gyro dynamic
     * calibration flag is disabled (the bno_app all-off policy). It is
     * only meaningful during the calibration flow itself, where the
     * flag is on.
     */
    uint8_t accelAccuracy;
    uint8_t gyroAccuracy;
    uint8_t magAccuracy;
    uint8_t rvAccuracy;

    /*
     * Estimated heading error from the rotation vector payload, in
     * RADIANS (lower is better; ~3 rad means "no idea"). This is the
     * field the SH-2 calls "accuracy" inside the RV report — it is NOT
     * the 0-3 calibrated scale. Diagnostic only.
     */
    float rvErrRad;

    bool haveMag;               /* a magnetic field report has arrived */

    /* Calibrated magnetic field (uT). */
    float magX_uT;
    float magY_uT;
    float magZ_uT;
} CalSample_t;

#endif /* CAL_CONTRACT_H */
