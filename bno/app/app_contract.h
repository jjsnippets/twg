#ifndef APP_CONTRACT_H
#define APP_CONTRACT_H

#include <stdint.h>

#define IMU_SAMPLE_STRUCT_VERSION 3

/*
 * app_contract.h — the shared data contract between the sensor
 * acquisition thread (sensor_reader.c) and downstream consumers
 * (main.c, and later thesis tasks: pressure sensing, video, network).
 *
 * All values are scaled to engineering units:
 *   - time: microseconds (host CLOCK_MONOTONIC and BNO085 timestamp)
 *   - orientation: radians (yaw, pitch, roll; see euler.h for convention)
 *   - linear acceleration: m/s^2 (gravity removed by BNO085 sensor fusion)
 *   - calibrated gyroscope: rad/s (bias-corrected by dynamic calibration)
 *
 * Sequence numbers:
 *   - seq: rolling uint32_t counter incremented on every decoded sensor
 *     event. If the consumer polls faster than the sensor reports, it
 *     sees the same seq — a duplicate, not a new sample.
 *   - report_seq: the 1-byte rolling sequence number from the SH-2 report
 *     itself (byte 1 of every input report), useful for detecting drops
 *     on the BNO085 side.
 *
 * Valid mask:
 *   Bitmask indicating which fields have been updated at least once since
 *   session start.
 */
#define IMU_SAMPLE_VALID_RV    (1u << 0)
#define IMU_SAMPLE_VALID_ACCEL (1u << 1)
#define IMU_SAMPLE_VALID_GYRO  (1u << 2)

typedef struct {
    uint8_t  version;        /* IMU_SAMPLE_STRUCT_VERSION */

    /* Monotonically increasing per real sensor event, incremented in
     * app_sensor.c's sensorCallback(). Starts at 0. */
    uint32_t seq;

    uint64_t tHost_uS;       /* host CLOCK_MONOTONIC at decode time */
    uint64_t tDevice_uS;     /* BNO085 event timestamp */

    /* Rotation vector (SH2_ROTATION_VECTOR) -> Euler angles (rad) */
    float    yaw;
    float    pitch;
    float    roll;

    /* Linear acceleration (SH2_LINEAR_ACCELERATION, m/s^2) */
    float    ax;
    float    ay;
    float    az;

    /* Calibrated gyroscope (SH2_GYROSCOPE_CALIBRATED, rad/s) */
    float    gx;
    float    gy;
    float    gz;

    /* Status byte from the newest SH-2 report (bits 1:0 = accuracy 0-3) */
    uint8_t  status;

    /* Rolling sequence from byte 1 of the newest SH-2 report */
    uint8_t  report_seq;

    /* Bitmask: IMU_SAMPLE_VALID_* */
    uint8_t  validMask;
} ImuSample_t;

#endif /* APP_CONTRACT_H */
