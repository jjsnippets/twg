#ifndef APP_CONTRACT_H
#define APP_CONTRACT_H

#include <stdint.h>

#define IMU_SAMPLE_STRUCT_VERSION 3

typedef struct {
    uint8_t  version;        /* IMU_SAMPLE_STRUCT_VERSION */

    /* Monotonically increasing per real sensor event, incremented in
     * app_sensor.c's sensorCallback(). Starts at 0 for the first
     * decoded event. Always increases by 1 per real event; if a
     * consumer polls faster than new events arrive, it will observe
     * the same seq value repeated (a genuine duplicate, not a new
     * event). Consumers can detect new data by comparing this
     * field instead of requiring a value-by-value comparison. */
    uint32_t seq;

    /* Sensor-reported timestamp (from sh2_SensorEvent_t.timestamp_uS).
     * Retained alongside seq: seq alone carries no timing information,
     * so timestamp_uS is still needed for rate/cadence verification
     * and for dt-based control logic. NOTE: this field can very
     * occasionally be non-monotonic by a few microseconds between
     * consecutive events -- this is an SH-2/SHTP-level artifact,
     * independent of host scheduling/RT tuning, so any dt computation
     * must still guard against dt <= 0. */
    uint64_t timestamp_uS;

    float yaw;
    float pitch;
    float roll;
    /* Estimated heading error from the rotation-vector payload, in
     * RADIANS (lower is better; ~3 rad means "no estimate"). This is
     * NOT the 0-3 status/accuracy scale used by the calibration
     * tools' status bytes. Renamed from orientationAccuracy (v2),
     * which misleadingly suggested the 0-3 scale.
     *
     * Note: When SH2 dynamic calibration is disabled (mask 0x00), the
     * BNO085 firmware reports the gyro status bit as 0 (unreliable)
     * by design because the real-time ZRO estimator is halted, while
     * the saved DCD keeps bias-correcting the gyro data itself.
     * Pre-flight health checks must evaluate rotation vector accuracy
     * and this error estimate rather than the gyro status bit. */
    float orientationErrRad;

    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    uint8_t validMask;       /* bit 0: orientation, 1: accel, 2: gyro */
} ImuSample_t;

#define IMU_SAMPLE_VALID_ORIENTATION (1u << 0)
#define IMU_SAMPLE_VALID_ACCEL       (1u << 1)
#define IMU_SAMPLE_VALID_GYRO        (1u << 2)

#endif /* APP_CONTRACT_H */
