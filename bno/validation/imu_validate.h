/*
 * imu_validate.h — data contract for the validation binary.
 *
 * Counterpart of app/imu_sample.h with one deliberate divergence: each
 * sensor group carries its OWN host/device timestamp pair and sequence
 * numbers. The app mailbox cannot express how stale each group is (one
 * ImuSample_t can mix values from up to three events ~10 ms apart, with
 * seq/timestamp reflecting only the newest), and phase comparison against
 * the encoder needs that per-group freshness.
 *
 * validMask bit meanings are identical to app/imu_sample.h.
 *
 * Part of the sensor_validate binary (see validation/Makefile).
 */

#ifndef IMU_VALIDATE_H
#define IMU_VALIDATE_H

#include <stdint.h>

#define IMU_VALIDATE_STRUCT_VERSION 1u

/* validMask bits — same meanings as app/imu_sample.h */
#define IMU_VALIDATE_VALID_RV    (1u << 0)
#define IMU_VALIDATE_VALID_ACCEL (1u << 1)
#define IMU_VALIDATE_VALID_GYRO  (1u << 2)

/*
 * Per-sensor-group freshness info, updated only by the thread that calls
 * sensor_validate_service() (the SH-2 event callback runs inside it).
 */
typedef struct {
    uint64_t host_ts_ns;    /* CLOCK_MONOTONIC ns, captured at decode time */
    uint64_t sensor_ts_us;  /* BNO085 device timestamp of the originating report */
    uint32_t seq;           /* global decode counter at this group's last update */
    uint8_t  report_seq;    /* device-side per-sensor rolling sequence (report byte 1) */
    uint8_t  status;        /* sh2 sensor status byte */
} ImuGroupMeta_t;

typedef struct {
    uint32_t version;       /* IMU_VALIDATE_STRUCT_VERSION */
    uint32_t seq;           /* +1 per decoded event (any of the three), since start */
    uint64_t host_ts_ns;    /* CLOCK_MONOTONIC ns of the newest decoded event */

    /* Rotation vector (SH2_ROTATION_VECTOR) */
    ImuGroupMeta_t rv;
    float yaw;              /* rad */
    float pitch;            /* rad */
    float roll;             /* rad */
    float rvAccuracy;       /* rad, rotation-vector accuracy estimate */

    /* Linear acceleration (SH2_LINEAR_ACCELERATION) */
    ImuGroupMeta_t accel;
    float ax;               /* m/s^2 */
    float ay;
    float az;

    /* Calibrated gyroscope (SH2_GYROSCOPE_CALIBRATED) */
    ImuGroupMeta_t gyro;
    float gx;               /* rad/s */
    float gy;
    float gz;

    uint8_t validMask;      /* IMU_VALIDATE_VALID_* bits, set once per group ever seen */
} ImuValidateSample_t;

#endif /* IMU_VALIDATE_H */
