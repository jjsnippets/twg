#ifndef SENSOR_ORIENT_H
#define SENSOR_ORIENT_H

#include <stdbool.h>
#include <stdint.h>

#define ORIENT_SAMPLE_STRUCT_VERSION 1

/*
 * sensor_orient.h — SH-2 session owner for the orientation (tare)
 * tool.
 *
 * Counterpart of sensor_calibrate.h for bno_orient: same Raspberry
 * Pi HAL (app/sh2_hal_rpi.c, compiled in via the Makefile), same
 * sh2_hal_rpi_init / sh2_open / sh2_setSensorCallback /
 * sh2_setSensorConfig sequence, but a different report set: only
 * SH2_ROTATION_VECTOR at 20 Hz, decoded in full (quaternion + its
 * payload accuracy), because the tare flow needs the actual fused
 * heading — CalSample_t deliberately carries accuracies only.
 *
 * The sample struct lives here instead of a separate orient_sample.h
 * (the cal_sample.h pattern): unlike CalSample_t it never crosses
 * between tools — bno_orient is the only producer and consumer.
 *
 * Deliberately does NOT own tare policy: sh2_setTareNow /
 * sh2_persistTare / sh2_clearTare are flow decisions made by
 * orient_main.c — exactly like cal_main.c owns the DCD policy and
 * bno_app's main.c owns the flight-time calibration policy.
 */

typedef struct {
    uint8_t version;            /* ORIENT_SAMPLE_STRUCT_VERSION */

    /*
     * Monotonically increasing per decoded sensor event, incremented
     * in sensor_orient.c's sensorCallback(). Starts at 0.
     */
    uint32_t seq;

    uint64_t tHost_uS;          /* host CLOCK_MONOTONIC at decode time */
    uint64_t tDevice_uS;        /* BNO085 event timestamp */

    bool haveRv;                /* a rotation vector report has arrived */

    uint8_t rvAccuracy;         /* 0-3, from the report payload */

    /* Rotation-vector quaternion (SH-2 order: i=x, j=y, k=z, real=w). */
    float quatW;
    float quatX;
    float quatY;
    float quatZ;
} OrientSample_t;

/*
 * Opens the SH-2 session and enables the rotation-vector report.
 * Does not create a thread; the caller must call
 * sensor_orient_service() periodically from its own loop.
 * May be called again after sensor_orient_stop() (the reopen
 * performs the HAL reset sequence, so the chip reboots — a volatile
 * tare is lost, a persisted one reloads from flash).
 */
bool sensor_orient_start(void);

/*
 * Services the SH-2 session. Call at approximately 1 ms cadence so
 * pending BNO085 H_INTN traffic is handled promptly.
 */
void sensor_orient_service(void);

/* Closes the SH-2 session and the Raspberry Pi HAL. */
void sensor_orient_stop(void);

/*
 * Copies the most recent decoded sample into outSample. Returns
 * false if no report has been decoded yet. Single-threaded: call
 * from the same loop that invokes sensor_orient_service().
 */
bool sensor_orient_getLatestSample(OrientSample_t *outSample);

#endif /* SENSOR_ORIENT_H */
