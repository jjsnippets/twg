/*
 * validate_sensor.h — SH-2 session owner for the validation binary.
 *
 * Validation counterpart of app/app_sensor.c: same HAL, same three
 * 100 Hz reports (rotation vector, linear acceleration, calibrated
 * gyro), but decodes into ImuValidateSample_t with per-group timestamps
 * so consumers can see exactly how stale each group is.
 *
 * Single-threaded, like the app reader: call sensor_validate_service()
 * and sensor_validate_getLatestSample() from the same ~1 ms loop.
 *
 * Also disables all dynamic calibration (sh2_setCalConfig(0)),
 * mirroring bno_app's flight policy
 *
 * Build: make validate   (Makefile)
 */

#ifndef VALIDATE_SENSOR_H
#define VALIDATE_SENSOR_H

#include <stdbool.h>

#include "validation/validate_contract.h"

/*
 * Brings up the Pi HAL, opens the SH-2 session, registers the sensor
 * callback and enables the three reports at 100 Hz. Returns false on
 * any failure (nothing to clean up afterwards in that case).
 */
bool sensor_validate_start(void);

/*
 * Services the SH-2 session. Call at approximately 1 ms cadence so
 * pending BNO085 H_INTN traffic is handled promptly.
 */
void sensor_validate_service(void);

/* Closes the SH-2 session (mirrors sensor_reader_stop). */
void sensor_validate_stop(void);

/*
 * Copies the most recent decoded sample into outSample. Returns false
 * if no event has been decoded yet.
 */
bool sensor_validate_getLatestSample(ImuValidateSample_t *outSample);

/*
 * Sets the sequence counter to 0. Call after the settle drain, before
 * the log loop, so the logged seq counts capture-window events only
 * (same contract as sensor_reader_resetSeq() in bno_app).
 */
void sensor_validate_resetSeq(void);

#endif /* VALIDATE_SENSOR_H */
