#ifndef SENSOR_CALIBRATE_H
#define SENSOR_CALIBRATE_H

#include <stdbool.h>

#include "cal_sample.h"

/*
 * sensor_calibrate.h — SH-2 session owner for the calibration tool.
 *
 * Calibration counterpart of app/sensor_reader.c: same Raspberry Pi
 * HAL (app/sh2_hal_rpi.c, compiled in via the Makefile), same
 * sh2_hal_rpi_init / sh2_open / sh2_setSensorCallback /
 * sh2_setSensorConfig sequence, but a different report set:
 *
 *   - SH2_MAGNETIC_FIELD_CALIBRATED at 50 Hz — the report rate CEVA's
 *     BNO085 calibration procedure (1000-4044) requires for
 *     magnetometer calibration work;
 *   - SH2_ACCELEROMETER, SH2_GYROSCOPE_CALIBRATED and
 *     SH2_ROTATION_VECTOR at 10 Hz, subscribed purely for their
 *     status/accuracy bits, which are the per-phase progress signal.
 *
 * Combined ~80 events/s, far below the app's ~300 events/s, so the
 * tool does not need SCHED_FIFO; a plain ~1 kHz usleep loop is enough.
 *
 * Deliberately does NOT own calibration policy: enabling dynamic
 * calibration (sh2_setCalConfig) and saving the DCD (sh2_saveDcdNow)
 * are flow decisions made by cal_main.c — exactly like bno_app's
 * main.c owns the flight-time "all dynamic calibration off" policy.
 */

/*
 * Opens the SH-2 session and enables the calibration report set.
 * Does not create a thread; the caller must call
 * sensor_calibrate_service() periodically from its own loop.
 * May be called again after sensor_calibrate_stop() (the reopen
 * performs the HAL reset sequence, so the chip reboots and reloads
 * the DCD from flash).
 */
bool sensor_calibrate_start(void);

/*
 * Services the SH-2 session. Call at approximately 1 ms cadence so
 * pending BNO085 H_INTN traffic is handled promptly.
 */
void sensor_calibrate_service(void);

/* Closes the SH-2 session and the Raspberry Pi HAL. */
void sensor_calibrate_stop(void);

/*
 * Copies the most recent decoded sample into outSample. Returns false
 * if no report has been decoded yet. Single-threaded: call from the
 * same loop that invokes sensor_calibrate_service().
 */
bool sensor_calibrate_getLatestSample(CalSample_t *outSample);

#endif /* SENSOR_CALIBRATE_H */
