#ifndef CAL_SENSOR_H
#define CAL_SENSOR_H

#include <stdbool.h>

#include "calibration/cal_contract.h"

/*
 * cal_sensor.h — SH-2 session owner for the calibration tool.
 *
 * Counterpart of app/app_sensor.h for bno_cal: same Raspberry
 * Pi HAL (app/sh2_hal_rpi.c, compiled in via the Makefile), same
 * sh2_hal_rpi_init / sh2_open / sh2_setSensorCallback /
 * sh2_setSensorConfig sequence, but a different report set:
 *
 *   - SH2_MAGNETIC_FIELD_CALIBRATED at 50 Hz (required by 1000-4044)
 *   - SH2_ACCELEROMETER             at 10 Hz
 *   - SH2_GYROSCOPE_CALIBRATED      at 10 Hz
 *   - SH2_ROTATION_VECTOR           at 10 Hz
 *
 * The last three only supply the per-report accuracy bits (0-3) that
 * the flow controller gates each phase on; the magnetic field also
 * supplies the vector.
 *
 * Deliberately does NOT own calibration policy: sh2_setCalConfig and
 * sh2_saveDcdNow are called by cal_main.c, which owns the flow.
 */

/*
 * Brings up the Pi HAL, opens the SH-2 session, registers the sensor
 * callback and enables the four calibration reports. Returns false on
 * any failure (no cleanup needed on failure).
 *
 * May be called again after cal_sensor_stop() (used by
 * bno_cal's step 6 to verify the DCD survives a chip reboot).
 */
bool cal_sensor_start(void);

/*
 * Services the SH-2 session. Call at approximately 1 ms cadence so
 * pending BNO085 H_INTN traffic is handled promptly.
 */
void cal_sensor_service(void);

/* Closes the SH-2 session and the Raspberry Pi HAL. */
void cal_sensor_stop(void);

/*
 * Copies the most recent decoded sample into outSample. Returns
 * false if no report has been decoded yet. Single-threaded: call
 * from the same loop that invokes cal_sensor_service().
 */
bool cal_sensor_getLatestSample(CalSample_t *outSample);

#endif /* CAL_SENSOR_H */
