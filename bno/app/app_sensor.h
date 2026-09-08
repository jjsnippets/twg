#ifndef APP_SENSOR_H
#define APP_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "app/app_contract.h"

/*
 * app_sensor.h — owns the SH-2 session and decodes BNO085 reports
 * into the latest ImuSample_t.
 *
 * Thread-safety contract:
 *   - sensor_reader_start() / stop() must be called from the main thread.
 *   - sensor_reader_service() runs the SH-2 event loop. In the single-
 *     threaded reference implementation it is called from the main loop.
 *     If moved to a background thread, getLatestSample() must be made
 *     safe against concurrent writes.
 *   - sensor_reader_getLatestSample() copies the most recent decoded
 *     sample into caller-provided memory.
 */

/*
 * Brings up the Pi HAL, opens the SH-2 session, registers the sensor
 * callback, and enables rotation vector, linear acceleration, and
 * calibrated gyroscope at 100 Hz.
 * Returns false on any failure (no cleanup needed on failure).
 */
bool sensor_reader_start(void);

/*
 * Services the SH-2 session. In single-threaded mode, call this from
 * the main loop at high frequency (~1 kHz) so H_INTN is drained promptly.
 */
void sensor_reader_service(void);

/* Closes the SH-2 session and releases HAL resources. */
void sensor_reader_stop(void);

/*
 * Copies the most recent decoded sample into outSample.
 * Returns false if no sample has been decoded yet.
 */
bool sensor_reader_getLatestSample(ImuSample_t *outSample);

/*
 * Resets the seq counter in ImuSample_t to 0. Call after the 300 ms
 * warm-up drain in main.c so seq=1 corresponds to the first sample the
 * application loop processes.
 */
void sensor_reader_resetSeq(void);

#endif /* APP_SENSOR_H */
