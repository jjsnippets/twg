#ifndef APP_SENSOR_H
#define APP_SENSOR_H

#include <stdbool.h>

#include "app/app_contract.h"

/*
 * sensor_reader.h
 *
 * Public interface to the BNO085 reader. Owns the SH-2 session and decodes
 * incoming reports into the latest ImuSample_t.
 *
 * Thread-safety contract:
 *   - sensor_reader_start() and sensor_reader_stop() are called by the owner
 *     thread during bring-up and tear-down.
 *   - sensor_reader_service() runs on the caller's thread; all SH-2 callbacks
 *     fire within its call stack.
 *   - sensor_reader_getLatestSample() copies the most recently decoded sample
 *     out to caller-owned storage.
 *
 * In the current single-threaded design, service() and getLatestSample()
 * run on the same thread so no synchronization is required. If service()
 * is ever moved to a background thread, getLatestSample() must be made
 * thread-safe.
 */

/*
 * Brings up the Raspberry Pi HAL, opens the SH-2 session, registers the
 * sensor event callback, and enables rotation vector, linear acceleration,
 * and calibrated gyroscope at 100 Hz.
 *
 * Returns true if the session opened and all three sensors were configured
 * successfully; returns false on any failure (no cleanup needed on failure).
 */
bool sensor_reader_start(void);

/*
 * Drives the SH-2 event loop. Calls sh2_service(), which checks the HAL
 * transport, reads any pending packets from the BNO085, and invokes the
 * registered sensor event callback for each decoded report.
 *
 * Call this from the real-time loop at the configured service rate (~1 kHz).
 */
void sensor_reader_service(void);

/*
 * Closes the SH-2 session and frees HAL resources.
 */
void sensor_reader_stop(void);

/*
 * Copies the most recently decoded sample into outSample.
 *
 * Returns true if at least one sample has been decoded since start(); returns
 * false if called before the first sample arrives or if outSample is NULL.
 */
bool sensor_reader_getLatestSample(ImuSample_t *outSample);

/*
 * Resets the seq counter in ImuSample_t to 0.
 *
 * Call this after the 300 ms settle phase in main.c so seq=1 corresponds
 * to the first sample the consumer loop processes, not the first sample
 * decoded during bring-up.
 */
void sensor_reader_resetSeq(void);

#endif /* APP_SENSOR_H */
