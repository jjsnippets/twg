#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include <stdbool.h>

#include "imu_sample.h"

/*
 * Opens the HAL and SH-2 session, then enables the configured IMU reports.
 * Does not create a thread; the caller must call sensor_reader_service()
 * periodically from its own loop.
 */
bool sensor_reader_start(void);

/*
 * Services the SH-2 session. Call at approximately 1 ms cadence so pending
 * BNO085 H_INTN traffic is handled promptly.
 */
void sensor_reader_service(void);

/* Closes the SH-2 session and the Raspberry Pi HAL. */
void sensor_reader_stop(void);

/*
 * Copies the most recent decoded sample into outSample.
 * Returns false until at least one sensor event has been received.
 *
 * This reader is single-threaded: call from the same loop that invokes
 * sensor_reader_service().
 */
bool sensor_reader_getLatestSample(ImuSample_t *outSample);

/*
 * Sets the sequence number to 0
 */
void sensor_reader_resetSeq();

#endif /* SENSOR_READER_H */