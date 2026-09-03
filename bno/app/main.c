#define _POSIX_C_SOURCE 200809L

/*
 * main.c
 *
 * Opens the IMU sensor reader, then runs a single real-time loop:
 *   - services the BNO085 / SH-2 session at 1 kHz; and
 *   - prints the latest combined sample at 100 Hz.
 *
 * The 300 ms settle phase services startup traffic without treating it as
 * application data. The timed acquisition window begins only afterward.
 */

#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include <time.h>

#include "imu_sample.h"
#include "realtime.h"
#include "sensor_reader.h"

#define RUN_DURATION_SEC       10
#define RT_PRIORITY            90
#define LOOP_DT_SEC            0.001
#define PRINT_EVERY_N_LOOPS    10
#define SETTLE_DURATION_MS     300
#define SETTLE_LOOPS           ((SETTLE_DURATION_MS * 1000) / \
                                (unsigned)(LOOP_DT_SEC * 1000000.0 + 0.5))

static double elapsedSeconds(const struct timespec *start,
                             const struct timespec *now)
{
    return (double)(now->tv_sec - start->tv_sec) +
           (double)(now->tv_nsec - start->tv_nsec) / 1e9;
}

int main(void)
{
    if (!sensor_reader_start()) {
        fprintf(stderr, "main: sensor_reader_start failed\n");
        return 1;
    }

    if (StartRT(RT_PRIORITY, LOOP_DT_SEC) != 0) {
        fprintf(stderr,
                "main: WARNING: StartRT failed; continuing at default scheduling\n");
    }

    /*
     * Service the BNO085 while its feature reports and fusion outputs
     * settle. Do not print or count samples during this phase.
     */
    for (unsigned i = 0; i < SETTLE_LOOPS; ++i) {
        sensor_reader_service();
        RT_SleepUntil(LOOP_DT_SEC);
    }

    printf("main: running IMU reader for %d seconds...\n", RUN_DURATION_SEC);

    ImuSample_t sample;
    int printCount = 0;
    long loopCount = 0;

    struct timespec t0;
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    double elapsed = 0.0;

    while (elapsed < (double)RUN_DURATION_SEC) {
        sensor_reader_service();
        loopCount++;

        if ((loopCount % PRINT_EVERY_N_LOOPS) == 0) {
            if (sensor_reader_getLatestSample(&sample) &&
                sample.version == IMU_SAMPLE_STRUCT_VERSION) {
                printf("seq=%" PRIu32 " ts=%" PRIu64,
                       sample.seq,
                       sample.timestamp_uS);

                if (sample.validMask & IMU_SAMPLE_VALID_ORIENTATION) {
                    printf(" yaw=%7.3f pitch=%7.3f roll=%7.3f",
                           sample.yaw,
                           sample.pitch,
                           sample.roll);
                }

                if (sample.validMask & IMU_SAMPLE_VALID_ACCEL) {
                    printf(" ax=%7.3f ay=%7.3f az=%7.3f",
                           sample.ax,
                           sample.ay,
                           sample.az);
                }

                if (sample.validMask & IMU_SAMPLE_VALID_GYRO) {
                    printf(" gx=%7.3f gy=%7.3f gz=%7.3f",
                           sample.gx,
                           sample.gy,
                           sample.gz);
                }

                printf("\n");
                printCount++;
            }
        }

        RT_SleepUntil(LOOP_DT_SEC);

        clock_gettime(CLOCK_MONOTONIC, &now);
        elapsed = elapsedSeconds(&t0, &now);
    }

    printf("main: finished. Printed %d lines.\n", printCount);

    sensor_reader_stop();
    return 0;
}