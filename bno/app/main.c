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

#include "sh2.h"
#include "sh2_err.h"

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

    /*
     * Flight-time calibration policy: fly on the saved DCD only.
     *
     * sh2_open() just reset the BNO085 and loaded the dynamic
     * calibration data (DCD) that bno_cal previously saved to flash.
     * The dynamic-calibration enable bits themselves are RAM-only
     * state that revert to chip defaults on every reset, so bno_cal
     * cannot set them on our behalf — every program must choose its
     * own policy at session start. Disable all dynamic calibration
     * here so the saved DCD is the only calibration input during
     * acquisition. (The gyro is still bias-corrected automatically
     * whenever the device is stationary, regardless of this setting.)
     *
     * Note: with all dynamic calibration disabled the BNO085 reports
     * the gyro status bit as 0 (unreliable) by design — the real-time
     * ZRO estimator is halted — while the saved DCD keeps
     * bias-correcting the gyro data itself. Health/readiness checks
     * must never gate on the gyro status bit under this policy; use
     * the rotation vector status and ImuSample_t.orientationErrRad
     * (expected <= ~0.35 rad once converged) instead.
     */
    if (sh2_setCalConfig(0) != SH2_OK) {
        fprintf(stderr, "main: sh2_setCalConfig(disable all) failed\n");
        sensor_reader_stop();
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
    sensor_reader_resetSeq();

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
