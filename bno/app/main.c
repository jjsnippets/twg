#define _POSIX_C_SOURCE 200809L

/*
 * main.c
 *
 * Opens the one IMU session, applies production reports and flight-cal
 * mask 0 through the session owner, then runs a single real-time loop:
 *   - services the BNO085 / SH-2 session at 1 kHz; and
 *   - prints the latest snapshot at 100 Hz.
 *
 * The 300 ms settle phase services startup traffic without treating it as
 * acquisition output. The timed window begins only after OPERATIONAL.
 */

#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include <time.h>

#include "rt/realtime.h"
#include "app/imu_session.h"

#define RUN_DURATION_SEC       10
#define RT_PRIORITY            90
#define LOOP_DT_SEC            0.001
#define PRINT_EVERY_N_LOOPS    10
#define SETTLE_DURATION_MS     300
#define SETTLE_LOOPS           ((SETTLE_DURATION_MS * 1000) / \
                                (unsigned)(LOOP_DT_SEC * 1000000.0 + 0.5))
#define FLIGHT_CAL_MASK        0u

static double elapsedSeconds(const struct timespec *start,
                             const struct timespec *now)
{
    return (double)(now->tv_sec - start->tv_sec) +
           (double)(now->tv_nsec - start->tv_nsec) / 1e9;
}

static void fail_and_close(const char *msg)
{
    fprintf(stderr, "main: %s\n", msg);
    imu_session_close();
}

int main(void)
{
    if (!imu_session_open()) {
        fprintf(stderr, "main: imu_session_open failed\n");
        imu_session_close();
        return 1;
    }

    if (!imu_session_configure_production(FLIGHT_CAL_MASK)) {
        fail_and_close("imu_session_configure_production failed");
        return 1;
    }

    if (!imu_session_begin_settle()) {
        fail_and_close("imu_session_begin_settle failed");
        return 1;
    }

    if (StartRT(RT_PRIORITY, LOOP_DT_SEC) != 0) {
        fprintf(stderr,
                "main: WARNING: StartRT failed; continuing at default scheduling\n");
    }

    for (unsigned i = 0; i < SETTLE_LOOPS; ++i) {
        imu_session_service();
        RT_SleepUntil(LOOP_DT_SEC);
    }

    if (!imu_session_mark_operational()) {
        fail_and_close("imu_session_mark_operational failed");
        return 1;
    }

    printf("main: running IMU reader for %d seconds...\n", RUN_DURATION_SEC);

    ImuSampleSnapshot_t sample;
    int printCount = 0;
    long loopCount = 0;

    struct timespec t0;
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    double elapsed = 0.0;

    while (elapsed < (double)RUN_DURATION_SEC) {
        imu_session_service();
        loopCount++;

        if ((loopCount % PRINT_EVERY_N_LOOPS) == 0) {
            if (imu_session_get_snapshot(&sample) &&
                sample.version == IMU_SAMPLE_CONTRACT_VERSION) {
                printf("epoch=%" PRIu32 " valid=%" PRIu8
                       " proc=%" PRIu64,
                       sample.configurationEpoch,
                       sample.validMask,
                       sample.processDecodeCount);

                if (sample.validMask & IMU_GROUP_BIT_ROTATION) {
                    printf(" rvseq=%" PRIu64 " rvts=%" PRIu64
                           " yaw=%7.3f pitch=%7.3f roll=%7.3f",
                           sample.rotationMeta.groupEventSeq,
                           sample.rotationMeta.sensorTimeUs,
                           sample.yaw,
                           sample.pitch,
                           sample.roll);
                }

                if (sample.validMask & IMU_GROUP_BIT_ACCEL) {
                    printf(" accseq=%" PRIu64
                           " ax=%7.3f ay=%7.3f az=%7.3f",
                           sample.accelMeta.groupEventSeq,
                           sample.ax,
                           sample.ay,
                           sample.az);
                }

                if (sample.validMask & IMU_GROUP_BIT_GYRO) {
                    printf(" gyrseq=%" PRIu64
                           " gx=%7.3f gy=%7.3f gz=%7.3f",
                           sample.gyroMeta.groupEventSeq,
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

    imu_session_close();
    return 0;
}