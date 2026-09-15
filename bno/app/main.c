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
 *
 * This executable is the sole StartRT / RT_SleepUntil owner. imu_session
 * is not allowed to schedule or terminate the process.
 */

#include <stdbool.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "rt/realtime.h"
#include "app/app_rt_policy.h"
#include "app/imu_session.h"

#define RUN_DURATION_SEC       10
#define RT_PRIORITY            90
#define LOOP_DT_SEC            0.001
#define PRINT_EVERY_N_LOOPS    10
#define SETTLE_DURATION_MS     300
#define SETTLE_LOOPS           ((SETTLE_DURATION_MS * 1000) / \
                                (unsigned)(LOOP_DT_SEC * 1000000.0 + 0.5))
#define FLIGHT_CAL_MASK        0u
#define NS_PER_SEC             1000000000ull

typedef struct {
    unsigned long loops;
    uint64_t      max_body_ns;
    unsigned long overrun_iters;
    unsigned long skipped_1ms;
    unsigned long sleep_err;
} LoopStats_t;

static double elapsedSeconds(const struct timespec *start,
                             const struct timespec *now)
{
    return (double)(now->tv_sec - start->tv_sec) +
           (double)(now->tv_nsec - start->tv_nsec) / 1e9;
}

static const char *policy_name(AppRtPolicy_t policy)
{
    switch (policy) {
        case APP_RT_POLICY_OK:
            return "ok";
        case APP_RT_POLICY_WARN_AND_CONTINUE:
            return "warn_and_continue";
        case APP_RT_POLICY_FATAL:
            return "fatal";
        default:
            return "unknown";
    }
}

static void print_scheduling_report(const LoopStats_t *settle,
                                    const LoopStats_t *run,
                                    RT_StartStatus_t rt_status,
                                    AppRtPolicy_t policy)
{
    fprintf(stderr, "main: scheduling\n");
    fprintf(stderr,
            "  settle: loops=%lu max_body_ns=%" PRIu64
            " overrun_iters=%lu skipped_1ms=%lu sleep_err=%lu\n",
            settle->loops,
            settle->max_body_ns,
            settle->overrun_iters,
            settle->skipped_1ms,
            settle->sleep_err);
    fprintf(stderr,
            "  run:    loops=%lu max_body_ns=%" PRIu64
            " overrun_iters=%lu skipped_1ms=%lu sleep_err=%lu\n",
            run->loops,
            run->max_body_ns,
            run->overrun_iters,
            run->skipped_1ms,
            run->sleep_err);
    fprintf(stderr,
            "  start_rt: status=%u policy=%s\n",
            (unsigned)rt_status,
            policy_name(policy));
}

static void fail_and_close(const char *msg)
{
    fprintf(stderr, "main: %s\n", msg);
    imu_session_close();
}

static int timespec_ns(const struct timespec *ts, uint64_t *out)
{
    if ((ts == NULL) || (out == NULL)) {
        return -1;
    }

    *out = ((uint64_t)ts->tv_sec * NS_PER_SEC) + (uint64_t)ts->tv_nsec;
    return 0;
}

/*
 * Finishes one 1 kHz body measurement and sleeps to the next deadline.
 * Work (service / optional print) must already have run after t_body0.
 * Returns 0 on success, -1 on clock or sleep failure.
 */
static int finish_tick(LoopStats_t *st, const struct timespec *t_body0)
{
    struct timespec t_body1;
    uint64_t ns0;
    uint64_t ns1;
    int rc;

    if (clock_gettime(CLOCK_MONOTONIC, &t_body1) != 0) {
        return -1;
    }
    if ((timespec_ns(t_body0, &ns0) != 0) ||
        (timespec_ns(&t_body1, &ns1) != 0) ||
        (ns1 < ns0)) {
        return -1;
    }

    st->loops++;
    if ((ns1 - ns0) > st->max_body_ns) {
        st->max_body_ns = ns1 - ns0;
    }

    rc = RT_SleepUntil(LOOP_DT_SEC);
    if (rc > 0) {
        st->overrun_iters++;
        st->skipped_1ms += (unsigned long)rc;
        return 0;
    }
    if (rc < 0) {
        st->sleep_err++;
        return -1;
    }
    return 0;
}

static void print_snapshot_line(const ImuSampleSnapshot_t *sample)
{
    printf("epoch=%" PRIu32 " valid=%" PRIu8 " proc=%" PRIu64,
           sample->configurationEpoch,
           sample->validMask,
           sample->processDecodeCount);

    if (sample->validMask & IMU_GROUP_BIT_ROTATION) {
        printf(" rvseq=%" PRIu64 " rvts=%" PRIu64
               " yaw=%7.3f pitch=%7.3f roll=%7.3f",
               sample->rotationMeta.groupEventSeq,
               sample->rotationMeta.sensorTimeUs,
               sample->yaw,
               sample->pitch,
               sample->roll);
    }

    if (sample->validMask & IMU_GROUP_BIT_ACCEL) {
        printf(" accseq=%" PRIu64
               " ax=%7.3f ay=%7.3f az=%7.3f",
               sample->accelMeta.groupEventSeq,
               sample->ax,
               sample->ay,
               sample->az);
    }

    if (sample->validMask & IMU_GROUP_BIT_GYRO) {
        printf(" gyrseq=%" PRIu64
               " gx=%7.3f gy=%7.3f gz=%7.3f",
               sample->gyroMeta.groupEventSeq,
               sample->gx,
               sample->gy,
               sample->gz);
    }

    printf("\n");
}

int main(void)
{
    LoopStats_t settle = {0};
    LoopStats_t run = {0};
    RT_StartStatus_t rt_status = RT_START_OK;
    AppRtPolicy_t policy = APP_RT_POLICY_OK;
    ImuSampleSnapshot_t sample;
    struct timespec t_body0;
    struct timespec t0;
    struct timespec now;
    int printCount = 0;
    long loopCount = 0;
    double elapsed = 0.0;
    unsigned i;

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

    rt_status = StartRT(RT_PRIORITY, LOOP_DT_SEC);
    policy = app_rt_policy_from_start(rt_status);
    if (policy == APP_RT_POLICY_FATAL) {
        fprintf(stderr,
                "main: StartRT failed fatally (status=%u policy=%s)\n",
                (unsigned)rt_status,
                policy_name(policy));
        imu_session_close();
        return 1;
    }
    if (policy == APP_RT_POLICY_WARN_AND_CONTINUE) {
        fprintf(stderr,
                "main: WARNING: StartRT failed; continuing at default scheduling\n");
    }

    for (i = 0; i < SETTLE_LOOPS; ++i) {
        if (clock_gettime(CLOCK_MONOTONIC, &t_body0) != 0) {
            fprintf(stderr, "main: CLOCK_MONOTONIC read failed\n");
            print_scheduling_report(&settle, &run, rt_status, policy);
            imu_session_close();
            return 1;
        }

        imu_session_service();

        if (finish_tick(&settle, &t_body0) != 0) {
            fprintf(stderr, "main: settle loop timing failed\n");
            print_scheduling_report(&settle, &run, rt_status, policy);
            imu_session_close();
            return 1;
        }
    }

    if (!imu_session_mark_operational()) {
        print_scheduling_report(&settle, &run, rt_status, policy);
        fail_and_close("imu_session_mark_operational failed");
        return 1;
    }

    printf("main: running IMU reader for %d seconds...\n", RUN_DURATION_SEC);

    if (clock_gettime(CLOCK_MONOTONIC, &t0) != 0) {
        fprintf(stderr, "main: CLOCK_MONOTONIC read failed\n");
        print_scheduling_report(&settle, &run, rt_status, policy);
        imu_session_close();
        return 1;
    }

    while (elapsed < (double)RUN_DURATION_SEC) {
        if (clock_gettime(CLOCK_MONOTONIC, &t_body0) != 0) {
            fprintf(stderr, "main: CLOCK_MONOTONIC read failed\n");
            print_scheduling_report(&settle, &run, rt_status, policy);
            imu_session_close();
            return 1;
        }

        imu_session_service();
        loopCount++;

        if ((loopCount % PRINT_EVERY_N_LOOPS) == 0) {
            if (imu_session_get_snapshot(&sample) &&
                sample.version == IMU_SAMPLE_CONTRACT_VERSION) {
                print_snapshot_line(&sample);
                printCount++;
            }
        }

        if (finish_tick(&run, &t_body0) != 0) {
            fprintf(stderr, "main: run loop timing failed\n");
            print_scheduling_report(&settle, &run, rt_status, policy);
            imu_session_close();
            return 1;
        }

        if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
            fprintf(stderr, "main: CLOCK_MONOTONIC read failed\n");
            print_scheduling_report(&settle, &run, rt_status, policy);
            imu_session_close();
            return 1;
        }
        elapsed = elapsedSeconds(&t0, &now);
    }

    print_scheduling_report(&settle, &run, rt_status, policy);
    printf("main: finished. Printed %d lines.\n", printCount);

    imu_session_close();
    return 0;
}