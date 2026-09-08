/*
 * validate_main.c — synchronized 100 Hz encoder + BNO085 IMU validation harness.
 *
 * Runs a single 1 kHz real-time thread (SCHED_FIFO, priority 90) that:
 *   1. Polls the AMT102 encoder via libgpiod edge events on GPIO 5, 6, 13
 *      and decodes x4 quadrature via quad_decode.c;
 *   2. Services the SH-2 session (SPI + H_INTN on GPIO 24) at 1 kHz via
 *      sensor_validate.c, decoding Rotation Vector, Linear Acceleration,
 *      and Calibrated Gyroscope at 100 Hz;
 *   3. Once every 10 ms (100 Hz), snapshots the current encoder count,
 *      combines it with the latest ImuValidateSample_t into a CsvRecord_t,
 *      and pushes the record into a lock-free ring buffer (csv_log.c).
 *      A separate SCHED_OTHER thread drains the ring to CSV on disk.
 *
 * Synchronization guarantee:
 *   Both sensors share the Pi's CLOCK_MONOTONIC domain. The encoder
 *   event timestamp (kernel event stamp from libgpiod) and the IMU
 *   arrival timestamp (CLOCK_MONOTONIC taken at sh2 decode time) live
 *   in the same nanosecond clock domain. The row's host_ts_ns is the
 *   exact CLOCK_MONOTONIC tick at which the pair was assembled.
 *
 * Usage:
 *   sudo ./bin/bno_validate -o run1.csv [-d seconds] [-a arm_m] [-n "notes"]
 *
 * Must not run concurrently with bno_app (one SPI HAL instance per process).
 *
 * Build: make validate (Makefile)
 */

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <inttypes.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "validation/amt102.h"
#include "validation/validate_logger.h"
#include "validation/validate_contract.h"
#include "app/realtime.h"
#include "validation/validate_sensor.h"

/* Real-time loop parameters */
#define RT_PRIORITY       90
#define LOOP_PERIOD_US    1000u                  /* 1 kHz service cadence */
#define LOOP_PERIOD_NS    (LOOP_PERIOD_US * 1000ull)
#define LOG_DIVIDER       10u                    /* 1 kHz / 10 = 100 Hz log cadence */
#define SETTLE_MS         500u                   /* post-open settle before logging */

/* Defaults */
#define DEFAULT_CSV_PATH  "validation.csv"
#define DEFAULT_ARM_LEN_M 0.25                   /* 25 cm default arm length */

static volatile sig_atomic_t sRunning = 1;

static void sigHandler(int sig)
{
    (void)sig;
    sRunning = 0;
}

static uint64_t nowns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000000000ull) + (uint64_t)ts.tv_nsec;
}

static void printUsage(const char *prog)
{
    fprintf(stderr,
            "Usage: %s [options]\n"
            "Options:\n"
            "  -o <file>     Output CSV path (default: %s)\n"
            "  -d <seconds>  Capture duration in seconds (default: run until Ctrl-C)\n"
            "  -a <meters>   Mechanical arm length in meters (default: %.2f)\n"
            "  -n <notes>    Free-form notes string for CSV header\n"
            "  -h            Show this help\n",
            prog, DEFAULT_CSV_PATH, DEFAULT_ARM_LEN_M);
}

int main(int argc, char **argv)
{
    const char *csvPath = DEFAULT_CSV_PATH;
    double armLen_m     = DEFAULT_ARM_LEN_M;
    const char *notes   = "";
    double duration_s   = 0.0; /* 0 = indefinite */

    int opt;
    while ((opt = getopt(argc, argv, "o:d:a:n:h")) != -1) {
        switch (opt) {
            case 'o': csvPath    = optarg; break;
            case 'd': duration_s = atof(optarg); break;
            case 'a': armLen_m   = atof(optarg); break;
            case 'n': notes      = optarg; break;
            case 'h':
            default:
                printUsage(argv[0]);
                return (opt == 'h') ? 0 : 1;
        }
    }

    printf("=== AMT102 Encoder vs BNO085 IMU Validation Harness ===\n");
    printf("CSV output:   %s\n", csvPath);
    printf("Arm length:   %.4f m\n", armLen_m);
    if (duration_s > 0.0) {
        printf("Duration:     %.1f s\n", duration_s);
    } else {
        printf("Duration:     indefinite (Ctrl-C to stop)\n");
    }
    if (notes && notes[0]) {
        printf("Notes:        %s\n", notes);
    }

    /* Signal handlers */
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sigHandler;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    /* 1. Start encoder hardware */
    printf("[1/4] Initializing AMT102 encoder on GPIO 5, 6, 13...\n");
    if (amt102_init() != 0) {
        fprintf(stderr, "error: amt102_init failed (%s)\n", strerror(errno));
        return 1;
    }

    /* 2. Start IMU session */
    printf("[2/4] Opening BNO085 SH-2 session over SPI @ 3 MHz...\n");
    if (!sensor_validate_start()) {
        fprintf(stderr, "error: sensor_validate_start failed (is bno_app running?)\n");
        amt102_close();
        return 1;
    }

    /* 3. Start CSV logger thread */
    printf("[3/4] Starting CSV logger thread -> %s...\n", csvPath);
    if (csv_log_start(csvPath, armLen_m, notes) != 0) {
        fprintf(stderr, "error: csv_log_start failed (%s)\n", strerror(errno));
        sensor_validate_stop();
        amt102_close();
        return 1;
    }

    /* 4. Elevate to SCHED_FIFO */
    printf("[4/4] Elevating main thread to SCHED_FIFO priority %d...\n", RT_PRIORITY);
    StartRT(RT_PRIORITY, LOOP_PERIOD_US);

    /*
     * Settle window: drain initial SHTP advertisements and encoder lines
     * before logging starts, then reset counters so log row 0 starts clean.
     */
    printf("Settling for %u ms...\n", SETTLE_MS);
    uint64_t settleEnd = nowns() + ((uint64_t)SETTLE_MS * 1000000ull);
    while (sRunning && nowns() < settleEnd) {
        amt102_poll();
        sensor_validate_service();
        RT_SleepUntil(LOOP_PERIOD_US);
    }

    amt102_reset_count();
    sensor_validate_resetSeq();

    printf("Capture running at 100 Hz. Press Ctrl-C to finish.\n");

    uint64_t tStart      = nowns();
    uint64_t tMaxEnd     = (duration_s > 0.0)
                               ? tStart + (uint64_t)(duration_s * 1e9)
                               : 0;
    uint32_t tickDivider = 0;
    uint64_t tickCount   = 0;

    /* Main real-time 1 kHz loop */
    while (sRunning) {
        /* Service encoder: read all pending edge events */
        amt102_poll();

        /* Service BNO085: drain pending SHTP packets */
        sensor_validate_service();

        /* 100 Hz log decimation (every 10 ticks) */
        if (++tickDivider >= LOG_DIVIDER) {
            tickDivider = 0;
            tickCount++;

            CsvRecord_t rec;
            memset(&rec, 0, sizeof(rec));

            rec.host_ts_ns = nowns();

            /* Encoder snapshot */
            Amt102Snapshot_t snap;
            amt102_snapshot(&snap);
            rec.enc_count       = snap.count;
            rec.enc_angle_deg   = snap.angle_deg;
            rec.enc_event_ts_ns = snap.last_edge_ts_ns;
            rec.enc_x_pulses    = snap.x_pulses;
            rec.enc_invalid     = snap.invalid_transitions;
            rec.enc_edges_ab    = snap.edges_ab;

            /* IMU snapshot */
            sensor_validate_getLatestSample(&rec.imu);

            rec.drops = csv_log_dropped();

            csv_log_push(&rec);

            /* Console heartbeat once per second */
            if ((tickCount % 100) == 0) {
                printf("  [%5.1f s] enc=%+8" PRId64 " (%+7.2f deg)  "
                       "imu_yaw=%+6.2f deg  rv_acc=%u (%.3f rad)  drops=%" PRIu64 "\r",
                       (double)(rec.host_ts_ns - tStart) / 1e9,
                       rec.enc_count,
                       rec.enc_angle_deg,
                       (double)(rec.imu.yaw * 57.29577951308232),
                       rec.imu.rv.status,
                       rec.imu.rvErrRad,
                       rec.drops);
                fflush(stdout);
            }
        }

        if (tMaxEnd > 0 && nowns() >= tMaxEnd) {
            printf("\nDuration limit (%.1f s) reached.\n", duration_s);
            break;
        }

        RT_SleepUntil(LOOP_PERIOD_US);
    }

    printf("\nStopping capture...\n");

    /* Teardown in reverse order */
    CsvLogStats_t stats;
    csv_log_stop(&stats);
    sensor_validate_stop();
    amt102_close();

    printf("\n=== Capture Summary ===\n");
    printf("Output file:      %s\n", csvPath);
    printf("Records written:  %" PRIu64 "\n", stats.records_written);
    printf("Records dropped:  %" PRIu64 "\n", stats.records_dropped);
    printf("Total duration:   %.3f s\n", stats.duration_sec);
    if (stats.records_written > 0 && stats.duration_sec > 0.0) {
        printf("Average rate:     %.2f Hz\n",
               (double)stats.records_written / stats.duration_sec);
    }

    return 0;
}
