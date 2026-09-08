/*
 * main.c — sensor_validate: synchronized AMT102 encoder + BNO085 capture.
 *
 * Three-thread model:
 *   - encoder thread (SCHED_OTHER): amt102_poll loop, publishes an
 *     amt102_state_t snapshot under a mutex
 *   - RT thread (this thread, SCHED_FIFO via app/realtime.c): 1 kHz
 *     sh2 service; every 10th iteration builds one 100 Hz CsvRecord
 *     (encoder snapshot + latest ImuValidateSample_t + host stamp) and
 *     pushes it to the csv_log ring
 *   - logger thread (SCHED_OTHER, owned by csv_log): drains the ring
 *     to the CSV file
 *
 * Threads are created BEFORE StartRT so they inherit default
 * scheduling; only this thread becomes SCHED_FIFO.
 *
 * Reuses the production decode chain (sh2/, app/sh2_hal_rpi.c,
 * app/realtime.c) plus the validated encoder modules in this
 * directory. Replaces bno_app while running: one SPI HAL instance
 * per process.
 *
 * Build: make validate   (Makefile)
 * Usage: ./bin/bno_validate [-o out.csv] [-d seconds]
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

#define SERVICE_PERIOD_US   1000      /* 1 kHz service cadence: ~1 ms sleep */
#define CONSUMER_DIVIDER    10        /* run consumer every 10 ticks = 100 Hz */
#define WARMUP_DRAIN_MS     10000     /* 10 s motion/warm-up before CSV logging */
#define DEFAULT_OUT_PATH    "validate.csv"

/* Rig facts (placeholders — override with -a / -n) */
#define DEFAULT_ARM_M       0.0       /* unknown arm length by default */
#define DEFAULT_NOTES       ""

static volatile sig_atomic_t sRunning = 1;
static void onSigint(int sig)
{
    (void)sig;
    sRunning = 0;
}

static uint64_t hostNowNs(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000000000ULL) +
           ((uint64_t)ts.tv_nsec);
}

/* ------------------------------------------------------------------ */
/* Encoder polling thread (SCHED_OTHER, priority default)             */
/* ------------------------------------------------------------------ */

typedef struct {
    pthread_mutex_t lock;
    amt102_state_t  st;
    bool            alive;
} SharedEncoder_t;

static SharedEncoder_t sEncShare = {
    .lock  = PTHREAD_MUTEX_INITIALIZER,
    .alive = false,
};

static void *encoderThread(void *arg)
{
    amt102_t *enc = (amt102_t *)arg;
    amt102_state_t local;

    while (sRunning) {
        /*
         * 5 ms poll timeout keeps event queue small (never overflows the
         * 256-event batch buffer even at peak swing angular velocity).
         */
        int n = amt102_poll(enc, 5);
        if (n < 0 && errno != EINTR) {
            fprintf(stderr, "sensor_validate: amt102_poll failed: %s\n",
                    strerror(errno));
            break;
        }
        amt102_get_state(enc, &local);

        pthread_mutex_lock(&sEncShare.lock);
        sEncShare.st = local;
        sEncShare.alive = true;
        pthread_mutex_unlock(&sEncShare.lock);
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* CLI argument parsing                                               */
/* ------------------------------------------------------------------ */

typedef struct {
    const char *outPath;
    unsigned    durationSec;   /* 0 = run until Ctrl-C */
    double      armLengthM;
    const char *notes;
} Args_t;

static bool parseArgs(int argc, char **argv, Args_t *out)
{
    out->outPath     = DEFAULT_OUT_PATH;
    out->durationSec = 0;
    out->armLengthM  = DEFAULT_ARM_M;
    out->notes       = DEFAULT_NOTES;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-o") == 0) {
            if (i + 1 >= argc) return false;
            out->outPath = argv[++i];
        } else if (strcmp(argv[i], "-d") == 0) {
            if (i + 1 >= argc) return false;
            char *end;
            unsigned long v = strtoul(argv[++i], &end, 10);
            if (*end != '\0' || v == 0) return false;
            out->durationSec = (unsigned)v;
        } else if (strcmp(argv[i], "-a") == 0) {
            if (i + 1 >= argc) return false;
            char *end;
            double v = strtod(argv[++i], &end);
            if (*end != '\0' || v < 0.0) return false;
            out->armLengthM = v;
        } else if (strcmp(argv[i], "-n") == 0) {
            if (i + 1 >= argc) return false;
            out->notes = argv[++i];
        } else if (strcmp(argv[i], "-h") == 0 ||
                   strcmp(argv[i], "--help") == 0) {
            return false;
        } else {
            fprintf(stderr, "unknown option: %s\n", argv[i]);
            return false;
        }
    }
    return true;
}

static void printUsage(const char *prog)
{
    fprintf(stderr,
            "usage: %s [-o file.csv] [-d seconds] [-a arm_length_m] "
            "[-n \"run notes\"]\n"
            "\n"
            "  -o file.csv   output path (default: %s)\n"
            "  -d seconds    capture duration (default: run until Ctrl-C)\n"
            "  -a meters     arm length from swing axis to IMU center (default: 0.0)\n"
            "  -n \"notes\"    free-form notes string saved in the CSV header\n",
            prog, DEFAULT_OUT_PATH);
}

/* ------------------------------------------------------------------ */
/* Main                                                               */
/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
    Args_t args;
    if (!parseArgs(argc, argv, &args)) {
        printUsage(argv[0]);
        return 1;
    }

    signal(SIGINT, onSigint);
    signal(SIGTERM, onSigint);

    printf("=== sensor_validate: synchronized encoder + IMU capture ===\n\n");
    printf("Configuration:\n");
    printf("  output CSV:     %s\n", args.outPath);
    if (args.durationSec > 0) {
        printf("  duration:       %u s (fixed)\n", args.durationSec);
    } else {
        printf("  duration:       indefinite (press Ctrl-C to finish)\n");
    }
    printf("  arm length:     %.4f m%s\n", args.armLengthM,
           (args.armLengthM == 0.0) ? "  [unspecified - set with -a]" : "");
    if (args.notes[0] != '\0') {
        printf("  run notes:      %s\n", args.notes);
    }
    printf("  cadence:        100 Hz synchronized tick (1 kHz service loop)\n\n");

    /* Step 1: Open the AMT102 encoder via libgpiod */
    printf("opening AMT102 encoder on GPIO 17 (A), 27 (B), 22 (X)...\n");
    amt102_t *enc = NULL;
    if (amt102_open(&enc) != 0) {
        perror("error: amt102_open failed");
        fprintf(stderr,
                "hint: is another process holding GPIO 17, 27 or 22? "
                "Check: gpioinfo gpiochip0 | grep -E '17|27|22'\n");
        return 1;
    }

    /* Step 2: Open the BNO085 SH-2 session over SPI */
    printf("opening BNO085 SH-2 session over SPI @ 3 MHz...\n");
    if (!sensor_validate_start()) {
        fprintf(stderr,
                "error: sensor_validate_start failed\n"
                "hint: is bno_app or another sh2 consumer still running? "
                "one SPI HAL instance per process\n");
        amt102_close(enc);
        return 1;
    }

    /* Step 3: Open the CSV logger */
    printf("opening CSV logger: %s...\n", args.outPath);
    if (csv_log_start(args.outPath, args.armLengthM, args.notes) != 0) {
        fprintf(stderr, "error: csv_log_start failed: %s\n",
                strerror(errno));
        sensor_validate_stop();
        amt102_close(enc);
        return 1;
    }

    /* Step 4: Spawn the encoder polling thread (SCHED_OTHER) BEFORE StartRT */
    pthread_t encThread;
    if (pthread_create(&encThread, NULL, encoderThread, enc) != 0) {
        fprintf(stderr, "error: pthread_create failed: %s\n",
                strerror(errno));
        csv_log_stop(NULL);
        sensor_validate_stop();
        amt102_close(enc);
        return 1;
    }

    /*
     * Step 5: Elevate THIS thread (only) to real-time priority (SCHED_FIFO 90).
     * The encoder thread and the logger thread remain SCHED_OTHER, so SD-card
     * write stalls cannot preempt or perturb the 1 kHz service cadence.
     */
    StartRT(90, SERVICE_PERIOD_US);

    /*
     * Step 6: Warm-up drain.
     * Service both sensors for WARMUP_DRAIN_MS (~10 s). The rotation vector
     * needs ~10 s of motion to converge after a reset under the all-off
     * dynamic cal policy (observed on this unit; see bno/calibration/readme.md).
     * The operator can move the fixture gently during this window.
     */
    printf("warming up sensors for %d s (move fixture gently to settle RV)...\n",
           WARMUP_DRAIN_MS / 1000);
    uint64_t tWarmupEnd = hostNowNs() + (uint64_t)WARMUP_DRAIN_MS * 1000000000ULL;
    while (sRunning && hostNowNs() < tWarmupEnd) {
        sensor_validate_service();
        RT_SleepUntil(SERVICE_PERIOD_US);
    }
    sensor_validate_resetSeq();
    printf("warm-up complete; entering 100 Hz capture loop\n");
    printf("press Ctrl-C to finish\n\n");

    /*
     * Step 7: Acquisition loop.
     * Runs at 1 kHz (~1 ms). Every CONSUMER_DIVIDER ticks (~10 ms = 100 Hz):
     *   - reads the latest encoder snapshot from the mutex
     *   - reads the latest ImuValidateSample_t from the SH-2 session
     *   - captures a fresh CLOCK_MONOTONIC host timestamp
     *   - pushes the combined row into the csv_log ring
     */
    uint32_t tickCount    = 0;
    uint32_t rowsPushed   = 0;
    uint64_t tCaptureStart = hostNowNs();
    uint64_t tCaptureEnd   = (args.durationSec > 0)
        ? tCaptureStart + (uint64_t)args.durationSec * 1000000000ULL
        : 0;

    while (sRunning) {
        sensor_validate_service();

        tickCount++;
        if ((tickCount % CONSUMER_DIVIDER) == 0) {
            CsvRecord_t rec;
            memset(&rec, 0, sizeof(rec));

            rec.host_ts_ns = hostNowNs();

            /* Snapshot the encoder */
            pthread_mutex_lock(&sEncShare.lock);
            amt102_state_t est = sEncShare.st;
            pthread_mutex_unlock(&sEncShare.lock);

            rec.enc_count        = est.count;
            rec.enc_angle_deg    = (double)est.count * (360.0 / (double)AMT102_COUNTS_PER_REV);
            rec.enc_event_ts_ns  = est.last_event_ts_ns;
            rec.enc_x_pulses     = est.x_pulses;
            rec.enc_invalid      = est.invalid;
            rec.enc_edges_ab     = est.a_edges + est.b_edges;

            /* Snapshot the IMU (per-group timestamps preserved) */
            sensor_validate_getLatestSample(&rec.imu);

            rec.drops = csv_log_dropped();

            csv_log_push(&rec);
            rowsPushed++;

            /* Live console heartbeat: refresh once per second (every 100 rows) */
            if ((rowsPushed % 100) == 0) {
                printf("  [t=%5.1f s] enc=%+8.2f deg (cnt %+" PRId64 ") | "
                       "rv_yaw=%+6.2f deg (acc %u, err %.2f rad) | "
                       "rows=%-6u drops=%" PRIu64 "\r",
                       (double)(rec.host_ts_ns - tCaptureStart) / 1e9,
                       rec.enc_angle_deg,
                       rec.enc_count,
                       (double)(rec.imu.yaw * 57.29577951308232),
                       rec.imu.rv.status,
                       (double)rec.imu.rvErrRad,
                       rowsPushed,
                       rec.drops);
                fflush(stdout);
            }

            if (tCaptureEnd > 0 && hostNowNs() >= tCaptureEnd) {
                printf("\nfixed duration (%u s) elapsed\n", args.durationSec);
                break;
            }
        }

        RT_SleepUntil(SERVICE_PERIOD_US);
    }

    printf("\nstopping capture; closing sessions...\n");

    /* Stop threads and tear down */
    sRunning = 0;
    pthread_join(encThread, NULL);

    CsvLogStats_t stats;
    csv_log_stop(&stats);

    sensor_validate_stop();
    amt102_close(enc);

    printf("\n=== Capture complete ===\n");
    printf("  CSV file:         %s\n", args.outPath);
    printf("  Rows written:     %" PRIu64 "\n", stats.records_written);
    printf("  Ring drops:       %" PRIu64 "\n", stats.records_dropped);
    printf("  Duration:         %.3f s\n", stats.duration_sec);
    if (stats.duration_sec > 0.0) {
        printf("  Effective rate:   %.1f Hz\n",
               (double)stats.records_written / stats.duration_sec);
    }
    printf("========================\n");
    return 0;
}
