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

#define RT_PRIORITY         90      /* keep in sync with app/main.c */
#define SETTLE_SEC          0.3     /* service-only drain after start */
#define LOOP_DT_SEC         0.001   /* 1 kHz service cadence */
#define LOG_DECIMATION      10      /* 1 kHz / 10 = 100 Hz records */
#define COUNTS_PER_REV      8192.0  /* 2048 PPR x 4 */
#define RAD2DEG             57.29577951308232

/* Placeholders: override in your copy per build */
#define ARM_LENGTH_M        0.0     /* axis to sensor center (placeholder) */
#define RUN_NOTES           ""

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

/* Shared encoder snapshot: written by the encoder thread, read by the RT thread. */
static pthread_mutex_t g_enc_mtx;
static amt102_state_t  g_enc_snap;
static volatile int    g_enc_err = 0;

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static int mutex_init_pi(pthread_mutex_t *m)
{
    pthread_mutexattr_t a;
    bool pi = false;

    if (pthread_mutexattr_init(&a) == 0) {
        if (pthread_mutexattr_setprotocol(&a, PTHREAD_PRIO_INHERIT) == 0) {
            pi = true;
        }
    }

    int rc = pthread_mutex_init(m, pi ? &a : NULL);
    pthread_mutexattr_destroy(&a);
    return rc;
}

static void *encoder_thread(void *arg)
{
    amt102_t *enc = (amt102_t *)arg;

    while (!g_stop) {
        int n = amt102_poll(enc, 100);
        if (n < 0) {
            fprintf(stderr, "\nencoder: event read failed: %s\n", strerror(errno));
            g_enc_err = 1;
            g_stop = 1;
            break;
        }

        amt102_state_t st;
        amt102_get_state(enc, &st);
        pthread_mutex_lock(&g_enc_mtx);
        g_enc_snap = st;
        pthread_mutex_unlock(&g_enc_mtx);
    }
    return NULL;
}

static void usage(const char *prog)
{
    fprintf(stderr,
            "usage: %s [-o out.csv] [-d seconds]\n"
            "  -o out.csv    output CSV (default: bno_validate_YYYYMMDD_HHMMSS.csv)\n"
            "  -d seconds    log duration after settle; 0 = run until Ctrl+C (default 0)\n",
            prog);
}

static void default_path(char *buf, size_t len)
{
    char stamp[16];
    time_t t = time(NULL);
    struct tm tmv;

    localtime_r(&t, &tmv);
    strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tmv);
    snprintf(buf, len, "bno_validate_%s.csv", stamp);
}

int main(int argc, char **argv)
{
    const char *out_path = NULL;
    double duration_sec = 0.0;
    char pathbuf[64];
    int opt;

    while ((opt = getopt(argc, argv, "o:d:h")) != -1) {
        switch (opt) {
        case 'o':
            out_path = optarg;
            break;
        case 'd':
            duration_sec = strtod(optarg, NULL);
            if (duration_sec < 0.0) {
                usage(argv[0]);
                return 1;
            }
            break;
        default:
            usage(argv[0]);
            return (opt == 'h') ? 0 : 1;
        }
    }
    if (!out_path) {
        default_path(pathbuf, sizeof(pathbuf));
        out_path = pathbuf;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    amt102_t *enc = NULL;
    if (amt102_open(&enc) != 0) {
        fprintf(stderr, "error: cannot open AMT102 on gpiochip0: %s\n", strerror(errno));
        fprintf(stderr, "hint: check that lines 17/27/22 are unused (gpioinfo gpiochip0)\n");
        return 2;
    }

    if (csv_log_start(out_path, ARM_LENGTH_M, RUN_NOTES) != 0) {
        fprintf(stderr, "error: cannot open %s: %s\n", out_path, strerror(errno));
        amt102_close(enc);
        return 2;
    }

    if (!sensor_validate_start()) {
        fprintf(stderr, "error: sensor_validate_start failed (BNO085/SPI)\n");
        fprintf(stderr, "hint: is bno_app or another sh2 consumer still running?"
                        " one HAL instance per process\n");
        csv_log_stop(NULL);
        amt102_close(enc);
        return 2;
    }

    if (mutex_init_pi(&g_enc_mtx) != 0) {
        pthread_mutex_init(&g_enc_mtx, NULL);
    }
    memset(&g_enc_snap, 0, sizeof(g_enc_snap));

    printf("bno_validate: csv=%s\n", out_path);
    printf("encoder AMT102 2048 PPR (8192 cpr) | imu RV+LA+GyroCal @100 Hz (dynamic cal off) | tick 100 Hz (struct v%u)\n",
           IMU_VALIDATE_STRUCT_VERSION);
    if (duration_sec > 0.0) {
        printf("logging %.1f s after %.1f s settle; Ctrl+C stops early\n",
               duration_sec, SETTLE_SEC);
    } else {
        printf("logging until Ctrl+C (%.1f s settle first)\n", SETTLE_SEC);
    }

    pthread_t enc_tid;
    if (pthread_create(&enc_tid, NULL, encoder_thread, enc) != 0) {
        fprintf(stderr, "error: cannot start encoder thread\n");
        csv_log_stop(NULL);
        sensor_validate_stop();
        amt102_close(enc);
        return 2;
    }

    /* Threads exist; now this thread (only) becomes SCHED_FIFO. */
    if (StartRT(RT_PRIORITY, LOOP_DT_SEC) != 0) {
        fprintf(stderr, "main: WARNING: StartRT failed; continuing at default scheduling\n");
    }

    /* Settle: service the session, do not log yet. */
    uint64_t t0 = now_ns();
    while (!g_stop && (now_ns() - t0) < (uint64_t)(SETTLE_SEC * 1e9)) {
        sensor_validate_service();
        RT_SleepUntil(LOOP_DT_SEC);
    }

    sensor_validate_resetSeq();

    uint64_t t_log0   = now_ns();
    uint64_t last_1hz = t_log0;
    uint64_t ticks    = 0;

    amt102_state_t st_prev;
    pthread_mutex_lock(&g_enc_mtx);
    st_prev = g_enc_snap;
    pthread_mutex_unlock(&g_enc_mtx);
    uint32_t imu_seq_prev = 0;

    unsigned loop = 0;
    while (!g_stop) {
        sensor_validate_service();

        if (duration_sec > 0.0 &&
            (now_ns() - t_log0) >= (uint64_t)(duration_sec * 1e9)) {
            break;
        }

        if ((loop % LOG_DECIMATION) == 0) {
            amt102_state_t st;
            pthread_mutex_lock(&g_enc_mtx);
            st = g_enc_snap;
            pthread_mutex_unlock(&g_enc_mtx);

            CsvRecord_t rec;
            memset(&rec, 0, sizeof(rec));
            rec.host_ts_ns      = now_ns();
            rec.enc_count       = st.count;
            rec.enc_angle_deg   = (double)st.count * (360.0 / COUNTS_PER_REV);
            rec.enc_event_ts_ns = st.last_event_ts_ns;
            rec.enc_x_pulses    = st.x_pulses;
            rec.enc_invalid     = st.invalid;
            rec.enc_edges_ab    = st.a_edges + st.b_edges;

            (void)sensor_validate_getLatestSample(&rec.imu);
            rec.drops = csv_log_dropped();

            (void)csv_log_push(&rec);
            ticks++;

            /* Heartbeat once per second */
            uint64_t t_now = now_ns();
            if ((t_now - last_1hz) >= 1000000000ull) {
                int64_t d_enc = st.count - st_prev.count;
                uint32_t d_imu = rec.imu.seq - imu_seq_prev;
                st_prev = st;
                imu_seq_prev = rec.imu.seq;

                double sec = (double)(t_now - t_log0) / 1e9;
                printf("[t=%5.1fs] enc=%+7.2f deg (d=%+5" PRId64 ") | "
                       "q=(%+.3f,%+.3f,%+.3f,%+.3f) | "
                       "ypr=(%+6.2f,%+6.2f,%+6.2f) deg | "
                       "acc=%u err=%.2frad | drops=%" PRIu64 "\n",
                       sec,
                       rec.enc_angle_deg,
                       d_enc,
                       rec.imu.rv_qw, rec.imu.rv_qx, rec.imu.rv_qy, rec.imu.rv_qz,
                       (double)(rec.imu.yaw * RAD2DEG),
                       (double)(rec.imu.pitch * RAD2DEG),
                       (double)(rec.imu.roll * RAD2DEG),
                       rec.imu.rv.status,
                       (double)rec.imu.rvErrRad,
                       rec.drops);
                fflush(stdout);
                last_1hz = t_now;
            }
        }

        RT_SleepUntil(LOOP_DT_SEC);
        loop++;
    }

    printf("\nsensor_validate: stopping...\n");

    /* Stop threads and tear down */
    g_stop = 1;
    pthread_join(enc_tid, NULL);

    CsvLogStats_t stats;
    csv_log_stop(&stats);

    sensor_validate_stop();
    amt102_close(enc);

    printf("done: records=%" PRIu64 " drops=%" PRIu64 " duration=%.3f s (%.2f Hz)\n",
           stats.records_written,
           stats.records_dropped,
           stats.duration_sec,
           stats.duration_sec > 0 ? (double)stats.records_written / stats.duration_sec : 0.0);

    return (g_enc_err != 0) ? 1 : 0;
}
