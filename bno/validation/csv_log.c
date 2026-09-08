/*
 * csv_log.c — bounded-ring CSV logger for the validation binary.
 *
 * Producer: the RT loop (csv_log_push, one 100 Hz record at a time).
 * Consumer: this module's own SCHED_OTHER thread (created in
 * csv_log_start BEFORE the caller calls StartRT, so it inherits
 * default scheduling, not SCHED_FIFO).
 *
 * Overflow policy: drop-newest. A dropped tick is visible to analysis
 * through the per-record cumulative 'drops' column and the footer.
 *
 * The producer/consumer mutex uses PTHREAD_PRIO_INHERIT when the
 * platform supports it so the SCHED_FIFO producer cannot be delayed
 * behind the logger thread's critical section.
 *
 * Build: make validate   (validation/Makefile)
 */

#include <errno.h>
#include <inttypes.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "csv_log.h"

#define RING_SIZE   4096u   /* 4096 records ~= 41 s of headroom at 100 Hz */
#define FLUSH_EVERY 128u    /* fflush cadence for the logger thread */
#define IOBUF_BYTES (1u << 18)

static const char CSV_COLUMNS[] =
    "host_ts_ns,enc_count,enc_angle_deg,enc_event_ts_ns,enc_x_pulses,"
    "enc_invalid,enc_edges_ab,"
    "rv_sensor_ts_us,rv_host_ts_ns,rv_seq,rv_qw,rv_qx,rv_qy,rv_qz,yaw,pitch,roll,rv_accuracy,"
    "acc_sensor_ts_us,acc_host_ts_ns,acc_seq,ax,ay,az,"
    "gyr_sensor_ts_us,gyr_host_ts_ns,gyr_seq,gx,gy,gz,"
    "valid_mask,drops";

typedef struct {
    CsvRecord_t    ring[RING_SIZE];
    unsigned       head, tail, count;
    pthread_mutex_t mtx;
    pthread_cond_t  cv;
    bool           running;
    bool           inited;
    bool           thread_started;
    pthread_t      thread;
    FILE          *fp;
    uint64_t       written, dropped;
    uint64_t       start_ns, stop_ns;
    double         duration_sec;
} csv_log_t;

static csv_log_t S;
static char      sIoBuf[IOBUF_BYTES];

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

static void write_header(double arm_len_m, const char *notes)
{
    char tbuf[32];
    time_t t = time(NULL);
    struct tm tmv;
    localtime_r(&t, &tmv);
    strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &tmv);

    fprintf(S.fp, "# sensor_validate synchronized capture\n");
    fprintf(S.fp, "# started_local: %s\n", tbuf);
    fprintf(S.fp, "# encoder: AMT102 2048 PPR (DIP all Off), x4 -> 8192 counts/rev\n");
    fprintf(S.fp, "# imu: BNO085 rotation vector + linear accel + calibrated gyro, 100 Hz each\n");
    fprintf(S.fp, "# tick: 100 Hz (every 10th iteration of the 1 kHz sh2 service loop)\n");
    fprintf(S.fp, "# arm_length_m: %.3f\n", arm_len_m);
    fprintf(S.fp, "# notes: %s\n", (notes && notes[0]) ? notes : "none");
    fprintf(S.fp, "# clocks: host_ts_ns/*_host_ts_ns/enc_event_ts_ns = CLOCK_MONOTONIC ns"
                 " (kernel event stamps, same domain); *_sensor_ts_us = BNO085 device stamps\n");
    fprintf(S.fp, "# units: enc_angle_deg = deg; rv_qw..rv_qz = unit quat; yaw/pitch/roll/rv_accuracy = rad;"
                 " ax..az = m/s^2; gx..gz = rad/s\n");
    fprintf(S.fp, "# *_seq = device per-sensor rolling sequence (mod 256)\n");
    fprintf(S.fp, "# enc_angle_deg is continuous (unwrapped): enc_count * 360/8192\n");
    fprintf(S.fp, "# drops = cumulative ring drops since start (missing ticks)\n");
    fprintf(S.fp, "%s\n", CSV_COLUMNS);
}

static void write_record(const CsvRecord_t *r)
{
    fprintf(S.fp,
        "%" PRIu64 ",%" PRId64 ",%.4f,%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ","
        "%" PRIu64 ",%" PRIu64 ",%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%" PRIu64 ",%" PRIu64 ",%u,%.6f,%.6f,%.6f,"
        "%" PRIu64 ",%" PRIu64 ",%u,%.6f,%.6f,%.6f,"
        "%u,%" PRIu64 "\n",
        r->host_ts_ns,
        r->enc_count,
        r->enc_angle_deg,
        r->enc_event_ts_ns,
        r->enc_x_pulses,
        r->enc_invalid,
        r->enc_edges_ab,
        r->imu.rv.sensor_ts_us,
        r->imu.rv.host_ts_ns,
        (unsigned)r->imu.rv.report_seq,
        r->imu.rv_qw,
        r->imu.rv_qx,
        r->imu.rv_qy,
        r->imu.rv_qz,
        r->imu.yaw,
        r->imu.pitch,
        r->imu.roll,
        r->imu.rvErrRad,
        r->imu.accel.sensor_ts_us,
        r->imu.accel.host_ts_ns,
        (unsigned)r->imu.accel.report_seq,
        r->imu.ax,
        r->imu.ay,
        r->imu.az,
        r->imu.gyro.sensor_ts_us,
        r->imu.gyro.host_ts_ns,
        (unsigned)r->imu.gyro.report_seq,
        r->imu.gx,
        r->imu.gy,
        r->imu.gz,
        (unsigned)r->imu.validMask,
        r->drops);
}

static void *logger_thread(void *arg)
{
    unsigned since_flush = 0;

    (void)arg;
    for (;;) {
        CsvRecord_t rec;
        bool have = false;
        bool done = false;

        pthread_mutex_lock(&S.mtx);
        while (S.count == 0 && S.running) {
            pthread_cond_wait(&S.cv, &S.mtx);
        }

        if (S.count > 0) {
            rec = S.ring[S.tail];
            S.tail = (S.tail + 1) % RING_SIZE;
            S.count--;
            S.written++;
            have = true;
        }
        if (S.count == 0 && !S.running) {
            done = true;
        }
        pthread_mutex_unlock(&S.mtx);

        if (have) {
            write_record(&rec);
            if (++since_flush >= FLUSH_EVERY) {
                fflush(S.fp);
                since_flush = 0;
            }
        }
        if (done) {
            break;
        }
    }
    return NULL;
}

int csv_log_start(const char *path, double arm_len_m, const char *notes)
{
    memset(&S, 0, sizeof(S));

    S.fp = fopen(path, "w");
    if (!S.fp) {
        return -1;
    }
    setvbuf(S.fp, sIoBuf, _IOFBF, sizeof(sIoBuf));

    write_header(arm_len_m, notes);

    if (mutex_init_pi(&S.mtx) != 0) {
        fclose(S.fp);
        S.fp = NULL;
        return -1;
    }
    pthread_cond_init(&S.cv, NULL);
    S.inited = true;

    S.running = true;
    S.start_ns = now_ns();
    if (pthread_create(&S.thread, NULL, logger_thread, NULL) != 0) {
        S.running = false;
        pthread_cond_destroy(&S.cv);
        pthread_mutex_destroy(&S.mtx);
        S.inited = false;
        fclose(S.fp);
        S.fp = NULL;
        return -1;
    }
    S.thread_started = true;
    return 0;
}

int csv_log_push(const CsvRecord_t *rec)
{
    int rc = 0;

    pthread_mutex_lock(&S.mtx);
    if (S.count >= RING_SIZE) {
        S.dropped++;
        rc = 1;
    } else {
        S.ring[S.head] = *rec;
        S.head = (S.head + 1) % RING_SIZE;
        S.count++;
        pthread_cond_signal(&S.cv);
    }
    pthread_mutex_unlock(&S.mtx);
    return rc;
}

uint64_t csv_log_dropped(void)
{
    uint64_t d;
    pthread_mutex_lock(&S.mtx);
    d = S.dropped;
    pthread_mutex_unlock(&S.mtx);
    return d;
}

void csv_log_stop(CsvLogStats_t *out)
{
    if (S.thread_started) {
        pthread_mutex_lock(&S.mtx);
        S.running = false;
        pthread_cond_broadcast(&S.cv);
        pthread_mutex_unlock(&S.mtx);
        pthread_join(S.thread, NULL);
        S.thread_started = false;
    }

    if (S.fp) {
        S.stop_ns = now_ns();
        S.duration_sec =
            (S.stop_ns >= S.start_ns) ? (double)(S.stop_ns - S.start_ns) / 1e9 : 0.0;
        fprintf(S.fp, "# records: %\" PRIu64 \"\n\", S.written);
        fprintf(S.fp, "# dropped: %\" PRIu64 \"\n\", S.dropped);
        fprintf(S.fp, "# duration_sec: %.3f\n\", S.duration_sec);
        fflush(S.fp);
        fsync(fileno(S.fp));
        fclose(S.fp);
        S.fp = NULL;
    }

    if (S.inited) {
        pthread_cond_destroy(&S.cv);
        pthread_mutex_destroy(&S.mtx);
        S.inited = false;
    }

    if (out) {
        out->records_written = S.written;
        out->records_dropped = S.dropped;
        out->duration_sec = S.duration_sec;
    }
}
