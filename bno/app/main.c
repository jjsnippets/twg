#define _POSIX_C_SOURCE 200809L

/*
 * main.c — BNO085 IMU acquisition application loop.
 *
 * Integration point for the Mantatow thesis data collection system.
 *
 * Current responsibilities:
 *   1. Brings up the sensor via sensor_reader_start() (100 Hz RV + Accel + Gyro).
 *   2. Enters real-time scheduling (SCHED_FIFO, priority 90) via realtime.c.
 *   3. Drains ~300 ms of warm-up traffic so filters settle.
 *   4. Runs the 1 kHz service loop, polling sensor_reader at ~1 ms cadence.
 *   5. Every 10 ms (100 Hz), snapshots the latest ImuSample_t.
 *   6. Prints a diagnostic line once per second.
 *
 * Future thesis integration:
 *   Replace the 100 Hz consumer block (marked below) with your actual data
 *   pipeline: pressure-sensor reading, video timestamp sync, logging, etc.
 *
 * Build:
 *   make bno_app
 *
 * Run:
 *   sudo ./bin/bno_app
 */

#include <inttypes.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "sh2.h"
#include "sh2_err.h"

#include "app/app_contract.h"
#include "app/realtime.h"
#include "app/app_sensor.h"

#define SERVICE_PERIOD_US   1000     /* 1 kHz service loop: ~1 ms sleep */
#define CONSUMER_DIVIDER    10       /* run consumer every 10 ticks = 100 Hz */
#define WARMUP_DRAIN_MS     300      /* let initial transients clear before consuming */

static volatile sig_atomic_t sRunning = 1;

static void onSigint(int sig)
{
    (void)sig;
    sRunning = 0;
}

int main(void)
{
    ImuSample_t sample;
    uint32_t tickCount = 0;
    uint32_t lastSeq = 0;
    uint32_t duplicates = 0;
    uint32_t totalSamples = 0;

    signal(SIGINT, onSigint);
    signal(SIGTERM, onSigint);

    printf("bno_app: starting BNO085 acquisition\n");

    /* Step 1: Open the SH-2 session and enable sensors at 100 Hz. */
    if (!sensor_reader_start()) {
        fprintf(stderr, "bno_app: ERROR: sensor_reader_start failed\n");
        return 1;
    }

    /*
     * Step 2: Elevate to real-time priority (SCHED_FIFO, priority 90).
     * If unprivileged, StartRT prints a warning and falls back to SCHED_OTHER.
     */
    StartRT(90, SERVICE_PERIOD_US);

    /*
     * Step 3: Warm-up drain.
     * Service the session for WARMUP_DRAIN_MS to allow the SHTP negotiation,
     * FRS reads, and filter warm-up to complete before the consumer runs.
     */
    printf("bno_app: draining %d ms of warm-up traffic...\n", WARMUP_DRAIN_MS);
    for (int i = 0; i < (WARMUP_DRAIN_MS * 1000) / SERVICE_PERIOD_US; ++i) {
        sensor_reader_service();
        RT_SleepUntil(SERVICE_PERIOD_US);
    }
    sensor_reader_resetSeq();
    printf("bno_app: warm-up complete, entering 100 Hz acquisition loop\n");

    /*
     * Step 4: Acquisition loop.
     * Services the SH-2 session at 1 kHz (~1 ms).
     * Dispatches the 100 Hz consumer every CONSUMER_DIVIDER ticks (~10 ms).
     */
    while (sRunning) {
        sensor_reader_service();

        tickCount++;
        if ((tickCount % CONSUMER_DIVIDER) == 0) {
            if (sensor_reader_getLatestSample(&sample)) {
                totalSamples++;

                if (sample.seq == lastSeq) {
                    duplicates++;
                }
                lastSeq = sample.seq;

                /*
                 * ---------------------------------------------------------
                 * 100 Hz INTEGRATION POINT
                 *
                 * sample contains the latest scaled engineering values:
                 *   sample.yaw, sample.pitch, sample.roll   (rad)
                 *   sample.ax,  sample.ay,    sample.az     (m/s^2)
                 *   sample.gx,  sample.gy,    sample.gz     (rad/s)
                 *   sample.tHost_uS                         (us, MONOTONIC)
                 *   sample.seq                              (monotonic)
                 *   sample.validMask                        (readiness)
                 *
                 * Hand off to your logging, networking, or sensor-fusion
                 * pipeline here.
                 * ---------------------------------------------------------
                 */

                /* Diagnostic print once per second (every 100 consumer ticks). */
                if ((totalSamples % 100) == 0) {
                    printf("[%8" PRIu64 " us] seq=%-6" PRIu32 " "
                           "yaw=%+6.2f deg pitch=%+6.2f deg roll=%+6.2f deg | "
                           "ax=%+6.2f ay=%+6.2f az=%+6.2f m/s^2 | "
                           "gx=%+6.2f gy=%+6.2f gz=%+6.2f rad/s | "
                           "dups=%" PRIu32 "\n",
                           sample.tHost_uS,
                           sample.seq,
                           (double)(sample.yaw   * 57.29577951308232),
                           (double)(sample.pitch * 57.29577951308232),
                           (double)(sample.roll  * 57.29577951308232),
                           (double)sample.ax,
                           (double)sample.ay,
                           (double)sample.az,
                           (double)sample.gx,
                           (double)sample.gy,
                           (double)sample.gz,
                           duplicates);
                }
            }
        }

        RT_SleepUntil(SERVICE_PERIOD_US);
    }

    printf("\nbno_app: stopping, closing SH-2 session...\n");
    sensor_reader_stop();
    printf("bno_app: done (processed %" PRIu32 " samples, %" PRIu32 " duplicates)\n",
           totalSamples, duplicates);
    return 0;
}
