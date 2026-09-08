/*
 * amt102_bringup.c — stand-alone bring-up diagnostic for AMT102-V encoder.
 *
 * Verifies quadrature decoding, count direction, and index-pulse
 * detection in real time without IMU or SPI dependencies.
 *
 * Usage:
 *   sudo ./bin/amt102_bringup
 *
 * Part of the sensor_validate harness (see Makefile).
 */

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <inttypes.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "validation/amt102.h"

static volatile sig_atomic_t sRunning = 1;

static void sigHandler(int sig)
{
    (void)sig;
    sRunning = 0;
}

int main(void)
{
    printf("=== AMT102-V Encoder Stand-Alone Bring-Up Diagnostic ===\n");
    printf("Initializing GPIO pins (A=5, B=6, X=13)...\n");

    signal(SIGINT, sigHandler);
    signal(SIGTERM, sigHandler);

    if (amt102_init() != 0) {
        fprintf(stderr, "error: amt102_init failed (%s)\n", strerror(errno));
        return 1;
    }

    printf("Rotate encoder shaft to observe counts. Press Ctrl-C to exit.\n\n");

    while (sRunning) {
        amt102_poll();

        Amt102Snapshot_t snap;
        amt102_snapshot(&snap);

        printf("  count=%+8" PRId64 "  angle=%+7.2f deg  x_pulses=%" PRIu64
               "  invalid=%" PRIu64 "  edges_ab=%" PRIu64 "\r",
               snap.count, snap.angle_deg, snap.x_pulses,
               snap.invalid_transitions, snap.edges_ab);
        fflush(stdout);

        usleep(10000); /* 10 ms refresh */
    }

    printf("\n\nClosing encoder lines...\n");
    amt102_close();
    printf("Done.\n");
    return 0;
}
