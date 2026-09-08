/*
 * amt102_bringup.c — AMT102-V encoder bring-up / hardware integration test
 *
 * Verifies encoder wiring, line states, x4 quadrature decoding, count direction,
 * and index pulses with live terminal telemetry refreshed every 500 ms.
 * Does not require rotating a full 360 degrees to observe counts.
 *
 * Wiring (BCM numbering):
 *   A -> 1k -> BCM 17 (pin 11)
 *   B -> 1k -> BCM 27 (pin 13)
 *   X -> 1k -> BCM 22 (pin 15)
 *   5V -> Pi 5V, GND -> Pi GND
 *   (encoder connector order: B, 5V, A, X, G)
 *
 * DIP preset: all four switches OFF = 2048 PPR = 8192 x4 counts/rev.
 *
 * Usage:
 *   sudo ./bin/amt102_bringup
 */

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <inttypes.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "validation/amt102.h"
#include "validation/quad_decode.h"

#define DISPLAY_PERIOD_US 500000u /* 500 ms refresh */

static volatile sig_atomic_t sAbort = 0;
static void onSigint(int sig)
{
    (void)sig;
    sAbort = 1;
}

static uint64_t hostNowUs(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000000ULL) +
           ((uint64_t)ts.tv_nsec / 1000ULL);
}

int main(void)
{
    amt102_t       *enc = NULL;
    amt102_state_t  st;
    uint64_t        total_events = 0;
    uint64_t        tLastDisplay = 0;

    signal(SIGINT, onSigint);
    signal(SIGTERM, onSigint);

    printf("=== AMT102-V Encoder Live Bring-up Diagnostic ===\n\n");
    printf("Hardware check:\n");
    printf("  A -> BCM 17 (pin 11),  B -> BCM 27 (pin 13),  X -> BCM 22 (pin 15)\n");
    printf("  Power: 5V (pin 2/4),   GND (pin 6/any)\n");
    printf("  DIP preset: all four OFF (2048 PPR -> 8192 x4 counts/rev)\n\n");

    if (amt102_open(&enc) != 0) {
        perror("amt102_open failed");
        fprintf(stderr,
                "\nCheck:\n"
                "  - is gpiochip0 accessible? (run under sudo or gpio group)\n"
                "  - is another process holding GPIO 17, 27 or 22? Check:\n"
                "      gpioinfo gpiochip0 | grep -E '17|27|22'\n"
                "  - is libgpiod-dev 1.6.x installed? (v2 is incompatible)\n");
        return 1;
    }

    amt102_get_state(enc, &st);
    if (st.bias_fallback) {
        printf("Note: kernel rejected BIAS_DISABLE; proceeding without it.\n\n");
    }

    printf("Rotate the shaft. Live status refreshes every 500 ms. Press Ctrl-C to stop.\n");
    printf("----------------------------------------------------------------------------------------\n");

    /* Prime initial display timestamp */
    tLastDisplay = hostNowUs();

    while (!sAbort) {
        /* Poll with short 20 ms timeout so transitions are captured promptly */
        int n = amt102_poll(enc, 20);
        if (n < 0 && errno != EINTR) {
            perror("\namt102_poll error");
            break;
        }
        total_events += (uint64_t)n;

        amt102_get_state(enc, &st);

        uint64_t now = hostNowUs();
        if ((now - tLastDisplay) >= DISPLAY_PERIOD_US) {
            double deg = (double)st.count * (360.0 / (double)AMT102_COUNTS_PER_REV);
            printf("  count=%+8" PRId64 " (%+7.2f deg) | edges: A=%-7" PRIu64 " B=%-7" PRIu64 " | x_pulses=%-3" PRIu64 " | inv=%" PRIu64 "  \r",
                   st.count, deg, st.a_edges, st.b_edges, st.x_pulses, st.invalid);
            fflush(stdout);
            tLastDisplay = now;
        }
    }

    /* Clear live line */
    printf("\n----------------------------------------------------------------------------------------\n");

    amt102_get_state(enc, &st);
    double final_deg = (double)st.count * (360.0 / (double)AMT102_COUNTS_PER_REV);

    printf("Session Summary:\n");
    printf("  Net count:           %+" PRId64 " (%.2f deg, %.3f rev)\n",
           st.count, final_deg, (double)st.count / (double)AMT102_COUNTS_PER_REV);
    printf("  Channel A edges:     %" PRIu64 "\n", st.a_edges);
    printf("  Channel B edges:     %" PRIu64 "\n", st.b_edges);
    printf("  Index (X) pulses:    %" PRIu64 "\n", st.x_pulses);
    printf("  Invalid transitions: %" PRIu64 "\n", st.invalid);
    printf("  Total events:        %" PRIu64 "\n\n", total_events);

    if (st.invalid > 0) {
        printf("RESULT: WARNING (invalid quadrature transitions detected)\n");
        printf("Check 1k series resistors, wiring contacts, and ground.\n");
    } else if (st.a_edges > 0 || st.b_edges > 0) {
        printf("RESULT: PASS (clean quadrature decoding confirmed)\n");
    } else {
        printf("RESULT: NO MOTION OBSERVED\n");
    }

    amt102_close(enc);
    return 0;
}
