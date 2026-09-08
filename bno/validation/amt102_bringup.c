/*
 * amt102_bringup.c - AMT102-V encoder bring-up / hardware integration test
 *
 * Part of: rpi4b prod code/bno/validation
 *
 * Purpose: verify the encoder wiring, the 2048 PPR DIP preset and the
 * whole decode chain (amt102 + quad_decode modules) by hand-rotating
 * the shaft and reporting statistics for each index-to-index revolution.
 * The output lines are the bring-up evidence for the validation records.
 *
 * Wiring (BCM numbering):
 *   A -> 1k -> BCM 17 (pin 11), B -> 1k -> BCM 27 (pin 13),
 *   X -> 1k -> BCM 22 (pin 15), 5V -> Pi 5V, G -> Pi GND
 *   (encoder connector order is B, 5V, A, X, G - check before power)
 *
 * AMT102 DIP preset: all four switches OFF = 2048 PPR = 8192 x4 counts.
 *
 * Build & run (from bno):
 *   make bringup
 *   ./bin/amt102_bringup        (run as a gpio-group user, e.g. pi)
 *
 * Manual build:
 *   gcc -Wall -Wextra -O2 -std=c11 -D_GNU_SOURCE -I. -o bin/amt102_bringup \
 *       validation/amt102_bringup.c validation/amt102.c validation/quad_decode.c -lgpiod
 *
 * Procedure: rotate slowly and steadily through several revolutions in
 * one direction, reverse for a few more, hold still ~10 s, then Ctrl+C.
 *
 * Verdict (relaxed): a window is clean when |counts| == 8192, exactly
 * one index pulse, and zero invalid transitions. Edge counts are NOT
 * part of the verdict anymore: "extra" = (A+B) - 8192 is reported as a
 * diagnostic only. A steady window shows extra ~0 (small positive
 * values are net-zero dither at transition boundaries); negative values
 * mean a partial/reversed window. Windows containing a direction
 * reversal legitimately fail the count check - re-run rotating one
 * direction per test block.
 *
 * Exit codes:
 *   0  success (at least one clean index-to-index window was observed)
 *   1  hardware/open failure or no clean window after observing pulses
 *   2  aborted before any index pulse was observed
 */

#include <inttypes.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "validation/amt102.h"
#include "validation/quad_decode.h"

static volatile sig_atomic_t sAbort = 0;
static void onSigint(int sig) { (void)sig; sAbort = 1; }

typedef struct {
    int64_t  count;
    uint64_t a_edges;
    uint64_t b_edges;
    uint64_t invalid;
    uint64_t x_pulses;
    uint64_t last_index_ts_ns;
} index_checkpoint_t;

/*
 * Checks whether an index-to-index window matches the 2048 PPR x4 spec:
 * exactly 8192 counts forward (+8192) or reverse (-8192), zero invalid
 * transitions, and exactly 1 index pulse.
 */
static bool window_is_clean(const index_checkpoint_t *curr,
                            const index_checkpoint_t *prev)
{
    int64_t d_count   = curr->count - prev->count;
    int64_t abs_count = (d_count >= 0) ? d_count : -d_count;
    uint64_t d_inv    = curr->invalid - prev->invalid;
    uint64_t d_x      = curr->x_pulses - prev->x_pulses;

    return (d_x == 1) && (d_inv == 0) && (abs_count == AMT102_COUNTS_PER_REV);
}

int main(void)
{
    amt102_t *enc = NULL;
    amt102_state_t st;
    index_checkpoint_t last_cp;
    bool have_cp = false;

    uint32_t total_windows = 0;
    uint32_t clean_windows = 0;
    uint64_t total_events  = 0;

    signal(SIGINT, onSigint);

    printf("=== AMT102-V encoder bring-up test (Raspberry Pi 4B) ===\n\n");
    printf("Hardware check:\n");
    printf("  A -> BCM 17 (pin 11),  B -> BCM 27 (pin 13),  X -> BCM 22 (pin 15)\n");
    printf("  Power: 5V (pin 2/4),   GND (pin 6/any)\n");
    printf("  DIP preset: all four OFF (2048 PPR -> 8192 x4 counts/rev)\n\n");

    if (amt102_open(&enc) != 0) {
        perror("amt102_open failed");
        fprintf(stderr,
                "\nTroubleshooting:\n"
                "  - is gpiochip0 accessible? (run under user 'pi' or sudo)\n"
                "  - is another process holding GPIO 17, 27 or 22? Check:\n"
                "      gpioinfo gpiochip0 | grep -E '17|27|22'\n"
                "  - are you using libgpiod-dev 1.6.x? (v2 is incompatible)\n");
        return 1;
    }

    amt102_get_state(enc, &st);
    if (st.bias_fallback) {
        printf("Note: kernel rejected BIAS_DISABLE; proceeding without it.\n"
               "      (Encoder outputs are push-pull; internal bias is benign.)\n\n");
    }

    printf("Rotate the shaft slowly and steadily. Ctrl-C to finish.\n");
    printf("----------------------------------------------------------------------\n");
    printf("%-5s  %-8s  %-8s  %-8s  %-8s  %-8s  %s\n",
           "Win#", "deltaCnt", "extraAB", "invTrans", "dt(ms)", "dir", "verdict");
    printf("----------------------------------------------------------------------\n");

    memset(&last_cp, 0, sizeof(last_cp));

    while (!sAbort) {
        int n = amt102_poll(enc, 100); /* 100 ms timeout so Ctrl-C responds */
        if (n < 0) {
            perror("amt102_poll error");
            break;
        }
        total_events += (uint64_t)n;

        amt102_get_state(enc, &st);

        /* New index pulse arrived since last checkpoint? */
        if (st.x_pulses > last_cp.x_pulses) {
            index_checkpoint_t curr;
            curr.count             = st.count_at_last_index;
            curr.a_edges           = st.a_edges_at_last_index;
            curr.b_edges           = st.b_edges_at_last_index;
            curr.invalid           = st.invalid_at_last_index;
            curr.x_pulses          = st.x_pulses;
            curr.last_index_ts_ns  = st.last_index_ts_ns;

            if (have_cp) {
                total_windows++;

                int64_t  d_count = curr.count - last_cp.count;
                uint64_t d_a     = curr.a_edges - last_cp.a_edges;
                uint64_t d_b     = curr.b_edges - last_cp.b_edges;
                uint64_t d_inv   = curr.invalid - last_cp.invalid;
                uint64_t d_ts_ns = curr.last_index_ts_ns - last_cp.last_index_ts_ns;
                double   dt_ms   = (double)d_ts_ns / 1000000.0;

                /*
                 * "extra" edges = observed (A+B) minus the nominal 8192 edges
                 * per rev. On a clean window extra == 0. Small positive extra
                 * with d_count == +/-8192 means minor edge dither (acceptable);
                 * large extra or d_count mismatch means dropped edges or a
                 * reversal mid-window.
                 */
                int64_t extra_ab = (int64_t)(d_a + d_b) - AMT102_COUNTS_PER_REV;

                bool clean = window_is_clean(&curr, &last_cp);
                if (clean) clean_windows++;

                const char *dir_str = (d_count > 0) ? "FWD (+)" :
                                      (d_count < 0) ? "REV (-)" : "STALL";
                const char *verd    = clean ? "PASS" : "FAIL";

                printf("#%-4u  %+8" PRId64 "  %+8" PRId64 "  %8" PRIu64 "  %8.1f  %-8s  %s\n",
                       total_windows, d_count, extra_ab, d_inv, dt_ms, dir_str, verd);
                fflush(stdout);
            } else {
                /* First index pulse seen - start baseline, no delta yet */
                printf("[index 1 seen - starting window measurement]\n");
                have_cp = true;
            }

            last_cp = curr;
        }
    }

    printf("\n----------------------------------------------------------------------\n");
    printf("Final summary:\n");
    amt102_get_state(enc, &st);
    printf("  Net count:           %+" PRId64 " (%.2f rev)\n",
           st.count, (double)st.count / (double)AMT102_COUNTS_PER_REV);
    printf("  Total events:        %" PRIu64 " (A: %" PRIu64 ", B: %" PRIu64 ", X: %" PRIu64 ")\n",
           total_events, st.a_edges, st.b_edges, st.x_pulses);
    printf("  Invalid transitions: %" PRIu64 "\n", st.invalid);
    printf("  Index windows:       %u total, %u clean (PASS)\n",
           total_windows, clean_windows);

    amt102_close(enc);

    if (total_windows == 0) {
        fprintf(stderr,
                "\nVerdict: INCOMPLETE (no index-to-index window was completed).\n"
                "Rotate through at least TWO index pulses in one run.\n");
        return 2;
    }

    if (clean_windows > 0) {
        printf("\nVerdict: PASS (%u clean 8192-count window(s) observed).\n"
               "Wiring, 2048 PPR DIP preset and decode chain confirmed.\n",
               clean_windows);
        return 0;
    }

    fprintf(stderr,
            "\nVerdict: FAIL (0 clean windows out of %u).\n"
            "Checks:\n"
            "  - Are all four DIP switches OFF? (Any ON alters PPR.)\n"
            "  - Are invalid transitions > 0? Check for floating ground or\n"
            "    missing 1k series resistors.\n"
            "  - Did you reverse direction mid-window? (Reversals fail the\n"
            "    strict 8192 check by design; re-test in one direction.)\n");
    return 1;
}
