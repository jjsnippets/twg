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
 * Build & run (from bno/validation):
 *   make
 *   ./bin/amt102_bringup        (run as a gpio-group user, e.g. pi)
 *
 * Manual build:
 *   gcc -Wall -Wextra -O2 -std=c11 -D_GNU_SOURCE -o bin/amt102_bringup \
 *       amt102_bringup.c amt102.c quad_decode.c -lgpiod
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
 * direction per pass for a formal PASS.
 *
 * The decoder needs one edge on each of A and B before counting starts
 * (initial levels are unknown), so the first fraction of a degree of
 * motion is used to sync - this is expected.
 *
 * Exit codes: 0 = PASS, 1 = CHECK (see hints), 2 = setup error.
 */

#include <errno.h>
#include <inttypes.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "amt102.h"

#define STATUS_INTERVAL_MS 500
#define POLL_TIMEOUT_MS    100

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static uint64_t now_ns(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

/* Counters frozen at the previous index pulse (window start). */
typedef struct {
    int64_t  count;
    uint64_t a_edges;
    uint64_t b_edges;
    uint64_t x_pulses;      /* total X count when the window closed */
    uint64_t invalid;
    uint64_t index_ts_ns;
} window_snap_t;

typedef struct {
    uint64_t total;
    uint64_t clean;
    int      saw_4096;      /* some window measured 4096 counts (DIP issue) */
    int      saw_partial;   /* some window was partial/reversed */
    int      saw_invalid;   /* some window had invalid transitions */
    int      saw_low_edges; /* some channel reported far fewer than 4096 edges */
} win_stats_t;

static void report_window(const window_snap_t *prev, const amt102_state_t *st,
                          win_stats_t *ws)
{
    int64_t  counts  = st->count_at_last_index - prev->count;
    uint64_t a_win   = st->a_edges_at_last_index - prev->a_edges;
    uint64_t b_win   = st->b_edges_at_last_index - prev->b_edges;
    uint64_t x_win   = st->x_pulses - prev->x_pulses;
    uint64_t inv_win = st->invalid_at_last_index - prev->invalid;
    uint64_t dt_ns   = st->last_index_ts_ns - prev->index_ts_ns;
    double   dt      = (double)dt_ns / 1e9;
    double   rpm     = dt > 0.0
                     ? 60.0 * (double)counts / (double)AMT102_COUNTS_PER_REV / dt
                     : 0.0;
    int64_t  extra   = (int64_t)a_win + (int64_t)b_win
                     - (int64_t)AMT102_COUNTS_PER_REV;
    int64_t  mag     = (counts < 0) ? -counts : counts;
    long     ppr     = (long)(mag / 4);
    int      ok      = (counts == AMT102_COUNTS_PER_REV ||
                        counts == -AMT102_COUNTS_PER_REV) &&
                       x_win == 1 && inv_win == 0;

    ws->total++;
    if (ok) {
        ws->clean++;
    } else {
        if (mag == AMT102_COUNTS_PER_REV / 2)
            ws->saw_4096 = 1;
        else if (mag != AMT102_COUNTS_PER_REV)
            ws->saw_partial = 1;
        if (inv_win > 0)
            ws->saw_invalid = 1;
        if (a_win < 3500 || b_win < 3500)
            ws->saw_low_edges = 1;
    }

    printf("REV %3" PRIu64 ": counts %+6" PRId64 " | ppr %5ld | "
           "A %4" PRIu64 " B %4" PRIu64 " | X %" PRIu64 " | "
           "extra %+5" PRId64 " | %5.2f s | %+6.1f RPM | "
           "invalid %" PRIu64 " | %s\n",
           ws->total, counts, ppr, a_win, b_win, x_win, extra, dt, rpm,
           inv_win, ok ? "OK" : "CHECK");
}

static void print_status(const amt102_state_t *st, uint64_t t0,
                         uint64_t *t_last, uint64_t *ev_last,
                         int64_t *count_last)
{
    uint64_t now    = now_ns();
    uint64_t ev_now = st->a_edges + st->b_edges + st->x_pulses;
    double   dt     = (double)(now - *t_last) / 1e9;
    double   rate   = dt > 0.0 ? (double)(ev_now - *ev_last) / dt : 0.0;
    double   elapsed = (double)(now - t0) / 1e9;
    int64_t  cdelta = st->count - *count_last;
    const char *dir = (cdelta > 0) ? "CCW+" : (cdelta < 0) ? "CW-" : "idle";
    int64_t  rem    = st->count % AMT102_COUNTS_PER_REV;
    double   angle;

    if (rem >  AMT102_COUNTS_PER_REV / 2)
        rem -= AMT102_COUNTS_PER_REV;
    if (rem < -AMT102_COUNTS_PER_REV / 2)
        rem += AMT102_COUNTS_PER_REV;
    angle = (double)rem * 360.0 / (double)AMT102_COUNTS_PER_REV;

    if (!st->synced) {
        printf("%6.1f s | waiting for first edges on A and B (decoder sync)"
               " | %6.0f ev/s\n", elapsed, rate);
    } else {
        printf("%6.1f s | count %+9" PRId64 " | angle %+8.2f deg | %-4s | "
               "%6.0f ev/s | invalid %" PRIu64 " | rev %" PRIu64 "\n",
               elapsed, st->count, angle, dir, rate, st->invalid,
               st->x_pulses);
    }

    *t_last     = now;
    *ev_last    = ev_now;
    *count_last = st->count;
}

static int print_summary(const amt102_state_t *st, uint64_t t0,
                         const win_stats_t *ws)
{
    double elapsed = (double)(now_ns() - t0) / 1e9;
    double avg_rpm = elapsed > 0.0
        ? 60.0 * (double)st->count / (double)AMT102_COUNTS_PER_REV / elapsed
        : 0.0;
    int pass = ws->total >= 1 && ws->clean == ws->total && st->invalid == 0;

    printf("\n==== SUMMARY ====\n");
    printf("run time            : %6.1f s\n", elapsed);
    printf("net counts          : %+" PRId64 "\n", st->count);
    printf("net revolutions     : %+.3f\n",
           (double)st->count / (double)AMT102_COUNTS_PER_REV);
    printf("index pulses        : %" PRIu64 "\n", st->x_pulses);
    printf("A / B edges         : %" PRIu64 " / %" PRIu64 "\n",
           st->a_edges, st->b_edges);
    printf("invalid transitions : %" PRIu64 "\n", st->invalid);
    printf("windows clean       : %" PRIu64 " of %" PRIu64 "\n",
           ws->clean, ws->total);
    printf("average speed       : %+.1f RPM\n", avg_rpm);

    if (pass) {
        printf("\nVERDICT: PASS - every measured revolution was exactly "
               "+/-8192 counts (2048 PPR) with zero invalid transitions.\n");
        return 0;
    }

    printf("\nVERDICT: CHECK\n");
    if (ws->total == 0)
        printf("hint: no index-to-index revolution recorded - rotate "
               "through at least one full revolution between index "
               "pulses.\n");
    if (ws->saw_4096)
        printf("hint: 4096 counts/rev detected - the DIP switches are not "
               "at the 2048 PPR preset (factory preset: all four switches "
               "OFF).\n");
    if (ws->saw_invalid || st->invalid > 0)
        printf("hint: invalid quadrature transitions - check A/B wiring, "
               "the 1k series resistors, and the common ground.\n");
    if (ws->saw_low_edges)
        printf("hint: a channel reported far fewer than 4096 edges/rev - "
               "check that channel's wire and resistor.\n");
    if (ws->saw_partial)
        printf("hint: partial windows (direction changed mid-revolution) "
               "legitimately fail the count check - for a formal PASS "
               "re-run rotating one direction per pass.\n");
    if (ws->total > 0 && !ws->saw_4096 && !ws->saw_invalid &&
        !ws->saw_low_edges && !ws->saw_partial && st->invalid == 0)
        printf("hint: windows deviated from +/-8192 without another "
               "diagnostic - re-run rotating slowly and steadily.\n");
    return 1;
}

int main(void)
{
    struct sigaction sa;
    amt102_t *enc;
    amt102_state_t st;
    window_snap_t prev;
    win_stats_t ws;
    uint64_t t0, t_status, ev_status, x_seen;
    int64_t count_status;
    int have_prev = 0;
    int rc;

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;                /* no SA_RESTART: poll() must EINTR */
    if (sigaction(SIGINT, &sa, NULL) != 0 ||
        sigaction(SIGTERM, &sa, NULL) != 0) {
        fprintf(stderr, "error: sigaction: %s\n", strerror(errno));
        return 2;
    }

    if (amt102_open(&enc) != 0) {
        fprintf(stderr, "error: cannot open AMT102 on gpiochip0: %s\n",
                strerror(errno));
        fprintf(stderr, "hint: check that lines 17/27/22 are unused "
                "(gpioinfo gpiochip0) and that this user is in the gpio "
                "group\n");
        return 2;
    }

    amt102_get_state(enc, &st);
    printf("AMT102 bring-up  |  A=BCM17  B=BCM27  X=BCM22  |  "
           "2048 PPR x4 = %d counts/rev\n", AMT102_COUNTS_PER_REV);
    printf("libgpiod v1 event API, bias-disable %s\n",
           st.bias_fallback
               ? "NOT accepted by kernel (default pulls active)"
               : "active (no internal pulls)");
    printf("rotate slowly and steadily through several revolutions, "
           "reverse, hold still, then Ctrl+C\n\n");

    memset(&ws, 0, sizeof(ws));
    memset(&prev, 0, sizeof(prev));
    t0           = now_ns();
    t_status     = t0;
    ev_status    = 0;
    count_status = 0;
    x_seen       = st.x_pulses;

    while (!g_stop) {
        int n = amt102_poll(enc, POLL_TIMEOUT_MS);

        if (n < 0) {
            fprintf(stderr, "\nerror: encoder event read failed: %s\n",
                    strerror(errno));
            amt102_close(enc);
            return 2;
        }

        amt102_get_state(enc, &st);

        while (st.x_pulses > x_seen) {
            if (st.x_pulses - x_seen > 1)
                printf("note: %" PRIu64 " index pulses in one batch - "
                       "intermediate windows skipped\n",
                       st.x_pulses - x_seen);

            if (have_prev) {
                report_window(&prev, &st, &ws);
            } else {
                printf("first index pulse - per-revolution reports start "
                       "at the next pulse\n");
            }

            prev.count       = st.count_at_last_index;
            prev.a_edges     = st.a_edges_at_last_index;
            prev.b_edges     = st.b_edges_at_last_index;
            prev.invalid     = st.invalid_at_last_index;
            prev.x_pulses    = st.x_pulses;
            prev.index_ts_ns = st.last_index_ts_ns;
            have_prev        = 1;
            x_seen           = st.x_pulses;
        }

        if (now_ns() - t_status >= (uint64_t)STATUS_INTERVAL_MS * 1000000ull)
            print_status(&st, t0, &t_status, &ev_status, &count_status);
    }

    rc = print_summary(&st, t0, &ws);
    amt102_close(enc);
    return rc;
}
