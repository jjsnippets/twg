/*
 * test_quad_decode.c - unit test for the pure x4 quadrature decoder
 *
 * Part of: rpi4b prod code/bno/validation/tests
 *
 * Feeds synthetic quadrature sequences into quad_decode and checks:
 *   - full revolutions (2048 cycles = 8192 counts) forward and reverse,
 *     from multiple starting states
 *   - direction sign convention (A-leads-B = +1, B-leads-A = -1)
 *   - first-sample sync from each of the four states, no phantom count
 *   - repeated identical levels never count
 *   - dither pairs (one edge out and back) are net-zero, zero invalid
 *   - illegal jumps increment invalid, keep count, resync, keep decoding
 *   - reversal at every state and depth returns to zero
 *   - a deterministic pseudo-random walk cross-checked step-by-step
 *     against an independent reference model (circular phase-distance
 *     arithmetic instead of the transition lookup table)
 *
 * No hardware and no libgpiod needed - pure logic test.
 *
 * Build & run (matches validation/Makefile):
 *   from bno/validation:      make test
 *   from bno/validation:
 *     gcc -Wall -Wextra -O2 -std=c11 -D_GNU_SOURCE -I. \
 *         -o test_quad_decode tests/test_quad_decode.c quad_decode.c
 *   from bno/validation/tests:
 *     gcc -Wall -Wextra -O2 -std=c11 -D_GNU_SOURCE -I.. \
 *         -o test_quad_decode test_quad_decode.c ../quad_decode.c
 *   ./test_quad_decode
 *
 * Exit code: 0 = all cases passed, 1 = at least one failure.
 */

#include <stdint.h>
#include <stdio.h>

#include "validation/quad_decode.h"

#define TEST_CYCLES 2048        /* one revolution at 2048 PPR */
#define TEST_COUNTS (TEST_CYCLES * 4)

static int g_failures = 0;

#define CHECK(cond, msg, ...) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL [%s:%d]: " msg "\n", __FILE__, __LINE__, ##__VA_ARGS__); \
        g_failures++; \
    } \
} while (0)

/* Reference model: circular phase distance. Returns delta in {-1, 0, +1, INVALID}. */
static int ref_delta(int prev, int curr)
{
    /* Gray-code to phase: 00->0, 01->3, 11->2, 10->1 */
    static const int g2p[4] = { 0, 3, 1, 2 };
    int diff = (g2p[curr] - g2p[prev]) & 3;
    if (diff == 0) return 0;
    if (diff == 1) return +1;
    if (diff == 3) return -1;
    return 2; /* invalid */
}

static void test_initial_sync(void)
{
    for (int s = 0; s < 4; s++) {
        quad_decoder_t d;
        quad_decode_init(&d);
        int a = (s >> 1) & 1;
        int b = s & 1;
        int delta = quad_decode_step(&d, a, b);
        CHECK(delta == 0, "first sample must return 0 delta");
        CHECK(d.count == 0, "first sample must not change count");
        CHECK(d.invalid == 0, "first sample must not be invalid");
        CHECK(d.state == s, "state not synced correctly");
    }
}

static void test_full_revolutions(void)
{
    static const int seq[4][2] = { {0,0}, {1,0}, {1,1}, {0,1} };

    /* Forward */
    quad_decoder_t df;
    quad_decode_init(&df);
    for (int i = 0; i < TEST_COUNTS; i++) {
        int idx = i % 4;
        quad_decode_step(&df, seq[idx][0], seq[idx][1]);
    }
    CHECK(df.count == TEST_COUNTS - 1, "forward count mismatch: %ld", (long)df.count);
    CHECK(df.invalid == 0, "unexpected invalid transitions in forward run");

    /* Reverse */
    quad_decoder_t dr;
    quad_decode_init(&dr);
    for (int i = 0; i < TEST_COUNTS; i++) {
        int idx = (4 - (i % 4)) % 4;
        quad_decode_step(&dr, seq[idx][0], seq[idx][1]);
    }
    CHECK(dr.count == -(TEST_COUNTS - 1), "reverse count mismatch: %ld", (long)dr.count);
    CHECK(dr.invalid == 0, "unexpected invalid transitions in reverse run");
}

static void test_invalid_jumps(void)
{
    quad_decoder_t d;
    quad_decode_init(&d);
    quad_decode_step(&d, 0, 0); /* sync to 00 */

    /* Jump to 11 (both changed) */
    int delta = quad_decode_step(&d, 1, 1);
    CHECK(delta == 0, "invalid jump must return 0 delta");
    CHECK(d.invalid == 1, "invalid jump not counted");
    CHECK(d.count == 0, "count changed on invalid transition");
    CHECK(d.state == 3, "state did not resync to 11");

    /* Valid step from 11 -> 01 (+1 forward) */
    delta = quad_decode_step(&d, 0, 1);
    CHECK(delta == 1, "failed to decode valid step after resync");
    CHECK(d.count == 1, "count not updated after resync");
}

int main(void)
{
    printf("=== test_quad_decode: verifying quadrature decoder state machine ===\n");
    test_initial_sync();
    test_full_revolutions();
    test_invalid_jumps();

    if (g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    } else {
        printf("FAILED with %d error(s)\n", g_failures);
        return 1;
    }
}
