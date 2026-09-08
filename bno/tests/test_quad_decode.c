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

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "validation/quad_decode.h"

#define REV_COUNTS 8192
#define REV_CYCLES 2048

static int g_checks   = 0;
static int g_failures = 0;

static void check(const char *name, int ok)
{
    printf("%-52s %s\n", name, ok ? "PASS" : "FAIL");
    g_checks++;
    if (!ok)
        g_failures++;
}

/* State circle helpers: forward order is 0 -> 2 -> 3 -> 1 -> 0. */
static const int NEXT_FWD[4] = { 2, 0, 3, 1 };
static const int NEXT_REV[4] = { 1, 3, 0, 2 };

/* One edge in the given direction from the current (synced) state. */
static int advance(quad_decoder_t *d, int forward)
{
    int s = forward ? NEXT_FWD[d->state] : NEXT_REV[d->state];
    return quad_decode_step(d, (s >> 1) & 1, s & 1);
}

/* Four edges = one full cycle from any synced state, returns +/-4. */
static int cycle(quad_decoder_t *d, int forward)
{
    int delta = 0;
    for (int i = 0; i < 4; i++)
        delta += advance(d, forward);
    return delta;
}

/* Sync the decoder from an arbitrary starting level pair. */
static void sync_to(quad_decoder_t *d, int a, int b)
{
    quad_decode_init(d);
    int delta = quad_decode_step(d, a, b);
    (void)delta;
}

/*
 * Independent reference model: circular phase arithmetic.
 *
 * Map Gray-code state (A<<1)|B to a 2-bit phase angle [0..3]:
 *   00 -> 0,  10 -> 1,  11 -> 2,  01 -> 3.
 *
 * Signed phase difference mod 4:
 *   (curr - prev) & 3:
 *     0 -> stationary (0)
 *     1 -> forward (+1)
 *     2 -> illegal (two-line jump)
 *     3 -> reverse (-1)
 *
 * Used only by the pseudo-random walk test to confirm that the lookup
 * table in quad_decode.c matches the circular phase definition across
 * arbitrary long sequences.
 */
typedef struct {
    int     phase;
    int64_t count;
    uint64_t invalid;
} ref_decoder_t;

static void ref_init(ref_decoder_t *r)
{
    r->phase   = -1;
    r->count   = 0;
    r->invalid = 0;
}

static int ref_step(ref_decoder_t *r, int a, int b)
{
    static const int G2P[4] = { 0, 3, 1, 2 }; /* [gray] -> phase */
    int gray = ((a ? 1 : 0) << 1) | (b ? 1 : 0);
    int p = G2P[gray];

    if (r->phase < 0) {
        r->phase = p;
        return 0;
    }

    int diff = (p - r->phase) & 3;
    r->phase = p;

    if (diff == 0) return 0;
    if (diff == 1) { r->count++; return +1; }
    if (diff == 3) { r->count--; return -1; }

    r->invalid++;
    return 0;
}

/*
 * Step both decoders on the same input, assert that their delta,
 * total count and invalid-transition count match at this step.
 */
static int feed_both(quad_decoder_t *d, ref_decoder_t *r, int a, int b)
{
    int d1 = quad_decode_step(d, a, b);
    int d2 = ref_step(r, a, b);
    if (d1 != d2) return 0;
    if (d->count != r->count) return 0;
    if (d->invalid != r->invalid) return 0;
    return 1;
}

/* ------------------------------------------------------------------ */
/* Test cases                                                         */
/* ------------------------------------------------------------------ */

static void test_initial_sync_and_stationary(void)
{
    quad_decoder_t d;

    /* First sample syncs, returns 0, zero count, zero invalid, state known */
    for (int s = 0; s < 4; s++) {
        quad_decode_init(&d);
        int a = (s >> 1) & 1, b = s & 1;
        int delta = quad_decode_step(&d, a, b);
        int ok = (delta == 0) && (d.count == 0) && (d.invalid == 0) &&
                 (d.state == s);
        char buf[64];
        snprintf(buf, sizeof(buf), "initial sync from state %d (A=%d B=%d)", s, a, b);
        check(buf, ok);
    }

    /* Repeated identical samples never count */
    quad_decode_init(&d);
    quad_decode_step(&d, 1, 0);
    int ok = 1;
    for (int i = 0; i < 50; i++) {
        if (quad_decode_step(&d, 1, 0) != 0 || d.count != 0 || d.invalid != 0)
            ok = 0;
    }
    check("stationary: repeated samples produce no counts", ok);
}

static void test_direction_conventions(void)
{
    quad_decoder_t d;

    /* Forward: 00 -> 10 -> 11 -> 01 -> 00 must produce four +1s */
    sync_to(&d, 0, 0);
    int d1 = quad_decode_step(&d, 1, 0);
    int d2 = quad_decode_step(&d, 1, 1);
    int d3 = quad_decode_step(&d, 0, 1);
    int d4 = quad_decode_step(&d, 0, 0);
    check("single forward cycle produces +1,+1,+1,+1",
          (d1 == 1) && (d2 == 1) && (d3 == 1) && (d4 == 1) && (d.count == 4));

    /* Reverse: 00 -> 01 -> 11 -> 10 -> 00 must produce four -1s */
    sync_to(&d, 0, 0);
    d1 = quad_decode_step(&d, 0, 1);
    d2 = quad_decode_step(&d, 1, 1);
    d3 = quad_decode_step(&d, 1, 0);
    d4 = quad_decode_step(&d, 0, 0);
    check("single reverse cycle produces -1,-1,-1,-1",
          (d1 == -1) && (d2 == -1) && (d3 == -1) && (d4 == -1) && (d.count == -4));
}

static void test_full_revolutions(void)
{
    quad_decoder_t d;

    /* Forward 8192 counts from state 00 */
    sync_to(&d, 0, 0);
    for (int c = 0; c < REV_CYCLES; c++)
        cycle(&d, 1);
    check("forward full revolution: exactly +8192 counts",
          (d.count == REV_COUNTS) && (d.invalid == 0));

    /* Reverse 8192 counts from state 00 */
    sync_to(&d, 0, 0);
    for (int c = 0; c < REV_CYCLES; c++)
        cycle(&d, 0);
    check("reverse full revolution: exactly -8192 counts",
          (d.count == -REV_COUNTS) && (d.invalid == 0));

    /* Full revolution from each of the other three starting states */
    for (int s = 1; s < 4; s++) {
        sync_to(&d, (s >> 1) & 1, s & 1);
        for (int c = 0; c < REV_CYCLES; c++)
            cycle(&d, 1);
        char buf[64];
        snprintf(buf, sizeof(buf), "forward revolution starting from state %d", s);
        check(buf, (d.count == REV_COUNTS) && (d.invalid == 0));
    }
}

static void test_dither_cancellation(void)
{
    quad_decoder_t d;
    sync_to(&d, 0, 0);

    /* Advance 100 counts, then dither back and forth 500 times on one edge */
    for (int i = 0; i < 100; i++)
        advance(&d, 1);
    int64_t baseline = d.count;

    int ok = 1;
    for (int i = 0; i < 500; i++) {
        advance(&d, 1);  /* +1 */
        advance(&d, 0);  /* -1 */
        if (d.count != baseline || d.invalid != 0)
            ok = 0;
    }
    check("dither: 500 single-edge oscillations cancel net-zero", ok);
}

static void test_reversals_at_every_depth(void)
{
    quad_decoder_t d;

    /*
     * For each depth from 1 to 16 edges: advance by depth, then reverse
     * by depth. Net count must return to 0 every time, zero invalid.
     */
    int all_ok = 1;
    for (int depth = 1; depth <= 16; depth++) {
        sync_to(&d, 0, 0);
        for (int i = 0; i < depth; i++) advance(&d, 1);
        for (int i = 0; i < depth; i++) advance(&d, 0);
        if (d.count != 0 || d.invalid != 0)
            all_ok = 0;
    }
    check("reversals: advance/reverse depths 1..16 return to zero", all_ok);
}

static void test_invalid_transitions(void)
{
    quad_decoder_t d;

    /*
     * The four illegal two-line jumps are:
     *   00 <-> 11,  01 <-> 10.
     * Each must:
     *   - increment d.invalid
     *   - return delta = 0
     *   - leave d.count unchanged
     *   - resync d.state to the new level
     *   - resume correct counting on the very next legal edge
     */
    static const int JUMPS[4][4] = {
        /* from_a, from_b, to_a, to_b */
        { 0, 0,  1, 1 },
        { 1, 1,  0, 0 },
        { 0, 1,  1, 0 },
        { 1, 0,  0, 1 }
    };

    for (int j = 0; j < 4; j++) {
        sync_to(&d, JUMPS[j][0], JUMPS[j][1]);
        int delta = quad_decode_step(&d, JUMPS[j][2], JUMPS[j][3]);
        int target_state = ((JUMPS[j][2] ? 1 : 0) << 1) | (JUMPS[j][3] ? 1 : 0);

        int ok = (delta == 0) &&
                 (d.count == 0) &&
                 (d.invalid == 1) &&
                 (d.state == target_state);

        /* Follow with one forward edge from the resynced state: must decode */
        int d_next = advance(&d, 1);
        ok = ok && (d_next == 1) && (d.count == 1);

        char buf[64];
        snprintf(buf, sizeof(buf), "illegal jump %d%d -> %d%d: logged, resynced, resumed",
                 JUMPS[j][0], JUMPS[j][1], JUMPS[j][2], JUMPS[j][3]);
        check(buf, ok);
    }
}

static void test_random_walk_vs_reference_model(void)
{
    quad_decoder_t d;
    ref_decoder_t  r;
    quad_decode_init(&d);
    ref_init(&r);

    /*
     * Deterministic pseudo-random walk: 50,000 steps using an LCG.
     * Each step chooses between:
     *   - stay (same levels): 10%
     *   - forward edge:       43%
     *   - reverse edge:       43%
     *   - illegal jump:        4% (exercises the error/resync path under load)
     *
     * Cross-checked step-by-step against the independent circular-phase
     * reference model.
     */
    uint32_t lcg = 0x12345678;
    int curr_a = 0, curr_b = 0;

    /* Initial sample */
    feed_both(&d, &r, curr_a, curr_b);

    int ok = 1;
    for (int i = 0; i < 50000; i++) {
        lcg = lcg * 1664525u + 1013904223u;
        uint32_t roll = lcg % 100;

        int next_a = curr_a, next_b = curr_b;

        if (roll < 10) {
            /* stationary */
        } else if (roll < 53) {
            /* forward */
            int s = NEXT_FWD[((curr_a ? 1 : 0) << 1) | (curr_b ? 1 : 0)];
            next_a = (s >> 1) & 1;
            next_b = s & 1;
        } else if (roll < 96) {
            /* reverse */
            int s = NEXT_REV[((curr_a ? 1 : 0) << 1) | (curr_b ? 1 : 0)];
            next_a = (s >> 1) & 1;
            next_b = s & 1;
        } else {
            /* illegal jump (invert both) */
            next_a = !curr_a;
            next_b = !curr_b;
        }

        if (!feed_both(&d, &r, next_a, next_b)) {
            ok = 0;
            break;
        }
        curr_a = next_a;
        curr_b = next_b;
    }

    check("50k-step random walk matches circular-phase reference", ok);
}

int main(void)
{
    printf("=== quad_decode unit test suite ===\n\n");

    test_initial_sync_and_stationary();
    test_direction_conventions();
    test_full_revolutions();
    test_dither_cancellation();
    test_reversals_at_every_depth();
    test_invalid_transitions();
    test_random_walk_vs_reference_model();

    printf("\nSummary: %d / %d checks passed.\n", g_checks - g_failures, g_checks);

    if (g_failures != 0) {
        printf("VERDICT: FAIL (%d failures)\n", g_failures);
        return 1;
    }

    printf("VERDICT: PASS\n");
    return 0;
}
