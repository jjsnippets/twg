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

#include "quad_decode.h"

#define TEST_CYCLES 2048        /* one revolution at 2048 PPR */
#define TEST_COUNTS (TEST_CYCLES * 4)
#define WALK_STEPS  200000

static int g_checks;
static int g_failures;

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
    int i, sum = 0;
    for (i = 0; i < 4; i++)
        sum += advance(d, forward);
    return sum;
}

/*
 * Independent reference model: states sit on a circle at positions
 * PHASE[state]; a transition's delta is the signed circular distance
 * between positions. Different arithmetic than quad_decode's table.
 */
static const int PHASE[4] = { 0, 3, 1, 2 };

typedef struct {
    int     state;    /* -1 until first sample */
    int64_t count;
    uint64_t invalid;
} ref_decoder_t;

static void ref_init(ref_decoder_t *r)
{
    r->state   = -1;
    r->count   = 0;
    r->invalid = 0;
}

static int ref_step(ref_decoder_t *r, int a, int b)
{
    int s = ((a ? 1 : 0) << 1) | (b ? 1 : 0);
    int d;

    if (r->state < 0) {
        r->state = s;
        return 0;
    }
    if (s == r->state)
        return 0;

    d = PHASE[s] - PHASE[r->state];
    if (d == 3)
        d = -1;
    else if (d == -3)
        d = 1;
    else if (d == 2 || d == -2) {          /* opposite state: illegal */
        r->invalid++;
        r->state = s;
        return 0;
    }
    r->state = s;
    r->count += d;
    return d;
}

/* Feed both models the same levels and require full agreement. */
static int feed_both(quad_decoder_t *d, ref_decoder_t *r, int a, int b)
{
    int d1 = quad_decode_step(d, a, b);
    int d2 = ref_step(r, a, b);

    return d1 == d2 && d->count == r->count && d->invalid == r->invalid;
}

static void test_full_revolution(void)
{
    quad_decoder_t d;
    int i;

    quad_decode_init(&d);
    quad_decode_step(&d, 0, 0);            /* sync at 00 */
    for (i = 0; i < TEST_CYCLES; i++)
        cycle(&d, 1);
    check("2048 forward cycles = +8192 counts", d.count == TEST_COUNTS);
    check("returned to starting state 00", d.state == 0);
    check("forward run has zero invalid", d.invalid == 0);

    for (i = 0; i < TEST_CYCLES; i++)
        cycle(&d, 0);
    check("2048 reverse cycles return to 0", d.count == 0);
    check("reverse run has zero invalid", d.invalid == 0);

    /* same revolution starting synced at a different state */
    quad_decode_init(&d);
    quad_decode_step(&d, 1, 1);            /* sync at 11 */
    for (i = 0; i < TEST_CYCLES; i++)
        cycle(&d, 1);
    check("2048 cycles from state 11 also +8192", d.count == TEST_COUNTS);
}

static void test_direction_sign(void)
{
    quad_decoder_t d;

    quad_decode_init(&d);
    quad_decode_step(&d, 0, 0);
    check("A-leads-B edge (00 -> 10) counts +1",
          quad_decode_step(&d, 1, 0) == +1);

    quad_decode_init(&d);
    quad_decode_step(&d, 0, 0);
    check("B-leads-A edge (00 -> 01) counts -1",
          quad_decode_step(&d, 0, 1) == -1);
}

static void test_first_sample_sync(void)
{
    static const int levels[4][2] = {
        { 0, 0 }, { 0, 1 }, { 1, 0 }, { 1, 1 }
    };
    int i, ok_first = 1, ok_state = 1, ok_after = 1;

    for (i = 0; i < 4; i++) {
        quad_decoder_t d;
        int s;

        quad_decode_init(&d);
        if (quad_decode_step(&d, levels[i][0], levels[i][1]) != 0)
            ok_first = 0;
        if (d.state != i || d.count != 0 || d.invalid != 0)
            ok_state = 0;

        s = NEXT_FWD[d.state];             /* one forward edge after sync */
        if (quad_decode_step(&d, (s >> 1) & 1, s & 1) != +1)
            ok_after = 0;
    }
    check("first sample from each state returns 0", ok_first);
    check("first sample syncs state, count/invalid stay 0", ok_state);
    check("edge after sync from each state counts +1", ok_after);
}

static void test_noop_repeats(void)
{
    static const int levels[4][2] = {
        { 0, 0 }, { 0, 1 }, { 1, 0 }, { 1, 1 }
    };
    int i, k, ok = 1;

    for (i = 0; i < 4; i++) {
        quad_decoder_t d;

        quad_decode_init(&d);
        quad_decode_step(&d, levels[i][0], levels[i][1]);
        for (k = 0; k < 100; k++)
            if (quad_decode_step(&d, levels[i][0], levels[i][1]) != 0)
                ok = 0;
        if (d.count != 0 || d.invalid != 0)
            ok = 0;
    }
    check("repeated identical levels never count", ok);
}

static void test_dither(void)
{
    static const int levels[4][2] = {
        { 0, 0 }, { 0, 1 }, { 1, 0 }, { 1, 1 }
    };
    int i, k, ok = 1;

    for (i = 0; i < 4; i++) {
        quad_decoder_t d;
        int s0;

        quad_decode_init(&d);
        quad_decode_step(&d, levels[i][0], levels[i][1]);
        s0 = d.state;

        for (k = 0; k < 1000; k++) {
            int out = NEXT_FWD[s0];
            quad_decode_step(&d, (out >> 1) & 1, out & 1);   /* edge out */
            quad_decode_step(&d, (s0 >> 1) & 1, s0 & 1);     /* edge back */
        }
        if (d.count != 0 || d.invalid != 0)
            ok = 0;
    }
    check("dither pairs are net-zero with no invalid", ok);
}

static void test_illegal_jump(void)
{
    quad_decoder_t d;

    quad_decode_init(&d);
    quad_decode_step(&d, 0, 0);
    check("illegal jump 00 -> 11 returns 0",
          quad_decode_step(&d, 1, 1) == 0);
    check("illegal jump increments invalid", d.invalid == 1);
    check("illegal jump leaves count unchanged", d.count == 0);
    check("decoder resyncs to jumped state", d.state == 3);
    check("decoding continues after jump (11 -> 01 = +1)",
          quad_decode_step(&d, 0, 1) == +1);
    cycle(&d, 1);
    check("full cycle after recovery adds +4", d.count == 5);
    check("no additional invalid during recovery", d.invalid == 1);

    quad_decode_init(&d);
    quad_decode_step(&d, 1, 1);            /* sync at 11 */
    (void)quad_decode_step(&d, 0, 0);      /* 11 -> 00 illegal */
    check("reverse jump 11 -> 00 also flagged",
          d.invalid == 1 && d.count == 0 && d.state == 0);
}

static void test_reversal_mid_cycle(void)
{
    static const int levels[4][2] = {
        { 0, 0 }, { 0, 1 }, { 1, 0 }, { 1, 1 }
    };
    int i, k, j, ok = 1;

    for (i = 0; i < 4; i++) {
        for (k = 1; k <= 4; k++) {
            quad_decoder_t d;

            quad_decode_init(&d);
            quad_decode_step(&d, levels[i][0], levels[i][1]);
            for (j = 0; j < k; j++)
                advance(&d, 1);
            if (d.count != k)
                ok = 0;
            for (j = 0; j < k; j++)
                advance(&d, 0);
            if (d.count != 0 || d.state != i || d.invalid != 0)
                ok = 0;
        }
    }
    check("reversal at every state/depth returns to zero", ok);
}

static uint32_t rng_state = 0x20260901u;

static uint32_t rng(void)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return rng_state;
}

static void test_random_walk(void)
{
    quad_decoder_t d;
    ref_decoder_t r;
    int i, dir = 1, ok = 1;

    quad_decode_init(&d);
    ref_init(&r);
    feed_both(&d, &r, 0, 0);               /* seed both at state 00 */

    for (i = 0; i < WALK_STEPS; i++) {
        uint32_t x = rng() % 1000u;

        if (x < 15u) {
            dir = -dir;                    /* occasional reversal */
        } else if (x < 50u) {
            int s0  = d.state;             /* dither: one edge out and back */
            int out = NEXT_FWD[s0];
            ok = feed_both(&d, &r, (out >> 1) & 1, out & 1);
            if (ok)
                ok = feed_both(&d, &r, (s0 >> 1) & 1, s0 & 1);
        } else {
            int s = (dir > 0) ? NEXT_FWD[d.state] : NEXT_REV[d.state];
            ok = feed_both(&d, &r, (s >> 1) & 1, s & 1);
        }
        if (!ok)
            break;
    }

    check("random walk matches reference model", ok);
    check("random walk has zero invalid transitions",
          d.invalid == 0 && r.invalid == 0);
}

int main(void)
{
    printf("test_quad_decode: x4 quadrature state machine\n");
    printf("----------------------------------------------------\n");

    test_full_revolution();
    test_direction_sign();
    test_first_sample_sync();
    test_noop_repeats();
    test_dither();
    test_illegal_jump();
    test_reversal_mid_cycle();
    test_random_walk();

    printf("----------------------------------------------------\n");
    printf("%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0)
        printf("ALL PASS\n");
    return g_failures ? 1 : 0;
}
