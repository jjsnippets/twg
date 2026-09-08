/*
 * test_quad_decode.c — unit test suite for quad_decode.c
 *
 * Exercises the decode table, step accumulator, index-pulse edge
 * detection, invalid-transition counting, and the edge-burst contract
 * against synthetic sequences. No hardware, no libgpiod, no root.
 *
 * Exit code 0 if all assertions pass, non-zero on first failure.
 *
 * Build:
 *   gcc -Wall -Wextra -O2 -std=c11 -I. -o test_quad_decode \
 *       test_quad_decode.c ../quad_decode.c
 *   ./test_quad_decode
 *
 * Part of the sensor_validate harness (see validation/Makefile).
 */

#include <assert.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "validation/quad_decode.h"

static unsigned g_tests_run    = 0;
static unsigned g_asserts_run  = 0;

#define TEST_BEGIN(name) \
    do { \
        printf("  [RUN ] %s\n", (name)); \
        g_tests_run++; \
    } while (0)

#define TEST_PASS(name) \
    do { \
        printf("  [PASS] %s\n", (name)); \
    } while (0)

#define CHECK(cond) \
    do { \
        g_asserts_run++; \
        if (!(cond)) { \
            fprintf(stderr, "FAIL: %s:%d: assertion failed: %s\n", \
                    __FILE__, __LINE__, #cond); \
            abort(); \
        } \
    } while (0)

#define CHECK_EQ_I64(a, b) \
    do { \
        g_asserts_run++; \
        int64_t _va = (int64_t)(a); \
        int64_t _vb = (int64_t)(b); \
        if (_va != _vb) { \
            fprintf(stderr, "FAIL: %s:%d: %s (%" PRId64 ") != %s (%" PRId64 ")\n", \
                    __FILE__, __LINE__, #a, _va, #b, _vb); \
            abort(); \
        } \
    } while (0)

#define CHECK_EQ_U64(a, b) \
    do { \
        g_asserts_run++; \
        uint64_t _va = (uint64_t)(a); \
        uint64_t _vb = (uint64_t)(b); \
        if (_va != _vb) { \
            fprintf(stderr, "FAIL: %s:%d: %s (%" PRIu64 ") != %s (%" PRIu64 ")\n", \
                    __FILE__, __LINE__, #a, _va, #b, _vb); \
            abort(); \
        } \
    } while (0)

/* ------------------------------------------------------------------ */
/* Test 1: Direct Table Verification                                  */
/* ------------------------------------------------------------------ */

static void test_transition_table_all_16(void)
{
    TEST_BEGIN("transition_table_all_16");

    /* 4 stationary cases */
    CHECK(quad_decode_step(0, 0) == 0);
    CHECK(quad_decode_step(1, 1) == 0);
    CHECK(quad_decode_step(2, 2) == 0);
    CHECK(quad_decode_step(3, 3) == 0);

    /* 4 forward cases: 00->01, 01->11, 11->10, 10->00 */
    CHECK(quad_decode_step(0, 1) == +1);
    CHECK(quad_decode_step(1, 3) == +1);
    CHECK(quad_decode_step(3, 2) == +1);
    CHECK(quad_decode_step(2, 0) == +1);

    /* 4 backward cases: 00->10, 10->11, 11->01, 01->00 */
    CHECK(quad_decode_step(0, 2) == -1);
    CHECK(quad_decode_step(2, 3) == -1);
    CHECK(quad_decode_step(3, 1) == -1);
    CHECK(quad_decode_step(1, 0) == -1);

    /* 4 double-transition error cases */
    CHECK(quad_decode_step(0, 3) == QUAD_DECODE_INVALID);
    CHECK(quad_decode_step(1, 2) == QUAD_DECODE_INVALID);
    CHECK(quad_decode_step(2, 1) == QUAD_DECODE_INVALID);
    CHECK(quad_decode_step(3, 0) == QUAD_DECODE_INVALID);

    TEST_PASS("transition_table_all_16");
}

/* ------------------------------------------------------------------ */
/* Test 2: Full Forward Revolution (2048 pulses = 8192 counts)        */
/* ------------------------------------------------------------------ */

static void test_forward_revolution_x4(void)
{
    TEST_BEGIN("forward_revolution_x4");

    QuadDecoder_t d;
    quad_decode_init(&d, 0, 0, 0);

    /* Cycle: (0,0)->(0,1)->(1,1)->(1,0)->(0,0)... */
    const uint8_t a_seq[4] = { 0, 1, 1, 0 };
    const uint8_t b_seq[4] = { 1, 1, 0, 0 };

    for (uint32_t i = 0; i < 2048; i++) {
        for (int phase = 0; phase < 4; phase++) {
            quad_decode_feed_edge(&d, a_seq[phase], b_seq[phase], 0);
        }
    }

    CHECK_EQ_I64(quad_decode_count(&d), 8192);
    CHECK_EQ_U64(quad_decode_invalid_count(&d), 0);
    CHECK_EQ_U64(quad_decode_x_pulses(&d), 0);

    TEST_PASS("forward_revolution_x4");
}

/* ------------------------------------------------------------------ */
/* Test 3: Full Reverse Revolution (-8192 counts)                     */
/* ------------------------------------------------------------------ */

static void test_reverse_revolution_x4(void)
{
    TEST_BEGIN("reverse_revolution_x4");

    QuadDecoder_t d;
    quad_decode_init(&d, 0, 0, 0);

    /* Reverse cycle: (0,0)->(1,0)->(1,1)->(0,1)->(0,0)... */
    const uint8_t a_seq[4] = { 1, 1, 0, 0 };
    const uint8_t b_seq[4] = { 0, 1, 1, 0 };

    for (uint32_t i = 0; i < 2048; i++) {
        for (int phase = 0; phase < 4; phase++) {
            quad_decode_feed_edge(&d, a_seq[phase], b_seq[phase], 0);
        }
    }

    CHECK_EQ_I64(quad_decode_count(&d), -8192);
    CHECK_EQ_U64(quad_decode_invalid_count(&d), 0);
    CHECK_EQ_U64(quad_decode_x_pulses(&d), 0);

    TEST_PASS("reverse_revolution_x4");
}

/* ------------------------------------------------------------------ */
/* Test 4: Direction Reversal (pendulum swing simulation)              */
/* ------------------------------------------------------------------ */

static void test_direction_reversal_hysteresis(void)
{
    TEST_BEGIN("direction_reversal_hysteresis");

    QuadDecoder_t d;
    quad_decode_init(&d, 0, 0, 0);

    /* Forward 100 counts */
    const uint8_t fwd_a[4] = { 0, 1, 1, 0 };
    const uint8_t fwd_b[4] = { 1, 1, 0, 0 };

    for (int i = 0; i < 25; i++) {
        for (int p = 0; p < 4; p++) {
            quad_decode_feed_edge(&d, fwd_a[p], fwd_b[p], 0);
        }
    }
    CHECK_EQ_I64(quad_decode_count(&d), 100);

    /* We are now at a=0, b=0, state=0. Reverse 100 counts back. */
    const uint8_t rev_a[4] = { 1, 1, 0, 0 };
    const uint8_t rev_b[4] = { 0, 1, 1, 0 };

    for (int i = 0; i < 25; i++) {
        for (int p = 0; p < 4; p++) {
            quad_decode_feed_edge(&d, rev_a[p], rev_b[p], 0);
        }
    }
    CHECK_EQ_I64(quad_decode_count(&d), 0);
    CHECK_EQ_U64(quad_decode_invalid_count(&d), 0);

    TEST_PASS("direction_reversal_hysteresis");
}

/* ------------------------------------------------------------------ */
/* Test 5: Invalid Transitions Are Rejected and Flagged                */
/* ------------------------------------------------------------------ */

static void test_invalid_transitions(void)
{
    TEST_BEGIN("invalid_transitions");

    QuadDecoder_t d;
    quad_decode_init(&d, 0, 0, 0);

    /* (0,0) -> (1,1): double transition, count must NOT change */
    quad_decode_feed_edge(&d, 1, 1, 0);
    CHECK_EQ_I64(quad_decode_count(&d), 0);
    CHECK_EQ_U64(quad_decode_invalid_count(&d), 1);

    /* Internal state should now be (1,1). From (1,1), another double jump:
       (1,1) -> (0,0) is invalid. */
    quad_decode_feed_edge(&d, 0, 0, 0);
    CHECK_EQ_I64(quad_decode_count(&d), 0);
    CHECK_EQ_U64(quad_decode_invalid_count(&d), 2);

    /* Now feed a valid transition: (0,0) -> (0,1) = +1 */
    quad_decode_feed_edge(&d, 0, 1, 0);
    CHECK_EQ_I64(quad_decode_count(&d), 1);
    CHECK_EQ_U64(quad_decode_invalid_count(&d), 2);

    TEST_PASS("invalid_transitions");
}

/* ------------------------------------------------------------------ */
/* Test 6: Stationary Edges (bounce on same line)                      */
/* ------------------------------------------------------------------ */

static void test_stationary_no_count(void)
{
    TEST_BEGIN("stationary_no_count");

    QuadDecoder_t d;
    quad_decode_init(&d, 0, 1, 0);

    /* Feeding the same state repeatedly must yield 0 counts, 0 errors */
    for (int i = 0; i < 100; i++) {
        quad_decode_feed_edge(&d, 0, 1, 0);
    }
    CHECK_EQ_I64(quad_decode_count(&d), 0);
    CHECK_EQ_U64(quad_decode_invalid_count(&d), 0);

    TEST_PASS("stationary_no_count");
}

/* ------------------------------------------------------------------ */
/* Test 7: Index Pulse Rising-Edge Detection                          */
/* ------------------------------------------------------------------ */

static void test_index_pulse_edge_detection(void)
{
    TEST_BEGIN("index_pulse_edge_detection");

    QuadDecoder_t d;
    quad_decode_init(&d, 0, 0, 0);

    /* x stays low */
    quad_decode_feed_edge(&d, 0, 1, 0);
    CHECK_EQ_U64(quad_decode_x_pulses(&d), 0);

    /* x goes high: rising edge -> +1 */
    quad_decode_feed_edge(&d, 1, 1, 1);
    CHECK_EQ_U64(quad_decode_x_pulses(&d), 1);

    /* x stays high: no new edge */
    quad_decode_feed_edge(&d, 1, 0, 1);
    CHECK_EQ_U64(quad_decode_x_pulses(&d), 1);

    /* x goes low: falling edge -> no pulse count increment */
    quad_decode_feed_edge(&d, 0, 0, 0);
    CHECK_EQ_U64(quad_decode_x_pulses(&d), 1);

    /* x rises again -> +1 */
    quad_decode_feed_edge(&d, 0, 1, 1);
    CHECK_EQ_U64(quad_decode_x_pulses(&d), 2);

    TEST_PASS("index_pulse_edge_detection");
}

/* ------------------------------------------------------------------ */
/* Test 8: Index Pulse with Initial High State                        */
/* ------------------------------------------------------------------ */

static void test_index_initial_high_no_spurious_edge(void)
{
    TEST_BEGIN("index_initial_high_no_spurious_edge");

    QuadDecoder_t d;
    /* Initialized with x=1 already */
    quad_decode_init(&d, 0, 0, 1);

    /* First event arrives with x still 1: must NOT count as a rising edge */
    quad_decode_feed_edge(&d, 0, 1, 1);
    CHECK_EQ_U64(quad_decode_x_pulses(&d), 0);

    /* Falls to 0 */
    quad_decode_feed_edge(&d, 1, 1, 0);
    CHECK_EQ_U64(quad_decode_x_pulses(&d), 0);

    /* Now rises: should count */
    quad_decode_feed_edge(&d, 1, 0, 1);
    CHECK_EQ_U64(quad_decode_x_pulses(&d), 1);

    TEST_PASS("index_initial_high_no_spurious_edge");
}

/* ------------------------------------------------------------------ */
/* Test 9: quad_decode_feed_burst                                     */
/* ------------------------------------------------------------------ */

static void test_burst_api(void)
{
    TEST_BEGIN("burst_api");

    QuadDecoder_t d;
    quad_decode_init(&d, 0, 0, 0);

    /* 4-edge burst: one full forward cycle with an index pulse on edge 2 */
    const uint8_t a[4] = { 0, 1, 1, 0 };
    const uint8_t b[4] = { 1, 1, 0, 0 };
    const uint8_t x[4] = { 0, 0, 1, 0 };

    quad_decode_feed_burst(&d, a, b, x, 4);

    CHECK_EQ_I64(quad_decode_count(&d), 4);
    CHECK_EQ_U64(quad_decode_invalid_count(&d), 0);
    CHECK_EQ_U64(quad_decode_x_pulses(&d), 1);

    TEST_PASS("burst_api");
}

/* ------------------------------------------------------------------ */
/* Test 10: Counter Reset                                             */
/* ------------------------------------------------------------------ */

static void test_reset_preserves_state(void)
{
    TEST_BEGIN("reset_preserves_state");

    QuadDecoder_t d;
    quad_decode_init(&d, 0, 0, 0);

    quad_decode_feed_edge(&d, 0, 1, 1);
    quad_decode_feed_edge(&d, 1, 1, 1);
    CHECK(quad_decode_count(&d) == 2);
    CHECK(quad_decode_x_pulses(&d) == 1);

    /* Reset counter only */
    quad_decode_reset_count(&d);
    CHECK_EQ_I64(quad_decode_count(&d), 0);
    CHECK_EQ_U64(quad_decode_x_pulses(&d), 0);
    CHECK_EQ_U64(quad_decode_invalid_count(&d), 0);

    /* Next edge from (1,1): (1,0) = +1 forward transition.
       If state was wiped, it would think prev was (0,0) and flag invalid! */
    quad_decode_feed_edge(&d, 1, 0, 0);
    CHECK_EQ_I64(quad_decode_count(&d), 1);
    CHECK_EQ_U64(quad_decode_invalid_count(&d), 0);

    TEST_PASS("reset_preserves_state");
}

/* ------------------------------------------------------------------ */
/* Test 11: 32-bit Integer Overflow / Large Count Sanity              */
/* ------------------------------------------------------------------ */

static void test_large_count_accumulation(void)
{
    TEST_BEGIN("large_count_accumulation");

    QuadDecoder_t d;
    quad_decode_init(&d, 0, 0, 0);

    /* 100 000 forward steps */
    const uint8_t a[4] = { 0, 1, 1, 0 };
    const uint8_t b[4] = { 1, 1, 0, 0 };

    for (int i = 0; i < 25000; i++) {
        quad_decode_feed_burst(&d, a, b, NULL, 4);
    }
    CHECK_EQ_I64(quad_decode_count(&d), 100000);
    CHECK_EQ_U64(quad_decode_invalid_count(&d), 0);

    TEST_PASS("large_count_accumulation");
}

/* ------------------------------------------------------------------ */
/* Main runner                                                        */
/* ------------------------------------------------------------------ */

int main(void)
{
    printf("=== test_quad_decode: AMT102 quadrature decoder unit tests ===\n");

    test_transition_table_all_16();
    test_forward_revolution_x4();
    test_reverse_revolution_x4();
    test_direction_reversal_hysteresis();
    test_invalid_transitions();
    test_stationary_no_count();
    test_index_pulse_edge_detection();
    test_index_initial_high_no_spurious_edge();
    test_burst_api();
    test_reset_preserves_state();
    test_large_count_accumulation();

    printf("\nALL %u TESTS PASSED (%u assertions checked).\n",
           g_tests_run, g_asserts_run);
    return 0;
}
