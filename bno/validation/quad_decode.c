/*
 * quad_decode.c — pure quadrature state machine implementation.
 *
 * See validation/quad_decode.h for transition table design and documentation.
 *
 * Part of the sensor_validate harness (see Makefile).
 */

#include "validation/quad_decode.h"

/*
 * 4x4 state transition lookup table.
 * Index = (prev_state << 2) | curr_state.
 * Output: +1 forward, -1 backward, 0 stationary, QUAD_DECODE_INVALID illegal.
 */
static const int8_t sLookup[16] = {
    /* prev=00 (0) */   0, +1, -1, QUAD_DECODE_INVALID,
    /* prev=01 (1) */  -1,  0, QUAD_DECODE_INVALID, +1,
    /* prev=10 (2) */  +1, QUAD_DECODE_INVALID,  0, -1,
    /* prev=11 (3) */  QUAD_DECODE_INVALID, -1, +1,  0,
};

int8_t quad_decode_step(uint8_t prev_state, uint8_t curr_state)
{
    return sLookup[((prev_state & 0x03u) << 2) | (curr_state & 0x03u)];
}

void quad_decode_init(QuadDecoder_t *d, uint8_t a, uint8_t b, uint8_t x)
{
    d->count = 0;
    d->invalid_transitions = 0;
    d->x_pulses = 0;
    d->prev_state = ((a ? 1u : 0u) << 1) | (b ? 1u : 0u);
    d->prev_x = (x ? 1u : 0u);
}

void quad_decode_feed_edge(QuadDecoder_t *d, uint8_t a, uint8_t b, uint8_t x)
{
    uint8_t curr_state = ((a ? 1u : 0u) << 1) | (b ? 1u : 0u);
    int8_t step = quad_decode_step(d->prev_state, curr_state);

    if (step == QUAD_DECODE_INVALID) {
        d->invalid_transitions++;
    } else {
        d->count += step;
    }
    d->prev_state = curr_state;

    uint8_t curr_x = (x ? 1u : 0u);
    if (!d->prev_x && curr_x) {
        d->x_pulses++;
    }
    d->prev_x = curr_x;
}

void quad_decode_feed_burst(QuadDecoder_t *d,
                           const uint8_t *a_buf,
                           const uint8_t *b_buf,
                           const uint8_t *x_buf,
                           size_t n)
{
    for (size_t i = 0; i < n; i++) {
        uint8_t x_val = x_buf ? x_buf[i] : d->prev_x;
        quad_decode_feed_edge(d, a_buf[i], b_buf[i], x_val);
    }
}

int64_t quad_decode_count(const QuadDecoder_t *d)
{
    return d->count;
}

uint64_t quad_decode_invalid_count(const QuadDecoder_t *d)
{
    return d->invalid_transitions;
}

uint64_t quad_decode_x_pulses(const QuadDecoder_t *d)
{
    return d->x_pulses;
}

void quad_decode_reset_count(QuadDecoder_t *d)
{
    d->count = 0;
    d->invalid_transitions = 0;
    d->x_pulses = 0;
}
