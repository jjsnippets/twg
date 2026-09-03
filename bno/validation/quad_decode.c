/*
 * quad_decode.c - pure x4 quadrature decoder state machine
 *
 * Part of: rpi4b prod code/bno/validation
 *
 * State encoding: s = (A << 1) | B.
 *
 *   Forward (A leads B, +1 per edge):  00 -> 10 -> 11 -> 01 -> 00
 *                                       0  -> 2  -> 3  -> 1  -> 0
 *   Reverse (B leads A, -1 per edge):  0  -> 1  -> 3  -> 2  -> 0
 *
 * One full cycle = 4 edges = +/-4 counts (x4 decoding). At the AMT102
 * factory preset of 2048 PPR that is 8192 counts per revolution.
 */

#include "quad_decode.h"

#define QDEC_INVALID 2

/*
 * Transition table, indexed by (prev_state << 2) | curr_state.
 * Rows = previous state 00, 01, 10, 11; columns = current state
 * 00, 01, 10, 11. Value is the count delta; QDEC_INVALID marks a
 * jump where both lines changed between consecutive samples.
 */
static const int8_t QDEC[16] = {
    /*        curr:  00   01   10   11 */
    /* prev 00 */   0,  -1,  +1,  QDEC_INVALID,
    /* prev 01 */  +1,   0,  QDEC_INVALID, -1,
    /* prev 10 */  -1,  QDEC_INVALID, 0,  +1,
    /* prev 11 */  QDEC_INVALID, +1, -1,   0
};

void quad_decode_init(quad_decoder_t *d)
{
    d->state   = QUAD_STATE_UNKNOWN;
    d->count   = 0;
    d->invalid = 0;
}

int quad_decode_step(quad_decoder_t *d, int a, int b)
{
    int curr = ((a ? 1 : 0) << 1) | (b ? 1 : 0);
    int delta;

    if (d->state == QUAD_STATE_UNKNOWN) {
        d->state = curr;          /* first sample: sync only, no count */
        return 0;
    }

    delta = QDEC[(d->state << 2) | curr];
    d->state = curr;

    if (delta == QDEC_INVALID) {
        d->invalid++;             /* both lines changed: count unreliable */
        return 0;                 /* count intentionally left unchanged */
    }

    d->count += delta;
    return delta;
}
