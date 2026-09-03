/*
 * quad_decode.h - pure x4 quadrature decoder state machine
 *
 * Part of: rpi4b prod code/bno/validation
 *
 * Pure decoder for incremental encoder A/B channels: no I/O, no globals,
 * no libgpiod - fully unit-testable (see tests/test_quad_decode.c).
 *
 * Sign convention: A-leads-B motion is positive (+1 per edge), which the
 * AMT102 datasheet defines as counter-clockwise viewed from the encoder
 * face. B-leads-A motion is negative.
 *
 * Sync semantics: line levels are unknown at start, so the first call
 * only records the state and counts nothing. Real counting begins on
 * the next edge (the first fraction of a degree of motion is used to
 * sync - this is expected and documented).
 */

#ifndef QUAD_DECODE_H
#define QUAD_DECODE_H

#include <stdint.h>

/* state value before the first sample */
#define QUAD_STATE_UNKNOWN (-1)

typedef struct {
    int     state;    /* current gray-code index (A<<1)|B, 0..3; UNKNOWN until first sync */
    int64_t count;    /* net x4 counts since init (one A-leads-B cycle = +4) */
    uint64_t invalid; /* count of illegal transitions (both lines changed at once) */
} quad_decoder_t;

/* Reset to the initial (unsynced, zero-count) state. */
void quad_decode_init(quad_decoder_t *d);

/*
 * Feed current line levels (any nonzero a/b is treated as high).
 * Returns the count delta for this step: -1, 0 or +1.
 *
 * - First call after init: syncs state only, returns 0.
 * - Same levels as before: returns 0.
 * - Valid single-line transition: returns -1 or +1, count updated.
 * - Illegal transition (both lines changed): increments invalid,
 *   resyncs state to the observed levels, leaves count unchanged,
 *   returns 0. Decoding continues normally from the resynced state.
 */
int quad_decode_step(quad_decoder_t *d, int a, int b);

#endif /* QUAD_DECODE_H */
