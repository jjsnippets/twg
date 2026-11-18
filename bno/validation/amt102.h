#ifndef AMT102_H
#define AMT102_H

#include <stdbool.h>
#include <stdint.h>

/*
 * AMT102-V, libgpiod v1.6 event API.
 *
 * BCM GPIO:
 *   A = 16
 *   B = 20
 *   X = 21
 *
 * Encoder DIP setting: 2048 PPR, decoded x4.
 *
 * One thread owns the handle. None of these functions is thread-safe.
 */
#define AMT102_COUNTS_PER_REV 8192

typedef struct amt102 amt102_t;

typedef struct {
    /* Lifetime position and diagnostics, preserved across amt102_zero(). */
    int64_t count;
    uint64_t a_edges;
    uint64_t b_edges;
    uint64_t x_pulses;
    uint64_t invalid;
    bool synced;
    bool bias_fallback;

    /* Lifetime values frozen at the most recent index pulse. */
    int64_t count_at_last_index;
    uint64_t a_edges_at_last_index;
    uint64_t b_edges_at_last_index;
    uint64_t invalid_at_last_index;
    uint64_t last_index_ts_ns;
    uint64_t last_event_ts_ns;

    /*
     * Software reference.
     *
     * relative_count = count - count captured by the latest zero.
     * zero_ts_ns is CLOCK_MONOTONIC.
     */
    int64_t relative_count;
    uint64_t zero_ts_ns;

    /*
     * CLOCK_MONOTONIC cutoff through which this snapshot was drained.
     * This is not the timestamp of a new physical encoder edge.
     */
    uint64_t snapshot_ts_ns;
} amt102_state_t;

/* Returns 0 on success, -1 on failure with errno set. */
int amt102_open(amt102_t **out);

/* NULL-safe. */
void amt102_close(amt102_t *enc);

/*
 * Wait up to timeout_ms, then process events through a monotonic cutoff.
 *
 * timeout_ms:
 *   -1: wait indefinitely
 *    0: do not wait
 *   >0: bounded wait in milliseconds
 *
 * Returns events processed, 0 on timeout/EINTR, or -1 on error.
 *
 * A bounded drain that cannot catch up fails with EOVERFLOW rather than
 * silently publishing a snapshot as though all pending events were read.
 *
 * The GPIO v1 API does not expose reliable kernel-queue overflow evidence.
 * A successful poll is therefore not proof that no kernel event was lost.
 */
int amt102_poll(amt102_t *enc, int timeout_ms);

/*
 * Drain through a fresh monotonic cutoff and establish the software zero.
 *
 * Preserves decoder phase, lifetime count, index values, and diagnostics.
 * Returns 0 on success, -1 on failure with errno set.
 */
int amt102_zero(amt102_t *enc);

void amt102_get_state(const amt102_t *enc, amt102_state_t *out);

#endif /* AMT102_H */
