/*
 * amt102.h - AMT102-V incremental encoder reader (libgpiod v1 event API)
 *
 * Part of: rpi4b prod code/bno/validation
 *
 * Opens the encoder's A/B/X lines on gpiochip0, registers edge events
 * (both edges on A and B, rising edge only on X), and decodes them into
 * an x4 position count via quad_decode.
 *
 * Wiring (BCM numbering, matches the agreed level-shifted chain):
 *   encoder A -> 1k -> BCM 17 (header pin 11)
 *   encoder B -> 1k -> BCM 27 (header pin 13)
 *   encoder X -> 1k -> BCM 22 (header pin 15)
 *   encoder 5V -> Pi 5V rail, encoder G -> Pi GND (common ground)
 *   (encoder connector pin order is B, 5V, A, X, G - check before power)
 *
 * Resolution: AMT102 DIP preset 2048 PPR -> 8192 x4 counts per rev.
 *
 * NOT thread-safe: one thread owns the handle (amt102_poll /
 * amt102_get_state). Threading belongs to the future sensor_validate
 * design, not to this module.
 */

#ifndef AMT102_H
#define AMT102_H

#include <stdbool.h>
#include <stdint.h>

/* 2048 PPR x 4 edges per cycle */
#define AMT102_COUNTS_PER_REV 8192

typedef struct amt102 amt102_t;      /* opaque handle */

/* Point-in-time snapshot, filled by amt102_get_state(). */
typedef struct {
    int64_t  count;                  /* net x4 counts since open */
    uint64_t a_edges;                /* both-edge events seen on A */
    uint64_t b_edges;                /* both-edge events seen on B */
    uint64_t x_pulses;               /* rising index pulses seen */
    uint64_t invalid;                /* illegal quadrature transitions */
    bool     synced;                 /* both line levels known (decoder counting) */
    bool     bias_fallback;          /* kernel rejected bias-disable request flag */

    /* Values frozen at the most recent index pulse - use these (not the
     * live counters) for exact index-to-index window statistics. */
    int64_t  count_at_last_index;
    uint64_t a_edges_at_last_index;
    uint64_t b_edges_at_last_index;
    uint64_t invalid_at_last_index;
    uint64_t last_index_ts_ns;       /* kernel timestamp of last index pulse */
    uint64_t last_event_ts_ns;       /* kernel timestamp of newest event */
} amt102_state_t;

/*
 * Open gpiochip0 and request A/B (both edges) and X (rising edge).
 * Bias-disable is requested first (encoder outputs are push-pull, no
 * pull wanted) and retried without the flag if the kernel rejects it.
 * Fails with EBUSY if any of the three lines is already in use.
 * Returns 0 on success, -1 on failure (errno set).
 */
int  amt102_open(amt102_t **enc);

/* Release all lines and free the handle. NULL-safe. */
void amt102_close(amt102_t *enc);

/*
 * Wait up to timeout_ms (-1 = forever) for encoder events, then drain
 * everything pending and process it in kernel-timestamp order (events
 * from the three lines are merged and sorted before decoding).
 * Returns the number of events processed this call, 0 on timeout or
 * EINTR, or -1 on error (errno set).
 */
int  amt102_poll(amt102_t *enc, int timeout_ms);

/* Copy the current snapshot into *out. */
void amt102_get_state(const amt102_t *enc, amt102_state_t *out);

#endif /* AMT102_H */
