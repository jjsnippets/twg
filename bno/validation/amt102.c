/*
 * amt102.c - AMT102-V incremental encoder reader (libgpiod v1)
 *
 * Part of: rpi4b prod code/bno/validation
 *
 * Requires libgpiod v1.6+ (libgpiod-dev 1.6.x per bno/readme.md):
 * uses gpiod_line_event_read_multiple() and the bias-disable request
 * flag, both introduced in 1.6.
 *
 * Event path: poll() the three per-line event fds, drain each readable
 * line with read_multiple (never reading an empty queue, so this works
 * regardless of fd blocking mode), merge all events into one batch,
 * sort by kernel timestamp, then feed the decoder in order. Sorting
 * matters: processing all of A's events before B's would collapse the
 * winding number and under-count a full revolution.
 */

#include <errno.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <gpiod.h>

#include "validation/amt102.h"
#include "validation/quad_decode.h"

#define AMT102_GPIOCHIP  "gpiochip0"
#define AMT102_LINE_A    17      /* BCM 17, header pin 11 */
#define AMT102_LINE_B    27      /* BCM 27, header pin 13 */
#define AMT102_LINE_X    22      /* BCM 22, header pin 15 */

#define AMT102_MAX_BATCH 64      /* events read per line per drain pass */
#define AMT102_EV_MAX    256     /* combined batch processed per pass */

struct line_slot {
    struct gpiod_line *line;
    int                fd;
    unsigned           pin;
};

struct amt102 {
    struct gpiod_chip *chip;
    struct line_slot   la;       /* channel A */
    struct line_slot   lb;       /* channel B */
    struct line_slot   lx;       /* index channel */

    quad_decoder_t     dec;
    amt102_state_t     state;

    /* Last known level for each line, seeded at open and tracked per edge */
    int                last_a;
    int                last_b;
};

/* Unified event record for timestamp-sorted processing */
typedef struct {
    uint64_t ts_ns;
    uint8_t  source;   /* 'A', 'B' or 'X' */
    uint8_t  type;     /* GPIOD_LINE_EVENT_RISING_EDGE / FALLING_EDGE */
} raw_event_t;

static uint64_t ts_to_ns(const struct timespec *ts)
{
    return ((uint64_t)ts->tv_sec * 1000000000ULL) + (uint64_t)ts->tv_nsec;
}

static int cmp_raw_events(const void *p1, const void *p2)
{
    const raw_event_t *e1 = p1;
    const raw_event_t *e2 = p2;
    if (e1->ts_ns < e2->ts_ns) return -1;
    if (e1->ts_ns > e2->ts_ns) return +1;
    return 0;
}

/*
 * Request a single line for edge events. Tries bias-disable first (the
 * AMT102 is push-pull CMOS; no internal pull is wanted), falls back
 * without the flag if the running kernel rejects it.
 */
static int request_line(struct gpiod_chip *chip,
                        struct line_slot *slot,
                        unsigned pin,
                        int event_type,
                        const char *consumer,
                        bool *bias_fallback)
{
    struct gpiod_line_request_config cfg;

    slot->pin = pin;
    slot->line = gpiod_chip_get_line(chip, pin);
    if (!slot->line) return -1;

    memset(&cfg, 0, sizeof(cfg));
    cfg.consumer = consumer;
    cfg.request_type = event_type;
    cfg.flags = GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE;

    if (gpiod_line_request(slot->line, &cfg, 0) < 0) {
        /* Retry without the bias flag */
        cfg.flags = 0;
        if (gpiod_line_request(slot->line, &cfg, 0) < 0) {
            slot->line = NULL;
            return -1;
        }
        *bias_fallback = true;
    }

    slot->fd = gpiod_line_event_get_fd(slot->line);
    if (slot->fd < 0) {
        gpiod_line_release(slot->line);
        slot->line = NULL;
        return -1;
    }

    return 0;
}

int amt102_open(amt102_t **out)
{
    amt102_t *e;
    bool bias_fallback = false;

    if (!out) { errno = EINVAL; return -1; }
    *out = NULL;

    e = calloc(1, sizeof(*e));
    if (!e) return -1;

    e->chip = gpiod_chip_open_lookup(AMT102_GPIOCHIP);
    if (!e->chip) {
        free(e);
        return -1;
    }

    if (request_line(e->chip, &e->la, AMT102_LINE_A,
                     GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES,
                     "amt102_a", &bias_fallback) < 0 ||
        request_line(e->chip, &e->lb, AMT102_LINE_B,
                     GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES,
                     "amt102_b", &bias_fallback) < 0 ||
        request_line(e->chip, &e->lx, AMT102_LINE_X,
                     GPIOD_LINE_REQUEST_EVENT_RISING_EDGE,
                     "amt102_x", &bias_fallback) < 0) {
        amt102_close(e);
        return -1;
    }

    quad_decode_init(&e->dec);
    memset(&e->state, 0, sizeof(e->state));
    e->state.bias_fallback = bias_fallback;

    /*
     * Read initial static levels so the decoder knows the starting state.
     * Line levels cannot be read while event-requested via gpiod_line_get_value
     * on some kernel versions, but on libgpiod 1.6 / Pi kernel 6.x this works.
     * If it returns -1, leave last_a/last_b as 0; the first edge event will
     * only sync the decoder state (no count) - see quad_decode.h.
     */
    e->last_a = gpiod_line_get_value(e->la.line);
    e->last_b = gpiod_line_get_value(e->lb.line);
    if (e->last_a >= 0 && e->last_b >= 0) {
        quad_decode_step(&e->dec, e->last_a, e->last_b);
        e->state.synced = (e->dec.state != QUAD_STATE_UNKNOWN);
    } else {
        e->last_a = 0;
        e->last_b = 0;
    }

    *out = e;
    return 0;
}

void amt102_close(amt102_t *enc)
{
    if (!enc) return;
    if (enc->la.line) gpiod_line_release(enc->la.line);
    if (enc->lb.line) gpiod_line_release(enc->lb.line);
    if (enc->lx.line) gpiod_line_release(enc->lx.line);
    if (enc->chip)    gpiod_chip_close(enc->chip);
    free(enc);
}

/*
 * Drains one line's event queue into ev_out using the batch read API.
 * Never blocks (caller only invokes this after poll reported POLLIN).
 */
static int drain_line(struct line_slot *slot, uint8_t src,
                      raw_event_t *ev_out, int max_ev)
{
    struct gpiod_line_event buf[AMT102_MAX_BATCH];
    int total = 0;

    while (total < max_ev) {
        int to_read = max_ev - total;
        if (to_read > AMT102_MAX_BATCH) to_read = AMT102_MAX_BATCH;

        int n = gpiod_line_event_read_multiple(slot->line, buf, to_read);
        if (n <= 0) break;

        for (int i = 0; i < n; i++) {
            ev_out[total + i].ts_ns  = ts_to_ns(&buf[i].ts);
            ev_out[total + i].source = src;
            ev_out[total + i].type   = buf[i].event_type;
        }
        total += n;
        if (n < to_read) break;   /* drained */
    }
    return total;
}

int amt102_poll(amt102_t *enc, int timeout_ms)
{
    struct pollfd pfd[3];
    raw_event_t events[AMT102_EV_MAX];
    int n_ev = 0;
    int ret;

    if (!enc) { errno = EINVAL; return -1; }

    pfd[0].fd = enc->la.fd;  pfd[0].events = POLLIN; pfd[0].revents = 0;
    pfd[1].fd = enc->lb.fd;  pfd[1].events = POLLIN; pfd[1].revents = 0;
    pfd[2].fd = enc->lx.fd;  pfd[2].events = POLLIN; pfd[2].revents = 0;

    ret = poll(pfd, 3, timeout_ms);
    if (ret <= 0) return ret;   /* 0 = timeout, <0 = error/EINTR */

    /* Drain all lines that have data waiting */
    if (pfd[0].revents & POLLIN) {
        n_ev += drain_line(&enc->la, 'A', events + n_ev, AMT102_EV_MAX - n_ev);
    }
    if (pfd[1].revents & POLLIN) {
        n_ev += drain_line(&enc->lb, 'B', events + n_ev, AMT102_EV_MAX - n_ev);
    }
    if (pfd[2].revents & POLLIN) {
        n_ev += drain_line(&enc->lx, 'X', events + n_ev, AMT102_EV_MAX - n_ev);
    }

    if (n_ev == 0) return 0;

    /* Sort combined batch by kernel timestamp to preserve transition order */
    if (n_ev > 1) {
        qsort(events, n_ev, sizeof(raw_event_t), cmp_raw_events);
    }

    /* Process in timestamp order */
    for (int i = 0; i < n_ev; i++) {
        const raw_event_t *e = &events[i];
        enc->state.last_event_ts_ns = e->ts_ns;

        if (e->source == 'X') {
            /* Rising edge on X: update pulse counter and snapshot */
            enc->state.x_pulses++;
            enc->state.count_at_last_index   = enc->dec.count;
            enc->state.a_edges_at_last_index = enc->state.a_edges;
            enc->state.b_edges_at_last_index = enc->state.b_edges;
            enc->state.invalid_at_last_index = enc->dec.invalid;
            enc->state.last_index_ts_ns      = e->ts_ns;
            continue;
        }

        /* Channel A or B edge */
        int level = (e->type == GPIOD_LINE_EVENT_RISING_EDGE) ? 1 : 0;
        if (e->source == 'A') {
            enc->last_a = level;
            enc->state.a_edges++;
        } else {
            enc->last_b = level;
            enc->state.b_edges++;
        }

        quad_decode_step(&enc->dec, enc->last_a, enc->last_b);
    }

    /* Publish current counters to public snapshot */
    enc->state.count   = enc->dec.count;
    enc->state.invalid = enc->dec.invalid;
    enc->state.synced  = (enc->dec.state != QUAD_STATE_UNKNOWN);

    return n_ev;
}

void amt102_get_state(const amt102_t *enc, amt102_state_t *out)
{
    if (enc && out) *out = enc->state;
}
