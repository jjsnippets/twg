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

#include "amt102.h"
#include "quad_decode.h"

#define AMT102_GPIOCHIP  "gpiochip0"
#define AMT102_LINE_A    17      /* BCM 17, header pin 11 */
#define AMT102_LINE_B    27      /* BCM 27, header pin 13 */
#define AMT102_LINE_X    22      /* BCM 22, header pin 15 */

#define AMT102_MAX_BATCH 64      /* events read per line per drain pass */
#define AMT102_EV_MAX    256     /* combined batch processed per pass */

struct amt102 {
    struct gpiod_chip *chip;
    struct gpiod_line *la, *lb, *lx;
    int                fd_a, fd_b, fd_x;
    quad_decoder_t     dec;
    int                last_a, last_b;    /* -1 until first event on that line */
    bool               bias_fallback;
    uint64_t           a_edges, b_edges, x_pulses;
    int64_t            count_at_last_index;
    uint64_t           a_edges_at_last_index;
    uint64_t           b_edges_at_last_index;
    uint64_t           invalid_at_last_index;
    uint64_t           last_index_ts_ns;
    uint64_t           last_event_ts_ns;
};

struct amt_event {
    uint64_t ts_ns;
    int      line;      /* 0 = A, 1 = B, 2 = X */
    int      rising;
};

static uint64_t ts_to_ns(const struct timespec *ts)
{
    return (uint64_t)ts->tv_sec * 1000000000ull + (uint64_t)ts->tv_nsec;
}

/* Order by kernel timestamp; equal timestamps fall back to line order. */
static int ev_cmp(const void *pa, const void *pb)
{
    const struct amt_event *a = pa;
    const struct amt_event *b = pb;

    if (a->ts_ns != b->ts_ns)
        return (a->ts_ns < b->ts_ns) ? -1 : 1;
    return a->line - b->line;
}

static int request_line(struct gpiod_line *line, int req_type,
                        bool bias_disable)
{
    struct gpiod_line_request_config cfg;

    memset(&cfg, 0, sizeof(cfg));
    cfg.consumer     = "amt102";
    cfg.request_type = req_type;
    if (bias_disable)
        cfg.flags = GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE;

    return gpiod_line_request(line, &cfg, 0);
}

static void process_event(struct amt102 *e, const struct amt_event *ev)
{
    e->last_event_ts_ns = ev->ts_ns;

    switch (ev->line) {
    case 0:                                   /* A */
        e->a_edges++;
        e->last_a = ev->rising ? 1 : 0;
        break;
    case 1:                                   /* B */
        e->b_edges++;
        e->last_b = ev->rising ? 1 : 0;
        break;
    case 2:                                   /* X (rising edge only) */
    default:
        e->x_pulses++;
        e->count_at_last_index   = e->dec.count;
        e->a_edges_at_last_index = e->a_edges;
        e->b_edges_at_last_index = e->b_edges;
        e->invalid_at_last_index = e->dec.invalid;
        e->last_index_ts_ns      = ev->ts_ns;
        return;
    }

    /* Feed the decoder once both levels are known. The first such call
     * only syncs the decoder state (no count) - see quad_decode.h. */
    if (e->last_a >= 0 && e->last_b >= 0)
        quad_decode_step(&e->dec, e->last_a, e->last_b);
}

/*
 * Drain one line's pending events into the combined batch. Reads only
 * after a zero-timeout poll confirms the queue is non-empty, so the
 * read can never block. Stops at the batch cap without reading, so no
 * event is ever dropped (the caller re-polls and continues).
 */
static int drain_line(struct amt102 *e, int line_idx,
                      struct amt_event *ev, int *nev)
{
    struct gpiod_line_event glev[AMT102_MAX_BATCH];
    struct gpiod_line *line;
    int fd;

    switch (line_idx) {
    case 0:  line = e->la; fd = e->fd_a; break;
    case 1:  line = e->lb; fd = e->fd_b; break;
    default: line = e->lx; fd = e->fd_x; break;
    }

    for (;;) {
        struct pollfd pfd;
        int n, i;

        if (*nev >= AMT102_EV_MAX)
            break;                          /* batch full: process, re-poll */

        pfd.fd      = fd;
        pfd.events  = POLLIN;
        pfd.revents = 0;
        if (poll(&pfd, 1, 0) <= 0)
            break;                          /* nothing more queued */
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
            return -1;

        n = gpiod_line_event_read_multiple(line, glev, AMT102_MAX_BATCH);
        if (n < 0)
            return (errno == EAGAIN) ? 0 : -1;
        if (n == 0)
            break;

        for (i = 0; i < n; i++) {
            if (*nev >= AMT102_EV_MAX)
                break;
            ev[*nev].ts_ns  = ts_to_ns(&glev[i].ts);
            ev[*nev].line   = line_idx;
            ev[*nev].rising =
                (glev[i].event_type == GPIOD_LINE_EVENT_RISING_EDGE);
            (*nev)++;
        }
    }
    return 0;
}

/*
 * One poll+drain+process pass.
 * Returns 1 if any events were processed (more may be pending),
 * 0 if nothing was pending (timeout, EINTR, or fully drained),
 * or -1 on error. Adds processed events to *total.
 */
static int poll_once(struct amt102 *e, int timeout_ms, int *total)
{
    struct pollfd pfd[3];
    struct amt_event ev[AMT102_EV_MAX];
    int nev = 0;
    int rc, i, line;

    pfd[0].fd      = e->fd_a;
    pfd[1].fd      = e->fd_b;
    pfd[2].fd      = e->fd_x;
    pfd[0].events  = POLLIN;
    pfd[1].events  = POLLIN;
    pfd[2].events  = POLLIN;

    rc = poll(pfd, 3, timeout_ms);
    if (rc < 0)
        return (errno == EINTR) ? 0 : -1;
    if (rc == 0)
        return 0;

    for (i = 0; i < 3; i++)
        if (pfd[i].revents & (POLLERR | POLLHUP | POLLNVAL))
            return -1;

    for (line = 0; line < 3; line++) {
        if (pfd[line].revents & POLLIN) {
            if (drain_line(e, line, ev, &nev) < 0)
                return -1;
        }
    }

    if (nev == 0)
        return 0;

    qsort(ev, (size_t)nev, sizeof(ev[0]), ev_cmp);
    for (i = 0; i < nev; i++)
        process_event(e, &ev[i]);

    *total += nev;
    return 1;
}

int amt102_poll(amt102_t *e, int timeout_ms)
{
    int total = 0;

    for (;;) {
        int rc = poll_once(e, total ? 0 : timeout_ms, &total);
        if (rc < 0)
            return -1;
        if (rc == 0)
            break;
    }
    return total;
}

int amt102_open(amt102_t **enc_out)
{
    struct amt102 *e;
    int rc;

    *enc_out = NULL;

    e = calloc(1, sizeof(*e));
    if (!e) {
        errno = ENOMEM;
        return -1;
    }
    e->last_a = -1;
    e->last_b = -1;
    quad_decode_init(&e->dec);

    e->chip = gpiod_chip_open_by_name(AMT102_GPIOCHIP);
    if (!e->chip)
        goto fail;

    e->la = gpiod_chip_get_line(e->chip, AMT102_LINE_A);
    e->lb = gpiod_chip_get_line(e->chip, AMT102_LINE_B);
    e->lx = gpiod_chip_get_line(e->chip, AMT102_LINE_X);
    if (!e->la || !e->lb || !e->lx)
        goto fail;

    if (gpiod_line_is_used(e->la) || gpiod_line_is_used(e->lb) ||
        gpiod_line_is_used(e->lx)) {
        errno = EBUSY;
        goto fail;
    }

    /* Bias-disable first (push-pull encoder outputs, no pull wanted).
     * Kernels older than 5.5 reject the flag - fall back without it. */
    rc = request_line(e->la, GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES, true);
    if (rc < 0) {
        e->bias_fallback = true;
        rc = request_line(e->la, GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES, false);
        if (rc < 0)
            goto fail;
    }
    rc = request_line(e->lb, GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES,
                      !e->bias_fallback);
    if (rc < 0)
        goto fail;
    rc = request_line(e->lx, GPIOD_LINE_REQUEST_EVENT_RISING_EDGE,
                      !e->bias_fallback);
    if (rc < 0)
        goto fail;

    e->fd_a = gpiod_line_event_get_fd(e->la);
    e->fd_b = gpiod_line_event_get_fd(e->lb);
    e->fd_x = gpiod_line_event_get_fd(e->lx);
    if (e->fd_a < 0 || e->fd_b < 0 || e->fd_x < 0)
        goto fail;

    *enc_out = e;
    return 0;

fail:
    {
        int saved = errno;
        amt102_close(e);
        errno = saved ? saved : EIO;
    }
    return -1;
}

void amt102_close(struct amt102 *e)
{
    if (!e)
        return;
    if (e->la)
        gpiod_line_release(e->la);
    if (e->lb)
        gpiod_line_release(e->lb);
    if (e->lx)
        gpiod_line_release(e->lx);
    if (e->chip)
        gpiod_chip_close(e->chip);
    free(e);
}

void amt102_get_state(const amt102_t *e, amt102_state_t *out)
{
    out->count                   = e->dec.count;
    out->a_edges                 = e->a_edges;
    out->b_edges                 = e->b_edges;
    out->x_pulses                = e->x_pulses;
    out->invalid                 = e->dec.invalid;
    out->synced                  = (e->last_a >= 0 && e->last_b >= 0);
    out->bias_fallback           = e->bias_fallback;
    out->count_at_last_index     = e->count_at_last_index;
    out->a_edges_at_last_index   = e->a_edges_at_last_index;
    out->b_edges_at_last_index   = e->b_edges_at_last_index;
    out->invalid_at_last_index   = e->invalid_at_last_index;
    out->last_index_ts_ns        = e->last_index_ts_ns;
    out->last_event_ts_ns        = e->last_event_ts_ns;
}
