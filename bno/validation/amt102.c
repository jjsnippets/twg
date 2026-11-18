#include "validation/amt102.h"
#include "validation/quad_decode.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <gpiod.h>

#define AMT102_GPIOCHIP "gpiochip0"
#define AMT102_LINE_A 16u
#define AMT102_LINE_B 20u
#define AMT102_LINE_X 21u

#define AMT102_DRAIN_LIMIT 4096

enum {
    SLOT_A = 0,
    SLOT_B = 1,
    SLOT_X = 2,
    SLOT_COUNT = 3
};

typedef struct {
    struct gpiod_line *line;
    int fd;
    bool requested;
    bool pending;
    struct gpiod_line_event event;
} LineSlot;

struct amt102 {
    struct gpiod_chip *chip;
    LineSlot slots[SLOT_COUNT];

    quad_decoder_t decoder;
    amt102_state_t state;

    int last_a;
    int last_b;
    int64_t zero_count;
};

static uint64_t event_ns(const struct gpiod_line_event *event)
{
    return (uint64_t)event->ts.tv_sec * UINT64_C(1000000000) +
           (uint64_t)event->ts.tv_nsec;
}

static int monotonic_ns(uint64_t *out)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return -1;
    }

    *out = (uint64_t)now.tv_sec * UINT64_C(1000000000) +
           (uint64_t)now.tv_nsec;
    return 0;
}

static void publish_state(amt102_t *enc, uint64_t cutoff)
{
    enc->state.count = enc->decoder.count;
    enc->state.invalid = enc->decoder.invalid;
    enc->state.synced =
        enc->decoder.state != QUAD_STATE_UNKNOWN;
    enc->state.relative_count =
        enc->decoder.count - enc->zero_count;
    enc->state.snapshot_ts_ns = cutoff;
}

static int request_line(
    amt102_t *enc,
    unsigned slot_index,
    unsigned pin,
    int request_type,
    const char *consumer)
{
    LineSlot *slot = &enc->slots[slot_index];
    struct gpiod_line_request_config config;
    int flags;

    slot->line = gpiod_chip_get_line(enc->chip, pin);
    if (slot->line == NULL) {
        return -1;
    }

    memset(&config, 0, sizeof(config));
    config.consumer = consumer;
    config.request_type = request_type;
    config.flags = GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE;

    if (gpiod_line_request(slot->line, &config, 0) != 0) {
        int saved_errno = errno;

        /*
         * Retry only failures compatible with unsupported request flags.
         * Do not disguise EBUSY or permission errors as bias fallback.
         */
        if (saved_errno != EINVAL &&
            saved_errno != EOPNOTSUPP &&
            saved_errno != ENOTTY) {
            errno = saved_errno;
            return -1;
        }

        config.flags = 0;
        if (gpiod_line_request(slot->line, &config, 0) != 0) {
            return -1;
        }
        enc->state.bias_fallback = true;
    }

    slot->requested = true;
    slot->fd = gpiod_line_event_get_fd(slot->line);
    if (slot->fd < 0) {
        return -1;
    }

    flags = fcntl(slot->fd, F_GETFL);
    if (flags < 0 ||
        fcntl(slot->fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        return -1;
    }

    return 0;
}

int amt102_open(amt102_t **out)
{
    amt102_t *enc;
    int saved_errno;
    unsigned i;
    uint64_t now;

    if (out == NULL) {
        errno = EINVAL;
        return -1;
    }
    *out = NULL;

    enc = calloc(1u, sizeof(*enc));
    if (enc == NULL) {
        return -1;
    }

    for (i = 0u; i < SLOT_COUNT; ++i) {
        enc->slots[i].fd = -1;
    }

    enc->chip = gpiod_chip_open_lookup(AMT102_GPIOCHIP);
    if (enc->chip == NULL) {
        goto fail;
    }

    if (request_line(
            enc, SLOT_A, AMT102_LINE_A,
            GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES, "amt102_a") != 0 ||
        request_line(
            enc, SLOT_B, AMT102_LINE_B,
            GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES, "amt102_b") != 0 ||
        request_line(
            enc, SLOT_X, AMT102_LINE_X,
            GPIOD_LINE_REQUEST_EVENT_RISING_EDGE, "amt102_x") != 0) {
        goto fail;
    }

    quad_decode_init(&enc->decoder);

    /*
     * Require real initial A/B levels. Do not invent an initial phase
     * when static line reads are unavailable on the running platform.
     *
     * Open the tool with the pendulum stationary.
     */
    enc->last_a =
        gpiod_line_get_value(enc->slots[SLOT_A].line);
    if (enc->last_a < 0) {
        goto fail;
    }

    enc->last_b =
        gpiod_line_get_value(enc->slots[SLOT_B].line);
    if (enc->last_b < 0) {
        goto fail;
    }

    (void)quad_decode_step(
        &enc->decoder, enc->last_a, enc->last_b);

    if (monotonic_ns(&now) != 0) {
        goto fail;
    }

    publish_state(enc, now);
    *out = enc;
    return 0;

fail:
    saved_errno = errno;
    amt102_close(enc);
    errno = saved_errno;
    return -1;
}

void amt102_close(amt102_t *enc)
{
    unsigned i;

    if (enc == NULL) {
        return;
    }

    for (i = 0u; i < SLOT_COUNT; ++i) {
        if (enc->slots[i].requested) {
            gpiod_line_release(enc->slots[i].line);
        }
    }

    if (enc->chip != NULL) {
        gpiod_chip_close(enc->chip);
    }
    free(enc);
}

/*
 * Keep one lookahead event per line. Event-request FDs are nonblocking.
 */
static int fill_pending(LineSlot *slot)
{
    int rc;

    if (slot->pending) {
        return 0;
    }

    do {
        rc = gpiod_line_event_read_fd(slot->fd, &slot->event);
    } while (rc < 0 && errno == EINTR);

    if (rc == 0) {
        slot->pending = true;
        return 0;
    }

    if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return 0;
    }

    return -1;
}

static void apply_event(amt102_t *enc, unsigned index)
{
    LineSlot *slot = &enc->slots[index];
    const struct gpiod_line_event *event = &slot->event;
    uint64_t timestamp = event_ns(event);
    int level =
        event->event_type == GPIOD_LINE_EVENT_RISING_EDGE ? 1 : 0;

    enc->state.last_event_ts_ns = timestamp;

    if (index == SLOT_X) {
        ++enc->state.x_pulses;
        enc->state.count_at_last_index = enc->decoder.count;
        enc->state.a_edges_at_last_index = enc->state.a_edges;
        enc->state.b_edges_at_last_index = enc->state.b_edges;
        enc->state.invalid_at_last_index = enc->decoder.invalid;
        enc->state.last_index_ts_ns = timestamp;
    } else {
        if (index == SLOT_A) {
            enc->last_a = level;
            ++enc->state.a_edges;
        } else {
            enc->last_b = level;
            ++enc->state.b_edges;
        }

        (void)quad_decode_step(
            &enc->decoder, enc->last_a, enc->last_b);
    }

    slot->pending = false;
}

static int drain_through(amt102_t *enc, uint64_t cutoff)
{
    int processed = 0;

    for (;;) {
        int selected = -1;
        uint64_t earliest = UINT64_MAX;
        unsigned i;

        for (i = 0u; i < SLOT_COUNT; ++i) {
            uint64_t timestamp;

            if (fill_pending(&enc->slots[i]) != 0) {
                return -1;
            }

            if (!enc->slots[i].pending) {
                continue;
            }

            timestamp = event_ns(&enc->slots[i].event);
            if (timestamp <= cutoff &&
                (selected < 0 || timestamp < earliest)) {
                earliest = timestamp;
                selected = (int)i;
            }
        }

        if (selected < 0) {
            publish_state(enc, cutoff);
            return processed;
        }

        if (processed >= AMT102_DRAIN_LIMIT) {
            errno = EOVERFLOW;
            return -1;
        }

        apply_event(enc, (unsigned)selected);
        ++processed;
    }
}

int amt102_poll(amt102_t *enc, int timeout_ms)
{
    struct pollfd descriptors[SLOT_COUNT];
    bool have_pending = false;
    uint64_t cutoff;
    unsigned i;
    int rc;

    if (enc == NULL || timeout_ms < -1) {
        errno = EINVAL;
        return -1;
    }

    for (i = 0u; i < SLOT_COUNT; ++i) {
        descriptors[i].fd = enc->slots[i].fd;
        descriptors[i].events = POLLIN;
        descriptors[i].revents = 0;
        have_pending = have_pending || enc->slots[i].pending;
    }

    rc = poll(
        descriptors, SLOT_COUNT,
        have_pending ? 0 : timeout_ms);

    if (rc < 0) {
        return errno == EINTR ? 0 : -1;
    }

    for (i = 0u; i < SLOT_COUNT; ++i) {
        if ((descriptors[i].revents &
             (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            errno = EIO;
            return -1;
        }
    }

    if (monotonic_ns(&cutoff) != 0) {
        return -1;
    }

    return drain_through(enc, cutoff);
}

int amt102_zero(amt102_t *enc)
{
    uint64_t cutoff;

    if (enc == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (monotonic_ns(&cutoff) != 0 ||
        drain_through(enc, cutoff) < 0) {
        return -1;
    }

    if (!enc->state.synced) {
        errno = EAGAIN;
        return -1;
    }

    enc->zero_count = enc->decoder.count;
    enc->state.zero_ts_ns = cutoff;
    publish_state(enc, cutoff);
    return 0;
}

void amt102_get_state(const amt102_t *enc, amt102_state_t *out)
{
    if (enc != NULL && out != NULL) {
        *out = enc->state;
    }
}
