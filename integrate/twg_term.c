#define _POSIX_C_SOURCE 200809L

#include "twg_term.h"

#include <errno.h>
#include <poll.h>
#include <sched.h>
#include <signal.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define TWG_TERM_POLL_MS 100

static uint64_t effective_tail(const TwgTerm *term)
{
    uint64_t tail =
        atomic_load_explicit(&term->tail, memory_order_acquire);
    uint64_t floor =
        atomic_load_explicit(&term->flushFloor, memory_order_acquire);

    return tail < floor ? floor : tail;
}

static bool offer(TwgTerm *term, char key)
{
    uint64_t head;
    uint64_t tail;

    head = atomic_load_explicit(&term->head, memory_order_relaxed);
    tail = effective_tail(term);

    if (head - tail >= TWG_TERM_QUEUE_CAPACITY) {
        atomic_fetch_add_explicit(
            &term->dropped, 1u, memory_order_relaxed);
        return false;
    }

    term->queue[head % TWG_TERM_QUEUE_CAPACITY].key = key;

    atomic_store_explicit(
        &term->head, head + 1u, memory_order_release);

    return true;
}

static bool enable_raw_mode(TwgTerm *term)
{
    struct termios raw;

    if (!isatty(term->fd)) {
        return true;
    }

    if (tcgetattr(term->fd, &term->savedTermios) != 0) {
        return false;
    }

    term->termiosSaved = true;
    raw = term->savedTermios;

    raw.c_iflag &=
        (tcflag_t)~(ICRNL | INLCR | IGNCR | IXON | IXOFF);
    raw.c_lflag &=
        (tcflag_t)~(ICANON | ECHO | ECHONL | IEXTEN);

    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(term->fd, TCSANOW, &raw) != 0) {
        term->termiosSaved = false;
        return false;
    }

    term->rawMode = true;
    return true;
}

static void restore_terminal(TwgTerm *term)
{
    if (term != NULL &&
        term->rawMode &&
        term->termiosSaved) {
        (void)tcsetattr(
            term->fd, TCSANOW, &term->savedTermios);
    }

    if (term != NULL) {
        term->rawMode = false;
    }
}

static bool acknowledge_flush(
    TwgTerm *term,
    uint64_t ticket)
{
    uint64_t head;

    if (isatty(term->fd) &&
        tcflush(term->fd, TCIFLUSH) != 0) {
        atomic_store(&term->inputError, true);
        return false;
    }

    head = atomic_load_explicit(
        &term->head, memory_order_acquire);

    atomic_store_explicit(
        &term->flushFloor, head, memory_order_release);

    atomic_store_explicit(
        &term->flushAcknowledged,
        ticket,
        memory_order_release);

    return true;
}

static void *worker_main(void *opaque)
{
    TwgTerm *term = opaque;

    atomic_store(
        &term->workerSchedOther,
        sched_getscheduler(0) == SCHED_OTHER);

    while (!atomic_load(&term->stopRequested)) {
        struct pollfd pfd;
        char bytes[32];
        uint64_t requested;
        uint64_t acknowledged;
        ssize_t count;
        ssize_t i;
        int ready;

        requested = atomic_load_explicit(
            &term->flushRequested, memory_order_acquire);

        acknowledged = atomic_load_explicit(
            &term->flushAcknowledged, memory_order_acquire);

        if (requested != acknowledged) {
            if (!acknowledge_flush(term, requested)) {
                break;
            }
            continue;
        }

        memset(&pfd, 0, sizeof(pfd));
        pfd.fd = term->fd;
        pfd.events = POLLIN | POLLHUP;

        ready = poll(&pfd, 1, TWG_TERM_POLL_MS);

        if (ready == 0) {
            continue;
        }

        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }

            atomic_store(&term->inputError, true);
            break;
        }

        if ((pfd.revents & (POLLERR | POLLNVAL)) != 0) {
            atomic_store(&term->inputError, true);
            break;
        }

        if ((pfd.revents & (POLLIN | POLLHUP)) == 0) {
            continue;
        }

        count = read(term->fd, bytes, sizeof(bytes));

        if (count == 0) {
            atomic_store(&term->eofSeen, true);
            break;
        }

        if (count < 0) {
            if (errno == EINTR || errno == EAGAIN) {
                continue;
            }

            atomic_store(&term->inputError, true);
            break;
        }

        for (i = 0; i < count; ++i) {
            char key = bytes[i];

            if (key == 0x03) {
                (void)raise(SIGINT);
                continue;
            }

            if (key == '\r') {
                key = '\n';
            }

            (void)offer(term, key);
        }
    }

    return NULL;
}

bool twg_term_init(TwgTerm *term, int fd)
{
    if (term == NULL || fd < 0) {
        return false;
    }

    memset(term, 0, sizeof(*term));
    term->fd = fd;

    atomic_init(&term->head, 0u);
    atomic_init(&term->tail, 0u);
    atomic_init(&term->flushFloor, 0u);
    atomic_init(&term->dropped, 0u);
    atomic_init(&term->flushRequested, 0u);
    atomic_init(&term->flushAcknowledged, 0u);

    atomic_init(&term->stopRequested, false);
    atomic_init(&term->eofSeen, false);
    atomic_init(&term->inputError, false);
    atomic_init(&term->workerSchedOther, false);

    term->initialized = true;
    return true;
}

bool twg_term_start(TwgTerm *term)
{
    pthread_attr_t attr;
    struct sched_param param;
    int rc;

    if (term == NULL ||
        !term->initialized ||
        term->workerStarted ||
        !enable_raw_mode(term)) {
        return false;
    }

    memset(&param, 0, sizeof(param));

    if (pthread_attr_init(&attr) != 0) {
        restore_terminal(term);
        return false;
    }

    rc = pthread_attr_setinheritsched(
        &attr, PTHREAD_EXPLICIT_SCHED);

    if (rc == 0) {
        rc = pthread_attr_setschedpolicy(&attr, SCHED_OTHER);
    }

    if (rc == 0) {
        rc = pthread_attr_setschedparam(&attr, &param);
    }

    if (rc == 0) {
        rc = pthread_create(
            &term->worker, &attr, worker_main, term);
    }

    (void)pthread_attr_destroy(&attr);

    if (rc != 0) {
        restore_terminal(term);
        return false;
    }

    term->workerStarted = true;
    return true;
}

void twg_term_request_stop(TwgTerm *term)
{
    if (term != NULL && term->initialized) {
        atomic_store(&term->stopRequested, true);
    }
}

bool twg_term_join(TwgTerm *term)
{
    if (term == NULL || !term->initialized) {
        return false;
    }

    if (!term->workerStarted) {
        return true;
    }

    if (pthread_join(term->worker, NULL) != 0) {
        return false;
    }

    term->workerStarted = false;
    return true;
}

void twg_term_destroy(TwgTerm *term)
{
    if (term == NULL || !term->initialized) {
        return;
    }

    twg_term_request_stop(term);
    (void)twg_term_join(term);

    restore_terminal(term);
    term->initialized = false;
}

bool twg_term_dequeue(TwgTerm *term, TwgTermKey *key)
{
    uint64_t tail;
    uint64_t floor;
    uint64_t head;

    if (term == NULL ||
        key == NULL ||
        !term->initialized) {
        return false;
    }

    tail = atomic_load_explicit(
        &term->tail, memory_order_relaxed);

    floor = atomic_load_explicit(
        &term->flushFloor, memory_order_acquire);

    if (tail < floor) {
        tail = floor;
        atomic_store_explicit(
            &term->tail, tail, memory_order_release);
    }

    head = atomic_load_explicit(
        &term->head, memory_order_acquire);

    if (tail == head) {
        return false;
    }

    *key = term->queue[tail % TWG_TERM_QUEUE_CAPACITY];

    atomic_store_explicit(
        &term->tail, tail + 1u, memory_order_release);

    return true;
}

uint64_t twg_term_request_flush(TwgTerm *term)
{
    if (term == NULL || !term->initialized) {
        return 0u;
    }

    return atomic_fetch_add_explicit(
               &term->flushRequested,
               1u,
               memory_order_acq_rel) +
           1u;
}

bool twg_term_flush_complete(
    const TwgTerm *term,
    uint64_t ticket)
{
    return term != NULL &&
           term->initialized &&
           ticket != 0u &&
           atomic_load_explicit(
               &term->flushAcknowledged,
               memory_order_acquire) >= ticket;
}

bool twg_term_eof_seen(const TwgTerm *term)
{
    return term != NULL &&
           atomic_load(&term->eofSeen);
}

bool twg_term_input_error(const TwgTerm *term)
{
    return term != NULL &&
           atomic_load(&term->inputError);
}

bool twg_term_worker_sched_other(const TwgTerm *term)
{
    return term != NULL &&
           atomic_load(&term->workerSchedOther);
}

uint64_t twg_term_dropped(const TwgTerm *term)
{
    return term != NULL
             ? atomic_load(&term->dropped)
             : 0u;
}
