#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include "app/imu_logger.h"

#include <errno.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "app/imu_csv.h"

#define LOGGER_VERSION 1u
#define QUEUE_MASK (IMU_LOGGER_QUEUE_CAPACITY - 1u)

_Static_assert((IMU_LOGGER_QUEUE_CAPACITY & (IMU_LOGGER_QUEUE_CAPACITY - 1u)) == 0u,
               "queue capacity must be a power of two");
_Static_assert(IMU_LOGGER_BATCH_BYTES >= 2u * IMU_CSV_LINE_CAPACITY,
               "batch must hold two worst-case lines");

static void sleep_ns(uint64_t ns)
{
    struct timespec ts;

    ts.tv_sec = (time_t)(ns / 1000000000ull);
    ts.tv_nsec = (long)(ns % 1000000000ull);
    (void)nanosleep(&ts, NULL);
}

static bool now_ns(uint64_t *out)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return false;
    }
    *out = (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
    return true;
}

bool imu_logger_init(ImuLogger_t *logger)
{
    if (logger == NULL) {
        return false;
    }
    memset(logger, 0, sizeof(*logger));
    logger->version = LOGGER_VERSION;
    atomic_store(&logger->startResult, (int)IMU_LOGGER_START_NOT_ATTEMPTED);
    return true;
}

static bool logger_usable(const ImuLogger_t *logger)
{
    return logger != NULL && logger->version == LOGGER_VERSION;
}

/* ---- worker ---- */

static void worker_flush(ImuLogger_t *l)
{
    if (l->batchUsed == 0u) {
        return;
    }
    if (atomic_load(&l->writeFailed) == 0u) {
        if (l->sink.write(l->sink.ctx, l->batch, l->batchUsed) != 0) {
            atomic_store(&l->writeFailed, 1u);
        } else {
            atomic_fetch_add_explicit(&l->written, l->batchRows,
                                      memory_order_release);
        }
    }
    l->batchUsed = 0u;
    l->batchRows = 0u;
}

static void worker_process(ImuLogger_t *l, const ImuPublicationRecord_t *rec)
{
    size_t n;

    if (atomic_load(&l->writeFailed) != 0u) {
        return;
    }
    n = imu_csv_format_row(rec, l->batch + l->batchUsed,
                           IMU_LOGGER_BATCH_BYTES - l->batchUsed);
    if (n == 0u && l->batchUsed != 0u) {
        worker_flush(l);
        if (atomic_load(&l->writeFailed) != 0u) {
            return;
        }
        n = imu_csv_format_row(rec, l->batch, IMU_LOGGER_BATCH_BYTES);
    }
    if (n == 0u) {
        atomic_store(&l->writeFailed, 1u);
        return;
    }
    l->batchUsed += n;
    l->batchRows += 1u;
}

static void *worker_main(void *arg)
{
    ImuLogger_t *l = (ImuLogger_t *)arg;

    for (;;) {
        unsigned stop = atomic_load_explicit(&l->stopRequested,
                                             memory_order_acquire);
        uint64_t tail = atomic_load_explicit(&l->tail, memory_order_relaxed);
        uint64_t head = atomic_load_explicit(&l->head, memory_order_acquire);

        if (head == tail) {
            worker_flush(l);
            if (stop != 0u) {
                break;
            }
            sleep_ns(IMU_LOGGER_POLL_NS);
            continue;
        }

        while (tail != head) {
            worker_process(l, &l->ring[tail & QUEUE_MASK]);
            ++tail;
            atomic_store_explicit(&l->tail, tail, memory_order_release);
        }
    }

    if (l->sink.sync != NULL && l->sink.sync(l->sink.ctx) != 0) {
        atomic_fetch_add(&l->shutdownErrors, 1u);
    }
    atomic_store_explicit(&l->workerDone, 1u, memory_order_release);
    return NULL;
}

/* ---- lifecycle ---- */

bool imu_logger_start(ImuLogger_t *logger, const ImuLoggerSink_t *sink)
{
    size_t n;

    if (!logger_usable(logger) || sink == NULL || sink->write == NULL ||
        atomic_load(&logger->startResult) !=
            (int)IMU_LOGGER_START_NOT_ATTEMPTED) {
        return false;
    }

    logger->sink = *sink;
    n = imu_csv_format_header(logger->batch, IMU_LOGGER_BATCH_BYTES);
    if (n == 0u || logger->sink.write(logger->sink.ctx, logger->batch, n) != 0) {
        atomic_store(&logger->startResult, (int)IMU_LOGGER_START_FAILED);
        return false;
    }

    if (pthread_create(&logger->thread, NULL, worker_main, logger) != 0) {
        atomic_store(&logger->startResult, (int)IMU_LOGGER_START_FAILED);
        return false;
    }
    logger->threadValid = 1u;
    atomic_store_explicit(&logger->startResult, (int)IMU_LOGGER_START_OK,
                          memory_order_release);
    return true;
}

bool imu_logger_enqueue(ImuLogger_t *l, const ImuPublicationRecord_t *record)
{
    uint64_t head;
    uint64_t tail;
    uint64_t drops;
    uint64_t depth;
    ImuPublicationRecord_t *slot;

    if (!logger_usable(l) || record == NULL ||
        record->version != IMU_PUBLICATION_CONTRACT_VERSION ||
        atomic_load_explicit(&l->startResult, memory_order_acquire) !=
            (int)IMU_LOGGER_START_OK ||
        atomic_load_explicit(&l->stopRequested, memory_order_relaxed) != 0u) {
        return false;
    }

    head = atomic_load_explicit(&l->head, memory_order_relaxed);
    tail = atomic_load_explicit(&l->tail, memory_order_acquire);
    drops = atomic_load_explicit(&l->dropped, memory_order_relaxed);

    if (head - tail >= IMU_LOGGER_QUEUE_CAPACITY) {
        atomic_store_explicit(&l->dropped, drops + 1u, memory_order_relaxed);
        return false;
    }

    slot = &l->ring[head & QUEUE_MASK];
    *slot = *record;
    slot->loggerDrops = drops;
    atomic_store_explicit(&l->head, head + 1u, memory_order_release);
    atomic_store_explicit(&l->enqueued,
        atomic_load_explicit(&l->enqueued, memory_order_relaxed) + 1u,
        memory_order_relaxed);

    depth = head + 1u - tail;
    if (depth > atomic_load_explicit(&l->highWater, memory_order_relaxed)) {
        atomic_store_explicit(&l->highWater, (uint32_t)depth,
                              memory_order_relaxed);
    }
    return true;
}

bool imu_logger_drain(ImuLogger_t *l, uint64_t timeoutNs)
{
    uint64_t start;
    uint64_t now;
    bool haveClock;

    if (!logger_usable(l) ||
        atomic_load_explicit(&l->startResult, memory_order_acquire) !=
            (int)IMU_LOGGER_START_OK) {
        return false;
    }
    if (atomic_load(&l->drainComplete) != 0u) {
        return true;
    }

    atomic_store(&l->drainStarted, 1u);
    atomic_store_explicit(&l->stopRequested, 1u, memory_order_release);

    haveClock = now_ns(&start);
    for (;;) {
        if (atomic_load_explicit(&l->workerDone, memory_order_acquire) != 0u) {
            break;
        }
        if (!haveClock || !now_ns(&now) || now - start >= timeoutNs) {
            atomic_store(&l->drainTimedOut, 1u);
            return false;
        }
        sleep_ns(1000000ull);
    }

    if (l->threadValid != 0u) {
        (void)pthread_join(l->thread, NULL);
        l->threadValid = 0u;
    }
    atomic_store(&l->drainTimedOut, 0u);
    atomic_store(&l->drainComplete, 1u);
    return true;
}

bool imu_logger_write_failed(const ImuLogger_t *l)
{
    return logger_usable(l) && atomic_load(&l->writeFailed) != 0u;
}

bool imu_logger_stats(const ImuLogger_t *l, ImuLoggerStats_t *out)
{
    if (!logger_usable(l) || out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->version = IMU_LOGGER_STATS_CONTRACT_VERSION;
    out->queueCapacity = IMU_LOGGER_QUEUE_CAPACITY;
    out->queueHighWater = atomic_load(&l->highWater);
    out->startResult = (ImuLoggerStartResult_t)atomic_load(&l->startResult);
    out->recordsEnqueued = atomic_load(&l->enqueued);
    out->recordsWritten = atomic_load(&l->written);
    out->recordsDropped = atomic_load(&l->dropped);
    out->shutdownErrors = atomic_load(&l->shutdownErrors);
    out->writeFailed = (uint8_t)(atomic_load(&l->writeFailed) != 0u);
    out->drainStarted = (uint8_t)(atomic_load(&l->drainStarted) != 0u);
    out->drainComplete = (uint8_t)(atomic_load(&l->drainComplete) != 0u);
    out->drainTimedOut = (uint8_t)(atomic_load(&l->drainTimedOut) != 0u);
    return true;
}

/* ---- fd sink ---- */

static int fd_write(void *ctx, const char *data, size_t length)
{
    int fd = *(int *)ctx;

    while (length != 0u) {
        ssize_t w = write(fd, data, length);

        if (w < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        data += (size_t)w;
        length -= (size_t)w;
    }
    return 0;
}

static int fd_sync(void *ctx)
{
    return fsync(*(int *)ctx) == 0 ? 0 : -1;
}

bool imu_logger_sink_fd(ImuLoggerSink_t *sink, int *fd)
{
    if (sink == NULL || fd == NULL || *fd < 0) {
        return false;
    }
    sink->write = fd_write;
    sink->sync = fd_sync;
    sink->ctx = fd;
    return true;
}
