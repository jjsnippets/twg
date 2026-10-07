#ifndef IMU_LOGGER_H
#define IMU_LOGGER_H

/*
 * Phase 9.6 logger queue and worker.
 *
 * One fixed single-producer/single-consumer ring of ImuPublicationRecord_t
 * plus one worker thread that formats rows (imu_csv) and writes them through
 * a sink. The producer is the publisher/owner loop; it never blocks, never
 * allocates, never takes a lock, and never makes a system call: enqueue is a
 * few atomic operations and a record copy. The worker polls the ring every
 * IMU_LOGGER_POLL_NS (it is never signalled, so the producer stays
 * wake-free) and is not a real-time thread.
 *
 * Overflow policy: if the ring is full the new record is dropped, never an
 * older one, and recordsDropped is incremented. The stored copy of the next
 * accepted record carries loggerDrops = drops counted before that attempt.
 *
 * Failure policy: the first sink write failure sets writeFailed (sticky).
 * The worker then keeps dequeuing and discarding so the producer is never
 * blocked; recordsWritten stops advancing. The owner is expected to poll
 * imu_logger_write_failed() and end the run as a logger failure.
 *
 * The ImuLogger_t object (about 0.5 MiB) must have static storage; the
 * module never allocates. After a drain timeout the object must stay valid
 * (the worker may still be running); call imu_logger_drain() again to retry.
 *
 * Header row is written synchronously by imu_logger_start() on the calling
 * (startup) thread before the worker exists.
 */

#include <pthread.h>
#include <stdalign.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app/imu_publish.h"

#define IMU_LOGGER_QUEUE_CAPACITY   2048u   /* power of two */
#define IMU_LOGGER_BATCH_BYTES      32768u
#define IMU_LOGGER_POLL_NS          5000000ull

/*
 * write: return 0 on success; must write all bytes or fail. Called only from
 * the start caller (header) and the worker (rows). sync: optional, called
 * once by the worker after the final flush; 0 on success.
 */
typedef struct {
    int  (*write)(void *ctx, const char *data, size_t length);
    int  (*sync)(void *ctx);
    void  *ctx;
} ImuLoggerSink_t;

typedef struct {
    uint32_t version;
    uint32_t reserved;

    alignas(64) _Atomic uint64_t head;       /* producer */
    alignas(64) _Atomic uint64_t tail;       /* consumer */

    alignas(64) _Atomic uint64_t enqueued;   /* producer */
    _Atomic uint64_t dropped;                /* producer */
    _Atomic uint32_t highWater;              /* producer */
    _Atomic uint64_t written;                /* worker */
    _Atomic uint32_t shutdownErrors;         /* worker */
    _Atomic int      startResult;            /* ImuLoggerStartResult_t */
    _Atomic unsigned writeFailed;
    _Atomic unsigned stopRequested;
    _Atomic unsigned workerDone;
    _Atomic unsigned drainStarted;
    _Atomic unsigned drainComplete;
    _Atomic unsigned drainTimedOut;

    ImuLoggerSink_t sink;
    pthread_t       thread;
    uint8_t         threadValid;
    uint8_t         reserved8[7];

    size_t   batchUsed;                      /* worker only */
    uint32_t batchRows;                      /* worker only */
    char     batch[IMU_LOGGER_BATCH_BYTES];

    ImuPublicationRecord_t ring[IMU_LOGGER_QUEUE_CAPACITY];
} ImuLogger_t;

/* Zero the object and set version/capacity. Not thread-safe; call before
 * start from the owning thread. */
bool imu_logger_init(ImuLogger_t *logger);

/* Write the CSV header through the sink, then start the worker. Returns true
 * only if both succeed (startResult OK). One attempt per init. */
bool imu_logger_start(ImuLogger_t *logger, const ImuLoggerSink_t *sink);

/* Producer only. Real-time safe. Returns true if the record was queued;
 * false if the logger is not running, draining, the record version is wrong,
 * or the ring was full (that last case counts one drop). */
bool imu_logger_enqueue(ImuLogger_t *logger, const ImuPublicationRecord_t *record);

/* Request shutdown, wait up to timeoutNs (CLOCK_MONOTONIC) for the worker
 * to flush, sync, and exit, then join it. true = drained and joined. A false
 * return with drainTimedOut set may be retried. After success further
 * enqueues are rejected. */
bool imu_logger_drain(ImuLogger_t *logger, uint64_t timeoutNs);

bool imu_logger_write_failed(const ImuLogger_t *logger);
bool imu_logger_stats(const ImuLogger_t *logger, ImuLoggerStats_t *out);

/* Convenience sink writing to a POSIX file descriptor (EINTR-safe loop,
 * fsync on sync). *fd must outlive the logger. Closing the fd is the
 * caller's job, after a successful drain. */
bool imu_logger_sink_fd(ImuLoggerSink_t *sink, int *fd);

#endif /* IMU_LOGGER_H */
