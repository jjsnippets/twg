#ifndef VALIDATE_LOGGER_H
#define VALIDATE_LOGGER_H

#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "validation/validate_contract.h"

/*
 * Owner appends records only before finish().
 * Worker reads the immutable buffer only after finish().
 *
 * No record-buffer access is concurrent between owner and worker.
 * done/failed provide completion and failure publication.
 */
typedef struct {
    ValidateRecord *records;
    size_t count;

    FILE *file;
    char filename[VALIDATE_FILENAME_CAPACITY];

    ValidateEnd end;

    pthread_t worker;
    bool thread_started;
    bool finishing;

    atomic_bool done;
    atomic_bool failed;
} ValidateLogger;

bool validate_logger_init(ValidateLogger *logger);

/*
 * Create imu_validate_YYYYMMDD_hhmmss.csv in the working directory.
 * Uses local time and O_EXCL. Existing files are never overwritten.
 */
bool validate_logger_open(ValidateLogger *logger);

const char *validate_logger_filename(const ValidateLogger *logger);

bool validate_logger_append(
    ValidateLogger *logger,
    const ValidateRecord *record);

/*
 * Freeze the buffer and start asynchronous finalization.
 *
 * If worker creation fails, performs a synchronous best-effort save,
 * marks the logger failed, and returns false.
 */
bool validate_logger_finish(
    ValidateLogger *logger,
    const ValidateEnd *end);

bool validate_logger_done(const ValidateLogger *logger);
bool validate_logger_failed(const ValidateLogger *logger);

/* Call after done(). Joins the worker if one was started. */
bool validate_logger_reap(ValidateLogger *logger);

/* Requires any active trial to have been finished and reaped. */
void validate_logger_destroy(ValidateLogger *logger);

#endif /* VALIDATE_LOGGER_H */
