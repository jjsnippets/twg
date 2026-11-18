#include "validation/validate_logger.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "app/imu_csv.h"

#define VALIDATE_TWO_PI 6.28318530717958647692

static const char *outcome_name(ValidateOutcome outcome)
{
    switch (outcome) {
    case VALIDATE_OUTCOME_COMPLETE:
        return "complete";
    case VALIDATE_OUTCOME_INTERRUPTED:
        return "interrupted";
    case VALIDATE_OUTCOME_FAILED:
        return "failed";
    case VALIDATE_OUTCOME_ABORTED_BEFORE_START:
    default:
        return "aborted_before_start";
    }
}

bool validate_logger_init(ValidateLogger *logger)
{
    if (logger == NULL) {
        errno = EINVAL;
        return false;
    }

    memset(logger, 0, sizeof(*logger));
    atomic_init(&logger->done, false);
    atomic_init(&logger->failed, false);

    logger->records = calloc(
        VALIDATE_RECORD_CAPACITY, sizeof(*logger->records));

    return logger->records != NULL;
}

bool validate_logger_open(ValidateLogger *logger)
{
    struct timespec wall;
    struct tm local;
    char stamp[32];
    char imu_header[IMU_CSV_LINE_CAPACITY];
    int fd;
    int saved_errno;

    if (logger == NULL ||
        logger->records == NULL ||
        logger->file != NULL ||
        logger->thread_started ||
        logger->finishing) {
        errno = EINVAL;
        return false;
    }

    if (clock_gettime(CLOCK_REALTIME, &wall) != 0 ||
        localtime_r(&wall.tv_sec, &local) == NULL ||
        strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &local) == 0u) {
        return false;
    }

    if (snprintf(
            logger->filename, sizeof(logger->filename),
            "imu_validate_%s.csv", stamp) >=
        (int)sizeof(logger->filename)) {
        errno = ENAMETOOLONG;
        return false;
    }

    if (imu_csv_format_header(
            imu_header, sizeof(imu_header)) == 0u) {
        errno = EINVAL;
        return false;
    }

    fd = open(
        logger->filename,
        O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC,
        0644);

    if (fd < 0) {
        return false;
    }

    logger->file = fdopen(fd, "w");
    if (logger->file == NULL) {
        saved_errno = errno;
        (void)close(fd);
        errno = saved_errno;
        return false;
    }

    if (fprintf(
            logger->file,
            "# validation_schema=%u\n"
            "# incomplete_unless_final_outcome_footer_is_present\n"
            "trial,elapsed_ns,encoder_snapshot_ns,encoder_zero_ns,"
            "encoder_count,encoder_angle_rad,encoder_raw_count,"
            "encoder_a_edges,encoder_b_edges,encoder_x_pulses,"
            "encoder_invalid,encoder_synced,encoder_bias_fallback,"
            "encoder_last_event_ns,encoder_last_index_ns,%s",
            VALIDATE_SCHEMA_VERSION, imu_header) < 0 ||
        fflush(logger->file) != 0) {
        saved_errno = errno != 0 ? errno : EIO;
        (void)fclose(logger->file);
        logger->file = NULL;
        errno = saved_errno;
        return false;
    }

    logger->count = 0u;
    memset(&logger->end, 0, sizeof(logger->end));
    atomic_store_explicit(
        &logger->done, false, memory_order_relaxed);
    atomic_store_explicit(
        &logger->failed, false, memory_order_relaxed);

    return true;
}

const char *validate_logger_filename(const ValidateLogger *logger)
{
    return logger != NULL ? logger->filename : "";
}

bool validate_logger_append(
    ValidateLogger *logger,
    const ValidateRecord *record)
{
    if (logger == NULL ||
        record == NULL ||
        logger->file == NULL ||
        logger->finishing) {
        errno = EINVAL;
        return false;
    }

    if (logger->count >= VALIDATE_RECORD_CAPACITY) {
        errno = ENOBUFS;
        return false;
    }

    logger->records[logger->count++] = *record;
    return true;
}

static bool write_record(FILE *file, const ValidateRecord *record)
{
    char imu_line[IMU_CSV_LINE_CAPACITY];
    const amt102_state_t *encoder = &record->encoder;
    double angle =
        (double)encoder->relative_count * VALIDATE_TWO_PI /
        (double)AMT102_COUNTS_PER_REV;

    if (imu_csv_format_row(
            &record->imu, imu_line, sizeof(imu_line)) == 0u) {
        return false;
    }

    return fprintf(
        file,
        "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ","
        "%" PRId64 ",%.17g,%" PRId64 ","
        "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ","
        "%u,%u,%" PRIu64 ",%" PRIu64 ",%s",
        record->trial,
        record->elapsed_ns,
        encoder->snapshot_ts_ns,
        encoder->zero_ts_ns,
        encoder->relative_count,
        angle,
        encoder->count,
        encoder->a_edges,
        encoder->b_edges,
        encoder->x_pulses,
        encoder->invalid,
        encoder->synced ? 1u : 0u,
        encoder->bias_fallback ? 1u : 0u,
        encoder->last_event_ts_ns,
        encoder->last_index_ts_ns,
        imu_line) >= 0;
}

static void *write_trial(void *opaque)
{
    ValidateLogger *logger = opaque;
    FILE *file = logger->file;
    size_t written = 0u;
    bool ok = true;
    uint64_t duration = 0u;

    if (logger->end.start_ns != 0u &&
        logger->end.end_ns >= logger->end.start_ns) {
        duration =
            logger->end.end_ns - logger->end.start_ns;
    }

    if (fprintf(
            file,
            "# trial=%" PRIu64
            " trial_start_ns=%" PRIu64
            " trial_end_ns=%" PRIu64
            " planned_window_ns=%" PRIu64 "\n",
            logger->end.trial,
            logger->end.start_ns,
            logger->end.end_ns,
            VALIDATE_WINDOW_NS) < 0) {
        ok = false;
    }

    while (ok && written < logger->count) {
        if (!write_record(file, &logger->records[written])) {
            ok = false;
            break;
        }
        ++written;
    }

    if (ok &&
        fprintf(
            file,
            "# outcome=%s"
            " duration_ns=%" PRIu64
            " observed_stop_ns=%" PRIu64
            " signal=%d"
            " rows=%zu"
            " gates_skipped=%" PRIu64 "\n",
            outcome_name(logger->end.outcome),
            duration,
            logger->end.observed_stop_ns,
            logger->end.signal_number,
            written,
            logger->end.gates_skipped) < 0) {
        ok = false;
    }

    if (fflush(file) != 0) {
        ok = false;
    }
    if (fsync(fileno(file)) != 0) {
        ok = false;
    }
    if (fclose(file) != 0) {
        ok = false;
    }

    logger->file = NULL;

    if (!ok) {
        atomic_store_explicit(
            &logger->failed, true, memory_order_release);
    }

    atomic_store_explicit(
        &logger->done, true, memory_order_release);
    return NULL;
}

bool validate_logger_finish(
    ValidateLogger *logger,
    const ValidateEnd *end)
{
    int rc;

    if (logger == NULL ||
        end == NULL ||
        logger->file == NULL ||
        logger->finishing) {
        errno = EINVAL;
        return false;
    }

    logger->end = *end;
    logger->finishing = true;

    rc = pthread_create(
        &logger->worker, NULL, write_trial, logger);

    if (rc != 0) {
        /*
         * Preserve buffered records if possible, but report failure.
         * Only this exceptional fallback performs trial writes in owner.
         */
        atomic_store_explicit(
            &logger->failed, true, memory_order_release);
        (void)write_trial(logger);
        errno = rc;
        return false;
    }

    logger->thread_started = true;
    return true;
}

bool validate_logger_done(const ValidateLogger *logger)
{
    return logger != NULL &&
           logger->finishing &&
           atomic_load_explicit(
               &logger->done, memory_order_acquire);
}

bool validate_logger_failed(const ValidateLogger *logger)
{
    return logger != NULL &&
           atomic_load_explicit(
               &logger->failed, memory_order_acquire);
}

bool validate_logger_reap(ValidateLogger *logger)
{
    int rc;

    if (!validate_logger_done(logger)) {
        errno = EBUSY;
        return false;
    }

    if (logger->thread_started) {
        rc = pthread_join(logger->worker, NULL);
        if (rc != 0) {
            errno = rc;
            return false;
        }
        logger->thread_started = false;
    }

    logger->finishing = false;
    return true;
}

void validate_logger_destroy(ValidateLogger *logger)
{
    if (logger == NULL) {
        return;
    }

    /*
     * The owner must finish/reap before destruction. Do not free memory
     * that a worker might still reference.
     */
    if (logger->thread_started || logger->finishing) {
        return;
    }

    if (logger->file != NULL) {
        (void)fclose(logger->file);
        logger->file = NULL;
    }

    free(logger->records);
    logger->records = NULL;
}
