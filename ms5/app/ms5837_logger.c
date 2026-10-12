#define _POSIX_C_SOURCE 200809L

#include "ms5837_logger.h"

#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include <time.h>

static int write_header(FILE *stream)
{
    return fprintf(
        stream,
        "schema_version,"
        "publication_sequence,"
        "scheduled_ns,"
        "scheduled_valid,"
        "actual_ns,"
        "actual_valid,"
        "deadline_missed,"
        "gates_skipped_before,"
        "phase,"
        "sample_ready,"
        "sample_fresh,"
        "sample_stale,"
        "age_valid,"
        "age_ns,"
        "driver_state,"
        "contract_version,"
        "status_flags,"
        "measurement_sequence,"
        "measurement_complete_time_ns,"
        "raw_pressure_d1,"
        "raw_temperature_d2,"
        "pressure_mbar,"
        "temperature_c,"
        "zero_requested,"
        "zero_valid,"
        "zero_valid_samples,"
        "zero_rejected_samples,"
        "surface_pressure_mbar,"
        "fluid_density_kg_m3,"
        "depth_m,"
        "logger_drops_before,"
        "not_ready\n") < 0
        ? -EIO
        : 0;
}

static int write_optional_u64(
    FILE *stream,
    bool valid,
    uint64_t value)
{
    if (valid) {
        return fprintf(stream, "%" PRIu64, value) < 0
            ? -EIO
            : 0;
    }

    return fputs(MS5837_CSV_MISSING_TOKEN, stream) == EOF
        ? -EIO
        : 0;
}

static int write_record(
    FILE *stream,
    const BaroLogRecord_t *record)
{
    const Ms5837PublicationRecord_t *publication;
    const BaroSample_t *sample;
    const char *phase;

    publication = &record->publication;
    sample = &publication->sample;

    phase = record->phase == BARO_CAPTURE_PHASE_ZERO
        ? "zero"
        : "run";

    if (fprintf(
            stream,
            "%" PRIu32 ","
            "%" PRIu64 ","
            "%" PRIu64 ","
            "%u,"
            "%" PRIu64 ","
            "%u,"
            "%u,"
            "%" PRIu32 ","
            "%s,"
            "%u,"
            "%u,"
            "%u,"
            "%u,",
            record->schema_version,
            publication->publication_sequence,
            publication->scheduled_ns,
            publication->scheduled_valid,
            publication->actual_ns,
            publication->actual_valid,
            publication->deadline_missed,
            publication->gates_skipped_before,
            phase,
            publication->sample_ready,
            publication->sample_fresh,
            publication->sample_stale,
            publication->age_valid) < 0) {
        return -EIO;
    }

    if (write_optional_u64(
            stream,
            publication->age_valid != 0U,
            publication->age_ns) < 0) {
        return -EIO;
    }

    if (fprintf(
            stream,
            ",%" PRId32
            ",%" PRIu32
            ",%" PRIu32
            ",%" PRIu64
            ",%" PRIu64
            ",%" PRIu32
            ",%" PRIu32
            ",%.6f"
            ",%.6f"
            ",%u"
            ",%u"
            ",%" PRIu64
            ",%" PRIu64
            ",%.6f"
            ",%.6f"
            ",%.6f"
            ",%" PRIu64
            ",%u\n",
            publication->driver_state,
            sample->contract_version,
            sample->status_flags,
            sample->measurement_sequence,
            sample->measurement_complete_time_ns,
            sample->raw_pressure_d1,
            sample->raw_temperature_d2,
            sample->pressure_mbar,
            sample->temperature_c,
            record->zero_requested ? 1U : 0U,
            record->zero_valid ? 1U : 0U,
            record->zero_valid_samples,
            record->zero_rejected_samples,
            record->surface_pressure_mbar,
            record->fluid_density_kg_m3,
            record->depth_m,
            record->logger_drops_before,
            publication->not_ready) < 0) {
        return -EIO;
    }

    return 0;
}

static void *logger_thread(void *opaque)
{
    Ms5837Logger_t *logger =
        (Ms5837Logger_t *)opaque;

    for (;;) {
        BaroLogRecord_t record;
        int status;

        (void)pthread_mutex_lock(&logger->mutex);

        while ((logger->count == 0U) &&
               !logger->stopping) {
            (void)pthread_cond_wait(
                &logger->ready,
                &logger->mutex);
        }

        if ((logger->count == 0U) &&
            logger->stopping) {
            (void)pthread_mutex_unlock(&logger->mutex);
            break;
        }

        record = logger->queue[logger->head];

        logger->head =
            (logger->head + 1U) %
            MS5837_LOGGER_CAPACITY;

        --logger->count;

        (void)pthread_mutex_unlock(&logger->mutex);

        status = write_record(logger->stream, &record);

        if (status < 0) {
            (void)pthread_mutex_lock(&logger->mutex);
            logger->write_failed = true;
            (void)pthread_mutex_unlock(&logger->mutex);
        }
    }

    return NULL;
}

int ms5837_logger_start_path(
    Ms5837Logger_t *logger,
    const char *path)
{
    int status;

    if ((logger == NULL) ||
        (path == NULL) ||
        (path[0] == '\0') ||
        (strlen(path) >= MS5837_LOGGER_FILENAME_SIZE)) {
        return -EINVAL;
    }

    memset(logger, 0, sizeof(*logger));

    (void)memcpy(
        logger->filename,
        path,
        strlen(path) + 1U);

    logger->stream = fopen(path, "wx");

    if (logger->stream == NULL) {
        return -errno;
    }

    if (write_header(logger->stream) < 0) {
        (void)fclose(logger->stream);
        logger->stream = NULL;
        return -EIO;
    }

    status = pthread_mutex_init(
        &logger->mutex,
        NULL);

    if (status != 0) {
        (void)fclose(logger->stream);
        logger->stream = NULL;
        return -status;
    }

    status = pthread_cond_init(
        &logger->ready,
        NULL);

    if (status != 0) {
        (void)pthread_mutex_destroy(&logger->mutex);
        (void)fclose(logger->stream);
        logger->stream = NULL;
        return -status;
    }

    status = pthread_create(
        &logger->thread,
        NULL,
        logger_thread,
        logger);

    if (status != 0) {
        (void)pthread_cond_destroy(&logger->ready);
        (void)pthread_mutex_destroy(&logger->mutex);
        (void)fclose(logger->stream);
        logger->stream = NULL;
        return -status;
    }

    logger->thread_started = true;
    return 0;
}

int ms5837_logger_start(Ms5837Logger_t *logger)
{
    struct timespec now;
    struct tm local;
    char path[MS5837_LOGGER_FILENAME_SIZE];

    if (logger == NULL) {
        return -EINVAL;
    }

    if ((clock_gettime(CLOCK_REALTIME, &now) < 0) ||
        (localtime_r(&now.tv_sec, &local) == NULL) ||
        (strftime(
             path,
             sizeof(path),
             "ms5_capture_%Y%m%d_%H%M%S.csv",
             &local) == 0U)) {
        return -EIO;
    }

    return ms5837_logger_start_path(logger, path);
}

int ms5837_logger_enqueue(
    Ms5837Logger_t *logger,
    const BaroLogRecord_t *record)
{
    size_t tail;
    int status;

    if ((logger == NULL) ||
        (record == NULL) ||
        !logger->thread_started) {
        return -EINVAL;
    }

    status = pthread_mutex_trylock(&logger->mutex);

    if (status != 0) {
        return -EAGAIN;
    }

    if (logger->stopping ||
        logger->write_failed ||
        (logger->count >= MS5837_LOGGER_CAPACITY)) {
        bool failed = logger->write_failed;

        (void)pthread_mutex_unlock(&logger->mutex);
        return failed ? -EIO : -EAGAIN;
    }

    tail =
        (logger->head + logger->count) %
        MS5837_LOGGER_CAPACITY;

    logger->queue[tail] = *record;
    ++logger->count;

    (void)pthread_cond_signal(&logger->ready);
    (void)pthread_mutex_unlock(&logger->mutex);

    return 0;
}

int ms5837_logger_stop(Ms5837Logger_t *logger)
{
    int result = 0;

    if (logger == NULL) {
        return -EINVAL;
    }

    if (logger->thread_started) {
        (void)pthread_mutex_lock(&logger->mutex);
        logger->stopping = true;
        (void)pthread_cond_signal(&logger->ready);
        (void)pthread_mutex_unlock(&logger->mutex);

        if (pthread_join(logger->thread, NULL) != 0) {
            result = -EIO;
        }

        logger->thread_started = false;

        if (logger->write_failed) {
            result = -EIO;
        }

        (void)pthread_cond_destroy(&logger->ready);
        (void)pthread_mutex_destroy(&logger->mutex);
    }

    if (logger->stream != NULL) {
        if ((fflush(logger->stream) != 0) ||
            (fclose(logger->stream) != 0)) {
            result = -EIO;
        }

        logger->stream = NULL;
    }

    return result;
}

const char *ms5837_logger_filename(
    const Ms5837Logger_t *logger)
{
    return logger != NULL
        ? logger->filename
        : NULL;
}
