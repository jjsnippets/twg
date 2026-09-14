#define _POSIX_C_SOURCE 200809L

#include "ms5837_logger.h"

#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include <time.h>

static int write_header(FILE *stream)
{
    return fprintf(stream,
        "publication_sequence,publication_time_ns,phase,sample_ready,"
        "sample_stale,fluid_density_kg_m3,surface_pressure_mbar,"
        "surface_pressure_valid,contract_version,status_flags,"
        "measurement_sequence,measurement_complete_time_ns,"
        "raw_pressure_d1,raw_temperature_d2,pressure_mbar,"
        "temperature_c,depth_m\n") < 0 ? -EIO : 0;
}

static int write_record(FILE *stream, const BaroLogRecord_t *record)
{
    const char *phase = (record->phase == BARO_CAPTURE_PHASE_ZERO)
        ? "zero" : "run";

    return fprintf(stream,
        "%" PRIu64 ",%" PRIu64 ",%s,%u,%u,%.6f,%.6f,%u,"
        "%" PRIu32 ",%" PRIu32 ",%" PRIu64 ",%" PRIu64 ","
        "%" PRIu32 ",%" PRIu32 ",%.6f,%.6f,%.6f\n",
        record->publication_sequence,
        record->publication_time_ns,
        phase,
        record->sample_ready ? 1U : 0U,
        record->sample_stale ? 1U : 0U,
        record->config.fluid_density_kg_m3,
        record->config.surface_pressure_mbar,
        record->config.surface_pressure_valid ? 1U : 0U,
        record->sample.contract_version,
        record->sample.status_flags,
        record->sample.measurement_sequence,
        record->sample.measurement_complete_time_ns,
        record->sample.raw_pressure_d1,
        record->sample.raw_temperature_d2,
        record->sample.pressure_mbar,
        record->sample.temperature_c,
        record->sample.depth_m) < 0 ? -EIO : 0;
}

static void *logger_thread(void *opaque)
{
    Ms5837Logger_t *logger = (Ms5837Logger_t *)opaque;

    for (;;) {
        BaroLogRecord_t record;
        int status;

        (void)pthread_mutex_lock(&logger->mutex);
        while ((logger->count == 0U) && !logger->stopping) {
            (void)pthread_cond_wait(&logger->ready, &logger->mutex);
        }
        if ((logger->count == 0U) && logger->stopping) {
            (void)pthread_mutex_unlock(&logger->mutex);
            break;
        }
        record = logger->queue[logger->head];
        logger->head = (logger->head + 1U) % MS5837_LOGGER_CAPACITY;
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

int ms5837_logger_start_path(Ms5837Logger_t *logger, const char *path)
{
    int status;

    if ((logger == NULL) || (path == NULL) || (path[0] == '\0') ||
        (strlen(path) >= MS5837_LOGGER_FILENAME_SIZE)) {
        return -EINVAL;
    }

    memset(logger, 0, sizeof(*logger));
    (void)memcpy(logger->filename, path, strlen(path) + 1U);
    logger->stream = fopen(path, "wx");
    if (logger->stream == NULL) {
        return -errno;
    }
    if (write_header(logger->stream) < 0) {
        (void)fclose(logger->stream);
        logger->stream = NULL;
        return -EIO;
    }

    status = pthread_mutex_init(&logger->mutex, NULL);
    if (status != 0) {
        (void)fclose(logger->stream);
        logger->stream = NULL;
        return -status;
    }
    status = pthread_cond_init(&logger->ready, NULL);
    if (status != 0) {
        (void)pthread_mutex_destroy(&logger->mutex);
        (void)fclose(logger->stream);
        logger->stream = NULL;
        return -status;
    }
    status = pthread_create(&logger->thread, NULL, logger_thread, logger);
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
        (strftime(path, sizeof(path),
                  "ms5_capture_%Y%m%d_%H%M%S.csv", &local) == 0U)) {
        return -EIO;
    }
    return ms5837_logger_start_path(logger, path);
}

int ms5837_logger_enqueue(Ms5837Logger_t *logger,
                          const BaroLogRecord_t *record)
{
    size_t tail;
    int status;

    if ((logger == NULL) || (record == NULL) || !logger->thread_started) {
        return -EINVAL;
    }
    status = pthread_mutex_trylock(&logger->mutex);
    if (status != 0) {
        return -EAGAIN;
    }
    if (logger->stopping || logger->write_failed ||
        (logger->count >= MS5837_LOGGER_CAPACITY)) {
        (void)pthread_mutex_unlock(&logger->mutex);
        return logger->write_failed ? -EIO : -EAGAIN;
    }

    tail = (logger->head + logger->count) % MS5837_LOGGER_CAPACITY;
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

const char *ms5837_logger_filename(const Ms5837Logger_t *logger)
{
    return (logger != NULL) ? logger->filename : NULL;
}
