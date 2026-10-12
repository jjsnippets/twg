#ifndef MS5837_LOGGER_H
#define MS5837_LOGGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <pthread.h>

#include "ms5837_publish.h"

#define MS5837_LOGGER_CAPACITY 2048U
#define MS5837_LOGGER_FILENAME_SIZE 40U

#define MS5837_CSV_SCHEMA_VERSION 2U
#define MS5837_CSV_COLUMN_COUNT 32U
#define MS5837_CSV_MISSING_TOKEN "NaN"

typedef enum {
    BARO_CAPTURE_PHASE_ZERO = 0,
    BARO_CAPTURE_PHASE_RUN = 1
} BaroCapturePhase_t;

typedef struct {
    uint32_t schema_version;
    BaroCapturePhase_t phase;
    Ms5837PublicationRecord_t publication;

    bool zero_requested;
    bool zero_valid;
    uint64_t zero_valid_samples;
    uint64_t zero_rejected_samples;

    double surface_pressure_mbar;
    double fluid_density_kg_m3;
    double depth_m;

    uint64_t logger_drops_before;
} BaroLogRecord_t;

typedef struct {
    FILE *stream;
    pthread_t thread;
    pthread_mutex_t mutex;
    pthread_cond_t ready;

    BaroLogRecord_t queue[MS5837_LOGGER_CAPACITY];
    size_t head;
    size_t count;

    bool stopping;
    bool thread_started;
    bool write_failed;

    char filename[MS5837_LOGGER_FILENAME_SIZE];
} Ms5837Logger_t;

int ms5837_logger_start(Ms5837Logger_t *logger);

int ms5837_logger_start_path(
    Ms5837Logger_t *logger,
    const char *path);

int ms5837_logger_enqueue(
    Ms5837Logger_t *logger,
    const BaroLogRecord_t *record);

int ms5837_logger_stop(Ms5837Logger_t *logger);

const char *ms5837_logger_filename(
    const Ms5837Logger_t *logger);

#endif /* MS5837_LOGGER_H */
