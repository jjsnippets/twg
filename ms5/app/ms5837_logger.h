#ifndef MS5837_LOGGER_H
#define MS5837_LOGGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <pthread.h>

#include "app_contract.h"

#define MS5837_LOGGER_CAPACITY 2048U
#define MS5837_LOGGER_FILENAME_SIZE 40U

typedef enum {
    BARO_CAPTURE_PHASE_ZERO = 0,
    BARO_CAPTURE_PHASE_RUN = 1
} BaroCapturePhase_t;

typedef struct {
    uint64_t publication_sequence;
    uint64_t publication_time_ns;
    BaroCapturePhase_t phase;
    bool sample_ready;
    bool sample_stale;
    BaroRunConfig_t config;
    BaroSample_t sample;
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
int ms5837_logger_start_path(Ms5837Logger_t *logger, const char *path);
int ms5837_logger_enqueue(Ms5837Logger_t *logger,
                          const BaroLogRecord_t *record);
int ms5837_logger_stop(Ms5837Logger_t *logger);
const char *ms5837_logger_filename(const Ms5837Logger_t *logger);

#endif
