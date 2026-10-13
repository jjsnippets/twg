#ifndef TWG_LOG_H
#define TWG_LOG_H

#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app/imu_publish.h"
#include "ms5837_contract.h"

#define TWG_LOG_SCHEMA_VERSION    1u
#define TWG_LOG_QUEUE_CAPACITY    2048u
#define TWG_LOG_FILENAME_CAPACITY 64u

typedef enum {
    TWG_LOG_PHASE_PRE = 0,
    TWG_LOG_PHASE_ZERO,
    TWG_LOG_PHASE_RUN,
    TWG_LOG_PHASE_CALIBRATION,
    TWG_LOG_PHASE_TARE_Z,
    TWG_LOG_PHASE_TARE_FULL,
    TWG_LOG_PHASE_CLEAR,
    TWG_LOG_PHASE_RESTORE
} TwgLogPhase;

typedef struct {
    uint32_t schemaVersion;
    uint64_t rowSequence;

    uint64_t scheduledNs;
    uint64_t actualNs;
    uint32_t gatesSkippedBefore;

    uint8_t scheduledValid;
    uint8_t actualValid;
    uint8_t deadlineMissed;

    TwgLogPhase phase;
    ImuPublicationRecord_t imu;

    uint8_t pressureReady;
    uint8_t pressureFresh;
    uint8_t pressureStale;
    uint8_t pressureAgeValid;

    uint64_t pressureAgeNs;
    int32_t pressureDriverState;
    BaroSample_t pressure;

    uint8_t zeroRequested;
    uint8_t zeroValid;
    uint64_t zeroValidSamples;
    uint64_t zeroRejectedSamples;

    double surfacePressureMbar;
    double fluidDensityKgM3;
    double depthM;

    uint64_t loggerDropsBefore;
} TwgLogRecord;

typedef struct {
    FILE *stream;
    pthread_t worker;

    TwgLogRecord ring[TWG_LOG_QUEUE_CAPACITY];

    _Atomic uint64_t head;
    _Atomic uint64_t tail;
    _Atomic uint64_t enqueued;
    _Atomic uint64_t written;
    _Atomic uint64_t dropped;
    _Atomic uint32_t highWater;

    _Atomic bool stopRequested;
    _Atomic bool workerDone;
    _Atomic bool writeFailed;

    char filename[TWG_LOG_FILENAME_CAPACITY];

    bool initialized;
    bool workerStarted;
} TwgLog;

bool twg_log_init(TwgLog *log);
bool twg_log_start(TwgLog *log);

bool twg_log_enqueue(
    TwgLog *log,
    const TwgLogRecord *record);

bool twg_log_stop(TwgLog *log);

bool twg_log_write_failed(const TwgLog *log);
const char *twg_log_filename(const TwgLog *log);

uint64_t twg_log_enqueued(const TwgLog *log);
uint64_t twg_log_written(const TwgLog *log);
uint64_t twg_log_dropped(const TwgLog *log);
uint32_t twg_log_high_water(const TwgLog *log);

#endif /* TWG_LOG_H */
