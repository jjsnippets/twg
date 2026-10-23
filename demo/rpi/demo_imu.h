#ifndef DEMO_IMU_H
#define DEMO_IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DEMO_PHASE_STARTING = 0,
    DEMO_PHASE_TRACKING = 1,
    DEMO_PHASE_TARE_SETTLE = 2,
    DEMO_PHASE_TARE_POST = 3,
    DEMO_PHASE_RESTORING = 4,
    DEMO_PHASE_FAULT = 5
} DemoPhase;

typedef enum {
    DEMO_ERROR_NONE = 0,
    DEMO_ERROR_STARTUP = 1,
    DEMO_ERROR_SESSION = 2,
    DEMO_ERROR_TARE_CONFIG = 3,
    DEMO_ERROR_TARE_COMMAND = 4,
    DEMO_ERROR_TARE_DATA = 5,
    DEMO_ERROR_RESTORE = 6,
    DEMO_ERROR_STARTUP_DATA = 7,
    DEMO_ERROR_SENSOR_RESET = 8
} DemoError;

/*
 * tare_result:
 *   0: no tare requested
 *   1: tare in progress
 *   2: tare completed
 *  -1: tare failed
 *
 * reference changes only after acquisition has resumed with fresh data.
 */
typedef struct {
    uint64_t serial;
    uint64_t sequence;
    uint64_t sample_ns;

    uint32_t reference;
    uint32_t epoch;
    uint32_t tare_id;

    DemoPhase phase;
    int tare_result;
    DemoError error;

    unsigned accuracy;
    bool valid;
    float q[4];                 /* w, x, y, z */
} DemoImuSnapshot;

/* Application-owned instance; all IMU functions run on the owner thread. */
typedef struct {
    DemoImuSnapshot output;
    uint64_t minimum_ns;
    uint64_t deadline_ns;
    bool reference_dirty;
    bool opened;
} DemoImu;

uint64_t demo_now_ns(void);

bool demo_imu_open(DemoImu *imu);
void demo_imu_service(DemoImu *imu);
bool demo_imu_request_tare(DemoImu *imu, uint32_t request_id);
void demo_imu_snapshot(const DemoImu *imu, DemoImuSnapshot *out);
void demo_imu_close(DemoImu *imu);

#endif /* DEMO_IMU_H */
