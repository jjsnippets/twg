#define _POSIX_C_SOURCE 200809L

/*
 * sensor_calibrate.c
 *
 * Owns the SH-2 session for the calibration tool and decodes four
 * BNO085 reports into one latest CalSample_t:
 *   - magnetic field calibrated at 50 Hz (the rate CEVA's calibration
 *     procedure requires for magnetometer work) — values + accuracy;
 *   - accelerometer, gyroscope calibrated, rotation vector at 10 Hz —
 *     status/accuracy bits only, as the per-phase progress signal.
 *
 * Mirrors app/sensor_reader.c (same sh2_hal_rpi_init / sh2_open /
 * sh2_setSensorCallback / sh2_setSensorConfig sequence) but does NOT
 * own calibration policy: sh2_setCalConfig / sh2_saveDcdNow are called
 * by cal_main.c, which owns the operator flow.
 *
 * Part of: rpi4b prod code/bno/calibration
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "sh2.h"
#include "sh2_err.h"
#include "sh2_hal.h"
#include "sh2_SensorValue.h"

#include "cal_sample.h"
#include "sensor_calibrate.h"

/* Provided by app/sh2_hal_rpi.c (same HAL as the app and the
 * validation binaries; see the calibration Makefile). */
extern sh2_Hal_t *sh2_hal_rpi_init(void);

/* Report rates. The magnetic field rate is fixed at 50 Hz because
 * CEVA's BNO085 calibration procedure requires that rate for proper
 * magnetometer calibration. The other three are slow: their per-report
 * status bits are all the flow controller needs. */
#define MAG_RATE_HZ       50U
#define SLOW_RATE_HZ      10U
#define MAG_INTERVAL_US   (1000000U / MAG_RATE_HZ)
#define SLOW_INTERVAL_US  (1000000U / SLOW_RATE_HZ)

static sh2_Hal_t *sHal = NULL;
static CalSample_t sLatestSample;
static bool sHasSample = false;

static uint64_t hostTimeUsec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000000ULL) +
           ((uint64_t)ts.tv_nsec / 1000ULL);
}

static void asyncEventCallback(void *cookie, sh2_AsyncEvent_t *pEvent)
{
    (void)cookie;
    (void)pEvent;
}

static void sensorCallback(void *cookie, sh2_SensorEvent_t *pEvent)
{
    (void)cookie;

    sh2_SensorValue_t value;
    memset(&value, 0, sizeof(value));

    if (sh2_decodeSensorEvent(&value, pEvent) != SH2_OK) {
        return;
    }

    sLatestSample.tHost_uS = hostTimeUsec();
    sLatestSample.tDevice_uS = pEvent->timestamp_uS;
    sLatestSample.seq++;

    switch (value.sensorId) {
        case SH2_ACCELEROMETER:
            /* accuracy bits live in the report status byte */
            sLatestSample.accelAccuracy = (uint8_t)value.status;
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            sLatestSample.gyroAccuracy = (uint8_t)value.status;
            break;

        case SH2_MAGNETIC_FIELD_CALIBRATED:
            sLatestSample.magAccuracy = (uint8_t)value.status;
            sLatestSample.magX_uT = value.un.magneticField.x;
            sLatestSample.magY_uT = value.un.magneticField.y;
            sLatestSample.magZ_uT = value.un.magneticField.z;
            sLatestSample.haveMag = true;
            break;

        case SH2_ROTATION_VECTOR:
            /*
             * Status bits from the report header, like every other
             * report. (The payload's 'accuracy' member is an error
             * estimate in RADIANS, not the 0-3 scale — it is kept
             * separately as rvErrRad. Reading the payload field as a
             * 0-3 accuracy inverted its meaning: a large value means
             * a LARGE heading error.)
             */
            sLatestSample.rvAccuracy = (uint8_t)value.status;
            sLatestSample.rvErrRad = value.un.rotationVector.accuracy;
            break;

        default:
            break;
    }

    sHasSample = true;
}

static bool configureSensor(sh2_SensorId_t sensorId, uint32_t interval_us)
{
    sh2_SensorConfig_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    cfg.reportInterval_us = interval_us;
    cfg.batchInterval_us = 0;

    return sh2_setSensorConfig(sensorId, &cfg) == SH2_OK;
}

bool sensor_calibrate_start(void)
{
    memset(&sLatestSample, 0, sizeof(sLatestSample));
    sLatestSample.version = CAL_SAMPLE_STRUCT_VERSION;
    sHasSample = false;

    sHal = sh2_hal_rpi_init();
    if (sHal == NULL) {
        return false;
    }

    if (sh2_open(sHal, asyncEventCallback, NULL) != SH2_OK) {
        sHal = NULL;
        return false;
    }

    if (sh2_setSensorCallback(sensorCallback, NULL) != SH2_OK) {
        sh2_close();
        sHal = NULL;
        return false;
    }

    if (!configureSensor(SH2_MAGNETIC_FIELD_CALIBRATED, MAG_INTERVAL_US) ||
        !configureSensor(SH2_ACCELEROMETER, SLOW_INTERVAL_US) ||
        !configureSensor(SH2_GYROSCOPE_CALIBRATED, SLOW_INTERVAL_US) ||
        !configureSensor(SH2_ROTATION_VECTOR, SLOW_INTERVAL_US)) {
        sh2_close();
        sHal = NULL;
        return false;
    }

    return true;
}

void sensor_calibrate_service(void)
{
    sh2_service();
}

void sensor_calibrate_stop(void)
{
    sh2_close();
    sHal = NULL;
}

bool sensor_calibrate_getLatestSample(CalSample_t *outSample)
{
    if (!outSample || !sHasSample) {
        return false;
    }
    *outSample = sLatestSample;
    return true;
}
