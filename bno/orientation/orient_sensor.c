#define _POSIX_C_SOURCE 200809L

/*
 * orient_sensor.c
 *
 * Owns the SH-2 session for the orientation (tare) tool and decodes
 * one BNO085 report into one latest OrientSample_t:
 *   - rotation vector at 20 Hz — the full quaternion plus its
 *     payload accuracy, which is both the settle gate and the
 *     before/after heading evidence for the tare flow.
 *
 * Mirrors sensor_calibrate.c (same sh2_hal_rpi_init / sh2_open /
 * sh2_setSensorCallback / sh2_setSensorConfig sequence) but does NOT
 * own tare policy: sh2_setTareNow / sh2_persistTare / sh2_clearTare
 * are called by orient_main.c, which owns the operator flow.
 *
 * Part of: rpi4b prod code/bno/orientation
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "sh2.h"
#include "sh2_err.h"
#include "sh2_hal.h"
#include "sh2_SensorValue.h"

#include "orientation/orient_sensor.h"

/* Provided by app/sh2_hal_rpi.c (same HAL as the app and the
 * validation binaries; see the calibration Makefile). */
extern sh2_Hal_t *sh2_hal_rpi_init(void);

/* Report rate. 20 Hz is far more than the operator-facing heading
 * display needs and keeps the bus load trivial next to bno_app. */
#define RV_RATE_HZ       20U
#define RV_INTERVAL_US   (1000000U / RV_RATE_HZ)

static sh2_Hal_t *sHal = NULL;
static OrientSample_t sLatestSample;
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
        case SH2_ROTATION_VECTOR:
            /* the rotation vector carries its accuracy in the payload */
            sLatestSample.quatW = value.un.rotationVector.real;
            sLatestSample.quatX = value.un.rotationVector.i;
            sLatestSample.quatY = value.un.rotationVector.j;
            sLatestSample.quatZ = value.un.rotationVector.k;
            sLatestSample.rvAccuracy =
                (uint8_t)value.un.rotationVector.accuracy;
            sLatestSample.haveRv = true;
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

bool orient_sensor_start(void)
{
    memset(&sLatestSample, 0, sizeof(sLatestSample));
    sLatestSample.version = ORIENT_SAMPLE_STRUCT_VERSION;
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

    if (!configureSensor(SH2_ROTATION_VECTOR, RV_INTERVAL_US)) {
        sh2_close();
        sHal = NULL;
        return false;
    }

    return true;
}

void orient_sensor_service(void)
{
    sh2_service();
}

void orient_sensor_stop(void)
{
    sh2_close();
    sHal = NULL;
}

bool orient_sensor_getLatestSample(OrientSample_t *outSample)
{
    if (!outSample || !sHasSample) {
        return false;
    }
    *outSample = sLatestSample;
    return true;
}
