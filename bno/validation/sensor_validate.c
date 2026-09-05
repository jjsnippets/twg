/*
 * sensor_validate.c — SH-2 session owner for the validation binary.
 *
 * Mirrors app/sensor_reader.c (same sh2_hal_rpi_init/sh2_open/
 * sh2_setSensorCallback/sh2_setSensorConfig sequence, same three
 * reports at the same 100 Hz interval) but decodes into
 * ImuValidateSample_t, which keeps a separate host/device timestamp
 * pair and sequence number per sensor group.
 *
 * The sensor set is deliberately identical to the app so validation
 * evidence describes exactly what bno_app consumes:
 *   SH2_ROTATION_VECTOR, SH2_LINEAR_ACCELERATION, SH2_GYROSCOPE_CALIBRATED
 *
 * Thread model: single-threaded, like the app reader. sensorCallback
 * runs inside sh2_service() on the caller's thread.
 *
 * Build: make validate   (validation/Makefile)
 *        links ../sh2 sources, ../app/sh2_hal_rpi.c, this file
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "sh2.h"
#include "sh2_err.h"
#include "sh2_hal.h"
#include "sh2_SensorValue.h"
#include "euler.h"

#include "imu_validate.h"
#include "sensor_validate.h"

/* Defined in ../app/sh2_hal_rpi.c (the transport has no public header). */
extern sh2_Hal_t *sh2_hal_rpi_init(void);

#define SENSOR_RATE_HZ     100U
#define SENSOR_INTERVAL_US (1000000U / SENSOR_RATE_HZ)

static sh2_Hal_t *sHal = NULL;
static ImuValidateSample_t sLatest;
static bool sHasSample = false;
static uint32_t sSeqCounter = 0;

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

/* Meta events (fls ready, resets, ...): unused here, same as the app. */
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

    uint64_t host_ns = now_ns();
    uint32_t seq = sSeqCounter++;

    sLatest.version    = IMU_VALIDATE_STRUCT_VERSION;
    sLatest.seq        = seq;
    sLatest.host_ts_ns = host_ns;

    switch (value.sensorId) {
    case SH2_ROTATION_VECTOR: {
        const sh2_RotationVectorWAcc_t *rv = &value.un.rotationVector;
        sLatest.rv.host_ts_ns   = host_ns;
        sLatest.rv.sensor_ts_us = pEvent->timestamp_uS;
        sLatest.rv.seq          = seq;
        sLatest.rv.report_seq   = value.sequence;
        sLatest.rv.status       = value.status;
        sLatest.yaw        = q_to_yaw(rv->real, rv->i, rv->j, rv->k);
        sLatest.pitch      = q_to_pitch(rv->real, rv->i, rv->j, rv->k);
        sLatest.roll       = q_to_roll(rv->real, rv->i, rv->j, rv->k);
        sLatest.rvAccuracy = rv->accuracy;
        sLatest.validMask |= IMU_VALIDATE_VALID_RV;
        sHasSample = true;
        break;
    }

    case SH2_LINEAR_ACCELERATION: {
        /* linearAcceleration reuses sh2_Accelerometer_t in this sh2 version. */
        const sh2_Accelerometer_t *la = &value.un.linearAcceleration;
        sLatest.accel.host_ts_ns   = host_ns;
        sLatest.accel.sensor_ts_us = pEvent->timestamp_uS;
        sLatest.accel.seq          = seq;
        sLatest.accel.report_seq   = value.sequence;
        sLatest.accel.status       = value.status;
        sLatest.ax = la->x;
        sLatest.ay = la->y;
        sLatest.az = la->z;
        sLatest.validMask |= IMU_VALIDATE_VALID_ACCEL;
        sHasSample = true;
        break;
    }

    case SH2_GYROSCOPE_CALIBRATED: {
        const sh2_Gyroscope_t *gy = &value.un.gyroscope;
        sLatest.gyro.host_ts_ns   = host_ns;
        sLatest.gyro.sensor_ts_us = pEvent->timestamp_uS;
        sLatest.gyro.seq          = seq;
        sLatest.gyro.report_seq   = value.sequence;
        sLatest.gyro.status       = value.status;
        sLatest.gx = gy->x;
        sLatest.gy = gy->y;
        sLatest.gz = gy->z;
        sLatest.validMask |= IMU_VALIDATE_VALID_GYRO;
        sHasSample = true;
        break;
    }

    default:
        break;
    }
}

static bool configureSensor(sh2_SensorId_t sensorId)
{
    sh2_SensorConfig_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.reportInterval_us = SENSOR_INTERVAL_US;
    cfg.batchInterval_us = 0;
    return sh2_setSensorConfig(sensorId, &cfg) == SH2_OK;
}

bool sensor_validate_start(void)
{
    memset(&sLatest, 0, sizeof(sLatest));
    sLatest.version = IMU_VALIDATE_STRUCT_VERSION;
    sHasSample = false;
    sSeqCounter = 0;

    sHal = sh2_hal_rpi_init();
    if (!sHal) {
        return false;
    }

    if (sh2_open(sHal, asyncEventCallback, NULL) != SH2_OK) {
        return false;
    }

    if (sh2_setSensorCallback(sensorCallback, NULL) != SH2_OK) {
        sh2_close();
        return false;
    }

    if (!configureSensor(SH2_ROTATION_VECTOR) ||
        !configureSensor(SH2_LINEAR_ACCELERATION) ||
        !configureSensor(SH2_GYROSCOPE_CALIBRATED)) {
        sh2_close();
        return false;
    }

    /*
     * Flight-time calibration policy: mirror bno_app exactly (all
     * dynamic calibration off, fly on the saved DCD) so validation
     * evidence describes the config the acquisition binary runs.
     * The enable bits are RAM-only and revert to chip defaults at
     * every reset, so every sh2 consumer must set its own policy.
     */
    if (sh2_setCalConfig(0) != SH2_OK) {
        sh2_close();
        return false;
    }

    return true;
}

void sensor_validate_service(void)
{
    sh2_service();
}

void sensor_validate_stop(void)
{
    sh2_close();
    sHal = NULL;
    sHasSample = false;
}

bool sensor_validate_getLatestSample(ImuValidateSample_t *outSample)
{
    if (!outSample || !sHasSample) {
        return false;
    }
    *outSample = sLatest;
    return true;
}
