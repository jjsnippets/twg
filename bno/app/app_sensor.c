#define _POSIX_C_SOURCE 200809L

/*
 * sensor_reader.c
 *
 * Owns the SH-2 session and decodes three BNO085 reports into one latest
 * ImuSample_t:
 *   - rotation vector;
 *   - linear acceleration; and
 *   - calibrated gyroscope.
 *
 * The design is intentionally single-threaded. The application owns the
 * service cadence by calling sensor_reader_service() from its real-time loop.
 *
 * Note: bno_app runs with all dynamic calibration disabled
 * (sh2_setCalConfig(0) in main.c). Under that policy the BNO085 reports
 * the gyro status bit as 0 (unreliable) by design — the real-time ZRO
 * estimator is halted — while the saved DCD keeps bias-correcting the
 * gyro data. ImuSample_t deliberately carries no status bits; consumers
 * judge readiness from the rotation-vector outputs (orientationErrRad),
 * never from a gyro status bit.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "sh2.h"
#include "sh2_err.h"
#include "sh2_hal.h"
#include "sh2_SensorValue.h"
#include "euler.h"

#include "app/app_contract.h"
#include "app/app_sensor.h"

extern sh2_Hal_t *sh2_hal_rpi_init(void);

#define SENSOR_RATE_HZ       100U
#define SENSOR_INTERVAL_US   (1000000U / SENSOR_RATE_HZ)

static sh2_Hal_t *sHal = NULL;
static ImuSample_t sLatestSample;
static bool sHasSample = false;
static uint32_t sSeqCounter = 0;

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

    sLatestSample.version = IMU_SAMPLE_STRUCT_VERSION;
    sLatestSample.seq = sSeqCounter++;
    sLatestSample.timestamp_uS = pEvent->timestamp_uS;

    switch (value.sensorId) {
        case SH2_ROTATION_VECTOR: {
            const sh2_RotationVectorWAcc_t *rv = &value.un.rotationVector;
            sLatestSample.yaw   = (float)q_to_yaw(rv->real, rv->i, rv->j, rv->k);
            sLatestSample.pitch = (float)q_to_pitch(rv->real, rv->i, rv->j, rv->k);
            sLatestSample.roll  = (float)q_to_roll(rv->real, rv->i, rv->j, rv->k);
            sLatestSample.orientationErrRad = rv->accuracy;
            sLatestSample.validMask |= IMU_SAMPLE_VALID_ORIENTATION;
            break;
        }

        case SH2_LINEAR_ACCELERATION:
            sLatestSample.ax = value.un.linearAcceleration.x;
            sLatestSample.ay = value.un.linearAcceleration.y;
            sLatestSample.az = value.un.linearAcceleration.z;
            sLatestSample.validMask |= IMU_SAMPLE_VALID_ACCEL;
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            sLatestSample.gx = value.un.gyroscope.x;
            sLatestSample.gy = value.un.gyroscope.y;
            sLatestSample.gz = value.un.gyroscope.z;
            sLatestSample.validMask |= IMU_SAMPLE_VALID_GYRO;
            break;

        default:
            return;
    }

    sHasSample = true;
}

static bool configureSensor(sh2_SensorId_t sensorId)
{
    sh2_SensorConfig_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    cfg.reportInterval_us = SENSOR_INTERVAL_US;
    cfg.batchInterval_us = 0;

    return sh2_setSensorConfig(sensorId, &cfg) == SH2_OK;
}

bool sensor_reader_start(void)
{
    memset(&sLatestSample, 0, sizeof(sLatestSample));
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

    return true;
}

void sensor_reader_service(void)
{
    sh2_service();
}

void sensor_reader_stop(void)
{
    sh2_close();
    sHal = NULL;
}

bool sensor_reader_getLatestSample(ImuSample_t *outSample)
{
    if (!outSample || !sHasSample) {
        return false;
    }

    *outSample = sLatestSample;
    return true;
}

void sensor_reader_resetSeq(void)
{
    sSeqCounter = 0;
}
