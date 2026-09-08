#define _POSIX_C_SOURCE 200809L

/*
 * app_sensor.c — BNO085 SH-2 session owner and report decoder.
 *
 * Implements the sensor interface defined in app_sensor.h.
 *
 * Sensor configuration:
 *   - Rotation Vector (fused orientation):        100 Hz (10,000 us)
 *   - Linear Acceleration (accel minus gravity):  100 Hz (10,000 us)
 *   - Calibrated Gyroscope (bias-corrected):       100 Hz (10,000 us)
 *   Combined rate: ~300 events/sec.
 *
 * Dynamic calibration policy:
 *   Disabled (mask 0x00). The sensor uses only the calibration saved in
 *   flash (DCD, FRS record 0x1F1F) from the offline calibration procedure.
 *   Runtime auto-calibration is off to prevent heading shifts during
 *   data-collection runs.
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

#include "app/app_contract.h"
#include "app/app_sensor.h"

/* Provided by sh2_hal_rpi.c (compiled into the binary). */
extern sh2_Hal_t *sh2_hal_rpi_init(void);

#define SENSOR_RATE_HZ     100U
#define SENSOR_INTERVAL_US (1000000U / SENSOR_RATE_HZ)

static sh2_Hal_t *sHal = NULL;
static ImuSample_t sLatestSample;
static bool sHasSample = false;
static uint32_t sSeqCounter = 0;

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

    sLatestSample.tHost_uS   = hostTimeUsec();
    sLatestSample.tDevice_uS = pEvent->timestamp_uS;
    sLatestSample.status     = value.status;
    sLatestSample.report_seq = value.sequence;
    sLatestSample.seq        = ++sSeqCounter;

    switch (value.sensorId) {
        case SH2_ROTATION_VECTOR: {
            const sh2_RotationVectorWAcc_t *rv = &value.un.rotationVector;
            sLatestSample.yaw   = q_to_yaw(rv->real, rv->i, rv->j, rv->k);
            sLatestSample.pitch = q_to_pitch(rv->real, rv->i, rv->j, rv->k);
            sLatestSample.roll  = q_to_roll(rv->real, rv->i, rv->j, rv->k);
            sLatestSample.validMask |= IMU_SAMPLE_VALID_RV;
            break;
        }

        case SH2_LINEAR_ACCELERATION: {
            const sh2_Accelerometer_t *la = &value.un.linearAcceleration;
            sLatestSample.ax = la->x;
            sLatestSample.ay = la->y;
            sLatestSample.az = la->z;
            sLatestSample.validMask |= IMU_SAMPLE_VALID_ACCEL;
            break;
        }

        case SH2_GYROSCOPE_CALIBRATED: {
            const sh2_Gyroscope_t *g = &value.un.gyroscope;
            sLatestSample.gx = g->x;
            sLatestSample.gy = g->y;
            sLatestSample.gz = g->z;
            sLatestSample.validMask |= IMU_SAMPLE_VALID_GYRO;
            break;
        }

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
    cfg.batchInterval_us  = 0;

    return sh2_setSensorConfig(sensorId, &cfg) == SH2_OK;
}

bool sensor_reader_start(void)
{
    memset(&sLatestSample, 0, sizeof(sLatestSample));
    sLatestSample.version = IMU_SAMPLE_STRUCT_VERSION;
    sHasSample   = false;
    sSeqCounter  = 0;

    sHal = sh2_hal_rpi_init();
    if (sHal == NULL) {
        return false;
    }

    if (sh2_open(sHal, asyncEventCallback, NULL) != SH2_OK) {
        sHal = NULL;
        return false;
    }

    if (sh2_setCalConfig(0) != SH2_OK) {
        sh2_close();
        sHal = NULL;
        return false;
    }

    if (sh2_setSensorCallback(sensorCallback, NULL) != SH2_OK) {
        sh2_close();
        sHal = NULL;
        return false;
    }

    if (!configureSensor(SH2_ROTATION_VECTOR, SENSOR_INTERVAL_US) ||
        !configureSensor(SH2_LINEAR_ACCELERATION, SENSOR_INTERVAL_US) ||
        !configureSensor(SH2_GYROSCOPE_CALIBRATED, SENSOR_INTERVAL_US)) {
        sh2_close();
        sHal = NULL;
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
    sLatestSample.seq = 0;
}
