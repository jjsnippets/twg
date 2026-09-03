#define _POSIX_C_SOURCE 200112L
/*
 * test_single_sensor.c
 *
 * Stage 3 minimal test harness: brings up sh2_lib on top of the
 * validated sh2_hal_rpi.c, configures a SINGLE sensor (rotation
 * vector) at a specified rate (e.g., 100 Hz), and logs decoded sensor
 * values via sh2_decodeSensorEvent().
 *
 * Goal: confirm end-to-end data flow for one sensor type:
 *   BNO085 -> SHTP over SPI -> HAL -> sh2_lib -> sensorCallback ->
 *   sh2_decodeSensorEvent() -> application-level quaternion.
 *
 * This is still diagnostic code, not the final application.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "sh2.h"
#include "sh2_hal.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"
#include "euler.h"

extern sh2_Hal_t *sh2_hal_rpi_init(void);

#define TEST_DURATION_SEC    10
#define SENSOR_RATE_HZ       100U
#define SENSOR_INTERVAL_US   (1000000U / SENSOR_RATE_HZ)

static int  sAsyncEventCount = 0;
static int  sSensorEventCount = 0;
static bool sResetSeen = false;

/* ------------------------------------------------------------------ */
/* Async event callback                                                */
/* ------------------------------------------------------------------ */

static void asyncEventCallback(void *cookie, sh2_AsyncEvent_t *pEvent)
{
    (void)cookie;
    sAsyncEventCount++;

    printf("[async %d] eventId=%d\n", sAsyncEventCount, pEvent->eventId);
    if (pEvent->eventId == SH2_RESET) {
        sResetSeen = true;
        printf("    -> SH2_RESET (device ready)\n");
    }
}

/* ------------------------------------------------------------------ */
/* Sensor callback: single sensor (Rotation Vector)                    */
/* ------------------------------------------------------------------ */

static void sensorCallback(void *cookie, sh2_SensorEvent_t *pEvent)
{
    (void)cookie;
    sSensorEventCount++;

    /* Decode into sh2_SensorValue_t, then pull out rotationVector. */
    sh2_SensorValue_t value;
    memset(&value, 0, sizeof(value));

    int rc = sh2_decodeSensorEvent(&value, pEvent);
    if (rc != SH2_OK) {
        fprintf(stderr, "WARN: sh2_decodeSensorEvent() returned %d\n", rc);
        return;
    }

    if (value.sensorId != SH2_ROTATION_VECTOR) {
        /* In this Stage 3 test, we expect only rotation vector events. */
        printf("[sensor %d] unexpected sensorId=%d (expected SH2_ROTATION_VECTOR=%d)\n",
               sSensorEventCount, value.sensorId, SH2_ROTATION_VECTOR);
        return;
    }

    /* Extract quaternion components (RV with accuracy) */
    const sh2_RotationVectorWAcc_t *rv = &value.un.rotationVector;

    /* Convert quaternion to yaw/pitch/roll via euler.c helpers. */
    double yaw = q_to_yaw(rv->real, rv->i, rv->j, rv->k);
    double pitch = q_to_pitch(rv->real, rv->i, rv->j, rv->k);
    double roll = q_to_roll(rv->real, rv->i, rv->j, rv->k);

    printf("[sensor %d] t_us=%u RV: w=%.6f x=%.6f y=%.6f z=%.6f acc=%.6f rad | ",
           sSensorEventCount,
           pEvent->timestamp_uS,
           rv->real, rv->i, rv->j, rv->k, rv->accuracy);
    printf("yaw=%.6f pitch=%.6f roll=%.6f (radians)\n", yaw, pitch, roll);
}

int main(void)
{
    printf("== Stage 3: single sensor (Rotation Vector @ %u Hz) ==\n", SENSOR_RATE_HZ);

    sh2_Hal_t *hal = sh2_hal_rpi_init();
    if (!hal) {
        fprintf(stderr, "FAIL: sh2_hal_rpi_init() returned NULL\n");
        return 1;
    }

    int rc = sh2_open(hal, asyncEventCallback, NULL);
    if (rc != SH2_OK) {
        fprintf(stderr, "FAIL: sh2_open() returned %d\n", rc);
        return 1;
    }
    printf("PASS: sh2_open() returned SH2_OK\n");

    rc = sh2_setSensorCallback(sensorCallback, NULL);
    if (rc != SH2_OK) {
        fprintf(stderr, "FAIL: sh2_setSensorCallback() returned %d\n", rc);
        sh2_close();
        return 1;
    }

    /* Configure Rotation Vector at SENSOR_RATE_HZ. */
    sh2_SensorConfig_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    // cfg.enable = 1;
    cfg.changeSensitivityEnabled = 0;
    cfg.changeSensitivity = 0;
    cfg.reportInterval_us = SENSOR_INTERVAL_US;
    cfg.batchInterval_us = 0;       /* no batching */
    cfg.sensorSpecific = 0;         /* default */

    rc = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &cfg);
    if (rc != SH2_OK) {
        fprintf(stderr, "FAIL: sh2_setSensorConfig(SH2_ROTATION_VECTOR) returned %d\n", rc);
        sh2_close();
        return 1;
    }
    printf("PASS: enabled SH2_ROTATION_VECTOR at %u Hz (interval %u us)\n",
           SENSOR_RATE_HZ, SENSOR_INTERVAL_US);

    printf("\n-- Running sh2_service() loop for %d seconds --\n", TEST_DURATION_SEC);

    time_t startTime = time(NULL);
    while (time(NULL) - startTime < TEST_DURATION_SEC) {
        sh2_service();
        // usleep(1000); /* 1 ms between service calls */
    }

    printf("\n-- Loop complete --\n");
    printf("Total async events: %d, sensor events: %d\n",
           sAsyncEventCount, sSensorEventCount);

    if (!sResetSeen) {
        fprintf(stderr, "FAIL: never observed SH2_RESET async event in Stage 3\n");
    }
    if (sSensorEventCount == 0) {
        fprintf(stderr,
                "FAIL: no rotation vector sensor events observed -- "
                "check WAKE/INT timing or sensor configuration.\n");
    } else {
        printf("PASS: observed rotation vector events at ~%u Hz (subject to host scheduling)\n",
               SENSOR_RATE_HZ);
    }

    sh2_close();
    printf("PASS: sh2_close() completed\n");

    printf("\n== Stage 3 test complete ==\n");
    return (sResetSeen && sSensorEventCount > 0) ? 0 : 1;
}
