/*
 * test_single_sensor.c
 *
 * Verifies that a single sensor report can be configured and delivered
 * cleanly before enabling multiple sensors simultaneously.
 *
 * Diagnostic program -- not part of production code.
 */

#define _POSIX_C_SOURCE 200112L

#include <inttypes.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "sh2.h"
#include "sh2_hal.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"
#include "euler.h"

extern sh2_Hal_t *sh2_hal_rpi_init(void);

static volatile sig_atomic_t sRunning = 1;
static int sSensorEventCount = 0;

static void onSigint(int sig)
{
    (void)sig;
    sRunning = 0;
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

    int rc = sh2_decodeSensorEvent(&value, pEvent);
    if (rc != SH2_OK) {
        fprintf(stderr, "WARN: sh2_decodeSensorEvent returned %d\n", rc);
        return;
    }

    sSensorEventCount++;

    printf("[sensor %d] t_us=%" PRIu64 " RV: w=%.6f x=%.6f y=%.6f z=%.6f acc=%.6f rad | ",
           sSensorEventCount,
           (uint64_t)pEvent->timestamp_uS,
           (double)value.un.rotationVector.real,
           (double)value.un.rotationVector.i,
           (double)value.un.rotationVector.j,
           (double)value.un.rotationVector.k,
           (double)value.un.rotationVector.accuracy);

    float yaw   = q_to_yaw(value.un.rotationVector.real,
                           value.un.rotationVector.i,
                           value.un.rotationVector.j,
                           value.un.rotationVector.k);
    float pitch = q_to_pitch(value.un.rotationVector.real,
                             value.un.rotationVector.i,
                             value.un.rotationVector.j,
                             value.un.rotationVector.k);
    float roll  = q_to_roll(value.un.rotationVector.real,
                            value.un.rotationVector.i,
                            value.un.rotationVector.j,
                            value.un.rotationVector.k);

    const float RAD2DEG = 57.29577951308232f;
    printf("Euler (deg): yaw=%7.2f pitch=%7.2f roll=%7.2f\n",
           (double)(yaw * RAD2DEG),
           (double)(pitch * RAD2DEG),
           (double)(roll * RAD2DEG));
}

int main(int argc, char **argv)
{
    int reportPeriodUs = 10000; /* default 10 ms = 100 Hz */
    if (argc > 1) {
        reportPeriodUs = atoi(argv[1]);
        if (reportPeriodUs <= 0) {
            fprintf(stderr, "Usage: %s [report_period_us]\n", argv[0]);
            return 1;
        }
    }

    signal(SIGINT, onSigint);
    signal(SIGTERM, onSigint);

    printf("Single Sensor Diagnostic (Rotation Vector)\n");
    printf("Report period: %d us (%.1f Hz)\n",
           reportPeriodUs, 1000000.0 / (double)reportPeriodUs);

    sh2_Hal_t *hal = sh2_hal_rpi_init();
    if (hal == NULL) {
        fprintf(stderr, "FAIL: sh2_hal_rpi_init returned NULL\n");
        return 1;
    }

    int rc = sh2_open(hal, asyncEventCallback, NULL);
    if (rc != SH2_OK) {
        fprintf(stderr, "FAIL: sh2_open returned %d\n", rc);
        return 1;
    }

    rc = sh2_setSensorCallback(sensorCallback, NULL);
    if (rc != SH2_OK) {
        fprintf(stderr, "FAIL: sh2_setSensorCallback returned %d\n", rc);
        sh2_close();
        return 1;
    }

    sh2_SensorConfig_t config;
    memset(&config, 0, sizeof(config));
    config.changeSensitivityEnabled = false;
    config.changeSensitivityRelative = false;
    config.wakeupEnabled = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.reportInterval_us = (uint32_t)reportPeriodUs;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;

    rc = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config);
    if (rc != SH2_OK) {
        fprintf(stderr, "FAIL: sh2_setSensorConfig returned %d\n", rc);
        sh2_close();
        return 1;
    }

    printf("Listening for Rotation Vector reports. Press Ctrl-C to stop.\n");

    while (sRunning) {
        sh2_service();
        usleep(500);
    }

    printf("\nStopping sensor...\n");
    memset(&config, 0, sizeof(config));
    sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config);
    sh2_close();
    printf("Diagnostic finished. Received %d events.\n", sSensorEventCount);
    return 0;
}
