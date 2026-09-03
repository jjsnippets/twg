/*
 * test_min_period.c
 *
 * Diagnostic tool: queries each sensor's Metadata FRS record via
 * sh2_getMetadata() and prints the "Minimum period" field, which
 * defines the true maximum sustainable report rate for that sensor
 * as configured/calibrated on this specific BNO085 unit.
 *
 * Run this BEFORE any of the per-sensor timing tests (test_timing_rv,
 * test_timing_accel, test_timing_gyro) so that the 100/200/400Hz
 * targets used in those tests can be checked against each sensor's
 * real minimum period. If a requested reportInterval_us is set below
 * a sensor's minimum period, sh2_setSensorConfig() will silently
 * clamp the achieved rate to the sensor's real maximum rather than
 * failing -- this test exists to catch that ahead of time.
 *
 * This is a throwaway diagnostic program -- not part of the final
 * application (main.c / sensor_reader.c).
 *
 * Build: make test_min_period
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "sh2.h"
#include "sh2_hal.h"
#include "sh2_err.h"

extern sh2_Hal_t *sh2_hal_rpi_init(void);

static bool sResetSeen = false;

static void asyncEventCallback(void *cookie, sh2_AsyncEvent_t *pEvent)
{
    (void)cookie;
    if (pEvent->eventId == SH2_RESET) {
        sResetSeen = true;
    }
}

/* No sensors are configured in this test, so this should never fire. */
static void sensorCallback(void *cookie, sh2_SensorEvent_t *pEvent)
{
    (void)cookie;
    (void)pEvent;
}

static void printMetadataFor(const char *label, int sensorId)
{
    sh2_SensorMetadata_t meta;
    memset(&meta, 0, sizeof(meta));

    int rc = sh2_getMetadata(sensorId, &meta);
    if (rc != SH2_OK) {
        fprintf(stderr, "FAIL: sh2_getMetadata(%s, id=%d) returned %d\n",
                label, sensorId, rc);
        return;
    }

    printf("%-24s sensorId=%3d  minPeriod_us=%8u  (max rate ~= %.2f Hz)\n",
           label, sensorId,
           (unsigned)meta.minPeriod_uS,
           meta.minPeriod_uS > 0 ? 1000000.0 / (double)meta.minPeriod_uS : 0.0);
}

int main(void)
{
    printf("== Minimum Period / Max Rate Diagnostic ==\n");

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

    rc = sh2_setSensorCallback(sensorCallback, NULL);
    if (rc != SH2_OK) {
        fprintf(stderr, "FAIL: sh2_setSensorCallback() returned %d\n", rc);
        sh2_close();
        return 1;
    }

    /* Drive the session briefly so the SH2_RESET async event and any
     * pending metadata responses are processed. */
    for (int i = 0; i < 200; i++) {
        sh2_service();
    }

    if (!sResetSeen) {
        fprintf(stderr,
            "WARN: never observed SH2_RESET async event -- metadata "
            "queries below may fail if the device is not ready.\n");
    }

    printf("\n-- Metadata (minimum period per sensor) --\n");
    printMetadataFor("ROTATION_VECTOR",     SH2_ROTATION_VECTOR);
    printMetadataFor("LINEAR_ACCELERATION", SH2_LINEAR_ACCELERATION);
    printMetadataFor("GYROSCOPE_CALIBRATED", SH2_GYROSCOPE_CALIBRATED);

    sh2_close();
    printf("\n== Diagnostic complete ==\n");
    return 0;
}
