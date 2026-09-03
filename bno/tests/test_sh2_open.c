#define _POSIX_C_SOURCE 200112L
/*
 * test_sh2_open.c
 *
 * Stage 2 minimal test harness: brings up sh2_lib (sh2_open) on top
 * of the already-validated sh2_hal_rpi.c, with NO sensors configured.
 *
 * Goal: confirm sh2_lib can successfully parse the HAL's raw output
 * (SHTP advertisement, reset-complete, etc.) via its own internal
 * shtp.c logic, and that sh2_service() drives the session correctly,
 * before any sensor configuration or sensor_app/control_loop code
 * is introduced.
 *
 * This is a throwaway diagnostic program -- not part of the final
 * application (main.c / sensor_app.c / control_loop.c).
 *
 * Build (example, from bno/app/):
 *   gcc -o test_sh2_open test_sh2_open.c sh2_hal_rpi.c \
 *    ../sh2/sh2.c ../sh2/shtp.c ../sh2/sh2_util.c ../sh2/sh2_SensorValue.c ../sh2/euler.c \
 *    -I../sh2 -lgpiod
 *
 * (Add any other .c files from sh2/ that sh2.c depends on, e.g.
 * sh2_util.c, sh2_SensorValue.c, if present in your checkout.)
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

extern sh2_Hal_t *sh2_hal_rpi_init(void);

#define TEST_DURATION_SEC   15

static bool sResetSeen = false;
static int  sEventCount = 0;

/* ------------------------------------------------------------------ */
/* sh2 async event callback -- no sensors configured, so we only      */
/* expect SH2_RESET / SH2_SHTP_EVENT / SH2_GET_FEATURE_RESP style     */
/* async events here, never SH2_SENSOR_EVENT.                         */
/* ------------------------------------------------------------------ */

static const char *eventIdToStr(int eventId)
{
    switch (eventId) {
        case SH2_RESET:            return "SH2_RESET";
        case SH2_SHTP_EVENT:       return "SH2_SHTP_EVENT";
        case SH2_GET_FEATURE_RESP: return "SH2_GET_FEATURE_RESP";
        default:                   return "UNKNOWN_ASYNC_EVENT";
    }
}

static void eventCallback(void *cookie, sh2_AsyncEvent_t *pEvent)
{
    (void)cookie;
    sEventCount++;

    printf("[async event %d] id=%d (%s)\n",
           sEventCount, pEvent->eventId, eventIdToStr(pEvent->eventId));

    if (pEvent->eventId == SH2_RESET) {
        sResetSeen = true;
        printf("    -> device reported RESET (expected once after sh2_open)\n");
    }
    if (pEvent->eventId == SH2_SHTP_EVENT) {
        printf("    -> SHTP-level event, shtpEvent=%d\n", pEvent->shtpEvent);
    }
}

/*
 * A sensor callback IS still required by sh2_open()'s signature in
 * some versions of sh2_lib, but since we configure zero sensors in
 * this stage, it should never fire. We register it anyway and flag
 * loudly if it does, since that would indicate something unexpected
 * (e.g. leftover feature reports from a prior run).
 */
static void sensorCallback(void *cookie, sh2_SensorEvent_t *pEvent)
{
    (void)cookie;
    fprintf(stderr,
        "UNEXPECTED: sensorCallback fired with reportId=%d "
        "-- no sensors were configured in Stage 2!\n",
        pEvent->reportId);
}

int main(void)
{
    printf("== Stage 2: sh2_open + async events + sh2_service (no sensors) ==\n");

    sh2_Hal_t *hal = sh2_hal_rpi_init();
    if (!hal) {
        fprintf(stderr, "FAIL: sh2_hal_rpi_init() returned NULL\n");
        return 1;
    }

    /*
     * sh2_open() internally calls hal->open(), then drives the SHTP
     * advertisement / reset-complete exchange via hal->read(), and
     * invokes eventCallback() with SH2_RESET once the device is
     * confirmed reset and ready.
     */
    int rc = sh2_open(hal, eventCallback, NULL);
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

    /*
     * Optionally query product IDs via sh2_lib itself (not the manual
     * bring-up path) to further confirm the library's parsing is
     * working correctly end-to-end.
     */
    sh2_ProductIds_t prodIds;
    memset(&prodIds, 0, sizeof(prodIds));
    rc = sh2_getProdIds(&prodIds);
    if (rc == SH2_OK) {
        printf("PASS: sh2_getProdIds() returned %d entries\n", prodIds.numEntries);
        for (int i = 0; i < prodIds.numEntries; i++) {
            printf("    entry %d: swVersionMajor=%d swVersionMinor=%d swPartNumber=%d\n",
                   i,
                   prodIds.entry[i].swVersionMajor,
                   prodIds.entry[i].swVersionMinor,
                   prodIds.entry[i].swPartNumber);
        }
    } else {
        fprintf(stderr, "WARN: sh2_getProdIds() returned %d (non-fatal for this stage)\n", rc);
    }

    /*
     * Drive the session with sh2_service() in a tight loop, as the
     * CEVA reference examples (sh2-demo-nucleo) do -- no sensors
     * configured, so we're purely validating session bring-up,
     * event delivery, and that the loop doesn't stall or error out.
     */
    printf("\n-- Running sh2_service() loop for %d seconds --\n", TEST_DURATION_SEC);

    time_t startTime = time(NULL);
    while (time(NULL) - startTime < TEST_DURATION_SEC) {
        sh2_service();
        usleep(1000); /* 1 ms between service calls */
    }

    printf("\n-- Loop complete --\n");
    printf("Total async events observed: %d\n", sEventCount);

    if (!sResetSeen) {
        fprintf(stderr,
            "FAIL: never observed SH2_RESET async event -- check that "
            "sh2_open() is correctly driving the HAL's read()/write().\n");
    } else {
        printf("PASS: observed SH2_RESET async event as expected\n");
    }

    sh2_close();
    printf("PASS: sh2_close() completed\n");

    printf("\n== Stage 2 test complete ==\n");
    return (sResetSeen && sEventCount > 0) ? 0 : 1;
}
