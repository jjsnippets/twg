#define _POSIX_C_SOURCE 200809L

/*
 * cal_clear.c — bno_cal_clear: erase ALL dynamic calibration data (DCD)
 * from the BNO085, from both flash (FRS) and RAM.
 *
 * Purpose: thesis documentation — capture the sensor's UNCALIBRATED
 * baseline (before/after accuracy printouts) and demonstrate what
 * calibration actually changes.
 *
 * Implements the SH-2 Reference Manual section 6.4.9 recommended
 * sequence for completely resetting DCD state:
 *
 *   1. reset the hub — done implicitly: opening the session performs
 *      the HAL pin-toggle reset;
 *   2. delete the flash copy of the DCD (FRS record 0x1F1F) via
 *      sh2_setFrs() with words = 0 ("0 to delete record", sh2.h);
 *   3. issue the Clear DCD and Reset command (sh2_clearDcdAndReset),
 *      which atomically clears the RAM DCD and resets the chip, so
 *      the old RAM state cannot re-persist to flash (at non-power-up
 *      reset the hub persists the last-stored RAM DCD to FRS — BNO08X
 *      datasheet section 3.4).
 *
 * Steps 2 and 3 are BOTH required: the Clear DCD and Reset command
 * alone only clears RAM — the flash copy would simply reload at the
 * next boot and the calibration would return. Conversely, if the
 * flash delete fails, the tool aborts BEFORE the RAM clear, so a
 * failed run leaves the existing calibration fully intact.
 *
 * Flow: open a session (all dynamic calibration disabled, like
 * bno_app, so the readings are policy-comparable), print the CURRENT
 * calibrated state for ~5 s, require the user to type CLEAR to
 * proceed, wipe flash + RAM, reopen on the freshly booted device and
 * print the UNCALIBRATED state for ~5 s.
 *
 * Exit codes:
 *   0  cleared and verified
 *   1  runtime error
 *   2  user declined / aborted
 *
 * Must not run at the same time as bno_app, bno_cal or the validation
 * binaries: one SPI HAL instance per process.
 *
 * Part of: rpi4b prod code/bno/calibration
 */

#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "sh2.h"
#include "sh2_err.h"

#include "cal_sample.h"
#include "sensor_calibrate.h"

/* DCD record ID — SH-2 Reference Manual, Figure 26 (Configuration
 * Records): 0x1F1F Dynamic calibration. */
#define FRS_RECORD_DCD (0x1F1F)

#define WATCH_SECONDS 5

static volatile sig_atomic_t sAbort = 0;
static void onSigint(int sig)
{
    (void)sig;
    sAbort = 1;
}

static const char *accName(unsigned v)
{
    switch (v) {
        case 0:  return "unreliable";
        case 1:  return "low";
        case 2:  return "medium";
        case 3:  return "high";
    }
    return "?";
}

/*
 * Services the session at ~1 kHz for WATCH_SECONDS and prints the
 * resulting accuracy state. Returns false on abort or if no reports
 * arrived.
 */
static bool watchState(const char *label)
{
    CalSample_t s;

    printf("%s (watching %u s; keep the device stationary)...\n",
           label, WATCH_SECONDS);
    for (unsigned sec = 0; sec < WATCH_SECONDS; ++sec) {
        if (sAbort) return false;
        for (int i = 0; i < 500; ++i) {
            sensor_calibrate_service();
            usleep(2000);
        }
    }
    if (sAbort) return false;

    if (!sensor_calibrate_getLatestSample(&s)) {
        fprintf(stderr, "error: no sensor reports arrived\n");
        return false;
    }

    printf("  accelerometer accuracy: %u (%s)\n",
           s.accelAccuracy, accName(s.accelAccuracy));
    printf("  gyroscope accuracy:     %u (%s)%s\n",
           s.gyroAccuracy, accName(s.gyroAccuracy),
           (s.gyroAccuracy < 2)
               ? "  [reads 0 while gyro dynamic cal is off - observed"
                 " on this unit]"
               : "");
    printf("  magnetometer accuracy:  %u (%s)\n",
           s.magAccuracy, accName(s.magAccuracy));
    printf("  rotation vector:        accuracy %u (%s), "
           "heading error est. %.2f rad\n",
           s.rvAccuracy, accName(s.rvAccuracy),
           (double)s.rvErrRad);
    return true;
}

/*
 * Prints the warning and requires the user to type CLEAR (exact,
 * uppercase) to proceed. Anything else, EOF or Ctrl-C aborts.
 */
static bool confirmClear(void)
{
    char buf[32];

    printf("\nWARNING: this permanently erases the BNO085's saved\n");
    printf("dynamic calibration (DCD) from BOTH flash and RAM. The\n");
    printf("sensor will be UNCALIBRATED afterwards and must be\n");
    printf("recalibrated with bno_cal before any data collection.\n");
    printf("\nType CLEAR (uppercase) to erase, anything else to abort: ");
    fflush(stdout);

    if (sAbort) return false;
    if (fgets(buf, sizeof(buf), stdin) == NULL) return false;
    if (sAbort) return false;
    buf[strcspn(buf, "\r\n")] = '\0';
    return strcmp(buf, "CLEAR") == 0;
}

static void reportOpenFailure(void)
{
    fprintf(stderr, "error: sensor_calibrate_start failed (BNO085/SPI)\n");
    fprintf(stderr, "hint: is bno_app or another sh2 consumer still "
                    "running? one HAL instance per process\n");
}

int main(void)
{
    uint32_t dummy = 0;

    signal(SIGINT, onSigint);

    printf("=== BNO085 DCD clear ===\n\n");

    /*
     * SH-2 RM 6.4.9 step 1: the session open performs the HAL reset.
     * Dynamic calibration is disabled (bno_app's policy) so the
     * before/after readings are comparable with bno_cal --check.
     */
    printf("bno_cal_clear: opening session...\n");
    if (!sensor_calibrate_start()) {
        reportOpenFailure();
        return 1;
    }
    if (sh2_setCalConfig(0) != SH2_OK) {
        fprintf(stderr, "error: sh2_setCalConfig failed\n");
        sensor_calibrate_stop();
        return 1;
    }

    /* The "before" documentation: the currently saved calibration. */
    if (!watchState("current state (before clear):")) {
        sensor_calibrate_stop();
        return sAbort ? 2 : 1;
    }

    if (!confirmClear()) {
        printf("\nbno_cal_clear: declined - nothing was erased (exit 2)\n");
        sensor_calibrate_stop();
        return 2;
    }

    /*
     * SH-2 RM 6.4.9 step 2: delete the flash copy of the DCD.
     * words = 0 deletes the record (sh2.h). If this fails, abort
     * BEFORE the RAM clear so the existing calibration stays intact.
     */
    if (sh2_setFrs(FRS_RECORD_DCD, &dummy, 0) != SH2_OK) {
        fprintf(stderr,
                "error: sh2_setFrs(delete DCD record 0x1F1F) failed; "
                "flash copy NOT erased - aborting before the RAM clear, "
                "calibration is intact\n");
        sensor_calibrate_stop();
        return 1;
    }
    printf("flash DCD record (FRS 0x1F1F) deleted.\n");

    /*
     * SH-2 RM 6.4.9 step 3: atomic clear-RAM-DCD + chip reset. No
     * response is sent; the chip reboots immediately and the session
     * dies with it. Clearing RAM first is what stops the old
     * calibration from re-persisting to flash at this reset
     * (BNO08X datasheet section 3.4).
     */
    if (sh2_clearDcdAndReset() != SH2_OK) {
        fprintf(stderr, "error: sh2_clearDcdAndReset failed\n");
        sensor_calibrate_stop();
        return 1;
    }
    printf("RAM DCD cleared and chip reset.\n");

    /* Close host-side resources, then reopen on the freshly booted,
     * now-uncalibrated device. */
    sensor_calibrate_stop();

    printf("\nbno_cal_clear: reopening session on the cleared device...\n");
    if (!sensor_calibrate_start()) {
        fprintf(stderr, "error: session reopen failed after clear\n");
        return 1;
    }
    if (sh2_setCalConfig(0) != SH2_OK) {
        fprintf(stderr, "error: sh2_setCalConfig failed after clear\n");
        sensor_calibrate_stop();
        return 1;
    }

    /* The "after" documentation: the uncalibrated baseline. */
    if (!watchState("uncalibrated state (after clear):")) {
        sensor_calibrate_stop();
        return sAbort ? 2 : 1;
    }
    sensor_calibrate_stop();

    printf("\nRESULT: DCD ERASED (exit 0)\n");
    printf("The sensor is now uncalibrated. Run bno_cal to recalibrate,\n");
    printf("then bno_cal --check to confirm before data collection.\n");
    return 0;
}
