#define _POSIX_C_SOURCE 200809L

/*
 * cal_main.c — bno_cal: guided BNO085 dynamic-calibration tool.
 *
 * Walks the operator through the CEVA "BNO085 IMU Sensor Calibration"
 * procedure (doc 1000-4044) using the vendored SH-2 library's dynamic
 * calibration API:
 *
 *   1. enable ME calibration for accelerometer + gyro + magnetometer
 *      (bitwise OR of the SH2_CAL_* bits — a logical OR collapses the
 *      mask to 0x01, the SparkFun Example_20 bug);
 *   2. accelerometer: six unique resting orientations, ~2 s each;
 *   3. gyroscope: device stationary on a surface for a few seconds;
 *   4. magnetometer: ~180-degree back-and-forth swings about each
 *      axis until the magnetic field accuracy reaches 2 or 3;
 *   5. hold still ~6 s (the hub snapshots dynamic cal data to RAM
 *      every 5 s and Save DCD persists the last snapshot), then
 *      sh2_saveDcdNow() writes the DCD to flash (FRS record 0x1F1F);
 *   6. close and reopen the session — the HAL open toggles RST, so
 *      the chip reboots and reloads the DCD from flash — then verify
 *      with all dynamic calibration disabled, exactly the way bno_app
 *      runs, so the verdict describes what acquisition will see.
 *
 * Verification gate: accelerometer AND magnetometer accuracy >= 2.
 * The gyro bit is deliberately NOT gated: it reads 0 whenever the
 * gyro dynamic calibration flag is disabled (i.e. under the bno_app
 * all-off policy), regardless of the DCD — measured on hardware and
 * a known BNO08x behavior. Gyro calibration is instead confirmed
 * during step 3, where the flag is on and the bit is meaningful.
 * The rotation vector's radian error estimate is printed as a
 * diagnostic but not gated.
 *
 * The old DCD is never cleared: Save DCD overwrites the flash record
 * wholesale, so recalibration is safe without a clear step.
 *
 * Usage:
 *   bno_cal            run the guided calibration; logs the accuracy
 *                      trace to bno_cal_<date>_<time>.csv in the
 *                      current directory
 *   bno_cal --check    read-only field go/no-go: open the session,
 *                      disable dynamic calibration like bno_app, print
 *                      the ME cal config and accuracy bits, exit
 *
 * Exit codes:
 *   0  success (calibrated, saved, verified) / --check: ready
 *   1  runtime error (SPI/SH-2 failure, DCD save failed)
 *   2  aborted by the user ('q' at a prompt or Ctrl-C)
 *   3  not calibrated (--check verdict) or verification failed
 *
 * Must not run at the same time as bno_app or the validation binaries:
 * one SPI HAL instance per process.
 *
 * Part of: rpi4b prod code/bno/calibration
 */

#include <inttypes.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "sh2.h"
#include "sh2_err.h"

#include "cal_sample.h"
#include "sensor_calibrate.h"

#define EXIT_OK              0
#define EXIT_ERROR           1
#define EXIT_ABORT           2
#define EXIT_NOT_CALIBRATED  3

/* Accuracy gate: 0 unreliable, 1 low, 2 medium, 3 high. */
#define ACC_GOAL 2

/* Masks for which accuracies a phase waits on. */
#define NEED_ACCEL (1u << 0)
#define NEED_GYRO  (1u << 1)
#define NEED_MAG   (1u << 2)

/*
 * Verify/--check gate: accel + mag only. The gyro bit reads 0 while
 * the gyro cal flag is off (the bno_app policy these modes mirror),
 * so it cannot be gated there.
 */
#define NEED_VERIFY (NEED_ACCEL | NEED_MAG)

#define SERVICE_LOOP_US   1000     /* ~1 kHz service, like the app loop */
#define DISPLAY_PERIOD_US 500000   /* live status line refresh */

typedef enum {
    PH_SETUP = 0,
    PH_ACCEL,
    PH_GYRO,
    PH_MAG,
    PH_HOLD,
    PH_SAVE,
    PH_VERIFY,
} CalPhase_t;

static const char *phaseName(CalPhase_t phase)
{
    switch (phase) {
        case PH_SETUP:  return "setup";
        case PH_ACCEL:  return "accel";
        case PH_GYRO:   return "gyro";
        case PH_MAG:    return "mag";
        case PH_HOLD:   return "hold";
        case PH_SAVE:   return "save";
        case PH_VERIFY: return "verify";
    }
    return "?";
}

static volatile sig_atomic_t sAbort = 0;
static void onSigint(int sig)
{
    (void)sig;
    sAbort = 1;
}

/* ------------------------------------------------------------------ */
/* CSV accuracy trace                                                  */
/* ------------------------------------------------------------------ */

static FILE *sCsv = NULL;
static uint32_t sCsvLastSeq = 0;

static void csvOpen(void)
{
    char name[64];
    time_t t = time(NULL);
    struct tm tmv;
    localtime_r(&t, &tmv);
    strftime(name, sizeof(name), "bno_cal_%Y%m%d_%H%M%S.csv", &tmv);

    sCsv = fopen(name, "w");
    if (sCsv == NULL) {
        fprintf(stderr,
                "bno_cal: WARNING: cannot open %s; continuing without CSV\n",
                name);
        return;
    }
    fprintf(sCsv,
            "host_us,device_us,phase,seq,accel_acc,gyro_acc,mag_acc,rv_acc,"
            "rv_err_rad,mag_ut_x,mag_ut_y,mag_ut_z\n");
    printf("bno_cal: logging accuracy trace to %s\n", name);
    sCsvLastSeq = 0;
}

static void csvWrite(const CalSample_t *s, CalPhase_t phase)
{
    if (sCsv == NULL || s == NULL || s->seq == sCsvLastSeq) {
        return;
    }
    sCsvLastSeq = s->seq;
    fprintf(sCsv,
            "%" PRIu64 ",%" PRIu64 ",%s,%" PRIu32 ",%u,%u,%u,%u,"
            "%.4f,%.3f,%.3f,%.3f\n",
            s->tHost_uS, s->tDevice_uS, phaseName(phase), s->seq,
            s->accelAccuracy, s->gyroAccuracy, s->magAccuracy,
            s->rvAccuracy, (double)s->rvErrRad,
            (double)s->magX_uT, (double)s->magY_uT, (double)s->magZ_uT);
}

static void csvClose(void)
{
    if (sCsv != NULL) {
        fclose(sCsv);
        sCsv = NULL;
    }
}

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static uint64_t hostNowUs(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000000ULL) +
           ((uint64_t)ts.tv_nsec / 1000ULL);
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

static bool accurateEnough(const CalSample_t *s, unsigned need)
{
    if ((need & NEED_ACCEL) && s->accelAccuracy < ACC_GOAL) return false;
    if ((need & NEED_GYRO)  && s->gyroAccuracy  < ACC_GOAL) return false;
    if ((need & NEED_MAG)   && s->magAccuracy   < ACC_GOAL) return false;
    return true;
}

static void printLiveLine(const CalSample_t *s)
{
    printf("  [acc %u  gyr %u  mag %u  rv %u]  rv_err=%5.2f rad  "
           "mag=(%7.2f %7.2f %7.2f) uT\r",
           s->accelAccuracy, s->gyroAccuracy, s->magAccuracy,
           s->rvAccuracy, (double)s->rvErrRad,
           (double)s->magX_uT, (double)s->magY_uT, (double)s->magZ_uT);
    fflush(stdout);
}

static void printVerdict(const CalSample_t *s)
{
    printf("  accelerometer accuracy: %u (%s)\n",
           s->accelAccuracy, accName(s->accelAccuracy));
    printf("  gyroscope accuracy:     %u (%s)%s\n",
           s->gyroAccuracy, accName(s->gyroAccuracy),
           (s->gyroAccuracy < ACC_GOAL)
               ? "  [reads 0 while gyro dynamic cal is disabled - expected]"
               : "");
    printf("  magnetometer accuracy:  %u (%s)\n",
           s->magAccuracy, accName(s->magAccuracy));
    printf("  rotation vector:        accuracy %u (%s), "
           "heading error est. %.2f rad\n",
           s->rvAccuracy, accName(s->rvAccuracy),
           (double)s->rvErrRad);
}

static void printCalConfig(uint8_t mask)
{
    printf("ME dynamic calibration config: accel %s, gyro %s, mag %s "
           "(mask 0x%02x)\n",
           (mask & SH2_CAL_ACCEL) ? "ON" : "off",
           (mask & SH2_CAL_GYRO)  ? "ON" : "off",
           (mask & SH2_CAL_MAG)   ? "ON" : "off",
           mask);
}

static void reportOpenFailure(void)
{
    fprintf(stderr, "error: sensor_calibrate_start failed (BNO085/SPI)\n");
    fprintf(stderr, "hint: is bno_app or another sh2 consumer still "
                    "running? one HAL instance per process\n");
}

/*
 * Prints the prompt and waits for Enter. Returns false to abort
 * ('q' line, EOF, or Ctrl-C).
 */
static bool promptEnter(const char *prompt)
{
    char buf[16];
    printf("%s\n  [Enter] = continue, q = abort: ", prompt);
    fflush(stdout);
    if (sAbort) return false;
    if (fgets(buf, sizeof(buf), stdin) == NULL) return false;
    if (sAbort) return false;
    return !(buf[0] == 'q' || buf[0] == 'Q');
}

/*
 * Services the session at ~1 kHz for duration_ms, logging each new
 * sample to the CSV. When live is true, refreshes a status line every
 * 500 ms. Returns false if the user aborted (Ctrl-C).
 */
static bool serviceFor(unsigned duration_ms, CalPhase_t phase, bool live)
{
    uint64_t tEnd = hostNowUs() + (uint64_t)duration_ms * 1000ULL;
    uint64_t tLastDisplay = 0;
    CalSample_t s;

    while (hostNowUs() < tEnd) {
        if (sAbort) return false;
        sensor_calibrate_service();
        if (sensor_calibrate_getLatestSample(&s)) {
            csvWrite(&s, phase);
            if (live && (hostNowUs() - tLastDisplay) >= DISPLAY_PERIOD_US) {
                printLiveLine(&s);
                tLastDisplay = hostNowUs();
            }
        }
        usleep(SERVICE_LOOP_US);
    }
    if (live) printf("\n");
    return !sAbort;
}

/*
 * Services the session until the requested accuracies reach ACC_GOAL,
 * the timeout (seconds) expires, or the user aborts.
 * Returns 0 = gate met, 1 = timeout, 2 = abort.
 */
static int waitAccurate(unsigned need, unsigned timeout_s, CalPhase_t phase)
{
    uint64_t tEnd = hostNowUs() + (uint64_t)timeout_s * 1000000ULL;
    uint64_t tLastDisplay = 0;
    CalSample_t s;

    while (hostNowUs() < tEnd) {
        if (sAbort) return 2;
        sensor_calibrate_service();
        if (sensor_calibrate_getLatestSample(&s)) {
            csvWrite(&s, phase);
            if ((hostNowUs() - tLastDisplay) >= DISPLAY_PERIOD_US) {
                printLiveLine(&s);
                tLastDisplay = hostNowUs();
            }
            if (accurateEnough(&s, need)) {
                printf("\n");
                return 0;
            }
        }
        usleep(SERVICE_LOOP_US);
    }
    printf("\n");
    return 1;
}

/* ------------------------------------------------------------------ */
/* --check: read-only field go/no-go                                  */
/* ------------------------------------------------------------------ */

static int doCheck(void)
{
    CalSample_t s;
    uint8_t calCfg = 0;

    printf("bno_cal --check: opening session (read-only inspection)...\n");
    if (!sensor_calibrate_start()) {
        reportOpenFailure();
        return EXIT_ERROR;
    }

    /*
     * Mirror bno_app's flight policy (all dynamic calibration off, fly
     * on the saved DCD) so the verdict describes exactly what the
     * acquisition binary will see.
     */
    if (sh2_setCalConfig(0) != SH2_OK) {
        fprintf(stderr, "error: sh2_setCalConfig failed\n");
        sensor_calibrate_stop();
        return EXIT_ERROR;
    }

    if (sh2_getCalConfig(&calCfg) == SH2_OK) {
        printCalConfig(calCfg);
    }
    printf("note: gyro accuracy reads 0 while gyro dynamic cal is off; "
           "it is not part of the verdict.\n");

    printf("monitoring accuracy for 5 s (keep the device stationary)...\n");
    if (!serviceFor(5000, PH_VERIFY, true)) {
        sensor_calibrate_stop();
        return EXIT_ABORT;
    }

    if (!sensor_calibrate_getLatestSample(&s)) {
        fprintf(stderr, "error: no sensor reports arrived\n");
        sensor_calibrate_stop();
        return EXIT_ERROR;
    }
    printVerdict(&s);
    sensor_calibrate_stop();

    if (accurateEnough(&s, NEED_VERIFY)) {
        printf("RESULT: READY - saved calibration looks good (exit 0)\n");
        return EXIT_OK;
    }
    printf("RESULT: NOT CALIBRATED - run bno_cal (exit 3)\n");
    return EXIT_NOT_CALIBRATED;
}

/* ------------------------------------------------------------------ */
/* Guided calibration flow                                             */
/* ------------------------------------------------------------------ */

static int doCalibrate(void)
{
    static const char *const accelPositions[6] = {
        "1/6: device flat, label side up",
        "2/6: device flat, label side down",
        "3/6: device resting on its left edge",
        "4/6: device resting on its right edge",
        "5/6: device resting on its top edge",
        "6/6: device resting on its bottom edge",
    };
    CalSample_t s;
    uint8_t calMask;

    printf("=== BNO085 guided calibration ===\n\n");
    printf("Before starting:\n");
    printf("  - move away from desks, PCs, monitors, cables and magnets\n");
    printf("    (the magnetometer bakes this environment into the DCD)\n");
    printf("  - calibrate the device in its final mounting, in the area\n");
    printf("    where it will be deployed\n");
    printf("  - make sure bno_app and the validation binaries are stopped\n");
    printf("  - abort any time: 'q' at a prompt or Ctrl-C\n\n");

    csvOpen();

    printf("opening SH-2 session...\n");
    if (!sensor_calibrate_start()) {
        reportOpenFailure();
        csvClose();
        return EXIT_ERROR;
    }

    /* Step 1: enable dynamic calibration for all three sensors.
     * Bitwise OR of the SH2_CAL_* bits — the gyro flag is required for
     * hand-held calibration per CEVA 1000-4044. */
    calMask = SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG;
    if (sh2_setCalConfig(calMask) != SH2_OK ||
        sh2_getCalConfig(&calMask) != SH2_OK) {
        fprintf(stderr, "error: sh2_setCalConfig/getCalConfig failed\n");
        goto fail;
    }
    printCalConfig(calMask);

    /* Step 2: accelerometer — six unique resting orientations. */
    printf("\n--- ACCELEROMETER: six resting orientations ---\n");
    for (unsigned round = 1; round <= 3; ++round) {
        if (round > 1) {
            printf("accelerometer accuracy below %d after round %u; "
                   "repeating the six positions\n",
                   ACC_GOAL, round - 1);
        }
        for (int i = 0; i < 6; ++i) {
            if (!promptEnter(accelPositions[i])) goto abort;
            if (!serviceFor(2000, PH_ACCEL, true)) goto abort;
        }
        {
            int rc = waitAccurate(NEED_ACCEL, 10, PH_ACCEL);
            if (rc == 2) goto abort;
            if (rc == 0) break;
            if (round == 3) {
                printf("accelerometer accuracy did not reach %d; "
                       "continuing (it may settle during later phases)\n",
                       ACC_GOAL);
            }
        }
    }

    /* Step 3: gyroscope — rest. The gyro accuracy bit is only
     * meaningful in this phase: the gyro cal flag is on here. */
    printf("\n--- GYROSCOPE: keep the device stationary on a surface ---\n");
    if (!promptEnter("place the device flat and do not touch it")) goto abort;
    {
        int rc = waitAccurate(NEED_GYRO, 15, PH_GYRO);
        if (rc == 2) goto abort;
        if (rc == 1) {
            printf("gyro accuracy did not reach %d; continuing\n", ACC_GOAL);
        }
    }

    /* Step 4: magnetometer — 180-degree back-and-forth swings. */
    printf("\n--- MAGNETOMETER: 180-degree back-and-forth swings ---\n");
    for (unsigned round = 1; round <= 5; ++round) {
        printf("round %u: hold the device in hand and swing it slowly\n"
               "(~2 s per swing) ~180 degrees and back about EACH axis\n"
               "in turn: roll it over, pitch it over, then yaw it left\n"
               "and right. Watch the mag accuracy below; target is %d.\n",
               round, ACC_GOAL);
        if (!promptEnter("start swinging (device in hand)")) goto abort;
        /* minimum 10 s of swings so fresh data is always gathered,
         * even if the accuracy gate is already met from an old DCD */
        if (!serviceFor(10000, PH_MAG, true)) goto abort;
        {
            int rc = waitAccurate(NEED_MAG, 50, PH_MAG);
            if (rc == 2) goto abort;
            if (rc == 0) break;
            if (round == 5) {
                fprintf(stderr,
                        "error: magnetometer accuracy did not reach %d; "
                        "check the magnetic environment and retry\n",
                        ACC_GOAL);
                goto fail;
            }
        }
    }

    /* Step 5: hold still so the hub's 5-second RAM snapshot includes
     * the final estimates, then save the DCD to flash. */
    printf("\n--- SAVE: hold the device still in its resting position ---\n");
    if (!serviceFor(6000, PH_HOLD, false)) goto abort;

    {
        int rc = sh2_saveDcdNow();
        if (rc != SH2_OK) {
            printf("save DCD failed (rc=%d); holding 5 s more and "
                   "retrying once...\n", rc);
            if (!serviceFor(5000, PH_HOLD, false)) goto abort;
            rc = sh2_saveDcdNow();
        }
        if (rc != SH2_OK) {
            fprintf(stderr,
                    "error: sh2_saveDcdNow failed (rc=%d); DCD NOT saved\n",
                    rc);
            goto fail;
        }
    }
    printf("DCD saved to flash (FRS record 0x1F1F).\n");

    /* Courtesy: leave the session in the flight-ready state. This does
     * not survive the reset below (or any future sh2_open) — bno_app
     * sets its own policy at startup. */
    if (sh2_setCalConfig(0) != SH2_OK) {
        printf("note: end-of-session sh2_setCalConfig(0) failed "
               "(cosmetic only)\n");
    }

    /* Step 6: verify exactly the way bno_app will run. Reopening the
     * session performs the HAL reset sequence, so the chip reboots and
     * reloads the DCD from flash; dynamic calibration stays disabled.
     * Gate: accel + mag (the gyro bit reads 0 by design with the gyro
     * cal flag off; gyro was confirmed in step 3). */
    printf("\n--- VERIFY: reopening session (chip reset, DCD reload) ---\n");
    sensor_calibrate_stop();
    if (!sensor_calibrate_start()) {
        fprintf(stderr, "error: session reopen failed during verification\n");
        csvClose();
        return EXIT_ERROR;
    }
    if (sh2_setCalConfig(0) != SH2_OK) {
        fprintf(stderr, "error: sh2_setCalConfig failed during "
                        "verification\n");
        goto fail;
    }

    printf("leave the device stationary; watching accuracy for up to "
           "15 s...\n");
    {
        int rc = waitAccurate(NEED_VERIFY, 15, PH_VERIFY);
        if (rc == 2) goto abort;
        if (rc == 1) {
            if (sensor_calibrate_getLatestSample(&s)) printVerdict(&s);
            fprintf(stderr,
                    "error: accuracy did not recover after DCD reload; "
                    "calibration may not have persisted\n");
            sensor_calibrate_stop();
            csvClose();
            return EXIT_NOT_CALIBRATED;
        }
    }

    if (sensor_calibrate_getLatestSample(&s)) printVerdict(&s);
    sensor_calibrate_stop();
    csvClose();
    printf("RESULT: CALIBRATED AND VERIFIED (exit 0)\n");
    return EXIT_OK;

abort:
    printf("\nbno_cal: aborted by user (exit 2)\n");
    sensor_calibrate_stop();
    csvClose();
    return EXIT_ABORT;

fail:
    sensor_calibrate_stop();
    csvClose();
    return EXIT_ERROR;
}

int main(int argc, char **argv)
{
    bool checkOnly = false;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--check") == 0) {
            checkOnly = true;
        } else {
            fprintf(stderr, "usage: bno_cal [--check]\n");
            return EXIT_ERROR;
        }
    }

    signal(SIGINT, onSigint);

    return checkOnly ? doCheck() : doCalibrate();
}
