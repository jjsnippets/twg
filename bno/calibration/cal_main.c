#define _POSIX_C_SOURCE 200809L

/*
 * cal_main.c — bno_cal: guided BNO085 dynamic-calibration and clear tool.
 *
 * Walks the operator through the CEVA "BNO08X Sensor Calibration
 * Procedure" (doc 1000-4044) using the vendored SH-2 library's dynamic
 * calibration API:
 *
 *   1. enable ME calibration for accelerometer + gyro + magnetometer
 *      (bitwise OR of the SH2_CAL_* bits). The gyro flag is
 *      required for hand-held calibration per 1000-4044;
 *   2. accelerometer: 4-6 unique resting orientations, ~1 s each;
 *   3. gyroscope: device stationary on a surface for ~2-3 s;
 *   4. magnetometer: ~180-degree back-and-forth rotations about each
 *      axis (roll, pitch, yaw), ~2 s per axis, repeated until the
 *      Magnetic Field status bit reads 2 or 3;
 *   5. hold still ~10 s, then sh2_saveDcdNow() writes the DCD to
 *      flash (FRS record 0x1F1F). The save is REFUSED if accuracy degraded;
 *   6. close and reopen session — verify with all dynamic calibration disabled,
 *      exactly the way bno_app runs.
 *
 * Verification gate: accelerometer AND magnetometer accuracy >= 2,
 * sustained for >= 3 s, plus rotation vector status >= 2 with heading error <= 0.35 rad.
 *
 * Usage:
 *   bno_cal                       run guided calibration
 *   bno_cal --clear               erase all dynamic calibration (flash + RAM)
 *   bno_cal --check               read-only field go/no-go check
 *   bno_cal --check --mask 0xNN   probe mode with specified ME cal mask
 *
 * Exit codes:
 *   0  success (calibrated, saved, verified / cleared / check ready)
 *   1  runtime error (SPI/SH-2 failure, DCD save failed)
 *   2  aborted by user ('n' or 'q' at a prompt or Ctrl-C)
 *   3  not calibrated (--check verdict) or verification failed
 *
 * Must not run at the same time as bno_app, bno_orient or bno_validate:
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

#include "calibration/cal_contract.h"
#include "calibration/cal_sensor.h"

#define EXIT_OK              0
#define EXIT_ERROR           1
#define EXIT_ABORT           2
#define EXIT_NOT_CALIBRATED  3

/* Accuracy gate: 0 unreliable, 1 low, 2 medium, 3 high. */
#define ACC_GOAL 2
#define MAX_HEADING_ERR_RAD 0.35f  /* ~20 degrees */

/* Masks for which accuracies a phase waits on. */
#define NEED_ACCEL (1u << 0)
#define NEED_GYRO  (1u << 1)
#define NEED_MAG   (1u << 2)
#define NEED_RV    (1u << 3)

#define NEED_VERIFY (NEED_ACCEL | NEED_MAG)
#define SUSTAIN_MS 3000

#define MAG_MIN_SWING_MS 25000
#define MAX_SAVE_ATTEMPTS 3
#define VERIFY_MOTION_MS 10000

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
    if ((need & NEED_RV)    && (s->rvAccuracy   < ACC_GOAL ||
                                s->rvErrRad > MAX_HEADING_ERR_RAD)) return false;
    return true;
}

static bool isFlightReady(const CalSample_t *s, uint8_t calMask)
{
    if (s->accelAccuracy < ACC_GOAL) return false;
    if (s->magAccuracy   < ACC_GOAL) return false;
    if (s->rvAccuracy    < ACC_GOAL) return false;
    if (s->rvErrRad      > MAX_HEADING_ERR_RAD) return false;

    if ((calMask & SH2_CAL_GYRO) && s->gyroAccuracy < ACC_GOAL) {
        return false;
    }
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
               ? "  [reads 0 while gyro dynamic cal is off - observed on"
                 " this unit; not gated]"
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
    fprintf(stderr, "error: cal_sensor_start failed (BNO085/SPI)\n");
    fprintf(stderr, "hint: is bno_app or another sh2 consumer still "
                    "running? one HAL instance per process\n");
}

#define FRS_RECORD_DCD (0x1F1F)

/*
 * Prints the prompt and waits for single-key [y/N] confirmation.
 * Returns true only if 'y' or 'Y' is entered; false on 'n', 'N', 'q', EOF, or Enter.
 */
static bool promptConfirm(const char *prompt)
{
    char buf[32];
    printf("%s [y/N]: ", prompt);
    fflush(stdout);
    if (sAbort) return false;
    if (fgets(buf, sizeof(buf), stdin) == NULL) return false;
    if (sAbort) return false;
    return (buf[0] == 'y' || buf[0] == 'Y');
}

static bool serviceFor(unsigned duration_ms, CalPhase_t phase, bool live)
{
    uint64_t tEnd = hostNowUs() + (uint64_t)duration_ms * 1000ULL;
    uint64_t tLastDisplay = 0;
    CalSample_t s;

    while (hostNowUs() < tEnd) {
        if (sAbort) return false;
        cal_sensor_service();
        if (cal_sensor_getLatestSample(&s)) {
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

static int waitAccurateSustained(unsigned need, unsigned timeout_s,
                                 CalPhase_t phase, bool live)
{
    uint64_t tEnd = hostNowUs() + (uint64_t)timeout_s * 1000000ULL;
    uint64_t tLastDisplay = 0;
    uint64_t tGoodSince = 0;
    CalSample_t s;

    while (hostNowUs() < tEnd) {
        if (sAbort) return 2;
        cal_sensor_service();
        if (cal_sensor_getLatestSample(&s)) {
            csvWrite(&s, phase);
            if (live && (hostNowUs() - tLastDisplay) >= DISPLAY_PERIOD_US) {
                printLiveLine(&s);
                tLastDisplay = hostNowUs();
            }
            if (accurateEnough(&s, need)) {
                if (tGoodSince == 0) {
                    tGoodSince = hostNowUs();
                } else if ((hostNowUs() - tGoodSince) >=
                           (uint64_t)SUSTAIN_MS * 1000ULL) {
                    printf("\n");
                    return 0;
                }
            } else {
                tGoodSince = 0;
            }
        }
        usleep(SERVICE_LOOP_US);
    }
    printf("\n");
    return 1;
}

/* ------------------------------------------------------------------ */
/* --check: read-only inspection / cal-config probe                   */
/* ------------------------------------------------------------------ */

static int doCheck(uint8_t mask)
{
    CalSample_t s;
    uint8_t calCfg = 0;
    int rc;

    printf("bno_cal --check: opening session (read-only inspection)...\n");
    if (!cal_sensor_start()) {
        reportOpenFailure();
        return EXIT_ERROR;
    }

    if (sh2_setCalConfig(mask) != SH2_OK) {
        fprintf(stderr, "error: sh2_setCalConfig failed\n");
        cal_sensor_stop();
        return EXIT_ERROR;
    }

    if (sh2_getCalConfig(&calCfg) == SH2_OK) {
        printCalConfig(calCfg);
    }
    if (!(mask & SH2_CAL_GYRO)) {
        printf("note: the gyro accuracy bit reads 0 while gyro dynamic "
               "cal is off (observed on this unit); it is not part of "
               "the verdict.\n");
    }
    printf("monitoring accuracy for up to 10 s (keep the device "
           "stationary; needs %d s of good readings)...\n",
           SUSTAIN_MS / 1000);
    rc = waitAccurateSustained(NEED_VERIFY, 10, PH_VERIFY, true);
    if (rc == 2) {
        cal_sensor_stop();
        return EXIT_ABORT;
    }

    if (!cal_sensor_getLatestSample(&s)) {
        fprintf(stderr, "error: no sensor reports arrived\n");
        cal_sensor_stop();
        return EXIT_ERROR;
    }
    printVerdict(&s);
    cal_sensor_stop();

    if (mask != 0) {
        printf("RESULT: probe complete (mask 0x%02x) - informational "
               "only (exit 0)\n",
               mask);
        return EXIT_OK;
    }
    if (rc == 0 && isFlightReady(&s, mask)) {
        printf("RESULT: READY - saved calibration looks good (exit 0)\n");
        return EXIT_OK;
    }
    printf("RESULT: NOT CALIBRATED - run bno_cal (exit 3)\n");
    return EXIT_NOT_CALIBRATED;
}

/* ------------------------------------------------------------------ */
/* Guided calibration flow                                             */
/* ------------------------------------------------------------------ */

static int magPhase(void)
{
    for (unsigned round = 1; round <= 5; ++round) {
        int rc;

        printf("round %u: hold the device in hand and perform FULL\n"
               "swing patterns: rotate it ~180 degrees and back about\n"
               "EACH axis in turn - roll it over, pitch it over, then\n"
               "yaw it left and right (~2 s per swing, per CEVA 1000-\n"
               "4044). Keep repeating patterns for the whole round.\n",
               round);
        if (!promptConfirm("start swinging (device in hand)")) return 2;

        if (!serviceFor(MAG_MIN_SWING_MS, PH_MAG, true)) return 2;

        rc = waitAccurateSustained(NEED_MAG, 30, PH_MAG, true);
        if (rc == 2) return 2;
        if (rc == 0) return 0;

        printf("mag accuracy not sustained at %d yet; do another round "
               "of full patterns.\n",
               ACC_GOAL);
    }
    return 1;
}

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
    bool saved = false;

    printf("=== BNO085 guided calibration ===\n\n");
    printf("Before starting:\n");
    printf("  - move away from desks, PCs, monitors, cables and magnets\n");
    printf("    (the magnetometer bakes this environment into the DCD)\n");
    printf("  - calibrate the device in its final mounting, in the area\n");
    printf("    where it will be deployed\n");
    printf("  - make sure bno_app and the validation binaries are stopped\n");
    printf("  - abort any time: 'n' or 'q' at a prompt or Ctrl-C\n\n");

    csvOpen();

    printf("opening SH-2 session...\n");
    if (!cal_sensor_start()) {
        reportOpenFailure();
        csvClose();
        return EXIT_ERROR;
    }

    calMask = SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG;
    if (sh2_setCalConfig(calMask) != SH2_OK ||
        sh2_getCalConfig(&calMask) != SH2_OK) {
        fprintf(stderr, "error: sh2_setCalConfig/getCalConfig failed\n");
        goto fail;
    }
    printCalConfig(calMask);

    printf("\n--- ACCELEROMETER: six resting orientations ---\n");
    for (unsigned round = 1; round <= 3; ++round) {
        if (round > 1) {
            printf("accelerometer accuracy below %d after round %u; "
                   "repeating the six positions\n",
                   ACC_GOAL, round - 1);
        }
        for (int i = 0; i < 6; ++i) {
            if (!promptConfirm(accelPositions[i])) goto abort;
            if (!serviceFor(2000, PH_ACCEL, true)) goto abort;
        }
        {
            int rc = waitAccurateSustained(NEED_ACCEL, 10, PH_ACCEL, true);
            if (rc == 2) goto abort;
            if (rc == 0) break;
            if (round == 3) {
                printf("accelerometer accuracy did not reach %d; "
                       "continuing (it may settle during later phases)\n",
                       ACC_GOAL);
            }
        }
    }

    printf("\n--- GYROSCOPE: keep the device stationary on a surface ---\n");
    if (!promptConfirm("place the device flat and do not touch it")) goto abort;
    {
        int rc = waitAccurateSustained(NEED_GYRO, 15, PH_GYRO, true);
        if (rc == 2) goto abort;
        if (rc == 1) {
            printf("gyro accuracy did not reach %d; continuing "
                   "(informational)\n",
                   ACC_GOAL);
        }
    }

    for (unsigned attempt = 1; attempt <= MAX_SAVE_ATTEMPTS && !saved;
         ++attempt) {
        int rc;

        printf("\n--- MAGNETOMETER: 180-degree swing patterns "
               "(attempt %u/%u) ---\n",
               attempt, MAX_SAVE_ATTEMPTS);
        rc = magPhase();
        if (rc == 2) goto abort;
        if (rc == 1) {
            fprintf(stderr,
                    "error: magnetometer accuracy did not sustain at %d; "
                    "check the magnetic environment and retry\n",
                    ACC_GOAL);
            goto fail;
        }

        printf("\n--- SAVE: hold the device still in its resting "
               "position (~10 s) ---\n");
        if (!serviceFor(10000, PH_HOLD, false)) goto abort;

        rc = waitAccurateSustained(NEED_VERIFY, 5, PH_HOLD, false);
        if (rc == 2) goto abort;
        if (rc == 1) {
            if (cal_sensor_getLatestSample(&s)) {
                printf("accuracy degraded at rest (acc=%u mag=%u); NOT "
                       "saving this state - swing again\n",
                       s.accelAccuracy, s.magAccuracy);
            }
            continue;
        }

        {
            int src = sh2_saveDcdNow();
            if (src != SH2_OK) {
                printf("save DCD failed (rc=%d); holding 5 s more and "
                       "retrying once...\n",
                       src);
                if (!serviceFor(5000, PH_HOLD, false)) goto abort;
                src = sh2_saveDcdNow();
            }
            if (src != SH2_OK) {
                fprintf(stderr,
                        "error: sh2_saveDcdNow failed (rc=%d); "
                        "DCD NOT saved\n",
                        src);
                goto fail;
            }
        }
        printf("DCD saved to flash (FRS record 0x1F1F).\n");
        saved = true;
    }
    if (!saved) {
        fprintf(stderr,
                "error: calibration state kept degrading at rest after "
                "%u attempts; nothing saved\n",
                MAX_SAVE_ATTEMPTS);
        goto fail;
    }

    if (sh2_setCalConfig(0) != SH2_OK) {
        printf("note: end-of-session sh2_setCalConfig(0) failed "
               "(cosmetic only)\n");
    }

    printf("\n--- VERIFY: reopening session (chip reset, DCD reload) ---\n");
    cal_sensor_stop();
    if (!cal_sensor_start()) {
        fprintf(stderr, "error: session reopen failed during verification\n");
        csvClose();
        return EXIT_ERROR;
    }
    if (sh2_setCalConfig(0) != SH2_OK) {
        fprintf(stderr, "error: sh2_setCalConfig failed during "
                        "verification\n");
        goto fail;
    }

    printf("motion window: slowly rotate the device in yaw back and\n"
           "forth for ~%u s (like the glider moving) and watch the rv "
           "line...\n",
           VERIFY_MOTION_MS / 1000);
    if (!promptConfirm("hold the device, ready to move it")) goto abort;
    if (!serviceFor(VERIFY_MOTION_MS, PH_VERIFY, true)) goto abort;

    printf("now set the device down stationary; watching accuracy for "
           "up to 20 s (needs %d s of good readings)...\n",
           SUSTAIN_MS / 1000);
    {
        int rc = waitAccurateSustained(NEED_VERIFY, 20, PH_VERIFY, true);
        if (rc == 2) goto abort;
        if (rc == 1) {
            if (cal_sensor_getLatestSample(&s)) printVerdict(&s);
            fprintf(stderr,
                    "error: accuracy did not recover after DCD reload; "
                    "calibration may not have persisted\n");
            cal_sensor_stop();
            csvClose();
            return EXIT_NOT_CALIBRATED;
        }
    }

    if (cal_sensor_getLatestSample(&s)) {
        printVerdict(&s);
        if (!isFlightReady(&s, 0)) {
            printf("note: rotation vector reads %u / %.2f rad "
                   "(expected >= %d and <= %.2f rad) - probe other "
                   "configs with 'bno_cal --check --mask 0xNN' before "
                   "relying on RV status in flight.\n",
                   s.rvAccuracy, (double)s.rvErrRad, ACC_GOAL,
                   (double)MAX_HEADING_ERR_RAD);
            cal_sensor_stop();
            csvClose();
            return EXIT_NOT_CALIBRATED;
        }
    }
    cal_sensor_stop();
    csvClose();
    printf("RESULT: CALIBRATED AND VERIFIED (exit 0)\n");
    return EXIT_OK;

abort:
    printf("\nbno_cal: aborted by user (exit 2)\n");
    cal_sensor_stop();
    csvClose();
    return EXIT_ABORT;

fail:
    cal_sensor_stop();
    csvClose();
    return EXIT_ERROR;
}

/* ------------------------------------------------------------------ */
/* --clear: erase all dynamic calibration (flash + RAM)               */
/* ------------------------------------------------------------------ */

static int doClear(void)
{
    uint32_t dummy = 0;

    printf("=== BNO085 dynamic-calibration (DCD) clear ===\n\n");
    printf("WARNING: this permanently erases the BNO085's saved\n");
    printf("dynamic calibration from BOTH flash (record 0x1F1F) and RAM.\n");
    printf("The sensor will be UNCALIBRATED afterwards.\n\n");

    if (!promptConfirm("permanently erase calibration and reset")) {
        printf("\nbno_cal: declined - nothing was erased (exit 2)\n");
        return EXIT_ABORT;
    }

    printf("opening SH-2 session...\n");
    if (!cal_sensor_start()) {
        reportOpenFailure();
        return EXIT_ERROR;
    }

    /*
     * SH-2 RM 6.4.9 step 2: delete the flash copy of the DCD.
     * words = 0 deletes the record (sh2.h).
     */
    if (sh2_setFrs(FRS_RECORD_DCD, &dummy, 0) != SH2_OK) {
        fprintf(stderr,
                "error: sh2_setFrs(delete DCD record 0x1F1F) failed; "
                "flash copy NOT erased\n");
        cal_sensor_stop();
        return EXIT_ERROR;
    }
    printf("flash DCD record (FRS 0x1F1F) deleted.\n");

    /*
     * SH-2 RM 6.4.9 step 3: atomic clear-RAM-DCD + chip reset.
     */
    if (sh2_clearDcdAndReset() != SH2_OK) {
        fprintf(stderr, "error: sh2_clearDcdAndReset failed\n");
        cal_sensor_stop();
        return EXIT_ERROR;
    }
    printf("RAM DCD cleared and chip reset.\n");

    cal_sensor_stop();
    printf("\nRESULT: DCD ERASED (exit 0)\n");
    printf("Run 'bno_cal' to perform a fresh guided calibration.\n");
    return EXIT_OK;
}

int main(int argc, char **argv)
{
    bool checkOnly = false;
    bool clearOnly = false;
    bool haveMask  = false;
    uint8_t mask   = 0;
    unsigned long tmp;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--check") == 0) {
            checkOnly = true;
        } else if (strcmp(argv[i], "--clear") == 0) {
            clearOnly = true;
        } else if (strcmp(argv[i], "--mask") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "error: --mask needs a value, e.g. "
                                "--mask 0x05\n");
                return EXIT_ERROR;
            }
            tmp = strtoul(argv[++i], NULL, 0);
            if (tmp > 0xFF) {
                fprintf(stderr, "error: --mask value out of range "
                                "(0x00-0xFF)\n");
                return EXIT_ERROR;
            }
            mask = (uint8_t)tmp;
            haveMask = true;
        } else {
            fprintf(stderr,
                    "usage: bno_cal [--clear] [--check [--mask 0xNN]]\n");
            return EXIT_ERROR;
        }
    }

    if (clearOnly && (checkOnly || haveMask)) {
        fprintf(stderr, "error: --clear cannot be combined with --check or --mask\n");
        return EXIT_ERROR;
    }

    if (haveMask && !checkOnly) {
        fprintf(stderr, "error: --mask is only valid together with "
                        "--check\n");
        return EXIT_ERROR;
    }

    signal(SIGINT, onSigint);

    if (clearOnly) return doClear();
    return checkOnly ? doCheck(mask) : doCalibrate();
}
