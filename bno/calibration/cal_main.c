#define _POSIX_C_SOURCE 200809L

/*
 * cal_main.c — guided BNO085 dynamic calibration CLI.
 *
 * Runs the guided calibration flow recommended by CEVA / Hillcrest Labs
 * (BNO080/BNO085 Tare Function and Dynamic Calibration procedure,
 * document 1000-4044):
 *
 *   1. enable dynamic calibration for accel, gyro and mag (0x07);
 *   2. guide the operator through the required motions:
 *        - accel: six unique resting orientations, e.g. on each face
 *                 of a cube, ~2 s per orientation (needs gravity to
 *                 separate from sensor bias);
 *        - gyro:  stationary rest on a surface for ~2-3 s (let the
 *                 zero-rate estimator converge);
 *        - mag:   full 180-degree swing patterns in yaw, pitch, and
 *                 roll (repeat until status reaches 3 / High);
 *   3. hold still at rest to let the hub's periodic DCD snapshot
 *      (taken every 5 s per the BNO08X datasheet section 3.4) capture
 *      the good state, and confirm accuracy does not degrade before
 *      saving;
 *   4. save to the DCD flash record via sh2_saveDcdNow();
 *   5. reset the sensor, query the DCD status, and confirm the
 *      calibration reloads and accuracy bits recover across reboot.
 *
 * Sudo & real-time policy:
 *   This tool requires root (or CAP_SYS_NICE and SPI device permissions)
 *   to access spidev and configure GPIO lines.
 *
 * Command line options:
 *   bno_cal                       interactive guided calibration
 *   bno_cal --check               read-only field go/no-go: open the
 *                                 session with all dynamic calibration
 *                                 disabled (mirroring bno_app flight
 *                                 mode) and check that accel+mag stay
 *                                 at accuracy >= 2; continuous live
 *                                 telemetry until Ctrl-C.
 *   bno_cal --check --mask 0xNN   probe mode: applies given ME cal mask (e.g. 0x05)
 *                                 and exits automatically after 10 s probe
 *   bno_cal --clear               erase all DCD calibration from flash and RAM
 *
 * Exit codes:
 *   0  success (calibrated, saved, verified) / --check: ready or probe
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

#include "cal_contract.h"
#include "cal_sensor.h"

#define EXIT_OK              0
#define EXIT_ERROR           1
#define EXIT_ABORT           2
#define EXIT_NOT_CALIBRATED  3

/* DCD record ID — SH-2 Reference Manual Figure 26: 0x1F1F Dynamic Calibration */
#define FRS_RECORD_DCD       0x1F1F

/* Accuracy gate: 0 unreliable, 1 low, 2 medium, 3 high. */
#define ACC_GOAL 2
#define MAX_HEADING_ERR_RAD 0.35f  /* ~20 degrees */

/*
 * Minimum sustained duration in each phase once accuracy reaches
 * ACC_GOAL. A single instantaneous report of 2 or 3 is not proof of
 * convergence: the mag status fluctuates at rest, and saving on a
 * transient spike persists a degraded calibration.
 */
#define SUSTAIN_MS 3000

/*
 * Fixed-duration motion windows during the mag phase and the verify
 * phase. These run unconditionally before the sustained-accuracy gate
 * is evaluated:
 *   - Mag: the sensor needs fresh angular excursions to populate its
 *     sphere fit; if the gate passes immediately on old data, the
 *     snapshot will not improve the calibration.
 *   - Verify: rotation vector needs motion to converge after a reset
 *     (under the all-off calibration policy, heading converges from
 *     the saved DCD + motion; sitting still leaves it stuck).
 */
#define MAG_MIN_SWING_MS  8000
#define VERIFY_MOTION_MS  10000

/* Save retry policy: hold still, retry up to this many times. */
#define MAX_SAVE_ATTEMPTS 3

#define SERVICE_LOOP_US   1000     /* ~1 kHz service loop */
#define DISPLAY_PERIOD_US 500000   /* 500 ms status line refresh */

/* Masks for which accuracies a phase waits on. */
#define NEED_ACCEL (1u << 0)
#define NEED_GYRO  (1u << 1)
#define NEED_MAG   (1u << 2)
#define NEED_RV    (1u << 3)

/*
 * Verify/--check and pre-save gate: accel + mag only. The gyro bit
 * reads 0 whenever gyro dynamic cal is disabled (the bno_app flight
 * policy, and the state after step 5) on this hardware unit, so
 * gating on gyro during verify would hang forever even though the
 * saved gyro bias is loaded. Confirmed 2026-09-07.
 */
#define NEED_VERIFY (NEED_ACCEL | NEED_MAG)

typedef enum {
    PH_STARTUP = 0,
    PH_ACCEL,
    PH_GYRO,
    PH_MAG,
    PH_HOLD,
    PH_SAVE,
    PH_RESET,
    PH_VERIFY,
} CalPhase_t;

static volatile sig_atomic_t sAbort = 0;
static FILE *sCsvFp = NULL;

static void onSigint(int sig)
{
    (void)sig;
    sAbort = 1;
}

static uint64_t hostNowUs(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
}

static const char *phaseName(CalPhase_t p)
{
    switch (p) {
        case PH_STARTUP: return "STARTUP";
        case PH_ACCEL:   return "ACCEL";
        case PH_GYRO:    return "GYRO";
        case PH_MAG:     return "MAG";
        case PH_HOLD:    return "HOLD";
        case PH_SAVE:    return "SAVE";
        case PH_RESET:   return "RESET";
        case PH_VERIFY:  return "VERIFY";
        default:         return "UNKNOWN";
    }
}

static const char *accName(uint8_t acc)
{
    switch (acc) {
        case 0: return "unreliable (0)";
        case 1: return "low (1)";
        case 2: return "medium (2)";
        case 3: return "high (3)";
        default: return "unknown";
    }
}

static void csvOpen(void)
{
    char fname[64];
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    strftime(fname, sizeof(fname), "cal_%Y%m%d_%H%M%S.csv", &tm);

    sCsvFp = fopen(fname, "w");
    if (!sCsvFp) {
        fprintf(stderr, "warning: cannot open %s for writing\n", fname);
        return;
    }
    fprintf(sCsvFp,
            "# bno_cal trajectory log\n"
            "# host_us,phase,accel_acc,gyro_acc,mag_acc,rv_acc,"
            "rv_err_rad,mag_x_uT,mag_y_uT,mag_z_uT\n");
    fflush(sCsvFp);
    printf("logging calibration trajectory to %s\n", fname);
}

static void csvWrite(const CalSample_t *s, CalPhase_t p)
{
    if (!sCsvFp) return;
    fprintf(sCsvFp,
            "%" PRIu64 ",%s,%u,%u,%u,%u,%.4f,%.2f,%.2f,%.2f\n",
            s->tHost_uS,
            phaseName(p),
            s->accelAccuracy,
            s->gyroAccuracy,
            s->magAccuracy,
            s->rvAccuracy,
            (double)s->rvErrRad,
            (double)s->magX_uT,
            (double)s->magY_uT,
            (double)s->magZ_uT);
}

static void csvClose(void)
{
    if (sCsvFp) {
        fclose(sCsvFp);
        sCsvFp = NULL;
    }
}

static bool accurateEnough(const CalSample_t *s, unsigned need)
{
    if ((need & NEED_ACCEL) && s->accelAccuracy < ACC_GOAL) return false;
    if ((need & NEED_GYRO)  && s->gyroAccuracy  < ACC_GOAL) return false;
    if ((need & NEED_MAG)   && s->magAccuracy   < ACC_GOAL) return false;
    if ((need & NEED_RV)    && (s->rvAccuracy < ACC_GOAL ||
                                s->rvErrRad > MAX_HEADING_ERR_RAD)) {
        return false;
    }
    return true;
}

static bool isFlightReady(const CalSample_t *s, uint8_t calMask)
{
    if (s->accelAccuracy < ACC_GOAL) return false;
    if (s->magAccuracy   < ACC_GOAL) return false;
    if ((calMask & SH2_CAL_GYRO) && s->gyroAccuracy < ACC_GOAL) return false;
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

static int doCheck(uint8_t mask, bool haveMask)
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
               "the verdict.\\n");
    }

    if (haveMask) {
        printf("monitoring accuracy for up to 10 s (keep the device "
               "stationary; needs %d s of good readings)...\\n",
               SUSTAIN_MS / 1000);
        rc = waitAccurateSustained(NEED_VERIFY, 10, PH_VERIFY, true);
        if (rc == 2) {
            cal_sensor_stop();
            return EXIT_ABORT;
        }

        if (!cal_sensor_getLatestSample(&s)) {
            fprintf(stderr, "error: no sensor reports arrived\\n");
            cal_sensor_stop();
            return EXIT_ERROR;
        }
        printVerdict(&s);
        cal_sensor_stop();

        printf("RESULT: probe complete (mask 0x%02x) - informational "
               "only (exit 0)\\n",
               mask);
        return EXIT_OK;
    }

    /* Continuous check mode: stream live telemetry until Ctrl+C */
    printf("monitoring accuracy live (press Ctrl+C to terminate)...\\n");
    uint64_t tLastDisplay = 0;
    bool gotSample = false;

    while (!sAbort) {
        cal_sensor_service();
        if (cal_sensor_getLatestSample(&s)) {
            gotSample = true;
            if ((hostNowUs() - tLastDisplay) >= DISPLAY_PERIOD_US) {
                printLiveLine(&s);
                tLastDisplay = hostNowUs();
            }
        }
        usleep(SERVICE_LOOP_US);
    }
    printf("\\n");

    if (!gotSample) {
        fprintf(stderr, "error: no sensor reports arrived\\n");
        cal_sensor_stop();
        return EXIT_ERROR;
    }
    printVerdict(&s);
    cal_sensor_stop();

    if (isFlightReady(&s, mask)) {
        printf("RESULT: READY - saved calibration looks good (exit 0)\\n");
        return EXIT_OK;
    }
    printf("RESULT: NOT CALIBRATED - run bno_cal (exit 3)\\n");
    return EXIT_NOT_CALIBRATED;
}

/* ------------------------------------------------------------------ */
/* --clear: erase DCD calibration from flash and RAM                  */
/* ------------------------------------------------------------------ */

static bool confirmClear(void)
{
    char buf[32];

    printf("\\nWARNING: this permanently erases the BNO085's saved\\n"
           "dynamic calibration (DCD) from BOTH flash and RAM. The\\n"
           "sensor will be UNCALIBRATED afterwards and must be\\n"
           "recalibrated with bno_cal before any data collection.\\n\\n"
           "Type CLEAR (uppercase) to erase, anything else to abort: ");
    fflush(stdout);

    if (sAbort) return false;
    if (fgets(buf, sizeof(buf), stdin) == NULL) return false;
    if (sAbort) return false;
    buf[strcspn(buf, "\\r\\n")] = '\\0';
    return strcmp(buf, "CLEAR") == 0;
}

static int doClear(void)
{
    uint32_t dummy = 0;
    CalSample_t s;

    printf("=== BNO085 DCD clear ===\\n\\n");
    printf("bno_cal --clear: opening session...\\n");
    if (!cal_sensor_start()) {
        reportOpenFailure();
        return EXIT_ERROR;
    }

    if (sh2_setCalConfig(0) != SH2_OK) {
        fprintf(stderr, "error: sh2_setCalConfig failed\\n");
        cal_sensor_stop();
        return EXIT_ERROR;
    }

    printf("current state (before clear; watching 5 s; keep device stationary)...\\n");
    for (int i = 0; i < 2500; ++i) {
        if (sAbort) break;
        cal_sensor_service();
        usleep(SERVICE_LOOP_US);
    }
    if (sAbort) {
        cal_sensor_stop();
        return EXIT_ABORT;
    }

    if (cal_sensor_getLatestSample(&s)) {
        printVerdict(&s);
    }

    if (!confirmClear()) {
        printf("\\nbno_cal --clear: declined - nothing was erased (exit 2)\\n");
        cal_sensor_stop();
        return EXIT_ABORT;
    }

    /* Delete flash DCD (FRS 0x1F1F) */
    if (sh2_setFrs(FRS_RECORD_DCD, &dummy, 0) != SH2_OK) {
        fprintf(stderr,
                "error: sh2_setFrs(delete DCD record 0x1F1F) failed; "
                "flash copy NOT erased - aborting before RAM clear\\n");
        cal_sensor_stop();
        return EXIT_ERROR;
    }
    printf("flash DCD record (FRS 0x1F1F) deleted.\\n");

    /* Clear RAM DCD and trigger reset */
    if (sh2_clearDcdAndReset() != SH2_OK) {
        fprintf(stderr, "error: sh2_clearDcdAndReset failed\\n");
        cal_sensor_stop();
        return EXIT_ERROR;
    }
    printf("RAM DCD cleared and chip reset.\\n");

    cal_sensor_stop();
    usleep(300000);

    printf("\\nbno_cal --clear: reopening session on cleared device...\\n");
    if (!cal_sensor_start()) {
        fprintf(stderr, "error: session reopen failed after clear\\n");
        return EXIT_ERROR;
    }

    if (sh2_setCalConfig(0) != SH2_OK) {
        fprintf(stderr, "error: sh2_setCalConfig failed after clear\\n");
        cal_sensor_stop();
        return EXIT_ERROR;
    }

    printf("uncalibrated state (after clear; watching 5 s)...\\n");
    for (int i = 0; i < 2500; ++i) {
        if (sAbort) break;
        cal_sensor_service();
        usleep(SERVICE_LOOP_US);
    }

    if (cal_sensor_getLatestSample(&s)) {
        printVerdict(&s);
    }
    cal_sensor_stop();

    printf("\\nRESULT: DCD ERASED (exit 0)\\n");
    printf("The sensor is now uncalibrated. Run bno_cal to recalibrate,\\n"
           "then bno_cal --check to confirm before data collection.\\n");
    return EXIT_OK;
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
        if (!promptEnter("start swinging (device in hand)")) return 2;

        /* Minimum fresh-motion window: two full roll/pitch/yaw
         * patterns, even if the accuracy gate is already met from a
         * previously saved DCD. */
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
    printf("  - abort any time: 'q' at a prompt or Ctrl-C\n\n");

    csvOpen();

    printf("opening SH-2 session...\n");
    if (!cal_sensor_start()) {
        reportOpenFailure();
        csvClose();
        return EXIT_ERROR;
    }

    /* Step 1: enable dynamic calibration for all three sensors. */
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

    /* Step 3: gyroscope — rest. */
    printf("\n--- GYROSCOPE: keep the device stationary on a surface ---\n");
    if (!promptEnter("place the device flat and do not touch it")) goto abort;
    {
        int rc = waitAccurateSustained(NEED_GYRO, 15, PH_GYRO, true);
        if (rc == 2) goto abort;
        if (rc == 1) {
            printf("gyro accuracy did not reach %d; continuing "
                   "(informational)\n",
                   ACC_GOAL);
        }
    }

    /* Steps 4 + 5: magnetometer swings, then hold + save. */
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

        /* Hold still so the hub's 5-second RAM snapshots all land
         * inside a good window, then confirm the state is still good
         * before saving. */
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

    /* Step 6: verify across reset */
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
    if (!promptEnter("hold the device, ready to move it")) goto abort;
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

int main(int argc, char **argv)
{
    bool checkOnly = false;
    bool clearOnly = false;
    bool haveMask = false;
    uint8_t mask = 0;
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
                    "usage: bno_cal [--check [--mask 0xNN]] [--clear]\n");
            return EXIT_ERROR;
        }
    }

    if (haveMask && !checkOnly) {
        fprintf(stderr, "error: --mask is only valid together with "
                        "--check\n");
        return EXIT_ERROR;
    }
    if (clearOnly && (checkOnly || haveMask)) {
        fprintf(stderr, "error: --clear cannot be combined with "
                        "--check or --mask\n");
        return EXIT_ERROR;
    }

    signal(SIGINT, onSigint);

    if (clearOnly) return doClear();
    return checkOnly ? doCheck(mask, haveMask) : doCalibrate();
}
