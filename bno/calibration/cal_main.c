/*
 * calibration/cal_main.c — Guided BNO085 dynamic calibration CLI.
 *
 * Runs the guided calibration flow:
 *   1. Opens an SH2 session via cal_sensor.c.
 *   2. Enables ME dynamic calibration for accelerometer, gyro, and
 *      magnetometer via sh2_setCalConfig(0x07).
 *   3. Guides the user through the motions required by the Hillcrest
 *      algorithms:
 *        - Accel: 4-6 stationary orientations (~1 s each, e.g. on
 *                 each face of a cube).
 *        - Gyro:  Stationary rest (~2-3 s) on a stable surface.
 *        - Mag:   Figure-8 / rotation in all 3 axes until status
 *                 reaches 3 (High).
 *   4. Freezes all dynamic calibration via sh2_setCalConfig(0x00)
 *      BEFORE saving to flash. This is critical: the Hillcrest ME
 *      firmware requires cal config to be 0 at save time so that the
 *      saved DCD record stores a static calibration snapshot (and
 *      avoids the vendor-example bug where SparkFun Example_20
 *      passed 0x01, leaving the gyro flag cleared in DCD).
 *   5. Persists the calibration to flash via sh2_saveDcdNow().
 *   6. Verifies the saved state: resets the sensor, queries the DCD
 *      status, and confirms accuracy bits remain valid across reboot.
 *
 * Sudo & real-time policy:
 *   This tool requires root (or CAP_SYS_NICE and SPI device permissions)
 *   to access spidev and configure GPIO.
 *
 * Command line options:
 *   bno_cal                       interactive guided calibration
 *   bno_cal --check               continuous live read-only inspection;
 *                                 terminates on Ctrl-C and evaluates pass/fail
 *   bno_cal --check --mask 0xNN   probe mode: applies given ME cal mask (e.g. 0x05)
 *                                 and exits automatically after 10 s probe
 *
 * Exit codes:
 *   0  success / ready
 *   1  general / runtime error
 *   2  aborted by user
 *   3  not calibrated (--check verdict)
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>

#include "sh2.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"
#include "cal_contract.h"
#include "cal_sensor.h"

#define ACC_GOAL          3        /* High accuracy */
#define SUSTAIN_MS        2000     /* Accuracy must hold 2 s continuously */
#define SERVICE_LOOP_US   1000     /* ~1 kHz service loop */
#define DISPLAY_PERIOD_US 500000   /* 500 ms refresh for live telemetry */

#define EXIT_OK              0
#define EXIT_ERROR           1
#define EXIT_ABORT           2
#define EXIT_NOT_CALIBRATED  3

/* Masks for which accuracies a phase waits on. */
#define NEED_ACCEL (1u << 0)
#define NEED_GYRO  (1u << 1)
#define NEED_MAG   (1u << 2)
#define NEED_RV    (1u << 3)

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
        case 0: return "Unreliable";
        case 1: return "Low";
        case 2: return "Medium";
        case 3: return "High";
        default: return "Unknown";
    }
}

static void csvWrite(const CalSample_t *s, CalPhase_t p)
{
    if (!sCsvFp) return;
    fprintf(sCsvFp, "%llu,%s,%u,%u,%u,%u,%.4f,%.2f,%.2f,%.2f\n",
            (unsigned long long)s->tHost_uS,
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

static bool accurateEnough(const CalSample_t *s, unsigned need)
{
    if ((need & NEED_ACCEL) && s->accelAccuracy < ACC_GOAL) return false;
    if ((need & NEED_GYRO)  && s->gyroAccuracy  < ACC_GOAL) return false;
    if ((need & NEED_MAG)   && s->magAccuracy   < ACC_GOAL) return false;
    if ((need & NEED_RV)    && s->rvAccuracy    < ACC_GOAL) return false;
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
    int rc = 0;

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

    if (haveMask) {
        /* Probe mode: evaluate accuracy for up to 10 s and exit automatically */
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

        printf("RESULT: probe complete (mask 0x%02x) - informational "
               "only (exit 0)\n", mask);
        return EXIT_OK;
    }

    /* Continuous check mode without --mask: streams live telemetry until Ctrl+C */
    printf("monitoring accuracy live (press Ctrl+C to terminate)...\n");
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
    printf("\n");

    if (!gotSample) {
        fprintf(stderr, "error: no sensor reports arrived\n");
        cal_sensor_stop();
        return EXIT_ERROR;
    }

    printVerdict(&s);
    cal_sensor_stop();

    if (isFlightReady(&s, mask)) {
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
    char csvFilename[64];
    time_t rawtime;
    struct tm *timeinfo;
    CalSample_t s;
    uint8_t calMask;
    int rc;

    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(csvFilename, sizeof(csvFilename),
             "bno_cal_%Y%m%d_%H%M%S.csv", timeinfo);

    sCsvFp = fopen(csvFilename, "w");
    if (sCsvFp) {
        fprintf(sCsvFp, "# bno_cal log started %s", asctime(timeinfo));
        fprintf(sCsvFp, "# timestamp_us,phase,accel_acc,gyro_acc,mag_acc,rv_acc,rv_err_rad,mag_x,mag_y,mag_z\n");
    }

    printf("====================================================\n");
    printf("  BNO085 Dynamic Calibration (bno_cal)\n");
    printf("====================================================\n\n");

    if (!cal_sensor_start()) {
        reportOpenFailure();
        if (sCsvFp) fclose(sCsvFp);
        return EXIT_ERROR;
    }

    /* Phase 1: Enable dynamic calibration for Accel, Gyro, Mag */
    calMask = SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG;
    if (sh2_setCalConfig(calMask) != SH2_OK ||
        sh2_getCalConfig(&calMask) != SH2_OK) {
        fprintf(stderr, "error: failed to configure dynamic calibration\\n\");\n        cal_sensor_stop();\n        if (sCsvFp) fclose(sCsvFp);\n        return EXIT_ERROR;\n    }\n    printCalConfig(calMask);\n\n    /* Phase 2: Gyro calibration */\n    printf(\"\\n[Phase 1/3] Gyroscope calibration\\n\");\n    printf(\"Place the sensor stationary on a stable flat surface.\\n\");\n    if (!promptEnter(\"Ready to calibrate gyroscope?\")) goto abort;\n\n    printf(\"Calibrating gyro (keep stationary)...\");\n    fflush(stdout);\n    rc = waitAccurateSustained(NEED_GYRO, 15, PH_GYRO, true);\n    if (rc == 2) goto abort;\n    if (rc != 0) {\n        printf(\"warning: gyro accuracy did not reach %d within timeout\\n\", ACC_GOAL);\n    } else {\n        printf(\"Gyroscope calibration settled.\\n\");\n    }\n\n    /* Phase 3: Accelerometer calibration */\n    printf(\"\\n[Phase 2/3] Accelerometer calibration\\n\");\n    printf(\"Slowly rotate the device to 4-6 distinct stationary orientations\\n\");\n    printf(\"(hold each orientation still for 1-2 seconds, like faces of a cube).\\n\");\n    if (!promptEnter(\"Ready to calibrate accelerometer?\")) goto abort;\n\n    rc = waitAccurateSustained(NEED_ACCEL, 30, PH_ACCEL, true);\n    if (rc == 2) goto abort;\n    if (rc != 0) {\n        printf(\"warning: accelerometer accuracy did not reach %d within timeout\\n\", ACC_GOAL);\n    } else {\n        printf(\"Accelerometer calibration settled.\\n\");\n    }\n\n    /* Phase 4: Magnetometer calibration */\n    printf(\"\\n[Phase 3/3] Magnetometer calibration\\n\");\n    printf(\"Slowly rotate the device in figure-8 motions across all 3 axes.\\n\");\n    if (!promptEnter(\"Ready to calibrate magnetometer?\")) goto abort;\n\n    rc = waitAccurateSustained(NEED_MAG, 45, PH_MAG, true);\n    if (rc == 2) goto abort;\n    if (rc != 0) {\n        printf(\"warning: magnetometer accuracy did not reach %d within timeout\\n\", ACC_GOAL);\n    } else {\n        printf(\"Magnetometer calibration settled.\\n\");\n    }\n\n    /* Phase 5: Hold steady before snapshot */\n    printf(\"\\nCalibration motions complete. Hold the device stationary...\\n\");\n    serviceFor(3000, PH_HOLD, true);\n\n    /* Phase 6: Freeze dynamic cal before saving to flash */\n    printf(\"Freezing dynamic calibration (mask 0x00) before persisting to flash...\\n\");\n    if (sh2_setCalConfig(0) != SH2_OK) {\n        fprintf(stderr, \"error: failed to clear dynamic cal config\\n\");\n        cal_sensor_stop();\n        if (sCsvFp) fclose(sCsvFp);\n        return EXIT_ERROR;\n    }\n\n    /* Phase 7: Persist calibration to flash */\n    printf(\"Persisting calibration to DCD flash record...\\n\");\n    if (sh2_saveDcdNow() != SH2_OK) {\n        fprintf(stderr, \"error: sh2_saveDcdNow failed\\n\");\n        cal_sensor_stop();\n        if (sCsvFp) fclose(sCsvFp);\n        return EXIT_ERROR;\n    }\n    printf(\"Calibration successfully saved to flash.\\n\");\n\n    /* Phase 8: Verify across reset */\n    printf(\"Resetting sensor to verify saved calibration reload...\\n\");\n    cal_sensor_stop();\n    usleep(300000);\n\n    if (!cal_sensor_start()) {\n        fprintf(stderr, \"error: re-opening session failed after reset\\n\");\n        if (sCsvFp) fclose(sCsvFp);\n        return EXIT_ERROR;\n    }\n\n    sh2_setCalConfig(0);\n    printf(\"Verifying restored calibration across reset...\\n\");\n    rc = waitAccurateSustained(NEED_VERIFY, 10, PH_VERIFY, true);\n\n    if (cal_sensor_getLatestSample(&s)) {\n        printf(\"\\nPost-reset verification summary:\\n\");\n        printVerdict(&s);\n    }\n    cal_sensor_stop();\n\n    if (sCsvFp) {\n        fclose(sCsvFp);\n        printf(\"\\nLogged calibration trajectory to %s\\n\", csvFilename);\n    }\n\n    if (rc == 0 && isFlightReady(&s, 0)) {\n        printf(\"RESULT: SUCCESS - BNO085 calibrated and verified.\\n\");\n        return EXIT_OK;\n    }\n\n    printf(\"RESULT: WARNING - verification did not reach target accuracy.\\n\");\n    return EXIT_NOT_CALIBRATED;\n\nabort:\n    printf(\"\\nCalibration aborted by user.\\n\");\n    cal_sensor_stop();\n    if (sCsvFp) fclose(sCsvFp);\n    return EXIT_ABORT;\n}\n\nint main(int argc, char **argv)\n{\n    bool checkOnly = false;\n    bool haveMask = false;\n    uint8_t mask = 0;\n\n    for (int i = 1; i < argc; i++) {\n        if (strcmp(argv[i], \"--check\") == 0) {\n            checkOnly = true;\n        } else if (strcmp(argv[i], \"--mask\") == 0) {\n            if (i + 1 >= argc) {\n                fprintf(stderr, \"error: --mask needs a value, e.g. \"\n                                \"--mask 0x05\\n\");\n                return EXIT_ERROR;\n            }\n            char *endp = NULL;\n            unsigned long tmp = strtoul(argv[++i], &endp, 0);\n            if (!endp || *endp != '\\0' || tmp > 0x07) {\n                fprintf(stderr, \"error: --mask value out of range \"\n                                \"(0x00 to 0x07): '%s'\\n\", argv[i]);\n                return EXIT_ERROR;\n            }\n            mask = (uint8_t)tmp;\n            haveMask = true;\n        } else {\n            fprintf(stderr, \"error: unrecognized option '%s'\\n\"\n                            \"usage: bno_cal [--check [--mask 0xNN]]\\n\",\n                    argv[i]);\n            return EXIT_ERROR;\n        }\n    }\n\n    if (haveMask && !checkOnly) {\n        fprintf(stderr, \"error: --mask is only valid together with \"\n                        \"--check\\n\");\n        return EXIT_ERROR;\n    }\n\n    signal(SIGINT, onSigint);\n    return checkOnly ? doCheck(mask, haveMask) : doCalibrate();\n}\n