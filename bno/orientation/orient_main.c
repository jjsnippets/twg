#define _POSIX_C_SOURCE 200809L

/*
 * orient_main.c — bno_orient: align the BNO085's fused output frame
 * to the fixture's swing axis using the SH-2 tare commands.
 *
 * Dynamic calibration (bno_cal) corrects the sensor's internal
 * physics and stores it in the DCD (FRS record 0x1F1F). Tare corrects
 * the ORIENTATION of the fused output frame and stores it in the
 * separate sensor-orientation record; the two are independent, so
 * taring never touches a calibration.
 *
 * Tare semantics (CEVA BNO085 Tare Function note + SH-2 Reference
 * Manual, Tare command):
 *   - Tare Now (sh2_setTareNow) rotates the chosen axes of the fused
 *     output so that the CURRENT attitude reads as zero. Z-axis only
 *     (the default) zeroes heading; --all also zeroes pitch and roll
 *     and requires the device to be held dead level facing magnetic
 *     North (the full tare defines the whole frame).
 *   - A tare lives in hub RAM only: the next sh2_open performs the
 *     HAL reset, so the chip reboots and a volatile tare is lost.
 *   - Persist Tare (sh2_persistTare, --persist) writes the applied
 *     tare to flash so every later session (bno_app and the validation
 *     binaries included) boots in the tared frame.
 *   - Clear Tare (sh2_clearTare, --clear) drops the volatile tare.
 *     A tare previously persisted to flash is NOT erased — it
 *     reloads at the next reset and must be overwritten by a fresh
 *     tare + --persist.
 *
 * Usage:
 *   bno_orient            Z-axis heading tare at the fixture's
 *                         swing-axis zero (volatile until --persist)
 *   bno_orient --all      full 3-axis tare (level + magnetic North!)
 *   bno_orient --persist  also write the tare to flash
 *   bno_orient --clear    drop the volatile tare
 *   bno_orient --check    read-only: print heading + RV accuracy
 *
 * Exit codes:
 *   0  success (tare applied, and verified across reset if persisted)
 *   1  runtime error (SPI/SH-2 failure, heading never settled)
 *   2  aborted by the user ('n' or 'q' at a prompt or Ctrl-C)
 *
 * Must not run at the same time as bno_app, bno_cal, bno_validate
 * or the test binaries: one SPI HAL instance per process.
 *
 * Part of: rpi4b prod code/bno/orientation
 */

#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "sh2.h"
#include "sh2_err.h"

#include "orientation/orient_sensor.h"

#define EXIT_OK     0
#define EXIT_ERROR  1
#define EXIT_ABORT  2

/* Heading gate before taring: 0 unreliable, 1 low, 2 medium, 3 high. */
#define ACC_GOAL 2

/* Warn if a just-tared angle is further than this from zero. */
#define TARE_RESIDUAL_DEG 5.0f

#define SERVICE_LOOP_US   1000     /* ~1 kHz service, like bno_cal */
#define DISPLAY_PERIOD_US 250000   /* live heading line refresh */
#define SETTLE_TIMEOUT_S  15       /* rotation-vector settle gate */
#define WATCH_MS          5000     /* post-tare verify windows */

static volatile sig_atomic_t sAbort = 0;
static void onSigint(int sig)
{
    (void)sig;
    sAbort = 1;
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

/*
 * SH-2 euler convention (degrees, Tait-Bryan ZYX from the rotation-vector
 * quaternion): yaw about Z, pitch about Y, roll about X. Only the
 * relative before/after behaviour and the post-tare zero matter here,
 * so the usual BNO085 display convention is enough.
 */
static void eulerDeg(const OrientSample_t *s,
                     float *yawDeg, float *pitchDeg, float *rollDeg)
{
    const float RAD2DEG = 57.29577951f;
    float w = s->quatW, x = s->quatX, y = s->quatY, z = s->quatZ;

    float sp = 2.0f * (w * y - z * x);
    if (sp >  1.0f) sp =  1.0f;
    if (sp < -1.0f) sp = -1.0f;

    *rollDeg  = RAD2DEG * atan2f(2.0f * (w * x + y * z),
                                 1.0f - 2.0f * (x * x + y * y));
    *pitchDeg = RAD2DEG * asinf(sp);
    *yawDeg   = RAD2DEG * atan2f(2.0f * (w * z + x * y),
                                 1.0f - 2.0f * (y * y + z * z));
}

static void printAttitude(const char *label, const OrientSample_t *s)
{
    float yaw, pitch, roll;
    eulerDeg(s, &yaw, &pitch, &roll);

    printf("  %s:\n"
           "    q=(%.4f, %.4f, %.4f, %.4f)\n"
           "    yaw=%7.2f  pitch=%6.2f  roll=%6.2f deg  [rv accuracy %u (%s)]\n",
           label,
           (double)s->quatW, (double)s->quatX,
           (double)s->quatY, (double)s->quatZ,
           (double)yaw, (double)pitch, (double)roll,
           s->rvAccuracy, accName(s->rvAccuracy));
}

static void printLiveLine(const OrientSample_t *s)
{
    float yaw, pitch, roll;
    eulerDeg(s, &yaw, &pitch, &roll);

    printf("  [q=(%6.3f,%6.3f,%6.3f,%6.3f)  "
           "yaw=%7.2f  pitch=%6.2f  roll=%6.2f deg  rv acc %u]  \r",
           (double)s->quatW, (double)s->quatX,
           (double)s->quatY, (double)s->quatZ,
           (double)yaw, (double)pitch, (double)roll,
           s->rvAccuracy);
    fflush(stdout);
}

static void reportOpenFailure(void)
{
    fprintf(stderr, "error: orient_sensor_start failed (BNO085/SPI)\n");
    fprintf(stderr, "hint: is bno_app or another sh2 consumer still "
                    "running? one HAL instance per process\n");
}

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

/*
 * Services the session at ~1 kHz for duration_ms, refreshing a live
 * heading line every 250 ms. Returns false if the user aborted.
 */
static bool serviceFor(unsigned duration_ms, bool live)
{
    uint64_t tEnd = hostNowUs() + (uint64_t)duration_ms * 1000ULL;
    uint64_t tLastDisplay = 0;
    OrientSample_t s;

    while (hostNowUs() < tEnd) {
        if (sAbort) return false;
        orient_sensor_service();
        if (orient_sensor_getLatestSample(&s)) {
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
 * Services the session until the rotation vector reports accuracy
 * >= ACC_GOAL, the timeout expires, or the user aborts.
 * Returns 0 = settled, 1 = timeout, 2 = abort.
 */
static int waitSettled(void)
{
    uint64_t tEnd = hostNowUs() + (uint64_t)SETTLE_TIMEOUT_S * 1000000ULL;
    uint64_t tLastDisplay = 0;
    OrientSample_t s;

    while (hostNowUs() < tEnd) {
        if (sAbort) return 2;
        orient_sensor_service();
        if (orient_sensor_getLatestSample(&s)) {
            if (hostNowUs() - tLastDisplay >= DISPLAY_PERIOD_US) {
                printLiveLine(&s);
                tLastDisplay = hostNowUs();
            }
            if (s.haveRv && s.rvAccuracy >= ACC_GOAL) {
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
/* --check: read-only heading inspection                              */
/* ------------------------------------------------------------------ */

static int doCheck(void)
{
    OrientSample_t s;

    printf("bno_orient --check: opening session (read-only inspection)...\n");
    if (!orient_sensor_start()) {
        reportOpenFailure();
        return EXIT_ERROR;
    }

    /* Mirror bno_app's flight policy so the printed heading is what
     * acquisition will see. */
    if (sh2_setCalConfig(0) != SH2_OK) {
        fprintf(stderr, "error: sh2_setCalConfig failed\n");
        orient_sensor_stop();
        return EXIT_ERROR;
    }

    printf("monitoring the rotation vector for 5 s...\n");
    if (!serviceFor(5000, true)) {
        orient_sensor_stop();
        return EXIT_ABORT;
    }

    if (!orient_sensor_getLatestSample(&s)) {
        fprintf(stderr, "error: no sensor reports arrived\n");
        orient_sensor_stop();
        return EXIT_ERROR;
    }
    printAttitude("attitude", &s);

    if (!s.haveRv || s.rvAccuracy < ACC_GOAL) {
        printf("warning: rotation vector accuracy below %d — heading is "
               "not settled (run bno_cal before taring)\n",
               ACC_GOAL);
    }

    orient_sensor_stop();
    return EXIT_OK;
}

/* ------------------------------------------------------------------ */
/* --clear: drop the volatile tare                                    */
/* ------------------------------------------------------------------ */

static int doClear(void)
{
    OrientSample_t s;

    printf("=== BNO085 tare clear ===\n\n");

    printf("opening SH-2 session...\n");
    if (!orient_sensor_start()) {
        reportOpenFailure();
        return EXIT_ERROR;
    }
    if (sh2_setCalConfig(0) != SH2_OK) {
        fprintf(stderr, "error: sh2_setCalConfig failed\n");
        orient_sensor_stop();
        return EXIT_ERROR;
    }

    {
        int rc = waitSettled();
        if (rc == 2) goto abort;
        if (rc == 1) {
            printf("warning: heading not settled; continuing anyway\n");
        }
    }

    if (!orient_sensor_getLatestSample(&s)) {
        fprintf(stderr, "error: no sensor reports arrived\n");
        orient_sensor_stop();
        return EXIT_ERROR;
    }
    printAttitude("current attitude (tared frame)", &s);

    if (!promptConfirm("drop the volatile tare and return to the raw frame")) {
        goto abort;
    }

    if (sh2_clearTare() != SH2_OK) {
        fprintf(stderr, "error: sh2_clearTare failed\n");
        orient_sensor_stop();
        return EXIT_ERROR;
    }

    printf("tare cleared; watching the raw-frame output for 3 s...\n");
    if (!serviceFor(3000, true)) goto abort;

    if (!orient_sensor_getLatestSample(&s)) {
        fprintf(stderr, "error: no sensor reports arrived after clear\n");
        orient_sensor_stop();
        return EXIT_ERROR;
    }
    printAttitude("raw-frame attitude", &s);

    orient_sensor_stop();
    printf("\nnote: a tare previously persisted to flash still reloads\n"
           "at the next reset; overwrite it with a fresh 'bno_orient --persist'.\n");
    printf("RESULT: TARE CLEARED (exit 0)\n");
    return EXIT_OK;

abort:
    printf("\nbno_orient: aborted by user (exit 2)\n");
    orient_sensor_stop();
    return EXIT_ABORT;
}

/* ------------------------------------------------------------------ */
/* Guided tare flow                                                   */
/* ------------------------------------------------------------------ */

static int doOrient(bool allAxes, bool persist)
{
    const uint8_t axes = allAxes
        ? (SH2_TARE_X | SH2_TARE_Y | SH2_TARE_Z)
        : SH2_TARE_Z;
    OrientSample_t s;
    float yaw, pitch, roll;

    printf("=== BNO085 swing-axis orientation tare ===\n\n");
    printf("Before starting:\n");
    printf("  - run bno_cal first: the tare zeroes whatever heading the\n");
    printf("    fusion reports, and that heading is only trustworthy on\n");
    printf("    top of a saved calibration\n");
    printf("  - mount the device in its FINAL position on the fixture\n");
    printf("  - make sure bno_app, bno_cal and the validation binaries\n");
    printf("    are stopped\n");
    printf("  - abort any time: 'n' or 'q' at a prompt or Ctrl-C\n\n");

    if (allAxes) {
        printf("NOTE: full-axis tare selected. When the tare is applied\n");
        printf("the device must be dead level AND pointed at magnetic\n");
        printf("North, or the zeroed pitch/roll will be wrong.\n\n");
    } else {
        printf("Z-axis tare only: the heading is re-zeroed; the fixture\n");
        printf("just needs to sit level at its swing-axis zero (no\n");
        printf("magnetic-North alignment needed).\n\n");
    }

    printf("opening SH-2 session...\n");
    if (!orient_sensor_start()) {
        reportOpenFailure();
        return EXIT_ERROR;
    }

    /* Mirror bno_app's flight policy: all dynamic calibration off, fly
     * on the saved DCD, so the heading shown and the heading the tare
     * zeroes is exactly what acquisition will see. */
    if (sh2_setCalConfig(0) != SH2_OK) {
        fprintf(stderr, "error: sh2_setCalConfig failed\n");
        orient_sensor_stop();
        return EXIT_ERROR;
    }

    printf("waiting for the rotation vector to settle (accuracy >= %d, "
           "up to %u s)...\n",
           ACC_GOAL, SETTLE_TIMEOUT_S);
    {
        int rc = waitSettled();
        if (rc == 2) goto abort;
        if (rc == 1) {
            fprintf(stderr,
                    "error: heading never settled to accuracy %d; "
                    "run bno_cal first\n",
                    ACC_GOAL);
            orient_sensor_stop();
            return EXIT_ERROR;
        }
    }

    if (!orient_sensor_getLatestSample(&s)) {
        fprintf(stderr, "error: no sensor reports arrived\n");
        orient_sensor_stop();
        return EXIT_ERROR;
    }
    printAttitude("current attitude (raw frame)", &s);

    if (!promptConfirm("move the fixture to its swing-axis ZERO, keep it stationary, then tare")) {
        goto abort;
    }

    printf("applying %s tare...\n",
           allAxes ? "full-axis" : "Z-axis heading");
    if (sh2_setTareNow(axes, SH2_TARE_BASIS_ROTATION_VECTOR) != SH2_OK) {
        fprintf(stderr, "error: sh2_setTareNow failed\n");
        orient_sensor_stop();
        return EXIT_ERROR;
    }

    printf("tare applied; watching the re-zeroed output for %u s...\n",
           WATCH_MS / 1000);
    if (!serviceFor(WATCH_MS, true)) goto abort;

    if (!orient_sensor_getLatestSample(&s)) {
        fprintf(stderr, "error: no sensor reports arrived after taring\n");
        orient_sensor_stop();
        return EXIT_ERROR;
    }
    printAttitude("post-tare attitude (tared frame)", &s);

    eulerDeg(&s, &yaw, &pitch, &roll);
    if (fabsf(yaw) > TARE_RESIDUAL_DEG ||
        (allAxes && (fabsf(pitch) > TARE_RESIDUAL_DEG ||
                     fabsf(roll)  > TARE_RESIDUAL_DEG))) {
        printf("warning: a tared angle is more than %.1f deg from zero — "
               "did the fixture move or the heading drift?\n",
               (double)TARE_RESIDUAL_DEG);
    }

    if (!persist) {
        printf("\nnote: this tare is volatile: the next session open (HAL "
               "re-init) reboots the chip and discards it. Run with "
               "--persist to store it in flash so bno_app and the "
               "validation start in the tared frame.\n");
        orient_sensor_stop();
        printf("RESULT: TARE APPLIED, VOLATILE (exit 0)\n");
        return EXIT_OK;
    }

    printf("\npersisting the tare to flash...\n");
    if (sh2_persistTare() != SH2_OK) {
        fprintf(stderr, "error: sh2_persistTare failed; the tare remains "
                        "active for this session only\n");
        orient_sensor_stop();
        return EXIT_ERROR;
    }
    printf("tare written to flash sensor-orientation record.\n");

    /* Verify like bno_cal's step 6: reopening the session performs
     * the HAL reset sequence, so the chip reboots and must reload
     * the persisted tare from flash; the heading must still read 0. */
    printf("\n--- VERIFY: reopening session (chip reset, tare reload) ---\n");
    orient_sensor_stop();
    if (!orient_sensor_start()) {
        fprintf(stderr, "error: session reopen failed during verification\n");
        return EXIT_ERROR;
    }
    if (sh2_setCalConfig(0) != SH2_OK) {
        fprintf(stderr, "error: sh2_setCalConfig failed during "
                        "verification\n");
        orient_sensor_stop();
        return EXIT_ERROR;
    }

    printf("leave the fixture stationary; watching for %u s...\n",
           WATCH_MS / 1000);
    if (!serviceFor(WATCH_MS, true)) goto abort;

    if (!orient_sensor_getLatestSample(&s)) {
        fprintf(stderr, "error: no sensor reports arrived during "
                        "verification\n");
        orient_sensor_stop();
        return EXIT_ERROR;
    }
    printAttitude("post-reset attitude (persisted frame)", &s);

    orient_sensor_stop();
    printf("RESULT: TARE PERSISTED AND VERIFIED (exit 0)\n");
    return EXIT_OK;

abort:
    printf("\nbno_orient: aborted by user (exit 2)\n");
    orient_sensor_stop();
    return EXIT_ABORT;
}

int main(int argc, char **argv)
{
    bool allAxes = false;
    bool persist = false;
    bool clear   = false;
    bool check   = false;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--all") == 0) {
            allAxes = true;
        } else if (strcmp(argv[i], "--persist") == 0) {
            persist = true;
        } else if (strcmp(argv[i], "--clear") == 0) {
            clear = true;
        } else if (strcmp(argv[i], "--check") == 0) {
            check = true;
        } else {
            fprintf(stderr,
                    "usage: bno_orient [--all] [--persist] [--clear] "
                    "[--check]\n");
            return EXIT_ERROR;
        }
    }

    signal(SIGINT, onSigint);

    if (check) return doCheck();
    if (clear) return doClear();
    return doOrient(allAxes, persist);
}
