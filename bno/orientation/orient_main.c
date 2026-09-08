/*
 * orientation/orient_main.c — BNO085 swing-axis tare CLI tool.
 *
 * Tares the BNO085 sensor orientation so that the current physical
 * attitude corresponds to the zero reference frame for swing-axis
 * validation or operation.
 *
 * Commands:
 *   bno_orient               interactive tare (prompts to align and confirm)
 *   bno_orient --check       continuous live read-only inspection; terminates
 *                            only on Ctrl-C
 *   bno_orient --persist     tare and write to sensor-orientation flash record
 *   bno_orient --clear       clear active tare and revert to default frame
 *   bno_orient --all         tare full 3-axis rotation (default is Z/heading only)
 *
 * Exit codes:
 *   0  success
 *   1  error
 *   2  aborted by user
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "orient_sensor.h"

#define ACC_GOAL          2
#define SERVICE_LOOP_US   1000
#define DISPLAY_PERIOD_US 500000

#define EXIT_OK     0
#define EXIT_ERROR  1
#define EXIT_ABORT  2

static volatile sig_atomic_t sAbort = 0;

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

static void eulerDeg(const OrientSample_t *s, float *yaw, float *pitch, float *roll)
{
    double q0 = s->quatW;
    double q1 = s->quatX;
    double q2 = s->quatY;
    double q3 = s->quatZ;

    double droll = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
    double sinp = 2.0 * (q0 * q2 - q3 * q1);
    double dpitch;
    if (fabs(sinp) >= 1.0)
        dpitch = copysign(M_PI / 2.0, sinp);
    else
        dpitch = asin(sinp);
    double dyaw = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));

    *yaw = (float)(dyaw * 180.0 / M_PI);
    *pitch = (float)(dpitch * 180.0 / M_PI);
    *roll = (float)(droll * 180.0 / M_PI);
}

static void printAttitude(const char *label, const OrientSample_t *s)
{
    float yaw, pitch, roll;
    eulerDeg(s, &yaw, &pitch, &roll);
    printf("%s: q=[%.4f, %.4f, %.4f, %.4f]  yaw %.2f  pitch %.2f  roll %.2f deg  "
           "(rv accuracy %u, %s)\n",
           label, (double)s->quatW, (double)s->quatX, (double)s->quatY, (double)s->quatZ,
           (double)yaw, (double)pitch, (double)roll,
           s->rvAccuracy, accName(s->rvAccuracy));
}

static void printLiveLine(const OrientSample_t *s)
{
    float yaw, pitch, roll;
    eulerDeg(s, &yaw, &pitch, &roll);
    printf("  q=[%6.3f,%6.3f,%6.3f,%6.3f] yaw %7.2f pitch %6.2f roll %6.2f deg [rv acc %u]  \r",
           (double)s->quatW, (double)s->quatX, (double)s->quatY, (double)s->quatZ,
           (double)yaw, (double)pitch, (double)roll, s->rvAccuracy);
    fflush(stdout);
}

static void reportOpenFailure(void)
{
    fprintf(stderr, "error: sensor_orient_start failed (BNO085/SPI)\n");
    fprintf(stderr, "hint: is bno_app or another sh2 consumer still "
                    "running? one HAL instance per process\n");
}

static bool promptEnter(const char *prompt)
{
    char buf[16];
    printf("%s\n  [y/N]: ", prompt);
    fflush(stdout);
    if (sAbort) return false;
    if (fgets(buf, sizeof(buf), stdin) == NULL) return false;
    if (sAbort) return false;
    return (buf[0] == 'y' || buf[0] == 'Y');
}

static bool serviceFor(unsigned duration_ms, bool live)
{
    uint64_t tEnd = hostNowUs() + (uint64_t)duration_ms * 1000ULL;
    uint64_t tLastDisplay = 0;
    OrientSample_t s;

    while (hostNowUs() < tEnd) {
        if (sAbort) return false;
        sensor_orient_service();
        if (sensor_orient_getLatestSample(&s)) {
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

static int doCheck(void)
{
    OrientSample_t s;

    printf("bno_orient --check: opening session (read-only inspection)...\n");
    if (!sensor_orient_start()) {
        reportOpenFailure();
        return EXIT_ERROR;
    }

    if (sh2_setCalConfig(0) != SH2_OK) {
        fprintf(stderr, "error: sh2_setCalConfig failed\n");
        sensor_orient_stop();
        return EXIT_ERROR;
    }

    printf("monitoring rotation vector live (press Ctrl+C to terminate)...\n");
    uint64_t tLastDisplay = 0;
    bool gotSample = false;

    while (!sAbort) {
        sensor_orient_service();
        if (sensor_orient_getLatestSample(&s)) {
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
        sensor_orient_stop();
        return EXIT_ERROR;
    }

    printAttitude("attitude", &s);
    if (!s.haveRv || s.rvAccuracy < ACC_GOAL) {
        printf("warning: rotation vector accuracy below %d; heading is "
               "not settled - run bno_cal before taring\n", ACC_GOAL);
    }
    sensor_orient_stop();
    return EXIT_OK;
}

static int doClear(void)
{
    printf("bno_orient --clear: clearing active tare...\n");
    if (!sensor_orient_start()) {
        reportOpenFailure();
        return EXIT_ERROR;
    }

    if (sh2_clearTare() != SH2_OK) {
        fprintf(stderr, "error: sh2_clearTare failed\n");
        sensor_orient_stop();
        return EXIT_ERROR;
    }

    printf("Active tare cleared. Reverting to raw frame.\n");
    sensor_orient_stop();
    return EXIT_OK;
}

static int doTare(bool persist, bool allAxes)
{
    OrientSample_t s;

    printf("====================================================\n");
    printf("  BNO085 Swing-Axis Tare (%s)\n", allAxes ? "All Axes" : "Z/Heading Only");
    printf("====================================================\n\n");

    if (!sensor_orient_start()) {
        reportOpenFailure();
        return EXIT_ERROR;
    }

    sh2_setCalConfig(0);

    printf("Align fixture to mechanical zero reference.\n");
    if (!promptEnter("Ready to tare?")) {
        printf("Aborted by user.\n");
        sensor_orient_stop();
        return EXIT_ABORT;
    }

    printf("Settling heading (2 seconds)...\n");
    if (!serviceFor(2000, true)) {
        sensor_orient_stop();
        return EXIT_ABORT;
    }

    uint8_t axes = allAxes ? (SH2_TARE_X | SH2_TARE_Y | SH2_TARE_Z) : SH2_TARE_Z;
    printf("Executing tare command...\n");
    if (sh2_setTareNow(axes, SH2_TARE_BASIS_ROTATION_VECTOR) != SH2_OK) {
        fprintf(stderr, "error: sh2_setTareNow failed\n");
        sensor_orient_stop();
        return EXIT_ERROR;
    }

    if (persist) {
        printf("Persisting tare transformation to sensor-orientation flash record...\n");
        if (sh2_persistTare() != SH2_OK) {
            fprintf(stderr, "error: sh2_persistTare failed\n");
            sensor_orient_stop();
            return EXIT_ERROR;
        }
        printf("Tare persisted successfully.\n");
    }

    /* Verify post-tare attitude */
    serviceFor(500, false);
    if (sensor_orient_getLatestSample(&s)) {
        printAttitude("post-tare attitude", &s);
    }

    sensor_orient_stop();
    printf("Tare operation complete.\n");
    return EXIT_OK;
}

int main(int argc, char **argv)
{
    bool check = false;
    bool persist = false;
    bool clear = false;
    bool allAxes = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--check") == 0) {
            check = true;
        } else if (strcmp(argv[i], "--persist") == 0) {
            persist = true;
        } else if (strcmp(argv[i], "--clear") == 0) {
            clear = true;
        } else if (strcmp(argv[i], "--all") == 0) {
            allAxes = true;
        } else {
            fprintf(stderr, "usage: bno_orient [--check] [--persist] [--clear] [--all]\n");
            return EXIT_ERROR;
        }
    }

    signal(SIGINT, onSigint);

    if (check)   return doCheck();
    if (clear)   return doClear();
    return doTare(persist, allAxes);
}
