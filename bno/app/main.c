#define _POSIX_C_SOURCE 200809L

/*
 * The only BNO085 executable/session-loop owner.
 * No CSV, logger, publisher freshness, or second HAL owner in Phase 8.
 */
#include <stdbool.h>
#include <stdint.h>

#include "app/imu_cmd.h"

#define APP_LOOP_PERIOD_NS 1000000ull
#define APP_ACQUIRE_PERIOD_TICKS 10u

typedef enum {
    APP_ADAPTER_NONE = 0,
    APP_ADAPTER_CAL,
    APP_ADAPTER_TARE,
    APP_ADAPTER_CHECK
} AppAdapter_t;

typedef enum {
    APP_TURN_OK = 0,
    APP_TURN_STOPPED,
    APP_TURN_ERROR
} AppTurnStatus_t;

/*
 * Main-private test entry. These callbacks wrap top-level operations,
 * not sensor policy. A normal turn calls each applicable callback once.
 */
typedef struct {
    bool (*stop_requested)(void *);
    void (*adopt_stop)(void *);
    void (*session_service)(void *);
    ImuCmdIdentity_t (*active)(void *);
    bool (*stage_action)(void *, AppAdapter_t, uint64_t);
    void (*session_failed)(void *);
    bool (*post_tick)(void *, uint64_t);
    bool (*post_one_input)(void *, uint64_t);
    void (*command_service)(void *);
    bool (*render)(void *, uint64_t);
    bool (*acquisition_due)(void *);
    bool (*acquire_snapshot)(void *);
    int (*sleep_until)(void *);
} AppTurnOps_t;

static AppAdapter_t app_adapter_for(ImuCmdIdentity_t id)
{
    switch (id) {
    case IMU_CMD_ID_CALIBRATION:
    case IMU_CMD_ID_DCD_CLEAR:
        return APP_ADAPTER_CAL;
    case IMU_CMD_ID_TARE:
    case IMU_CMD_ID_TARE_CLEAR:
    case IMU_CMD_ID_TARE_CHECK:
        return APP_ADAPTER_TARE;
    case IMU_CMD_ID_CHECK:
    case IMU_CMD_ID_PROBE:
        return APP_ADAPTER_CHECK;
    case IMU_CMD_ID_NONE:
    case IMU_CMD_ID_SETTLE:
    case IMU_CMD_ID_ACQUISITION:
    default:
        return APP_ADAPTER_NONE;
    }
}

static AppTurnStatus_t app_owner_turn(const AppTurnOps_t *ops,
                                      void *context, uint64_t nowNs)
{
    ImuCmdIdentity_t id;

    /* A signal already adopted here cannot be followed by a pump. */
    if (ops->stop_requested(context)) {
        ops->adopt_stop(context);
        return APP_TURN_STOPPED;
    }

    ops->session_service(context);

    /* Also catch a stop that arrived during SH-2 service. */
    if (ops->stop_requested(context)) {
        ops->adopt_stop(context);
        return APP_TURN_STOPPED;
    }

    id = ops->active(context);
    if (!ops->stage_action(context, app_adapter_for(id), nowNs)) {
        ops->session_failed(context);
        return APP_TURN_ERROR;
    }
    if (!ops->post_tick(context, nowNs) ||
        !ops->post_one_input(context, nowNs)) {
        return APP_TURN_ERROR;
    }

    ops->command_service(context);
    if (!ops->render(context, nowNs)) {
        return APP_TURN_ERROR;
    }
    if (ops->acquisition_due(context) &&
        !ops->acquire_snapshot(context)) {
        return APP_TURN_ERROR;
    }

    /* Exactly one sleep call on each completed normal turn. */
    if (ops->sleep_until(context) < 0) {
        return APP_TURN_ERROR;
    }
    return APP_TURN_OK;
}

#ifndef IMU_APP_OWNER_TEST

#include <inttypes.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "rt/realtime.h"
#include "app/app_rt_policy.h"
#include "app/imu_cal.h"
#include "app/imu_cal_adapter.h"
#include "app/imu_check.h"
#include "app/imu_check_adapter.h"
#include "app/imu_cli.h"
#include "app/imu_console.h"
#include "app/imu_session.h"
#include "app/imu_tare.h"
#include "app/imu_tare_adapter.h"

#define APP_RT_PRIORITY 90
#define APP_LOOP_PERIOD_SEC 0.001
#define APP_PROGRESS_PERIOD_NS 500000000ull
#define APP_SETTLE_NS 300000000ull

static volatile sig_atomic_t s_signalStop;

typedef struct {
    ImuConsole_t console;
    ImuCmdPlan_t plan;
    bool sessionOpened;
    bool productionConfigured;
    bool settleStarted;
    bool settleDue;
    bool activeAtTurnStartWasAcquisition;
    uint64_t settleDeadlineNs;
    uint64_t lastProgressNs;
    uint64_t lastAcquisitionPrintNs;
    uint64_t observedFrames;
    uint64_t skippedDeadlines;
    uint64_t consoleDropsShown;
    unsigned acquisitionTicks;
    bool haveProgress;
    bool haveSample;
    ImuCmdProgress_t previousProgress;
    ImuSampleSnapshot_t latestSample;
    bool printed[IMU_CMD_ID_COUNT];
} AppContext_t;

static void signal_stop(int number)
{
    (void)number;
    s_signalStop = 1;
}

static bool install_stop_signals(void)
{
    struct sigaction action;

    memset(&action, 0, sizeof(action));
    action.sa_handler = signal_stop;
    sigemptyset(&action.sa_mask);
    return sigaction(SIGINT, &action, NULL) == 0 &&
           sigaction(SIGTERM, &action, NULL) == 0;
}

static bool monotonic_ns(uint64_t *out)
{
    struct timespec ts;

    if (out == NULL || clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return false;
    }
    *out = (uint64_t)ts.tv_sec * 1000000000ull +
           (uint64_t)ts.tv_nsec;
    return true;
}

static void print_usage(void)
{
    puts("usage: bno_app [--cal-imu [--clear]]"
         " [--tare-imu [--full|--check|--clear]]"
         " [--check-imu [--mask 0..255|0x00..0xFF]]"
         " [--duration 1..3600]");
    puts("       bno_app --help");
    puts("Order of operation families does not change stage order.");
    puts("--clear with both --cal-imu and --tare-imu is ambiguous.");
    puts("During a command: q ends that stage; type exact CLEAR when"
         " destructive confirmation is requested.");
    puts("Acquisition defaults to 10 seconds; duration affects only acquisition.");
}

static bool stop_requested(void *unused)
{
    (void)unused;
    return s_signalStop != 0;
}

static void adopt_stop(void *unused)
{
    ImuCmdEvent_t event;

    (void)unused;
    memset(&event, 0, sizeof(event));
    event.type = IMU_CMD_EVENT_PROCESS_STOP;
    (void)imu_cmd_post(&event);
}

static void session_failed(void *unused)
{
    ImuCmdEvent_t event;

    (void)unused;
    memset(&event, 0, sizeof(event));
    event.type = IMU_CMD_EVENT_SESSION_UNRESTORABLE;
    (void)imu_cmd_post(&event);
}

static void service_session(void *unused)
{
    (void)unused;
    imu_session_service();
}

static ImuCmdIdentity_t active_command(void *unused)
{
    ImuCmdProgress_t progress;

    (void)unused;
    if (!imu_cmd_get_progress(&progress)) {
        return IMU_CMD_ID_NONE;
    }
    return progress.active;
}

static bool stage_action(void *opaque, AppAdapter_t adapter,
                         uint64_t nowNs)
{
    AppContext_t *app = opaque;
    ImuSampleSnapshot_t snapshot;
    ImuCmdIdentity_t id = active_command(app);
    ImuCmdProgress_t machineProgress;

    app->activeAtTurnStartWasAcquisition =
        id == IMU_CMD_ID_ACQUISITION;

    if (!imu_session_get_snapshot(&snapshot) ||
        snapshot.readerState == IMU_READER_STATE_FAULTED ||
        snapshot.readerState == IMU_READER_STATE_CLOSED) {
        return false;
    }

    if (id == IMU_CMD_ID_SETTLE) {
        if (!app->settleStarted) {
            if (!app->productionConfigured) {
                if (!imu_session_configure_production(
                        app->plan.flightCalMask)) {
                    return false;
                }
                app->productionConfigured = true;
            }
            if (!imu_session_begin_settle()) {
                return false;
            }
            app->settleDeadlineNs = nowNs + APP_SETTLE_NS;
            app->settleStarted = true;
        }
        app->settleDue = nowNs >= app->settleDeadlineNs;
        return true;
    }

    switch (adapter) {
    case APP_ADAPTER_CAL:
        /* First coordinator tick initializes the machine. Do not pump
         * its request in that same owner turn. */
        if (!imu_cal_get_progress(&machineProgress) ||
            machineProgress.active != id || imu_cal_complete()) {
            return true;
        }
        return imu_cal_adapter_pump();
    case APP_ADAPTER_TARE:
        if (!imu_tare_get_progress(&machineProgress) ||
            machineProgress.active != id || imu_tare_complete()) {
            return true;
        }
        return imu_tare_adapter_pump();
    case APP_ADAPTER_CHECK:
        if (!imu_check_get_progress(&machineProgress) ||
            machineProgress.active != id || imu_check_complete()) {
            return true;
        }
        return imu_check_adapter_pump();
    case APP_ADAPTER_NONE:
    default:
        return true;
    }
}

static bool post_tick(void *unused, uint64_t nowNs)
{
    ImuCmdEvent_t event;

    (void)unused;
    memset(&event, 0, sizeof(event));
    event.type = IMU_CMD_EVENT_TICK;
    event.monotonicNs = nowNs;
    return imu_cmd_post(&event);
}

static bool post_one_input(void *opaque, uint64_t nowNs)
{
    AppContext_t *app = opaque;
    ImuConsoleInput_t token;
    ImuConsoleObservation_t observation;
    ImuCmdProgress_t progress;
    ImuCmdEvent_t event;

    (void)nowNs;

    if (app->settleDue) {
        if (!imu_session_mark_operational()) {
            return false;
        }
        memset(&event, 0, sizeof(event));
        event.type = IMU_CMD_EVENT_SETTLE_DONE;
        if (!imu_cmd_post(&event)) {
            return false;
        }
        app->settleDue = false;
    }

    if (!imu_console_observe(&app->console, &observation)) {
        return false;
    }
    if (observation.inputError) {
        /* Process-level I/O failure, not OPERATOR_Q and not a claim
         * that the SH-2 session failed. */
        return false;
    }
    if (!imu_console_dequeue(&app->console, &token)) {
        return true;
    }
    if (!imu_cmd_get_progress(&progress)) {
        return false;
    }
    if (!imu_console_translate(token, &progress, &event)) {
        return true; /* Irrelevant or malformed line is ignored. */
    }
    return imu_cmd_post(&event);
}

static void service_command(void *unused)
{
    (void)unused;
    imu_cmd_service();
}

static bool progress_changed(const ImuCmdProgress_t *a,
                             const ImuCmdProgress_t *b)
{
    return a->active != b->active ||
           a->requiredAction != b->requiredAction ||
           a->cal.phase != b->cal.phase ||
           a->tare.phase != b->tare.phase ||
           a->check.phase != b->check.phase ||
           a->check.requestedMaskValid != b->check.requestedMaskValid ||
           a->check.requestedMask != b->check.requestedMask ||
           a->check.actualMaskValid != b->check.actualMaskValid ||
           a->check.actualMask != b->check.actualMask ||
           a->check.gateReached != b->check.gateReached;
}

static bool render_outputs(void *opaque, uint64_t nowNs)
{
    AppContext_t *app = opaque;
    ImuCmdProgress_t progress;
    ImuCmdResult_t result;
    ImuConsoleObservation_t observation;
    char text[2048];
    unsigned id;

    if (!imu_cmd_get_progress(&progress)) {
        return false;
    }
    if (progress.active != IMU_CMD_ID_NONE &&
        progress.active != IMU_CMD_ID_SETTLE &&
        progress.active != IMU_CMD_ID_ACQUISITION &&
        (!app->haveProgress ||
         progress_changed(&progress, &app->previousProgress) ||
         nowNs - app->lastProgressNs >= APP_PROGRESS_PERIOD_NS)) {
        if (!imu_console_render_progress(&progress, text,
                                          sizeof(text)) ||
            fputs(text, stdout) == EOF) {
            return false;
        }
        app->lastProgressNs = nowNs;
    }
    app->previousProgress = progress;
    app->haveProgress = true;

    for (id = (unsigned)IMU_CMD_ID_CALIBRATION;
         id < (unsigned)IMU_CMD_ID_COUNT; ++id) {
        if (app->printed[id] ||
            !imu_cmd_get_result((ImuCmdIdentity_t)id, &result) ||
            result.state == IMU_CMD_STATE_NOT_REQUESTED ||
            result.state == IMU_CMD_STATE_RUNNING) {
            continue;
        }
        if (!imu_console_render_result(&result,
                                       imu_cmd_do_not_acquire(),
                                       text, sizeof(text)) ||
            fputs(text, stdout) == EOF) {
            return false;
        }
        app->printed[id] = true;
        if (result.restoredProduction) {
            app->productionConfigured = true;
        }
    }

    if (!imu_console_observe(&app->console, &observation)) {
        return false;
    }
    if (observation.droppedLines != app->consoleDropsShown) {
        fprintf(stderr, "main: consoleDroppedLines=%" PRIu64 "\n",
                observation.droppedLines);
        app->consoleDropsShown = observation.droppedLines;
    }
    if (progress.active == IMU_CMD_ID_ACQUISITION &&
        app->haveSample &&
        (app->lastAcquisitionPrintNs == 0ull ||
         nowNs - app->lastAcquisitionPrintNs >=
             APP_PROGRESS_PERIOD_NS)) {
        const ImuSampleSnapshot_t *s = &app->latestSample;

        if (printf("acquisition epoch=%" PRIu32
                   " validMask=0x%02x observations=%" PRIu64,
                   s->configurationEpoch, s->validMask,
                   app->observedFrames) < 0) {
            return false;
        }
        if (s->validMask & IMU_GROUP_BIT_ROTATION) {
            if (printf(" yaw=%.3f pitch=%.3f roll=%.3f",
                       s->yaw, s->pitch, s->roll) < 0) {
                return false;
            }
        } else if (fputs(" orientation=not_seen", stdout) == EOF) {
            return false;
        }
        if (s->validMask & IMU_GROUP_BIT_ACCEL) {
            if (printf(" ax=%.3f ay=%.3f az=%.3f",
                       s->ax, s->ay, s->az) < 0) {
                return false;
            }
        } else if (fputs(" linear_accel=not_seen", stdout) == EOF) {
            return false;
        }
        if (s->validMask & IMU_GROUP_BIT_GYRO) {
            if (printf(" gx=%.3f gy=%.3f gz=%.3f",
                       s->gx, s->gy, s->gz) < 0) {
                return false;
            }
        } else if (fputs(" calibrated_gyro=not_seen", stdout) == EOF) {
            return false;
        }
        if (putchar('\n') == EOF) {
            return false;
        }
        app->lastAcquisitionPrintNs = nowNs;
    }
    return true;
}

static bool acquisition_due(void *opaque)
{
    AppContext_t *app = opaque;
    ImuSampleSnapshot_t snapshot;

    if (!app->activeAtTurnStartWasAcquisition ||
        imu_cmd_do_not_acquire() ||
        active_command(app) != IMU_CMD_ID_ACQUISITION ||
        !imu_session_get_snapshot(&snapshot) ||
        snapshot.version != IMU_SAMPLE_CONTRACT_VERSION ||
        snapshot.readerState != IMU_READER_STATE_OPERATIONAL) {
        app->acquisitionTicks = 0u;
        return false;
    }
    ++app->acquisitionTicks;
    return app->acquisitionTicks % APP_ACQUIRE_PERIOD_TICKS == 0u;
}

static bool acquire_snapshot(void *opaque)
{
    AppContext_t *app = opaque;

    if (!imu_session_get_snapshot(&app->latestSample) ||
        app->latestSample.version != IMU_SAMPLE_CONTRACT_VERSION ||
        app->latestSample.readerState !=
            IMU_READER_STATE_OPERATIONAL) {
        return false;
    }
    app->haveSample = true;
    ++app->observedFrames;
    return true;
}

static int sleep_until(void *opaque)
{
    AppContext_t *app = opaque;
    int rc = RT_SleepUntil(APP_LOOP_PERIOD_SEC);

    if (rc > 0) {
        app->skippedDeadlines += (uint64_t)rc;
    }
    /* A signal-interrupted sleep is handled by the stop guard next turn. */
    if (rc < 0 && s_signalStop) {
        return 0;
    }
    return rc;
}

static const AppTurnOps_t s_turnOps = {
    stop_requested,
    adopt_stop,
    service_session,
    active_command,
    stage_action,
    session_failed,
    post_tick,
    post_one_input,
    service_command,
    render_outputs,
    acquisition_due,
    acquire_snapshot,
    sleep_until
};

int main(int argc, char **argv)
{
    ImuCliParse_t parsed;
    AppContext_t app;
    RT_StartStatus_t rtStatus;
    AppRtPolicy_t rtPolicy;
    AppTurnStatus_t turnStatus = APP_TURN_OK;
    uint64_t nowNs;
    int exitStatus = 0;
    bool consoleReady = false;
    bool consoleStarted = false;

    memset(&app, 0, sizeof(app));
    if (imu_cli_parse(argc, argv, &parsed) != IMU_CLI_STATUS_OK) {
        if (parsed.status == IMU_CLI_STATUS_HELP) {
            print_usage();
            return 0;
        }
        fprintf(stderr, "main: CLI error=%d argvIndex=%d\n",
                (int)parsed.error, parsed.errorIndex);
        return 2;
    }
    app.plan = parsed.plan;

    if (!install_stop_signals()) {
        fprintf(stderr, "main: signal setup failed\n");
        return 1;
    }
    if (!imu_console_init(&app.console, STDIN_FILENO)) {
        fprintf(stderr, "main: console init failed\n");
        return 1;
    }
    consoleReady = true;
    if (!imu_console_start(&app.console)) {
        fprintf(stderr, "main: console worker start failed\n");
        exitStatus = 1;
        goto cleanup;
    }
    consoleStarted = true;

    if (!imu_session_open()) {
        fprintf(stderr, "main: imu_session_open failed\n");
        exitStatus = 1;
        goto cleanup;
    }
    app.sessionOpened = true;

    if (!imu_cmd_init(&app.plan)) {
        fprintf(stderr, "main: illegal coordinator plan reason=%d\n",
                (int)imu_cmd_init_reason());
        exitStatus = 1;
        goto cleanup;
    }

    rtStatus = StartRT(APP_RT_PRIORITY, APP_LOOP_PERIOD_SEC);
    rtPolicy = app_rt_policy_from_start(rtStatus);
    if (rtPolicy == APP_RT_POLICY_FATAL) {
        fprintf(stderr, "main: fatal StartRT status=%u\n",
                (unsigned)rtStatus);
        exitStatus = 1;
        goto cleanup;
    }
    if (rtPolicy == APP_RT_POLICY_WARN_AND_CONTINUE) {
        fprintf(stderr, "main: StartRT warning status=%u;"
                        " continuing with fallback\n",
                (unsigned)rtStatus);
    }

    while (!imu_cmd_plan_complete()) {
        if (!monotonic_ns(&nowNs)) {
            fprintf(stderr, "main: monotonic clock failed\n");
            exitStatus = 1;
            break;
        }
        turnStatus = app_owner_turn(&s_turnOps, &app, nowNs);
        if (turnStatus != APP_TURN_OK) {
            if (turnStatus == APP_TURN_ERROR) {
                fprintf(stderr, "main: owner turn failed\n");
                exitStatus = 1;
            }
            break;
        }
    }

    /* A signal may have abandoned a command before the normal render step. */
    if (turnStatus == APP_TURN_STOPPED && !render_outputs(&app, nowNs)) {
        exitStatus = 1;
    }
    if (imu_cmd_do_not_acquire() && !imu_cmd_process_stop_seen()) {
        exitStatus = 1;
    }

cleanup:
    if (consoleStarted) {
        imu_console_request_stop(&app.console);
        if (!imu_console_join(&app.console)) {
            fprintf(stderr, "main: console worker join failed\n");
            exitStatus = 1;
        }
    }
    if (consoleReady) {
        imu_console_destroy(&app.console);
    }
    if (app.sessionOpened) {
        imu_session_close();
    }
    printf("main: shutdown status=%d observations=%" PRIu64
           " skippedDeadlines=%" PRIu64 "\n",
           exitStatus, app.observedFrames, app.skippedDeadlines);
    return exitStatus;
}
#endif /* IMU_APP_OWNER_TEST */
