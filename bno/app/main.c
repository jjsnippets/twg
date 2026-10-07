#define _POSIX_C_SOURCE 200809L

/*
 * The only BNO085 executable/session-loop owner. Phase 9 keeps publication
 * scheduling here while the logger worker owns CSV formatting and I/O.
 */
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_cmd.h"
#include "app/imu_console.h"
#include "app/imu_contract.h"
#include "app/imu_publish.h"
#include "app/imu_session.h"

#define APP_LOOP_PERIOD_NS IMU_PUBLISH_SERVICE_PERIOD_NS

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
    bool (*publication_step)(void *, uint64_t);
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

typedef struct {
    ImuCmdIdentity_t stage;
    uint64_t flushTicket;
} AppInputGate_t;

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
    if (!ops->publication_step(context, nowNs)) {
        return APP_TURN_ERROR;
    }

    /* Exactly one sleep call on each completed normal turn. */
    if (ops->sleep_until(context) < 0) {
        return APP_TURN_ERROR;
    }
    return APP_TURN_OK;
}

static bool app_input_stage_ready(AppInputGate_t *gate,
                                  ImuConsole_t *console,
                                  ImuCmdIdentity_t stage)
{
    if (gate == NULL || console == NULL) {
        return false;
    }
    if (gate->stage != stage) {
        gate->stage = stage;
        gate->flushTicket =
            imu_console_request_stage_flush(console);
    }
    return imu_console_stage_flush_complete(console,
                                            gate->flushTicket);
}

static bool app_format_acquisition(const ImuSampleSnapshot_t *s,
                                   uint64_t publications,
                                   char *out, size_t capacity)
{
    size_t used = 0u;
    int n;

    if (s == NULL || out == NULL || capacity == 0u ||
        s->version != IMU_SAMPLE_CONTRACT_VERSION) {
        return false;
    }
    out[0] = '\0';
#define APP_APPEND(...)                                                   \
    do {                                                                  \
        n = snprintf(out + used, capacity - used, __VA_ARGS__);          \
        if (n < 0 || (size_t)n >= capacity - used) return false;         \
        used += (size_t)n;                                                \
    } while (0)

    APP_APPEND("acquisition epoch=%" PRIu32
               " validMask=0x%02x publications=%" PRIu64,
               s->configurationEpoch, s->validMask, publications);
    if (s->validMask & IMU_GROUP_BIT_ROTATION) {
        APP_APPEND(" yaw=%.3f pitch=%.3f roll=%.3f",
                   s->yaw, s->pitch, s->roll);
    } else {
        APP_APPEND(" orientation=not_seen");
    }
    if (s->validMask & IMU_GROUP_BIT_ACCEL) {
        APP_APPEND(" ax=%.3f ay=%.3f az=%.3f",
                   s->ax, s->ay, s->az);
    } else {
        APP_APPEND(" linear_accel=not_seen");
    }
    if (s->validMask & IMU_GROUP_BIT_GYRO) {
        APP_APPEND(" gx=%.3f gy=%.3f gz=%.3f",
                   s->gx, s->gy, s->gz);
    } else {
        APP_APPEND(" calibrated_gyro=not_seen");
    }
    APP_APPEND("\n");
#undef APP_APPEND
    return true;
}

/*
 * Presentation-only: style instruction lines without changing the
 * renderer's plain R8 text or placing escape codes in captured logs.
 * Continuation lines inherit GUIDE/WARNING style up to the data line.
 */
static bool app_style_progress(const char *source, bool useColor,
                               char *out, size_t capacity)
{
    static const char cyan[] = "\x1b[1;36m";
    static const char yellow[] = "\x1b[1;33m";
    static const char reset[] = "\x1b[0m";
    const char *line;
    const char *style = NULL;
    size_t used = 0u;

    if (source == NULL || out == NULL || capacity == 0u) {
        return false;
    }
    out[0] = '\0';

    for (line = source; *line != '\0'; ) {
        const char *end = strchr(line, '\n');
        size_t length = end != NULL
                      ? (size_t)(end - line) + 1u : strlen(line);
        size_t prefixLength;
        size_t resetLength;
        size_t required;

        if (strncmp(line, "R8 ", 3u) == 0 ||
            strncmp(line, "cal pose=", 9u) == 0 ||
            strncmp(line, "tare axes=", 10u) == 0 ||
            strncmp(line, "check ", 6u) == 0) {
            style = NULL;
        } else if (strncmp(line, "GUIDE:", 6u) == 0) {
            style = cyan;
        } else if (strncmp(line, "WARNING:", 8u) == 0) {
            style = yellow;
        }

        prefixLength = useColor && style != NULL
                     ? strlen(style) : 0u;
        resetLength = prefixLength != 0u
                    ? sizeof(reset) - 1u : 0u;
        required = prefixLength + length + resetLength + 1u;
        if (used >= capacity || required > capacity - used) {
            return false;
        }

        if (prefixLength != 0u) {
            memcpy(out + used, style, prefixLength);
            used += prefixLength;
        }
        memcpy(out + used, line, length);
        used += length;
        if (resetLength != 0u) {
            memcpy(out + used, reset, resetLength);
            used += resetLength;
        }
        out[used] = '\0';
        line += length;
    }
    return true;
}

static bool app_retain_check_diagnostics(
    ImuCheckDiagnostics_t *retained,
    const ImuCheckDiagnostics_t *candidate,
    uint32_t expectedEpoch,
    ImuSessionCheckMode_t expectedMode)
{
    if (retained == NULL || candidate == NULL ||
        candidate->version != IMU_CHECK_DIAGNOSTICS_VERSION ||
        !candidate->eligible || expectedEpoch == 0u ||
        candidate->configurationEpoch != expectedEpoch ||
        candidate->mode != expectedMode ||
        (expectedMode != IMU_SESSION_CHECK_MODE_CHECK &&
         expectedMode != IMU_SESSION_CHECK_MODE_PROBE)) {
        return false;
    }
    *retained = *candidate;
    return true;
}

static bool app_format_check_diagnostics(
    const ImuCheckDiagnostics_t *d, char *out, size_t capacity)
{
    const ImuCheckReportDiagnostic_t *reports[4];
    static const char *const names[4] = {
        "mag", "accel", "gyro", "rv"
    };
    static const unsigned requestedHz[4] = {
        IMU_CHECK_MAG_RATE_HZ, IMU_CHECK_SLOW_RATE_HZ,
        IMU_CHECK_SLOW_RATE_HZ, IMU_CHECK_SLOW_RATE_HZ
    };
    size_t used;
    unsigned i;
    int n;

    if (out == NULL || capacity == 0u) {
        return false;
    }
    out[0] = '\0';
    if (d == NULL ||
        d->version != IMU_CHECK_DIAGNOSTICS_VERSION ||
        !d->eligible || d->configurationEpoch == 0u ||
        (d->mode != IMU_SESSION_CHECK_MODE_CHECK &&
         d->mode != IMU_SESSION_CHECK_MODE_PROBE)) {
        return false;
    }

    reports[0] = &d->mag;
    reports[1] = &d->accel;
    reports[2] = &d->gyro;
    reports[3] = &d->rv;
    n = snprintf(out, capacity,
                 "checkReceipt version=%" PRIu32
                 " mode=%s epoch=%" PRIu32
                 " snapshot=retained_last_eligible"
                 " clock=host_decode_ns\n",
                 d->version,
                 d->mode == IMU_SESSION_CHECK_MODE_CHECK
                     ? "CHECK" : "PROBE",
                 d->configurationEpoch);
    if (n < 0 || (size_t)n >= capacity) {
        out[0] = '\0';
        return false;
    }
    used = (size_t)n;

    for (i = 0u; i < 4u; ++i) {
        const ImuCheckReportDiagnostic_t *r = reports[i];

        n = snprintf(out + used, capacity - used,
                     "checkReceipt report=%s requestedHz=%u"
                     " epochDecodes=%" PRIu64
                     " processDecodes=%" PRIu64
                     " hostTimesValid=%u"
                     " firstHostDecodeNs=%" PRIu64
                     " latestHostDecodeNs=%" PRIu64 "\n",
                     names[i], requestedHz[i],
                     r->epochDecodeCount, r->processDecodeCount,
                     r->hostTimesValid ? 1u : 0u,
                     r->firstHostDecodeNs, r->latestHostDecodeNs);
        if (n < 0 || (size_t)n >= capacity - used) {
            out[0] = '\0';
            return false;
        }
        used += (size_t)n;
    }
    return true;
}

static void app_count_publication(ImuRunStats_t *stats,
                                  const ImuPublicationRecord_t *record)
{
    unsigned i;

    if (stats == NULL || record == NULL) {
        return;
    }
    ++stats->publicationsAttempted;
    if (record->validMask == IMU_GROUP_MASK_REQUIRED) {
        ++stats->publicationsAllValid;
    }
    if (record->freshMask == IMU_GROUP_MASK_REQUIRED) {
        ++stats->publicationsAllFresh;
    } else if (record->freshMask != 0u) {
        ++stats->publicationsPartiallyFresh;
    }
    if (record->notReady != 0u) {
        ++stats->publicationsNotReady;
    }
    stats->deadlineMisses += record->deadlineMissed != 0u ? 1u : 0u;
    stats->gatesSkipped += record->gatesSkippedBefore;
    for (i = 0u; i < IMU_PUBLISH_GROUP_COUNT; ++i) {
        uint8_t bit = (uint8_t)IMU_PUBLISH_GROUP_BIT(i);

        if ((record->staleMask & bit) != 0u) {
            ++stats->staleByGroup[i];
        }
        if ((record->multiUpdateMask & bit) != 0u) {
            ++stats->multiUpdateByGroup[i];
        }
    }
}

static bool app_publication_turn_eligible(
    bool activeAtTurnStartWasAcquisition, ImuCmdIdentity_t current,
    const ImuCmdResult_t *acquisition)
{
    if (current == IMU_CMD_ID_ACQUISITION) {
        return true;
    }
    return activeAtTurnStartWasAcquisition && acquisition != NULL &&
           acquisition->state == IMU_CMD_STATE_SUCCEEDED &&
           acquisition->reason == IMU_CMD_REASON_OK;
}

static ImuTerminationReason_t app_termination_reason(
    bool loggerFailure, bool startupFailure, bool ownerFailure,
    bool sessionUnrecovered, bool processStop, bool windowOpened)
{
    if (loggerFailure) {
        return IMU_TERM_REASON_LOGGER_FAILURE;
    }
    if (startupFailure) {
        return IMU_TERM_REASON_STARTUP_FAILURE;
    }
    if (ownerFailure) {
        return IMU_TERM_REASON_OWNER_FAILURE;
    }
    if (sessionUnrecovered) {
        return IMU_TERM_REASON_SESSION_UNRECOVERED;
    }
    if (processStop) {
        return IMU_TERM_REASON_PROCESS_STOP;
    }
    if (!windowOpened) {
        return IMU_TERM_REASON_NO_ACQUISITION;
    }
    return IMU_TERM_REASON_COMPLETED;
}

/*
 * CSV archive completeness. A drained, undamaged archive is complete when
 * the window ran to its end, or when a process stop ended it early after at
 * least one row. The stop reason stays in R12; it explains the shorter run.
 */
static bool app_csv_finalizes_complete(ImuTerminationReason_t reason,
                                       bool windowOpened, bool windowClosed,
                                       bool loggerStarted, bool drained,
                                       uint8_t writeFailed,
                                       uint32_t shutdownErrors,
                                       uint64_t publications)
{
    bool ranToEnd = reason == IMU_TERM_REASON_COMPLETED && windowClosed;
    bool stoppedWithRows = reason == IMU_TERM_REASON_PROCESS_STOP &&
                           windowOpened && publications != 0u;

    return (ranToEnd || stoppedWithRows) && loggerStarted && drained &&
           writeFailed == 0u && shutdownErrors == 0u;
}

#ifndef IMU_APP_OWNER_TEST

#include <inttypes.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
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
#include "app/imu_logfile.h"
#include "app/imu_logger.h"
#include "app/imu_meta.h"
#include "app/imu_session.h"
#include "app/imu_tare.h"
#include "app/imu_tare_adapter.h"

#define APP_RT_PRIORITY 90
#define APP_LOOP_PERIOD_SEC 0.001
#define APP_PROGRESS_PERIOD_NS 500000000ull
#define APP_SETTLE_NS 300000000ull
#define APP_LOGGER_DRAIN_TIMEOUT_NS 5000000000ull
#define APP_CONSOLE_PUBLICATION_PERIOD 50ull

static volatile sig_atomic_t s_signalStop;
static volatile sig_atomic_t s_signalNumber;
static ImuLogger_t s_logger;
static char s_metaJson[IMU_META_JSON_CAPACITY];

typedef struct {
    ImuConsole_t console;
    AppInputGate_t inputGate;
    ImuCmdPlan_t plan;
    ImuPublisherState_t publisher;
    ImuPublishGate_t publishGate;
    ImuRunStats_t stats;
    ImuLogfile_t logfile;
    char filenameUtc[IMU_META_UTC_STAMP_CAPACITY];
    bool colorInstructions;
    bool sessionOpened;
    bool productionConfigured;
    bool settleStarted;
    bool settleDue;
    bool activeAtTurnStartWasAcquisition;
    bool loggerInitialized;
    bool logfileOpen;
    bool loggerStarted;
    bool outputAttempted;
    bool windowOpened;
    bool windowClosed;
    bool windowJustOpened;
    bool loggerFailure;
    bool startupFailure;
    bool ownerFailure;
    uint64_t settleDeadlineNs;
    uint64_t lastProgressNs;
    uint64_t lastConsolePublication;
    uint64_t consoleDropsShown;
    uint64_t nextTurnTicks;
    bool haveProgress;
    bool haveSample;
    bool postClearDiagnostic;
    bool postClearNoticeShown;
    ImuCmdProgress_t previousProgress;
    ImuSampleSnapshot_t latestSample;
    ImuCheckDiagnostics_t checkDiagnostics[IMU_CMD_ID_COUNT];
    bool haveCheckDiagnostics[IMU_CMD_ID_COUNT];
    bool printed[IMU_CMD_ID_COUNT];
} AppContext_t;

static void signal_stop(int number)
{
    s_signalNumber = number;
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

static bool local_wall_clock_now(ImuMetaUtc_t *out)
{
    struct timespec ts;
    struct tm local;

    if (out == NULL || clock_gettime(CLOCK_REALTIME, &ts) != 0) {
        return false;
    }

    /* Filename stamp is the system's local time; no offset is encoded. */
    tzset();
    if (localtime_r(&ts.tv_sec, &local) == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->year = (uint16_t)(local.tm_year + 1900);
    out->month = (uint8_t)(local.tm_mon + 1);
    out->day = (uint8_t)local.tm_mday;
    out->hour = (uint8_t)local.tm_hour;
    out->minute = (uint8_t)local.tm_min;
    out->second = (uint8_t)local.tm_sec;
    return true;
}

static void metadata_base(AppContext_t *app, ImuRunMetadata_t *metadata,
                          ImuMetaCompletion_t completion)
{
    memset(metadata, 0, sizeof(*metadata));
    metadata->version = IMU_RUN_METADATA_CONTRACT_VERSION;
    metadata->metaSchemaVersion = IMU_META_SCHEMA_VERSION;
    metadata->csvSchemaVersion = IMU_CSV_SCHEMA_VERSION;
    metadata->completion = completion;
    metadata->tLateNs = IMU_PUBLISH_T_LATE_NS;
    metadata->servicePeriodNs = IMU_PUBLISH_SERVICE_PERIOD_NS;
    metadata->publicationPeriodNs =
        IMU_PUBLISH_SERVICE_PERIOD_NS * IMU_PUBLISH_GATE_TICKS;
    metadata->settleDurationNs = APP_SETTLE_NS;
    metadata->stats = app->stats;
    (void)imu_logger_stats(&s_logger, &metadata->logger);
    (void)snprintf(metadata->baseName, sizeof(metadata->baseName), "%s",
                   imu_logfile_base_name(&app->logfile));
    (void)snprintf(metadata->filenameUtc, sizeof(metadata->filenameUtc), "%s",
                   app->filenameUtc);
}

static void collect_results(
    ImuCmdResult_t storage[IMU_CMD_ID_COUNT],
    const ImuCmdResult_t *results[IMU_CMD_ID_COUNT])
{
    unsigned i;

    memset(storage, 0, sizeof(ImuCmdResult_t) * IMU_CMD_ID_COUNT);
    memset(results, 0, sizeof(*results) * IMU_CMD_ID_COUNT);
    for (i = (unsigned)IMU_CMD_ID_CALIBRATION;
         i < (unsigned)IMU_CMD_ID_COUNT; ++i) {
        if (imu_cmd_get_result((ImuCmdIdentity_t)i, &storage[i])) {
            results[i] = &storage[i];
        }
    }
}

static bool write_running_metadata(AppContext_t *app)
{
    ImuRunMetadata_t metadata;
    size_t length;

    metadata_base(app, &metadata, IMU_META_COMPLETION_RUNNING);
    length = imu_meta_format_json(&metadata, &app->plan, NULL,
                                  s_metaJson, sizeof(s_metaJson));
    return length != 0u &&
           imu_logfile_write_running(&app->logfile, s_metaJson, length) ==
               IMU_LOGFILE_OK;
}

static void print_usage(void)
{
    puts("usage: bno_app [--cal-imu [--clear]]\n"
         " [--tare-imu [--full|--check|--clear]]\n"
         " [--check-imu [--mask 0..255|0x00..0xFF]]\n"
         " [--duration 1..3600]\n\n");
    puts("       bno_app --help");
    puts("Order of operation families does not change stage order.");
    puts("One --clear with both --cal-imu and --tare-imu clears"
         " DCD first, then active/saved tare.");
    puts("Each clear stage requires its own fresh CLEAR confirmation.");
    puts("After clearing, acquisition is post-clear diagnostic data,"
         " not a calibrated flight-data verdict.");
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
        snapshot.readerState == IMU_READER_STATE_CLOSED ||
        (snapshot.readerState == IMU_READER_STATE_FAULTED &&
         id != IMU_CMD_ID_ACQUISITION)) {
        return false;
    }

    if (id == IMU_CMD_ID_CHECK || id == IMU_CMD_ID_PROBE) {
        ImuCheckDiagnostics_t candidate;
        ImuSessionCheckMode_t mode =
            id == IMU_CMD_ID_CHECK
                ? IMU_SESSION_CHECK_MODE_CHECK
                : IMU_SESSION_CHECK_MODE_PROBE;

        if (imu_session_get_check_diagnostics(&candidate) &&
            app_retain_check_diagnostics(
                &app->checkDiagnostics[id], &candidate,
                snapshot.configurationEpoch, mode)) {
            app->haveCheckDiagnostics[id] = true;
        }
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
    if (!imu_cmd_get_progress(&progress)) {
        return false;
    }
    if (!app_input_stage_ready(&app->inputGate,
                               &app->console, progress.active)) {
        return true; /* Worker has not acknowledged stage flush. */
    }
    if (!imu_console_dequeue(&app->console, &token)) {
        return true;
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
    char styled[4096];
    unsigned id;

    if (!imu_cmd_get_progress(&progress)) {
        return false;
    }
    /*
     * Coordinator service may have selected a new stage this turn.
     * Arm its worker-side input flush before showing a destructive
     * prompt; never accept or display that prompt before the ack.
     */
    {
        bool inputReady = app_input_stage_ready(&app->inputGate,
                                                &app->console,
                                                progress.active);
        if (progress.active != IMU_CMD_ID_NONE &&
            progress.active != IMU_CMD_ID_SETTLE &&
            progress.active != IMU_CMD_ID_ACQUISITION &&
            inputReady &&
            (!app->haveProgress ||
            progress_changed(&progress, &app->previousProgress) ||
            nowNs - app->lastProgressNs >= APP_PROGRESS_PERIOD_NS)) {
        if (!imu_console_render_progress(&progress, text,
                                          sizeof(text)) ||
            !app_style_progress(text, app->colorInstructions,
                                styled, sizeof(styled)) ||
            fputs(styled, stdout) == EOF) {
                return false;
            }
            app->lastProgressNs = nowNs;
        }
    }

    /* Do not consume the material-change edge before its prompt prints. */
    if (imu_console_stage_flush_complete(&app->console,
                                         app->inputGate.flushTicket)) {
        app->previousProgress = progress;
        app->haveProgress = true;
    }

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
        if (id == (unsigned)IMU_CMD_ID_CHECK ||
            id == (unsigned)IMU_CMD_ID_PROBE) {
            if (app->haveCheckDiagnostics[id]) {
                if (!app_format_check_diagnostics(
                        &app->checkDiagnostics[id],
                        text, sizeof(text)) ||
                    fputs(text, stdout) == EOF) {
                    return false;
                }
            } else if (printf(
                           "checkReceipt mode=%s snapshot=unavailable\n",
                           id == (unsigned)IMU_CMD_ID_CHECK
                               ? "CHECK" : "PROBE") < 0) {
                return false;
            }
        }
        app->printed[id] = true;
        if (result.restoredProduction) {
            app->productionConfigured = true;
        }
        if ((result.identity == IMU_CMD_ID_DCD_CLEAR ||
             result.identity == IMU_CMD_ID_TARE_CLEAR) &&
            (result.state == IMU_CMD_STATE_SUCCEEDED ||
             result.state == IMU_CMD_STATE_FAILED)) {
            /* A failed destructive operation can still be partial. */
            app->postClearDiagnostic = true;
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
        app->stats.publicationsAttempted - app->lastConsolePublication >=
            APP_CONSOLE_PUBLICATION_PERIOD) {
        const ImuSampleSnapshot_t *s = &app->latestSample;

        if (app->postClearDiagnostic &&
            !app->postClearNoticeShown) {
            if (fputs("WARNING: post-clear acquisition is diagnostic;"
                      " calibration/tare quality is not established"
                      " by validMask or this output.\n",
                      stdout) == EOF) {
                return false;
            }
            app->postClearNoticeShown = true;
        }

        if (!app_format_acquisition(
                s, app->stats.publicationsAttempted,
                text, sizeof(text)) ||
            fputs(text, stdout) == EOF) {
            return false;
        }
        app->lastConsolePublication = app->stats.publicationsAttempted;
    }
    return true;
}

static bool begin_acquisition_window(AppContext_t *app, uint64_t nowNs)
{
    ImuMetaUtc_t utc;
    ImuLoggerSink_t sink;
    ImuSampleSnapshot_t baseline;
    uint64_t windowNs;
    uint64_t tick1Ns;
    uint64_t freshNs = 0u;

    (void)nowNs;
    app->outputAttempted = true;
    if (!local_wall_clock_now(&utc) ||
        !imu_meta_utc_stamp(&utc, app->filenameUtc,
                            sizeof(app->filenameUtc)) ||
        imu_logfile_open(&app->logfile, NULL, app->filenameUtc, true) !=
            IMU_LOGFILE_OK) {
        app->loggerFailure = true;
        return false;
    }
    app->logfileOpen = true;

    if (!imu_logfile_sink(&app->logfile, &sink) ||
        !imu_logger_start(&s_logger, &sink)) {
        app->loggerFailure = true;
        return false;
    }
    app->loggerStarted = true;
    if (!write_running_metadata(app)) {
        app->loggerFailure = true;
        return false;
    }

    if (!imu_session_get_snapshot(&baseline) ||
        !imu_publish_begin_window(&app->publisher, &baseline)) {
        app->ownerFailure = true;
        return false;
    }

    /* Read the clock only after the file and logger setup is finished. */
    if (!monotonic_ns(&freshNs)) {
        ++app->stats.clockReadFailures;
        app->ownerFailure = true;
        return false;
    }
    if (freshNs > UINT64_MAX - IMU_PUBLISH_SERVICE_PERIOD_NS) {
        app->ownerFailure = true;
        return false;
    }

    windowNs = (uint64_t)app->plan.acquisitionDurationS * 1000000000ull;
    tick1Ns = freshNs + IMU_PUBLISH_SERVICE_PERIOD_NS;
    if (!imu_publish_gate_begin(&app->publishGate, tick1Ns, true, windowNs)) {
        app->ownerFailure = true;
        return false;
    }

    app->windowOpened = true;
    app->windowJustOpened = true;
    app->nextTurnTicks = 1u;
    return true;
}

static bool publication_step(void *opaque, uint64_t nowNs)
{
    AppContext_t *app = opaque;
    ImuPublishGateDecision_t decision;
    ImuPublishGateStatus_t gateStatus;
    ImuPublicationRecord_t record;
    ImuSampleSnapshot_t snapshot;
    ImuCmdResult_t acquisition;
    const ImuCmdResult_t *acquisitionPtr = NULL;
    ImuCmdIdentity_t current;
    uint64_t actualNs = 0u;
    bool actualValid;
    ++app->stats.serviceTicks;

    if (!app->windowOpened) {
        if (active_command(app) != IMU_CMD_ID_ACQUISITION ||
            imu_cmd_do_not_acquire()) {
            return true;
        }
        return begin_acquisition_window(app, nowNs);
    }
    if (app->windowClosed) {
        return true;
    }
    current = active_command(app);
    if (current != IMU_CMD_ID_ACQUISITION &&
        imu_cmd_get_result(IMU_CMD_ID_ACQUISITION, &acquisition)) {
        acquisitionPtr = &acquisition;
    }
    if (!app_publication_turn_eligible(
            app->activeAtTurnStartWasAcquisition, current,
            acquisitionPtr)) {
        app->windowClosed = true;
        return true;
    }
    if (imu_logger_write_failed(&s_logger)) {
        app->loggerFailure = true;
        app->windowClosed = true;
        return false;
    }

    gateStatus = imu_publish_gate_advance(
        &app->publishGate, app->nextTurnTicks, &decision);
    if (gateStatus == IMU_PUBLISH_GATE_IDLE) {
        return true;
    }
    if (gateStatus == IMU_PUBLISH_GATE_COMPLETE) {
        app->windowClosed = true;
        return true;
    }
    if (gateStatus != IMU_PUBLISH_GATE_DUE) {
        app->ownerFailure = true;
        return false;
    }

    if (!imu_session_get_snapshot(&snapshot) ||
        snapshot.version != IMU_SAMPLE_CONTRACT_VERSION) {
        app->ownerFailure = true;
        return false;
    }
    actualValid = monotonic_ns(&actualNs);
    if (!actualValid) {
        ++app->stats.clockReadFailures;
    }
    if (!imu_publish_evaluate_snapshot(
            &app->publisher, &snapshot,
            decision.scheduledNs, decision.scheduledValid != 0u,
            actualNs, actualValid, &record) ||
        !imu_publish_apply_timing(&record,
                                  decision.gatesSkippedBefore)) {
        app->ownerFailure = true;
        return false;
    }

    app_count_publication(&app->stats, &record);
    app->latestSample = snapshot;
    app->haveSample = true;
    if (imu_logger_enqueue(&s_logger, &record)) {
        ++app->stats.publicationsEnqueued;
    }

    if (imu_publish_gate_window_done(&app->publishGate) ||
        active_command(app) != IMU_CMD_ID_ACQUISITION) {
        app->windowClosed = true;
    }
    return true;
}

static int sleep_until(void *opaque)
{
    AppContext_t *app = opaque;
    int rc = RT_SleepUntil(APP_LOOP_PERIOD_SEC);

    if (rc > 0) {
        app->stats.serviceOverruns += (uint64_t)rc;
        app->nextTurnTicks = 1u + (uint64_t)rc;
    } else {
        app->nextTurnTicks = 1u;
    }
    /* Setup-turn overrun is not gate time; the grid starts next turn. */
    if (app->windowJustOpened) {
        app->nextTurnTicks = 1u;
        app->windowJustOpened = false;
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
    publication_step,
    sleep_until
};

static int finalize_outputs(AppContext_t *app)
{
    ImuRunMetadata_t metadata;
    ImuTermination_t termination;
    ImuLoggerStats_t loggerStats;
    ImuSampleSnapshot_t finalSnapshot;
    ImuCmdResult_t resultStorage[IMU_CMD_ID_COUNT];
    const ImuCmdResult_t *results[IMU_CMD_ID_COUNT];
    ImuMetaCompletion_t completion;
    ImuTerminationReason_t reason;
    bool drained = false;
    bool sessionUnrecovered = false;
    bool processStop = imu_cmd_process_stop_seen();
    bool csvComplete;
    size_t jsonLength;
    int exitStatus;

    if (app->loggerStarted) {
        drained = imu_logger_drain(&s_logger,
                                   APP_LOGGER_DRAIN_TIMEOUT_NS);
        if (!drained || imu_logger_write_failed(&s_logger)) {
            app->loggerFailure = true;
        }
    }
    if (!imu_logger_stats(&s_logger, &loggerStats)) {
        memset(&loggerStats, 0, sizeof(loggerStats));
        loggerStats.version = IMU_LOGGER_STATS_CONTRACT_VERSION;
        app->ownerFailure = true;
    }

    memset(&finalSnapshot, 0, sizeof(finalSnapshot));
    finalSnapshot.readerState = IMU_READER_STATE_CLOSED;
    if (app->sessionOpened && !imu_session_get_snapshot(&finalSnapshot)) {
        app->ownerFailure = true;
    }
    if (app->windowOpened &&
        finalSnapshot.readerState != IMU_READER_STATE_OPERATIONAL) {
        sessionUnrecovered = true;
    }

    reason = app_termination_reason(
        app->loggerFailure, app->startupFailure, app->ownerFailure,
        sessionUnrecovered, processStop, app->windowOpened);
    exitStatus = (reason == IMU_TERM_REASON_COMPLETED ||
                  reason == IMU_TERM_REASON_PROCESS_STOP) ? 0 : 1;

    if (app->logfileOpen && app->logfile.wantCsv != 0u &&
        app->logfile.csvFinalized == 0u) {
        csvComplete = app_csv_finalizes_complete(
            reason, app->windowOpened, app->windowClosed,
            app->loggerStarted, drained, loggerStats.writeFailed,
            loggerStats.shutdownErrors, app->stats.publicationsAttempted);
        if (imu_logfile_close_csv(&app->logfile, csvComplete,
                                  !app->loggerStarted || drained) !=
            IMU_LOGFILE_OK) {
            app->loggerFailure = true;
            reason = IMU_TERM_REASON_LOGGER_FAILURE;
            exitStatus = 1;
        }
    }

    if (!app->logfileOpen && !app->outputAttempted && app->sessionOpened) {
        ImuMetaUtc_t utc;

        app->outputAttempted = true;
        if (!local_wall_clock_now(&utc) ||
            !imu_meta_utc_stamp(&utc, app->filenameUtc,
                                sizeof(app->filenameUtc)) ||
            imu_logfile_open(&app->logfile, NULL, app->filenameUtc, false) !=
                IMU_LOGFILE_OK) {
            app->loggerFailure = true;
            return 1;
        }
        app->logfileOpen = true;
    }
    if (!app->logfileOpen) {
        return 1;
    }

    if (app->logfile.wantCsv == 0u) {
        completion = IMU_META_COMPLETION_NO_CSV;
    } else if (!app->loggerFailure &&
               app->stats.publicationsAttempted != 0u &&
               imu_logfile_csv_outcome(&app->logfile) ==
                   IMU_LOGFILE_CSV_COMPLETE) {
        completion = IMU_META_COMPLETION_COMPLETE;
    } else {
        completion = IMU_META_COMPLETION_INCOMPLETE;
    }

    memset(&termination, 0, sizeof(termination));
    termination.version = IMU_TERMINATION_CONTRACT_VERSION;
    termination.reason = reason;
    termination.signalNumber = processStop ? (int32_t)s_signalNumber : 0;
    termination.exitStatus = exitStatus;
    termination.finalReaderState = finalSnapshot.readerState;
    termination.sessionUnrecovered = sessionUnrecovered ? 1u : 0u;
    termination.acquisitionWindowClosed = app->windowClosed ? 1u : 0u;

    metadata_base(app, &metadata, completion);
    metadata.logger = loggerStats;
    metadata.termination = termination;
    metadata.terminationPresent = 1u;
    collect_results(resultStorage, results);
    jsonLength = imu_meta_format_json(&metadata, &app->plan, results,
                                      s_metaJson, sizeof(s_metaJson));
    if (jsonLength == 0u ||
        imu_logfile_commit_json(&app->logfile, s_metaJson, jsonLength) !=
            IMU_LOGFILE_OK) {
        fprintf(stderr, "main: companion finalization failed error=%d errno=%d\n",
                (int)(jsonLength == 0u ? IMU_LOGFILE_ERR_WRITE
                                      : IMU_LOGFILE_ERR_RENAME),
                imu_logfile_last_errno(&app->logfile));
        exitStatus = 1;
    }

    if (loggerStats.recordsDropped != 0u) {
        fprintf(stderr, "main: logger dropped=%" PRIu64 " record(s)\n",
                loggerStats.recordsDropped);
    }
    printf("main: publications=%" PRIu64 " enqueued=%" PRIu64
           " written=%" PRIu64 " dropped=%" PRIu64
           " deadlineMisses=%" PRIu64 " gatesSkipped=%" PRIu64 "\n",
           app->stats.publicationsAttempted,
           app->stats.publicationsEnqueued,
           loggerStats.recordsWritten, loggerStats.recordsDropped,
           app->stats.deadlineMisses, app->stats.gatesSkipped);
    return exitStatus;
}

int main(int argc, char **argv)
{
    ImuCliParse_t parsed;
    AppContext_t app;
    RT_StartStatus_t rtStatus;
    AppRtPolicy_t rtPolicy;
    AppTurnStatus_t turnStatus = APP_TURN_OK;
    uint64_t nowNs = 0u;
    int exitStatus = 0;
    bool consoleReady = false;
    bool consoleStarted = false;

    memset(&app, 0, sizeof(app));
    app.stats.version = IMU_RUN_STATS_CONTRACT_VERSION;
    app.nextTurnTicks = 1u;
    if (!imu_logger_init(&s_logger)) {
        fprintf(stderr, "main: logger init failed\n");
        return 1;
    }
    app.loggerInitialized = true;

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
    app.colorInstructions = isatty(STDOUT_FILENO) &&
                            getenv("NO_COLOR") == NULL;

    if ((app.plan.slot1 == IMU_CMD_ID_DCD_CLEAR ||
         app.plan.slot2 == IMU_CMD_ID_TARE_CLEAR) &&
        !isatty(STDIN_FILENO)) {
        fprintf(stderr, "main: destructive clear requires"
                        " an interactive terminal\n");
        return 1;
    }

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
        app.startupFailure = true;
        goto cleanup;
    }

    rtStatus = StartRT(APP_RT_PRIORITY, APP_LOOP_PERIOD_SEC);
    rtPolicy = app_rt_policy_from_start(rtStatus);
    if (rtPolicy == APP_RT_POLICY_FATAL) {
        fprintf(stderr, "main: fatal StartRT status=%u\n",
                (unsigned)rtStatus);
        app.startupFailure = true;
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
            ++app.stats.clockReadFailures;
            app.ownerFailure = true;
            break;
        }
        turnStatus = app_owner_turn(&s_turnOps, &app, nowNs);
        if (turnStatus != APP_TURN_OK) {
            if (turnStatus == APP_TURN_ERROR && !app.loggerFailure) {
                fprintf(stderr, "main: owner turn failed\n");
                app.ownerFailure = true;
            }
            break;
        }
    }

    /* A signal may have abandoned a command before the normal render step. */
    if (turnStatus == APP_TURN_STOPPED && !render_outputs(&app, nowNs)) {
        app.ownerFailure = true;
    }

cleanup:
    if (consoleStarted) {
        imu_console_request_stop(&app.console);
        if (!imu_console_join(&app.console)) {
            fprintf(stderr, "main: console worker join failed\n");
            app.ownerFailure = true;
        }
    }

    if (consoleReady) {
        imu_console_destroy(&app.console);
    }

    if (app.sessionOpened) {
        exitStatus = finalize_outputs(&app);
        imu_session_close();
    }
    printf("main: shutdown status=%d serviceTicks=%" PRIu64
           " serviceOverruns=%" PRIu64 "\n",
           exitStatus, app.stats.serviceTicks,
           app.stats.serviceOverruns);
    return exitStatus;
}
#endif /* IMU_APP_OWNER_TEST */
