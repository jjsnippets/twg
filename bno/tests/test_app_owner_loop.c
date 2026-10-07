/* bno/tests/test_app_owner_loop.c */
#define IMU_APP_OWNER_TEST
#include "../app/main.c"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

typedef struct {
    char trace[128];
    unsigned length;
    bool stop;
    bool failAdapter;
    bool initialized;
    unsigned inputCalls;
    unsigned serviceCalls;
    unsigned sleepCalls;
    unsigned publishCalls;
    ImuCmdIdentity_t active;
    AppAdapter_t pumped;
} Fixture_t;

static int failures;

static void check(bool ok, const char *id, const char *what)
{
    if (!ok) {
        fprintf(stderr, "FAIL %s: %s\n", id, what);
        ++failures;
    }
}

static void note(Fixture_t *f, char symbol)
{
    if (f->length + 1u < sizeof(f->trace)) {
        f->trace[f->length++] = symbol;
        f->trace[f->length] = '\0';
    }
}

static bool stopped(void *context)
{
    return ((Fixture_t *)context)->stop;
}

static void adopt(void *context)
{
    note(context, 'P');
}

static void session(void *context)
{
    note(context, 'S');
}

static ImuCmdIdentity_t active(void *context)
{
    return ((Fixture_t *)context)->active;
}

static bool stage(void *context, AppAdapter_t adapter, uint64_t nowNs)
{
    Fixture_t *f = context;
    (void)nowNs;

    if (adapter != APP_ADAPTER_NONE && f->initialized) {
        f->pumped = adapter;
        note(f, 'A');
    }
    return !f->failAdapter;
}

static void fail_session(void *context)
{
    note(context, 'U');
}

static bool tick(void *context, uint64_t nowNs)
{
    (void)nowNs;
    note(context, 'T');
    return true;
}

static bool input(void *context, uint64_t nowNs)
{
    Fixture_t *f = context;
    (void)nowNs;
    note(f, 'I');
    ++f->inputCalls;
    return true;
}

static void coordinator(void *context)
{
    Fixture_t *f = context;
    note(f, 'C');
    ++f->serviceCalls;
    /* A request emitted now cannot be pumped until the next turn. */
    f->initialized = true;
}

static bool render(void *context, uint64_t nowNs)
{
    (void)nowNs;
    note(context, 'R');
    return true;
}

static bool publish(void *context, uint64_t nowNs)
{
    Fixture_t *f = context;
    (void)nowNs;
    note(f, 'E');
    ++f->publishCalls;
    return true;
}

static int sleep_once(void *context)
{
    Fixture_t *f = context;
    note(f, 'Z');
    ++f->sleepCalls;
    return 0;
}

static const AppTurnOps_t ops = {
    stopped, adopt, session, active, stage, fail_session,
    tick, input, coordinator, render, publish, sleep_once
};

static void test_normal_order_and_next_turn_request(void)
{
    Fixture_t f;

    memset(&f, 0, sizeof(f));
    f.active = IMU_CMD_ID_CALIBRATION;

    check(app_owner_turn(&ops, &f, 100ull) == APP_TURN_OK,
          "OL01", "first turn");
    check(strcmp(f.trace, "STICREZ") == 0,
          "OL01", "service then tick/input/service/render/gate/sleep");
    check(f.pumped == APP_ADAPTER_NONE,
          "OL01", "first turn did not pump new request");
    check(f.publishCalls == 1u,
          "OL01", "publication seam checked every turn");
    check(f.inputCalls == 1u &&
          f.serviceCalls == 1u && f.sleepCalls == 1u,
          "OL01", "one input attempt, service, and sleep");

    f.length = 0u;
    f.trace[0] = '\0';
    check(app_owner_turn(&ops, &f, 101ull) == APP_TURN_OK,
          "OL02", "second turn");
    check(strcmp(f.trace, "SATICREZ") == 0 &&
          f.pumped == APP_ADAPTER_CAL,
          "OL02", "pending calibration pumped only next turn");
    check(f.serviceCalls == 2u && f.sleepCalls == 2u,
          "OL02", "one service and sleep each normal turn");
}

static void test_exclusive_selection_and_10ms_gate(void)
{
    const struct {
        ImuCmdIdentity_t identity;
        AppAdapter_t adapter;
    } cases[] = {
        { IMU_CMD_ID_CALIBRATION, APP_ADAPTER_CAL },
        { IMU_CMD_ID_DCD_CLEAR, APP_ADAPTER_CAL },
        { IMU_CMD_ID_TARE, APP_ADAPTER_TARE },
        { IMU_CMD_ID_TARE_CLEAR, APP_ADAPTER_TARE },
        { IMU_CMD_ID_TARE_CHECK, APP_ADAPTER_TARE },
        { IMU_CMD_ID_CHECK, APP_ADAPTER_CHECK },
        { IMU_CMD_ID_PROBE, APP_ADAPTER_CHECK },
        { IMU_CMD_ID_SETTLE, APP_ADAPTER_NONE },
        { IMU_CMD_ID_ACQUISITION, APP_ADAPTER_NONE },
        { IMU_CMD_ID_NONE, APP_ADAPTER_NONE }
    };
    unsigned i;
    Fixture_t f;

    for (i = 0u; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        memset(&f, 0, sizeof(f));
        f.active = cases[i].identity;
        f.initialized = true;
        check(app_adapter_for(f.active) == cases[i].adapter,
              "OL03", "one matching adapter kind");
        check(app_owner_turn(&ops, &f, 100ull) == APP_TURN_OK,
              "OL03", "selection turn");
        check(f.pumped == cases[i].adapter,
              "OL03", "no competing adapter");
        check(f.publishCalls == 1u,
              "OL03", "one publication decision per normal turn");
    }

    memset(&f, 0, sizeof(f));
    f.active = IMU_CMD_ID_ACQUISITION;
    check(app_owner_turn(&ops, &f, 100ull) == APP_TURN_OK,
          "OL04", "acquisition turn");
    check(f.publishCalls == 1u && strstr(f.trace, "REZ") != NULL,
          "OL04", "publication decision after rendering, before sleep");
}

static void test_stop_and_adapter_failure(void)
{
    Fixture_t f;

    memset(&f, 0, sizeof(f));
    f.active = IMU_CMD_ID_DCD_CLEAR;
    f.initialized = true;
    f.stop = true;
    check(app_owner_turn(&ops, &f, 100ull) == APP_TURN_STOPPED,
          "OL05", "stop turn");
    check(strcmp(f.trace, "P") == 0 &&
          f.pumped == APP_ADAPTER_NONE &&
          f.sleepCalls == 0u,
          "OL05", "adopt stop before session or adapter");

    memset(&f, 0, sizeof(f));
    f.active = IMU_CMD_ID_CHECK;
    f.initialized = true;
    f.failAdapter = true;
    check(app_owner_turn(&ops, &f, 100ull) == APP_TURN_ERROR,
          "OL06", "adapter failure");
    check(strcmp(f.trace, "SAU") == 0 &&
          f.serviceCalls == 0u &&
          f.publishCalls == 0u,
          "OL06", "unrestorable routed without coordinator pass");
}

static void test_owner_input_gate_requires_worker_ack(void)
{
    ImuConsole_t console;
    AppInputGate_t gate;
    ImuConsoleInput_t input;
    int fds[2];
    unsigned i;

    memset(&gate, 0, sizeof(gate));
    check(pipe(fds) == 0, "OL07", "pipe");
    check(imu_console_init(&console, fds[0]), "OL07", "init");
    check(imu_console_start(&console), "OL07", "worker");
    check(imu_console_offer_line(&console, "CLEAR"),
          "OL07", "prequeued CLEAR");
    check(!app_input_stage_ready(&gate, &console,
                                  IMU_CMD_ID_DCD_CLEAR),
          "OL07", "stage initially disarmed");
    for (i = 0u; i < 200u &&
                 !app_input_stage_ready(&gate, &console,
                                         IMU_CMD_ID_DCD_CLEAR);
         ++i) {
        struct timespec delay = {0, 5000000L};
        (void)nanosleep(&delay, NULL);
    }
    check(app_input_stage_ready(&gate, &console,
                                 IMU_CMD_ID_DCD_CLEAR) &&
          !imu_console_dequeue(&console, &input),
          "OL07", "prior token removed before DCD stage");

    check(imu_console_offer_line(&console, "CLEAR"),
          "OL07", "typed-ahead next-stage token");
    check(!app_input_stage_ready(&gate, &console,
                                  IMU_CMD_ID_TARE_CLEAR),
          "OL07", "new tare stage disarmed");
    for (i = 0u; i < 200u &&
                 !app_input_stage_ready(&gate, &console,
                                         IMU_CMD_ID_TARE_CLEAR);
         ++i) {
        struct timespec delay = {0, 5000000L};
        (void)nanosleep(&delay, NULL);
    }
    check(app_input_stage_ready(&gate, &console,
                                 IMU_CMD_ID_TARE_CLEAR) &&
          !imu_console_dequeue(&console, &input),
          "OL07", "no carry-over confirmation");
    imu_console_request_stop(&console);
    check(imu_console_join(&console), "OL07", "join");
    imu_console_destroy(&console);
    (void)close(fds[0]);
    (void)close(fds[1]);
}

static void test_acquisition_formatter_validity(void)
{
    ImuSampleSnapshot_t s;
    char out[512];

    memset(&s, 0, sizeof(s));
    s.version = IMU_SAMPLE_CONTRACT_VERSION;
    s.configurationEpoch = 7u;
    check(app_format_acquisition(&s, 1u, out, sizeof(out)) &&
          strstr(out, "publications=1") != NULL &&
          strstr(out, "orientation=not_seen") != NULL &&
          strstr(out, "linear_accel=not_seen") != NULL &&
          strstr(out, "calibrated_gyro=not_seen") != NULL &&
          strstr(out, " ax=") == NULL &&
          strstr(out, " gx=") == NULL,
          "OL08", "missing groups never print zero as valid");

    s.validMask = IMU_GROUP_MASK_REQUIRED;
    s.yaw = 1.0f;
    s.ax = 2.0f;
    s.gx = 3.0f;
    check(app_format_acquisition(&s, 2u, out, sizeof(out)) &&
          strstr(out, "yaw=1.000") != NULL &&
          strstr(out, "ax=2.000") != NULL &&
          strstr(out, "gx=3.000") != NULL &&
          strstr(out, "not_seen") == NULL,
          "OL08", "all three eligible groups represented");
    check(!app_format_acquisition(&s, 2u, out, 4u),
          "OL08", "undersized destination rejected");
}

static void test_instruction_color_is_tty_only(void)
{
    const char *guide =
        "R8 version=4 identity=CALIBRATION\n"
        "GUIDE: pose 1/6: place device flat.\n"
        "Press Enter to start this face.\n"
        "cal pose=1/6 accelStatus=2\n";
    const char *warning =
        "R8 version=4 identity=DCD_CLEAR\n"
        "WARNING: erase saved DCD calibration.\n"
        "Type CLEAR then Enter.\n"
        "cal pose=0/6\n";
    char out[512];

    check(app_style_progress(guide, false, out, sizeof(out)) &&
          strcmp(out, guide) == 0 &&
          strchr(out, '\x1b') == NULL,
          "OL09", "captured/non-TTY R8 remains plain");
    check(app_style_progress(guide, true, out, sizeof(out)) &&
          strstr(out, "\x1b[1;36mGUIDE:") != NULL &&
          strstr(out, "\x1b[1;36mPress Enter") != NULL &&
          strstr(out, "\x1b[0mcal pose=") != NULL,
          "OL09", "guide stands out; telemetry resets");
    check(app_style_progress(warning, true, out, sizeof(out)) &&
          strstr(out, "\x1b[1;33mWARNING:") != NULL &&
          strstr(out, "\x1b[1;33mType CLEAR") != NULL &&
          strstr(out, "\x1b[0mcal pose=") != NULL,
          "OL09", "destructive warning distinct and reset");
    check(!app_style_progress(guide, true, out, 4u),
          "OL09", "insufficient output storage rejected");
}

static void test_check_diagnostics_retention(void)
{
    ImuCheckDiagnostics_t candidate;
    ImuCheckDiagnostics_t retained;

    memset(&candidate, 0, sizeof(candidate));
    memset(&retained, 0, sizeof(retained));
    candidate.version = IMU_CHECK_DIAGNOSTICS_VERSION;
    candidate.configurationEpoch = 2u;
    candidate.eligible = true;
    candidate.mode = IMU_SESSION_CHECK_MODE_PROBE;
    candidate.mag.epochDecodeCount = 499u;

    check(app_retain_check_diagnostics(
              &retained, &candidate, 2u,
              IMU_SESSION_CHECK_MODE_PROBE),
          "OL10", "retain matching eligible diagnostics");
    check(retained.mag.epochDecodeCount == 499u,
          "OL10", "actual decode count retained");

    candidate.eligible = false;
    candidate.configurationEpoch = 3u;
    candidate.mag.epochDecodeCount = 0u;
    check(!app_retain_check_diagnostics(
              &retained, &candidate, 3u,
              IMU_SESSION_CHECK_MODE_PROBE) &&
          retained.configurationEpoch == 2u &&
          retained.mag.epochDecodeCount == 499u,
          "OL10", "restoration cannot overwrite retained evidence");

    candidate.eligible = true;
    check(!app_retain_check_diagnostics(
              &retained, &candidate, 2u,
              IMU_SESSION_CHECK_MODE_PROBE),
          "OL10", "different epoch rejected");
    check(!app_retain_check_diagnostics(
              &retained, &candidate, 3u,
              IMU_SESSION_CHECK_MODE_CHECK),
          "OL10", "different mode rejected");
    candidate.version = 0u;
    check(!app_retain_check_diagnostics(
              &retained, &candidate, 3u,
              IMU_SESSION_CHECK_MODE_PROBE),
          "OL10", "wrong diagnostics version rejected");
}

static void test_check_diagnostics_formatter(void)
{
    ImuCheckDiagnostics_t d;
    char out[2048];

    memset(&d, 0, sizeof(d));
    d.version = IMU_CHECK_DIAGNOSTICS_VERSION;
    d.configurationEpoch = 2u;
    d.eligible = true;
    d.mode = IMU_SESSION_CHECK_MODE_PROBE;
    d.mag.epochDecodeCount = 499u;
    d.mag.processDecodeCount = 700u;
    d.mag.hostTimesValid = true;
    d.mag.firstHostDecodeNs = 1000000000ull;
    d.mag.latestHostDecodeNs = 10960000000ull;
    d.accel.epochDecodeCount = 100u;
    d.gyro.epochDecodeCount = 99u;
    d.rv.epochDecodeCount = 98u;

    check(app_format_check_diagnostics(&d, out, sizeof(out)) &&
          strstr(out, "mode=PROBE epoch=2") != NULL &&
          strstr(out, "snapshot=retained_last_eligible") != NULL &&
          strstr(out, "clock=host_decode_ns") != NULL,
          "OL11", "diagnostic provenance is explicit");
    check(strstr(out,
                 "report=mag requestedHz=50 epochDecodes=499"
                 " processDecodes=700 hostTimesValid=1"
                 " firstHostDecodeNs=1000000000"
                 " latestHostDecodeNs=10960000000") != NULL,
          "OL11", "mag counts and 64-bit timestamps preserved");
    check(strstr(out,
                 "report=accel requestedHz=10 epochDecodes=100") != NULL &&
          strstr(out,
                 "report=gyro requestedHz=10 epochDecodes=99") != NULL &&
          strstr(out,
                 "report=rv requestedHz=10 epochDecodes=98") != NULL,
          "OL11", "report counts remain independent");
    check(strstr(out,
                 "hostTimesValid=0 firstHostDecodeNs=0"
                 " latestHostDecodeNs=0") != NULL &&
          strchr(out, '\x1b') == NULL,
          "OL11", "missing timestamps and plain output remain truthful");

    d.mode = IMU_SESSION_CHECK_MODE_CHECK;
    check(app_format_check_diagnostics(&d, out, sizeof(out)) &&
          strstr(out, "mode=CHECK epoch=2") != NULL,
          "OL11", "continuous check labelled separately");
    check(!app_format_check_diagnostics(&d, out, 4u) &&
          out[0] == '\0',
          "OL11", "truncation rejected without usable partial output");
    d.eligible = false;
    check(!app_format_check_diagnostics(&d, out, sizeof(out)),
          "OL11", "ineligible snapshot rejected");
    check(!app_format_check_diagnostics(NULL, out, sizeof(out)) &&
          !app_format_check_diagnostics(&d, NULL, sizeof(out)) &&
          !app_format_check_diagnostics(&d, out, 0u),
          "OL11", "invalid arguments rejected");
}

static void test_publication_counters(void)
{
    ImuRunStats_t stats;
    ImuPublicationRecord_t record;

    memset(&stats, 0, sizeof(stats));
    memset(&record, 0, sizeof(record));
    record.validMask = IMU_GROUP_MASK_REQUIRED;
    record.freshMask = IMU_GROUP_BIT_ROTATION | IMU_GROUP_BIT_ACCEL;
    record.staleMask = IMU_GROUP_BIT_GYRO;
    record.multiUpdateMask = IMU_GROUP_BIT_ACCEL;
    record.deadlineMissed = 1u;
    record.gatesSkippedBefore = 2u;

    app_count_publication(&stats, &record);
    check(stats.publicationsAttempted == 1u &&
          stats.publicationsAllValid == 1u &&
          stats.publicationsPartiallyFresh == 1u &&
          stats.publicationsAllFresh == 0u &&
          stats.deadlineMisses == 1u && stats.gatesSkipped == 2u,
          "OL12", "publication totals and skipped deadlines");
    check(stats.staleByGroup[IMU_PUBLISH_GROUP_GYRO] == 1u &&
          stats.multiUpdateByGroup[IMU_PUBLISH_GROUP_ACCEL] == 1u,
          "OL12", "per-group counters");
}

static void test_terminal_acquisition_publication(void)
{
    ImuCmdResult_t result;

    memset(&result, 0, sizeof(result));
    result.state = IMU_CMD_STATE_SUCCEEDED;
    result.reason = IMU_CMD_REASON_OK;

    check(app_publication_turn_eligible(
              true, IMU_CMD_ID_ACQUISITION, NULL),
          "OL13", "active acquisition is eligible");
    check(app_publication_turn_eligible(true, IMU_CMD_ID_NONE, &result),
          "OL13", "natural terminal turn emits final gate");
    check(!app_publication_turn_eligible(false, IMU_CMD_ID_NONE, &result),
          "OL13", "later turn cannot publish");
    result.state = IMU_CMD_STATE_FAILED;
    check(!app_publication_turn_eligible(true, IMU_CMD_ID_NONE, &result),
          "OL13", "failed acquisition cannot publish terminal gate");
}

static void test_exit_reason_precedence(void)
{
    check(app_termination_reason(false, false, false, false, false, true) ==
              IMU_TERM_REASON_COMPLETED,
          "OL13", "normal completion");
    check(app_termination_reason(false, false, false, false, true, true) ==
              IMU_TERM_REASON_PROCESS_STOP,
          "OL13", "graceful process stop");
    check(app_termination_reason(false, false, false, false, false, false) ==
              IMU_TERM_REASON_NO_ACQUISITION,
          "OL13", "pre-window stop");
    check(app_termination_reason(false, false, false, true, false, true) ==
              IMU_TERM_REASON_SESSION_UNRECOVERED,
          "OL13", "unrecovered session");
    check(app_termination_reason(false, false, true, false, false, true) ==
              IMU_TERM_REASON_OWNER_FAILURE,
          "OL13", "owner failure");
    check(app_termination_reason(false, true, false, false, false, false) ==
              IMU_TERM_REASON_STARTUP_FAILURE,
          "OL13", "startup failure");
    check(app_termination_reason(true, true, true, true, true, true) ==
              IMU_TERM_REASON_LOGGER_FAILURE,
          "OL13", "logger failure takes precedence");
}

static void test_csv_finalization_policy(void)
{
    check(app_csv_finalizes_complete(IMU_TERM_REASON_COMPLETED,
                                     true, true, true, true, 0u, 0u, 1000u),
          "OL14", "natural completion finalizes the CSV");
    check(app_csv_finalizes_complete(IMU_TERM_REASON_PROCESS_STOP,
                                     true, false, true, true, 0u, 0u, 1405u),
          "OL14", "graceful stop with rows finalizes the CSV");
    check(!app_csv_finalizes_complete(IMU_TERM_REASON_PROCESS_STOP,
                                      true, false, true, true, 0u, 0u, 0u),
          "OL14", "stop before any row stays partial");
    check(!app_csv_finalizes_complete(IMU_TERM_REASON_PROCESS_STOP,
                                      false, false, true, true, 0u, 0u, 10u),
          "OL14", "stop before the window opened stays partial");
    check(!app_csv_finalizes_complete(IMU_TERM_REASON_COMPLETED,
                                      true, false, true, true, 0u, 0u, 10u),
          "OL14", "completed reason without a closed window stays partial");
    check(!app_csv_finalizes_complete(IMU_TERM_REASON_PROCESS_STOP,
                                      true, false, true, false, 0u, 0u, 1405u),
          "OL14", "incomplete drain stays partial");
    check(!app_csv_finalizes_complete(IMU_TERM_REASON_PROCESS_STOP,
                                      true, false, true, true, 1u, 0u, 1405u),
          "OL14", "sticky write failure stays partial");
    check(!app_csv_finalizes_complete(IMU_TERM_REASON_COMPLETED,
                                      true, true, true, true, 0u, 1u, 1000u),
          "OL14", "shutdown error stays partial");
    check(!app_csv_finalizes_complete(IMU_TERM_REASON_PROCESS_STOP,
                                      true, false, false, true, 0u, 0u, 1405u),
          "OL14", "logger never started stays partial");
    check(!app_csv_finalizes_complete(IMU_TERM_REASON_SESSION_UNRECOVERED,
                                      true, true, true, true, 0u, 0u, 1000u) &&
          !app_csv_finalizes_complete(IMU_TERM_REASON_LOGGER_FAILURE,
                                      true, true, true, true, 0u, 0u, 1000u) &&
          !app_csv_finalizes_complete(IMU_TERM_REASON_OWNER_FAILURE,
                                      true, true, true, true, 0u, 0u, 1000u) &&
          !app_csv_finalizes_complete(IMU_TERM_REASON_STARTUP_FAILURE,
                                      true, true, true, true, 0u, 0u, 1000u) &&
          !app_csv_finalizes_complete(IMU_TERM_REASON_NO_ACQUISITION,
                                      false, false, true, true, 0u, 0u, 0u),
          "OL14", "failure reasons never finalize as complete");
}

int main(void)
{
    test_normal_order_and_next_turn_request();
    test_exclusive_selection_and_10ms_gate();
    test_stop_and_adapter_failure();
    test_owner_input_gate_requires_worker_ack();
    test_acquisition_formatter_validity();
    test_instruction_color_is_tty_only();
    test_check_diagnostics_retention();
    test_check_diagnostics_formatter();
    test_publication_counters();
    test_terminal_acquisition_publication();
    test_exit_reason_precedence();
    test_csv_finalization_policy();

    if (failures != 0) {
        fprintf(stderr, "test_app_owner_loop: %d failure(s)\n", failures);
        return 1;
    }
    puts("test_app_owner_loop: pass");
    return 0;
}
