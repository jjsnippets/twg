/* bno/tests/test_app_owner_loop.c */
#define IMU_APP_OWNER_TEST
#include "../app/main.c"

#include <stdio.h>
#include <string.h>

typedef struct {
    char trace[128];
    unsigned length;
    bool stop;
    bool failAdapter;
    bool due;
    bool initialized;
    unsigned inputCalls;
    unsigned serviceCalls;
    unsigned sleepCalls;
    unsigned acquireCalls;
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

static bool due(void *context)
{
    Fixture_t *f = context;
    note(f, 'E');
    return f->due;
}

static bool acquire(void *context)
{
    Fixture_t *f = context;
    note(f, 'O');
    ++f->acquireCalls;
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
    tick, input, coordinator, render, due, acquire, sleep_once
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
        check(f.acquireCalls == 0u, "OL03", "no early acquisition");
    }

    memset(&f, 0, sizeof(f));
    f.active = IMU_CMD_ID_ACQUISITION;
    check(app_owner_turn(&ops, &f, 100ull) == APP_TURN_OK,
          "OL04", "non-gated acquisition turn");
    check(f.acquireCalls == 0u, "OL04", "no observation before gate");
    f.due = true; /* Production due callback owns the tenth-tick gate. */
    check(app_owner_turn(&ops, &f, 110ull) == APP_TURN_OK,
          "OL04", "due acquisition turn");
    check(f.acquireCalls == 1u &&
          strstr(f.trace, "REOZ") != NULL,
          "OL04", "one observation after rendering, before sleep");
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
          f.acquireCalls == 0u,
          "OL06", "unrestorable routed without coordinator pass");
}

int main(void)
{
    test_normal_order_and_next_turn_request();
    test_exclusive_selection_and_10ms_gate();
    test_stop_and_adapter_failure();

    if (failures != 0) {
        fprintf(stderr, "test_app_owner_loop: %d failure(s)\n", failures);
        return 1;
    }
    puts("test_app_owner_loop: pass");
    return 0;
}
