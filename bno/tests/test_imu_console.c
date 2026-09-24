/* bno/tests/test_imu_console.c */
#define _POSIX_C_SOURCE 200809L

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "app/imu_console.h"

static int failures;

static void check(bool ok, const char *id, const char *what)
{
    if (!ok) {
        fprintf(stderr, "FAIL %s: %s\n", id, what);
        ++failures;
    }
}

static ImuCmdProgress_t progress(ImuCmdIdentity_t identity,
                                  ImuCmdOperatorAction_t action)
{
    ImuCmdProgress_t p;

    memset(&p, 0, sizeof(p));
    p.version = IMU_CMD_PROGRESS_VERSION;
    p.active = identity;
    p.requiredAction = action;
    return p;
}

static void test_translation_is_exact_and_contextual(void)
{
    ImuCmdProgress_t p;
    ImuCmdEvent_t event;

    p = progress(IMU_CMD_ID_DCD_CLEAR, IMU_CMD_ACTION_CONFIRM);
    check(imu_console_translate(IMU_CONSOLE_INPUT_CLEAR, &p, &event) &&
          event.type == IMU_CMD_EVENT_OPERATOR_CONFIRM,
          "CO01", "CLEAR confirms DCD clear");
    check(imu_console_translate(IMU_CONSOLE_INPUT_Q, &p, &event) &&
          event.type == IMU_CMD_EVENT_OPERATOR_Q,
          "CO01", "q cancels confirm prompt");
    check(!imu_console_translate(IMU_CONSOLE_INPUT_Y, &p, &event),
          "CO01", "y cannot clear DCD");

    p = progress(IMU_CMD_ID_TARE_CLEAR, IMU_CMD_ACTION_CONFIRM);
    check(imu_console_translate(IMU_CONSOLE_INPUT_CLEAR, &p, &event) &&
          event.type == IMU_CMD_EVENT_OPERATOR_CONFIRM,
          "CO02", "CLEAR confirms tare clear");

    p = progress(IMU_CMD_ID_TARE, IMU_CMD_ACTION_ALIGN_AND_CONFIRM);
    check(imu_console_translate(IMU_CONSOLE_INPUT_Y, &p, &event) &&
          event.type == IMU_CMD_EVENT_OPERATOR_CONFIRM,
          "CO03", "y confirms aligned tare");
    check(!imu_console_translate(IMU_CONSOLE_INPUT_CLEAR, &p, &event),
          "CO03", "CLEAR cannot confirm tare-now");

    p = progress(IMU_CMD_ID_CALIBRATION,
                 IMU_CMD_ACTION_PRESS_Q_TO_END);
    p.cal.phase = IMU_CMD_CAL_PHASE_ACCEL;
    check(imu_console_translate(IMU_CONSOLE_INPUT_ENTER, &p, &event) &&
          event.type == IMU_CMD_EVENT_OPERATOR_CONFIRM,
          "CO04", "Enter confirms accel face prompt");
    p.cal.deadlineNs = 10ull;
    check(!imu_console_translate(IMU_CONSOLE_INPUT_ENTER, &p, &event),
          "CO04", "Enter cannot skip active motion window");

    p = progress(IMU_CMD_ID_CHECK, IMU_CMD_ACTION_PRESS_Q_TO_END);
    check(imu_console_translate(IMU_CONSOLE_INPUT_Q, &p, &event) &&
          event.type == IMU_CMD_EVENT_OPERATOR_Q,
          "CO05", "q ends active check");
    check(!imu_console_translate(IMU_CONSOLE_INPUT_CLEAR, &p, &event),
          "CO05", "queued CLEAR irrelevant in check");

    p = progress(IMU_CMD_ID_SETTLE, IMU_CMD_ACTION_NONE);
    check(!imu_console_translate(IMU_CONSOLE_INPUT_Q, &p, &event),
          "CO06", "q ignored during settle");
    p.active = IMU_CMD_ID_ACQUISITION;
    check(!imu_console_translate(IMU_CONSOLE_INPUT_Q, &p, &event),
          "CO06", "q ignored during acquisition");
    p.active = IMU_CMD_ID_NONE;
    check(!imu_console_translate(IMU_CONSOLE_INPUT_Q, &p, &event),
          "CO06", "q ignored when idle");
    check(!imu_console_translate(IMU_CONSOLE_INPUT_INVALID, &p, &event),
          "CO06", "invalid line cannot confirm");
}

static void test_fifo_invalid_lines_and_overflow(void)
{
    ImuConsole_t console;
    ImuConsoleInput_t input;
    ImuConsoleObservation_t observation;
    int fds[2];
    unsigned i;

    check(pipe(fds) == 0, "CO07", "pipe");
    check(imu_console_init(&console, fds[0]), "CO07", "init");
    check(imu_console_offer_line(&console, "q"), "CO07", "enqueue q");
    check(imu_console_offer_line(&console, "clear"), "CO07",
          "wrong case enqueued as invalid");
    check(imu_console_offer_line(&console, "CLEAR "), "CO07",
          "whitespace payload enqueued as invalid");
    check(imu_console_offer_line(&console, "CLEAR"), "CO07",
          "enqueue canonical CLEAR");
    check(imu_console_dequeue(&console, &input) &&
          input == IMU_CONSOLE_INPUT_Q, "CO07", "FIFO first q");
    check(imu_console_dequeue(&console, &input) &&
          input == IMU_CONSOLE_INPUT_INVALID, "CO07",
          "wrong case did not confirm");
    check(imu_console_dequeue(&console, &input) &&
          input == IMU_CONSOLE_INPUT_INVALID, "CO07",
          "whitespace did not confirm");
    check(imu_console_dequeue(&console, &input) &&
          input == IMU_CONSOLE_INPUT_CLEAR, "CO07",
          "FIFO canonical CLEAR");

    for (i = 0u; i < IMU_CONSOLE_QUEUE_CAPACITY; ++i) {
        check(imu_console_offer_line(&console, "q"),
              "CO07", "fill bounded queue");
    }
    check(!imu_console_offer_line(&console, "CLEAR"),
          "CO07", "overflow rejects rather than replaces");
    check(imu_console_observe(&console, &observation) &&
          observation.droppedLines == 1u,
          "CO07", "drop observed");
    for (i = 0u; i < IMU_CONSOLE_QUEUE_CAPACITY; ++i) {
        check(imu_console_dequeue(&console, &input) &&
              input == IMU_CONSOLE_INPUT_Q,
              "CO07", "accepted items unchanged");
    }
    check(!imu_console_dequeue(&console, &input),
          "CO07", "empty queue is nonblocking");
    imu_console_destroy(&console);
    (void)close(fds[0]);
    (void)close(fds[1]);
}

static void wait_briefly(void)
{
    struct timespec delay;

    delay.tv_sec = 0;
    delay.tv_nsec = 5000000L;
    (void)nanosleep(&delay, NULL);
}

static void test_worker_lines_eof_and_join(void)
{
    ImuConsole_t console;
    ImuConsoleInput_t input;
    ImuConsoleObservation_t observation;
    int fds[2];
    unsigned attempt;
    unsigned received = 0u;
    const ImuConsoleInput_t expected[] = {
        IMU_CONSOLE_INPUT_Q,
        IMU_CONSOLE_INPUT_CLEAR,
        IMU_CONSOLE_INPUT_Y,
        IMU_CONSOLE_INPUT_ENTER,
        IMU_CONSOLE_INPUT_INVALID
    };
    const char payload[] = "q\nCLEAR\r\ny\n\n CLEAR\n";

    check(pipe(fds) == 0, "CO08", "pipe");
    check(imu_console_init(&console, fds[0]), "CO08", "init");
    check(imu_console_start(&console), "CO08", "start worker");
    check(write(fds[1], payload, sizeof(payload) - 1u) ==
              (ssize_t)(sizeof(payload) - 1u), "CO08", "write lines");

    for (attempt = 0u; attempt < 200u &&
                       received < sizeof(expected) / sizeof(expected[0]);
         ++attempt) {
        if (imu_console_dequeue(&console, &input)) {
            check(input == expected[received], "CO08", "FIFO line mapping");
            ++received;
        } else {
            wait_briefly();
        }
    }
    check(received == sizeof(expected) / sizeof(expected[0]),
          "CO08", "all worker events consumed");
    check(imu_console_observe(&console, &observation) &&
          observation.workerIsSchedOther,
          "CO08", "worker explicitly SCHED_OTHER");

    (void)close(fds[1]);
    for (attempt = 0u; attempt < 200u; ++attempt) {
        (void)imu_console_observe(&console, &observation);
        if (observation.eofObserved) {
            break;
        }
        wait_briefly();
    }
    check(observation.eofObserved && !observation.inputError,
          "CO08", "EOF is worker shutdown, not process stop");
    check(imu_console_join(&console), "CO08", "join after EOF");
    imu_console_destroy(&console);
    (void)close(fds[0]);
}

static void test_stop_unblocks_idle_worker(void)
{
    ImuConsole_t console;
    ImuConsoleObservation_t observation;
    int fds[2];

    check(pipe(fds) == 0, "CO09", "pipe");
    check(imu_console_init(&console, fds[0]), "CO09", "init");
    check(imu_console_start(&console), "CO09", "start");
    imu_console_request_stop(&console);
    check(imu_console_join(&console), "CO09", "join without Enter");
    check(imu_console_observe(&console, &observation) &&
          !observation.inputError,
          "CO09", "stop did not fabricate input failure");
    imu_console_destroy(&console);
    (void)close(fds[0]);
    (void)close(fds[1]);
}

static void test_rendering_keeps_r8_and_r7_distinct(void)
{
    ImuCmdProgress_t p;
    ImuCmdResult_t r;
    char buffer[2048];

    p = progress(IMU_CMD_ID_PROBE, IMU_CMD_ACTION_PRESS_Q_TO_END);
    p.check.phase = IMU_CMD_CHECK_PHASE_MONITOR;
    p.check.requestedMaskValid = true;
    p.check.requestedMask = 0u;
    p.check.effectiveMask = 0u;
    p.check.actualMaskValid = false;
    p.check.haveAccel = true;
    p.check.haveMag = true;
    p.check.accelStatus = 3u;
    p.check.magStatus = 2u;
    p.check.gatePassingNow = false;
    p.check.gateReached = true;
    check(imu_console_render_progress(&p, buffer, sizeof(buffer)),
          "CO10", "render R8");
    check(strstr(buffer, "identity=PROBE") != NULL &&
          strstr(buffer, "requestedMaskValid=1 requestedMask=0x00") != NULL &&
          strstr(buffer, "actualMaskValid=0") != NULL &&
          strstr(buffer, "gatePassingNow=0 gateReached=1") != NULL,
          "CO10", "R8 preserves mask and current/sticky verdicts");

    memset(&r, 0, sizeof(r));
    r.version = IMU_CMD_RESULT_VERSION;
    r.identity = IMU_CMD_ID_PROBE;
    r.state = IMU_CMD_STATE_TIMED_OUT;
    r.reason = IMU_CMD_REASON_PROBE_DEADLINE;
    r.sub.probeTimedOut = true;
    r.sub.probeReachedGate = true;
    r.terminalProgress.check.gateReached = true;
    r.terminalProgress.check.gatePassingNow = false;
    check(imu_console_render_result(&r, false, buffer, sizeof(buffer)),
          "CO11", "render R7");
    check(strstr(buffer, "state=TIMED_OUT") != NULL &&
          strstr(buffer, "reason=PROBE_DEADLINE") != NULL &&
          strstr(buffer, "gatePassingNow=0 gateReached=1") != NULL &&
          strstr(buffer, "probeTimedOut=1") != NULL,
          "CO11", "timeout not disguised as success");

    r.state = IMU_CMD_STATE_CANCELLED;
    r.reason = IMU_CMD_REASON_OPERATOR_Q;
    check(imu_console_render_result(&r, false, buffer, sizeof(buffer)) &&
          strstr(buffer, "state=CANCELLED") != NULL,
          "CO12", "cancellation distinct");
    r.state = IMU_CMD_STATE_FAILED;
    r.reason = IMU_CMD_REASON_CONFIG_FAILED;
    check(imu_console_render_result(&r, false, buffer, sizeof(buffer)) &&
          strstr(buffer, "state=FAILED") != NULL,
          "CO12", "ordinary failure distinct");
    r.state = IMU_CMD_STATE_RECOVERY_FAILED;
    r.reason = IMU_CMD_REASON_SESSION_UNUSABLE;
    check(imu_console_render_result(&r, true, buffer, sizeof(buffer)) &&
          strstr(buffer, "state=RECOVERY_FAILED") != NULL &&
          strstr(buffer, "doNotAcquire=1") != NULL,
          "CO12", "recovery failure prevents acquire");
    r.state = IMU_CMD_STATE_SUCCEEDED;
    r.reason = IMU_CMD_REASON_OK;
    check(imu_console_render_result(&r, false, buffer, sizeof(buffer)) &&
          strstr(buffer, "state=SUCCEEDED") != NULL,
          "CO12", "success distinct");

    check(!imu_console_render_result(&r, false, buffer, 4u),
          "CO13", "small destination rejected");
    p.version = 0u;
    check(!imu_console_render_progress(&p, buffer, sizeof(buffer)),
          "CO13", "wrong progress version rejected");
}

int main(void)
{
    test_translation_is_exact_and_contextual();
    test_fifo_invalid_lines_and_overflow();
    test_worker_lines_eof_and_join();
    test_stop_unblocks_idle_worker();
    test_rendering_keeps_r8_and_r7_distinct();

    if (failures != 0) {
        fprintf(stderr, "test_imu_console: %d failure(s)\n", failures);
        return 1;
    }
    puts("test_imu_console: pass");
    return 0;
}
