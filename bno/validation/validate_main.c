#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "app/imu_cal.h"
#include "app/imu_cal_adapter.h"
#include "app/imu_cmd.h"
#include "app/imu_console.h"
#include "app/imu_publish.h"
#include "app/imu_session.h"
#include "app/imu_tare.h"
#include "app/imu_tare_adapter.h"

#include "validation/amt102.h"
#include "validation/validate_contract.h"
#include "validation/validate_logger.h"

#define VALIDATE_SETTLE_NS UINT64_C(300000000)
#define VALIDATE_WATCHDOG_NS UINT64_C(1000000000)
#define VALIDATE_RAD_TO_DEG 57.2957795130823208768

typedef enum {
    VALIDATE_READY = 0,
    VALIDATE_RECORDING,
    VALIDATE_FINALIZING
} ValidateMode;

typedef struct {
    struct termios saved;
    int saved_flags;
    bool attributes_changed;
    bool flags_changed;
} RawTerminal;

typedef struct {
    bool calibration_ok;
    bool tare_ok;
    bool settle_started;
    bool printed_calibration;
    bool printed_tare;

    uint64_t settle_deadline_ns;
    uint64_t last_status_ns;
    uint64_t flush_ticket;

    ImuCmdIdentity_t input_stage;
    ImuCmdIdentity_t rendered_stage;
} Preparation;

static volatile sig_atomic_t stop_requested;
static volatile sig_atomic_t stop_signal;

static void signal_handler(int number)
{
    stop_signal = number;
    stop_requested = 1;
}

static bool install_signals(void)
{
    struct sigaction action;

    memset(&action, 0, sizeof(action));
    action.sa_handler = signal_handler;
    sigemptyset(&action.sa_mask);

    return sigaction(SIGINT, &action, NULL) == 0 &&
           sigaction(SIGTERM, &action, NULL) == 0;
}

static bool monotonic_ns(uint64_t *out)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return false;
    }

    *out = (uint64_t)now.tv_sec * UINT64_C(1000000000) +
           (uint64_t)now.tv_nsec;
    return true;
}

/*
 * Best-effort absolute 1 ms service pacing.
 * Publication and trial duration use elapsed monotonic time, not turns.
 */
static bool sleep_turn(uint64_t *next_wake_ns)
{
    struct timespec deadline;
    uint64_t now;
    int rc;

    if (!monotonic_ns(&now)) {
        return false;
    }

    if (*next_wake_ns <= now) {
        *next_wake_ns = now + VALIDATE_SERVICE_NS;
    }

    deadline.tv_sec =
        (time_t)(*next_wake_ns / UINT64_C(1000000000));
    deadline.tv_nsec =
        (long)(*next_wake_ns % UINT64_C(1000000000));

    do {
        rc = clock_nanosleep(
            CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL);
    } while (rc == EINTR && !stop_requested);

    *next_wake_ns += VALIDATE_SERVICE_NS;

    if (rc != 0 && !(rc == EINTR && stop_requested)) {
        errno = rc;
        return false;
    }

    return true;
}

static bool post_event(ImuCmdEventType_t type, uint64_t now)
{
    ImuCmdEvent_t event;

    memset(&event, 0, sizeof(event));
    event.type = type;
    event.monotonicNs = now;

    return imu_cmd_post(&event);
}

static bool pump_preparation_adapter(ImuCmdIdentity_t active)
{
    ImuCmdProgress_t machine;

    if (active == IMU_CMD_ID_CALIBRATION) {
        if (!imu_cal_get_progress(&machine) ||
            machine.active != active ||
            imu_cal_complete()) {
            return true;
        }
        return imu_cal_adapter_pump();
    }

    if (active == IMU_CMD_ID_TARE) {
        if (!imu_tare_get_progress(&machine) ||
            machine.active != active ||
            imu_tare_complete()) {
            return true;
        }
        return imu_tare_adapter_pump();
    }

    return true;
}

static bool check_preparation_results(Preparation *preparation)
{
    const ImuCmdIdentity_t identities[2] = {
        IMU_CMD_ID_CALIBRATION,
        IMU_CMD_ID_TARE
    };
    unsigned i;

    for (i = 0u; i < 2u; ++i) {
        ImuCmdResult_t result;
        bool *printed =
            i == 0u
                ? &preparation->printed_calibration
                : &preparation->printed_tare;
        bool acceptable;
        char text[2048];

        if (*printed ||
            !imu_cmd_get_result(identities[i], &result) ||
            result.state == IMU_CMD_STATE_NOT_REQUESTED ||
            result.state == IMU_CMD_STATE_RUNNING) {
            continue;
        }

        if (!imu_console_render_result(
                &result, imu_cmd_do_not_acquire(),
                text, sizeof(text))) {
            return false;
        }

        fputs(text, stdout);
        *printed = true;

        acceptable =
            result.state == IMU_CMD_STATE_SUCCEEDED &&
            result.verified &&
            result.restoredProduction;

        if (i == 0u) {
            acceptable = acceptable && result.dcdSaved;
        } else {
            acceptable =
                acceptable &&
                result.sub.tareNow == IMU_CMD_SUB_SUCCEEDED &&
                result.sub.persist == IMU_CMD_SUB_SUCCEEDED;
        }

        if (!acceptable) {
            fprintf(
                stderr,
                "Startup stopped: %s did not complete successfully.\n",
                i == 0u ? "calibration" : "Z tare");
            return false;
        }

        if (i == 0u) {
            preparation->calibration_ok = true;
        } else {
            preparation->tare_ok = true;
        }
    }

    return true;
}

static bool preparation_status(
    Preparation *preparation,
    ImuConsole_t *console,
    const ImuCmdProgress_t *progress,
    uint64_t now)
{
    char text[2048];
    bool due =
        preparation->last_status_ns == 0u ||
        now - preparation->last_status_ns >= VALIDATE_STATUS_NS;
    bool changed =
        preparation->rendered_stage != progress->active;

    if (!due && !changed) {
        return true;
    }

    if (progress->active == IMU_CMD_ID_CALIBRATION) {
        if (!imu_console_stage_flush_complete(
                console, preparation->flush_ticket)) {
            return true;
        }

        if (!imu_console_render_progress(
                progress, text, sizeof(text))) {
            return false;
        }

        fputs(text, stdout);

        printf(
            "CAL status: accel=%u gyro=%u mag=%u rv=%u "
            "gate=%u sustained_ms=%" PRIu64 "\n",
            (unsigned)progress->cal.accelAccuracy,
            (unsigned)progress->cal.gyroAccuracy,
            (unsigned)progress->cal.magAccuracy,
            (unsigned)progress->cal.rvAccuracy,
            progress->cal.gatePassingNow ? 1u : 0u,
            progress->cal.sustainedGoodNs / UINT64_C(1000000));
    } else if (progress->active == IMU_CMD_ID_TARE) {
        printf(
            "TARE-Z: phase=%d remaining_ms=%" PRIu64
            "; keep the rig aligned at rest.\n",
            (int)progress->tare.phase,
            progress->tare.remainingNs / UINT64_C(1000000));
    } else if (progress->active == IMU_CMD_ID_SETTLE) {
        puts("SETTLE: restoring the production observation window.");
    } else {
        puts("STARTUP: preparing the next stage.");
    }

    preparation->last_status_ns = now;
    preparation->rendered_stage = progress->active;
    return fflush(stdout) == 0;
}

/*
 * Returns true only after successful calibration, successful persisted
 * and verified Z tare, and production settle.
 *
 * No CSV is opened here.
 */
static bool prepare_imu(
    amt102_t *encoder,
    ImuConsole_t *console)
{
    Preparation preparation;
    ImuCmdPlan_t plan;
    uint64_t next_wake = 0u;

    memset(&preparation, 0, sizeof(preparation));

    if (!imu_cmd_plan_acquire_default(&plan)) {
        return false;
    }

    plan.slot1 = IMU_CMD_ID_CALIBRATION;
    plan.slot2 = IMU_CMD_ID_TARE;
    plan.tareAxes = IMU_CMD_TARE_AXES_Z;
    plan.persistTare = true;
    plan.confirmTare = true;
    plan.flightCalMask = 0u;

    /*
     * The coordinator is used only through its acquisition-entry
     * boundary. Runtime trials are owned by this executable.
     */
    plan.acquisitionDurationS = 20u;

    if (!imu_cmd_init(&plan)) {
        fprintf(
            stderr, "Coordinator initialization failed: reason=%d\n",
            (int)imu_cmd_init_reason());
        return false;
    }

    puts(
        "Startup: guided calibration, then automatic persisted Z tare.\n"
        "Calibration retains its existing q and Enter inputs.\n"
        "Finish calibration with the rig in its intended resting pose.");

    while (!stop_requested) {
        ImuCmdProgress_t progress;
        ImuSampleSnapshot_t snapshot;
        ImuConsoleObservation_t observation;
        uint64_t now;

        imu_session_service();

        if (amt102_poll(encoder, 0) < 0) {
            perror("startup encoder poll");
            return false;
        }

        if (!imu_cmd_get_progress(&progress) ||
            !pump_preparation_adapter(progress.active) ||
            !monotonic_ns(&now)) {
            return false;
        }

        if (!imu_session_get_snapshot(&snapshot) ||
            snapshot.readerState == IMU_READER_STATE_CLOSED ||
            snapshot.readerState == IMU_READER_STATE_FAULTED ||
            snapshot.readerState == IMU_READER_STATE_RECOVERING) {
            fprintf(stderr, "Startup IMU session became unusable.\n");
            return false;
        }

        if (stop_requested) {
            break;
        }

        /*
         * A terminal result is checked before any subsequent stage can
         * be selected by another coordinator service call.
         */
        if (!check_preparation_results(&preparation)) {
            return false;
        }

        if (progress.active != preparation.input_stage) {
            preparation.input_stage = progress.active;
            preparation.flush_ticket =
                imu_console_request_stage_flush(console);

            if (preparation.flush_ticket == 0u) {
                return false;
            }
        }

        /*
         * TICK initializes a newly selected machine. Adapter pumping
         * occurred before this, so no new request is pumped in the same
         * turn in which its machine is initialized.
         */
        if (!post_event(IMU_CMD_EVENT_TICK, now) ||
            !imu_cmd_get_progress(&progress)) {
            return false;
        }

        if (!imu_console_observe(console, &observation) ||
            observation.inputError ||
            observation.eofObserved) {
            fprintf(stderr, "Startup terminal input unavailable.\n");
            return false;
        }

        if (progress.active == IMU_CMD_ID_CALIBRATION &&
            imu_console_stage_flush_complete(
                console, preparation.flush_ticket)) {
            ImuConsoleInput_t token;
            ImuCmdEvent_t event;

            if (imu_console_dequeue(console, &token) &&
                imu_console_translate(token, &progress, &event) &&
                !imu_cmd_post(&event)) {
                return false;
            }
        }

        /*
         * The coordinator enforces tare confirmation. This validation
         * owner supplies it automatically after successful calibration.
         */
        if (progress.active == IMU_CMD_ID_TARE &&
            progress.requiredAction ==
                IMU_CMD_ACTION_ALIGN_AND_CONFIRM &&
            progress.tare.confirmationPending) {
            if (!preparation.calibration_ok ||
                !post_event(IMU_CMD_EVENT_OPERATOR_CONFIRM, now)) {
                return false;
            }
        }

        if (progress.active == IMU_CMD_ID_SETTLE) {
            if (!preparation.calibration_ok || !preparation.tare_ok) {
                return false;
            }

            if (!preparation.settle_started) {
                /*
                 * Successful machine results already restored the
                 * production reports with flightCalMask == 0.
                 */
                if (!imu_session_begin_settle() ||
                    !monotonic_ns(&now)) {
                    return false;
                }

                preparation.settle_deadline_ns =
                    now + VALIDATE_SETTLE_NS;
                preparation.settle_started = true;
            }

            if (now >= preparation.settle_deadline_ns) {
                if (!imu_session_mark_operational() ||
                    !post_event(IMU_CMD_EVENT_SETTLE_DONE, now)) {
                    return false;
                }
            }
        }

        imu_cmd_service();

        if (!check_preparation_results(&preparation) ||
            !imu_cmd_get_progress(&progress) ||
            !monotonic_ns(&now)) {
            return false;
        }

        if (progress.active == IMU_CMD_ID_ACQUISITION) {
            return preparation.calibration_ok &&
                   preparation.tare_ok;
        }

        if (imu_cmd_plan_complete()) {
            fprintf(stderr, "Startup plan ended before READY.\n");
            return false;
        }

        if (!preparation_status(
                &preparation, console, &progress, now) ||
            !sleep_turn(&next_wake)) {
            return false;
        }
    }

    /*
     * Ctrl+C is a process stop, not an operator-q cancellation.
     * q cancellation instead remains in the loop through restoration.
     */
    {
        uint64_t now = 0u;
        (void)monotonic_ns(&now);
        (void)post_event(IMU_CMD_EVENT_PROCESS_STOP, now);
    }

    return false;
}

static void raw_terminal_close(RawTerminal *terminal)
{
    if (terminal->flags_changed) {
        (void)fcntl(
            STDIN_FILENO, F_SETFL, terminal->saved_flags);
        terminal->flags_changed = false;
    }

    if (terminal->attributes_changed) {
        (void)tcsetattr(
            STDIN_FILENO, TCSANOW, &terminal->saved);
        terminal->attributes_changed = false;
    }
}

static bool raw_terminal_open(RawTerminal *terminal)
{
    struct termios raw;

    memset(terminal, 0, sizeof(*terminal));

    if (tcgetattr(STDIN_FILENO, &terminal->saved) != 0) {
        return false;
    }

    terminal->saved_flags = fcntl(STDIN_FILENO, F_GETFL);
    if (terminal->saved_flags < 0) {
        return false;
    }

    raw = terminal->saved;

    /*
     * Preserve ISIG: Ctrl+C must still generate SIGINT.
     * Disable canonical input so 's' does not require Enter.
     */
    raw.c_lflag &= (tcflag_t)~(ICANON | ECHO);
    raw.c_lflag |= ISIG;
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
        return false;
    }
    terminal->attributes_changed = true;

    if (fcntl(
            STDIN_FILENO, F_SETFL,
            terminal->saved_flags | O_NONBLOCK) != 0) {
        raw_terminal_close(terminal);
        return false;
    }
    terminal->flags_changed = true;

    if (tcflush(STDIN_FILENO, TCIFLUSH) != 0) {
        raw_terminal_close(terminal);
        return false;
    }

    return true;
}

/*
 * Bound terminal work per service turn.
 * During RECORDING/FINALIZING all ordinary keys are discarded.
 */
static bool read_runtime_keys(bool ready, bool *start)
{
    unsigned pass;

    *start = false;

    for (pass = 0u; pass < 4u; ++pass) {
        char buffer[64];
        ssize_t length = read(STDIN_FILENO, buffer, sizeof(buffer));
        ssize_t i;

        if (length > 0) {
            for (i = 0; i < length; ++i) {
                if (ready && buffer[i] == 's') {
                    *start = true;
                }
            }
            continue;
        }

        if (length == 0) {
            errno = EIO;
            return false;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return true;
        }

        if (errno == EINTR) {
            return true;
        }

        return false;
    }

    return true;
}

static bool runtime_healthy(
    const ImuSampleSnapshot_t *snapshot,
    uint32_t expected_epoch,
    uint64_t ready_since,
    uint64_t now)
{
    const ImuGroupMeta_t *groups[3] = {
        &snapshot->rotationMeta,
        &snapshot->accelMeta,
        &snapshot->gyroMeta
    };
    const unsigned bits[3] = {
        IMU_GROUP_BIT_ROTATION,
        IMU_GROUP_BIT_ACCEL,
        IMU_GROUP_BIT_GYRO
    };
    unsigned i;

    /*
     * Do not silently recover/reconfigure between validation attempts.
     * A reset or epoch change ends this process and requires preparation
     * again on the next launch.
     */
    if (snapshot->readerState != IMU_READER_STATE_OPERATIONAL ||
        snapshot->configurationEpoch != expected_epoch) {
        return false;
    }

    for (i = 0u; i < 3u; ++i) {
        bool present = (snapshot->validMask & bits[i]) != 0u;
        uint64_t host_ns = groups[i]->hostDecodeNs;

        if (!present || host_ns == 0u) {
            if (now - ready_since > VALIDATE_WATCHDOG_NS) {
                return false;
            }
            continue;
        }

        if (host_ns > now ||
            now - host_ns > VALIDATE_WATCHDOG_NS) {
            return false;
        }
    }

    return true;
}

static const char *mode_name(ValidateMode mode)
{
    switch (mode) {
    case VALIDATE_READY:
        return "READY";
    case VALIDATE_RECORDING:
        return "RECORDING";
    case VALIDATE_FINALIZING:
    default:
        return "FINALIZING";
    }
}

static void print_runtime_status(
    ValidateMode mode,
    uint64_t trial,
    uint64_t start_ns,
    uint64_t now,
    const ImuSampleSnapshot_t *imu,
    const amt102_state_t *encoder,
    const ValidateLogger *logger)
{
    double elapsed = 0.0;
    double remaining = 0.0;
    double yaw = NAN;
    double pitch = NAN;
    double roll = NAN;
    double encoder_degrees =
        (double)encoder->relative_count * 360.0 /
        (double)AMT102_COUNTS_PER_REV;

    if (mode == VALIDATE_RECORDING && now >= start_ns) {
        elapsed = (double)(now - start_ns) / 1e9;
        remaining = 20.0 - elapsed;
        if (remaining < 0.0) {
            remaining = 0.0;
        }
    }

    if ((imu->validMask & IMU_GROUP_BIT_ROTATION) != 0u) {
        yaw = (double)imu->yaw * VALIDATE_RAD_TO_DEG;
        pitch = (double)imu->pitch * VALIDATE_RAD_TO_DEG;
        roll = (double)imu->roll * VALIDATE_RAD_TO_DEG;
    }

    printf(
        "%s trial=%" PRIu64
        " elapsed=%.3f remaining=%.3f"
        " enc=%.3fdeg y/p/r=%.3f/%.3f/%.3fdeg"
        " valid=0x%02x invalid=%" PRIu64
        " rows=%zu%s\n",
        mode_name(mode),
        trial,
        elapsed,
        remaining,
        encoder_degrees,
        yaw, pitch, roll,
        (unsigned)imu->validMask,
        encoder->invalid,
        logger->count,
        mode == VALIDATE_READY ? "  [s=start, Ctrl+C=exit]" : "");

    (void)fflush(stdout);
}

static bool finish_trial(
    ValidateLogger *logger,
    uint64_t trial,
    uint64_t start_ns,
    uint64_t end_ns,
    uint64_t observed_ns,
    uint64_t skipped,
    ValidateOutcome outcome)
{
    ValidateEnd end;

    memset(&end, 0, sizeof(end));
    end.trial = trial;
    end.start_ns = start_ns;
    end.end_ns = end_ns;
    end.observed_stop_ns = observed_ns;
    end.gates_skipped = skipped;
    end.signal_number = (int)stop_signal;
    end.outcome = outcome;

    return validate_logger_finish(logger, &end);
}

int main(int argc, char **argv)
{
    amt102_t *encoder = NULL;
    amt102_state_t encoder_state;
    ImuSampleSnapshot_t snapshot;
    ImuPublisherState_t publisher;
    ImuConsole_t console;
    RawTerminal terminal;
    ValidateLogger logger;

    ValidateMode mode = VALIDATE_READY;

    bool logger_initialized = false;
    bool logger_open = false;
    bool logger_finishing = false;
    bool session_open = false;
    bool console_initialized = false;
    bool console_started = false;

    uint64_t trial = 0u;
    uint64_t start_ns = 0u;
    uint64_t deadline_ns = 0u;
    uint64_t next_gate = 1u;
    uint64_t skipped_total = 0u;
    uint64_t next_wake = 0u;
    uint64_t last_status = 0u;
    uint64_t ready_since = 0u;
    uint64_t now = 0u;
    uint32_t expected_epoch = 0u;

    int exit_status = 0;

    (void)argv;
    memset(&terminal, 0, sizeof(terminal));
    memset(&logger, 0, sizeof(logger));
    memset(&encoder_state, 0, sizeof(encoder_state));
    memset(&snapshot, 0, sizeof(snapshot));
    memset(&publisher, 0, sizeof(publisher));

    if (argc != 1) {
        fprintf(
            stderr,
            "Usage: bno_validate\n"
            "No terminal arguments are accepted.\n");
        return 2;
    }

    if (!isatty(STDIN_FILENO)) {
        fprintf(stderr, "An interactive terminal is required.\n");
        return 1;
    }

    if (!install_signals()) {
        perror("signal setup");
        return 1;
    }

    if (!validate_logger_init(&logger)) {
        perror("logger initialization");
        return 1;
    }
    logger_initialized = true;

    if (amt102_open(&encoder) != 0) {
        perror("encoder open");
        exit_status = 1;
        goto cleanup;
    }

    if (!imu_console_init(&console, STDIN_FILENO)) {
        fprintf(stderr, "Console initialization failed.\n");
        exit_status = 1;
        goto cleanup;
    }
    console_initialized = true;

    if (!imu_console_start(&console)) {
        fprintf(stderr, "Console worker start failed.\n");
        exit_status = 1;
        goto cleanup;
    }
    console_started = true;

    puts("OPENING: initializing the IMU session.");
    (void)fflush(stdout);

    if (!imu_session_open()) {
        fprintf(stderr, "IMU session open failed.\n");
        exit_status = 1;
        goto cleanup;
    }
    session_open = true;

    if (!prepare_imu(encoder, &console)) {
        exit_status = stop_requested ? 0 : 1;
        goto cleanup;
    }

    imu_console_request_stop(&console);
    if (!imu_console_join(&console)) {
        fprintf(stderr, "Console worker join failed.\n");
        exit_status = 1;
        goto cleanup;
    }
    console_started = false;
    imu_console_destroy(&console);
    console_initialized = false;

    if (stop_requested) {
        goto cleanup;
    }

    if (!raw_terminal_open(&terminal)) {
        perror("single-key terminal setup");
        exit_status = 1;
        goto cleanup;
    }

    if (!imu_session_get_snapshot(&snapshot) ||
        !monotonic_ns(&ready_since)) {
        exit_status = 1;
        goto cleanup;
    }

    expected_epoch = snapshot.configurationEpoch;

    puts(
        "READY: no recording is active.\n"
        "Return the arm to rest before each s.\n"
        "Each s records 20 seconds; Ctrl+C exits.");

    while (!stop_requested) {
        bool start_key = false;

        imu_session_service();

        if (amt102_poll(encoder, 0) < 0) {
            perror("encoder poll");
            exit_status = 1;
            break;
        }
        amt102_get_state(encoder, &encoder_state);

        if (!imu_session_get_snapshot(&snapshot) ||
            !monotonic_ns(&now)) {
            fprintf(stderr, "Runtime snapshot or clock failed.\n");
            exit_status = 1;
            break;
        }

        if (!runtime_healthy(
                &snapshot, expected_epoch, ready_since, now)) {
            fprintf(
                stderr,
                "IMU reset, epoch change, unusable state, or "
                "report watchdog failure; stopping validation.\n");
            exit_status = 1;
            break;
        }

        if (!read_runtime_keys(
                mode == VALIDATE_READY, &start_key)) {
            perror("runtime terminal input");
            exit_status = 1;
            break;
        }

        if (stop_requested) {
            break;
        }

        if (mode == VALIDATE_READY && start_key) {
            /*
             * Prepare the file before establishing the timed window.
             * No sensor reconfiguration or IMU retare happens here.
             */
            if (!validate_logger_open(&logger)) {
                perror("CSV creation");
                exit_status = 1;
                break;
            }
            logger_open = true;
            ++trial;
            start_ns = 0u;
            deadline_ns = 0u;
            skipped_total = 0u;

            printf(
                "CSV: %s\n",
                validate_logger_filename(&logger));
            (void)fflush(stdout);

            if (stop_requested) {
                break;
            }

            if (amt102_zero(encoder) != 0) {
                perror("encoder zero");
                exit_status = 1;
                break;
            }
            amt102_get_state(encoder, &encoder_state);

            start_ns = encoder_state.zero_ts_ns;
            deadline_ns = start_ns + VALIDATE_WINDOW_NS;

            if (!imu_publish_begin_window(&publisher, &snapshot)) {
                fprintf(stderr, "Publisher baseline failed.\n");
                exit_status = 1;
                break;
            }

            next_gate = 1u;
            mode = VALIDATE_RECORDING;
            last_status = 0u;

            printf(
                "RECORDING trial=%" PRIu64
                ": zero established; 20-second window started.\n",
                trial);
        } else if (mode == VALIDATE_RECORDING) {
            if (now >= deadline_ns) {
                /*
                 * The admission boundary is the exact deadline.
                 * observed_stop_ns records owner-loop scheduling delay.
                 */
                bool finish_ok = finish_trial(
                    &logger, trial, start_ns, deadline_ns, now,
                    skipped_total, VALIDATE_OUTCOME_COMPLETE);

                logger_finishing = true;
                mode = VALIDATE_FINALIZING;
                last_status = 0u;

                if (!finish_ok) {
                    perror("logger finalization start");
                    exit_status = 1;
                    break;
                }
            } else {
                uint64_t due_gate =
                    (now - start_ns) / VALIDATE_PUBLICATION_NS;

                if (due_gate >= next_gate) {
                    ValidateRecord record;
                    uint64_t skipped = due_gate - next_gate;

                    memset(&record, 0, sizeof(record));
                    record.trial = trial;
                    record.elapsed_ns = now - start_ns;
                    record.encoder = encoder_state;

                    if (!imu_publish_evaluate_snapshot(
                            &publisher,
                            &snapshot,
                            start_ns +
                                due_gate * VALIDATE_PUBLICATION_NS,
                            true,
                            now,
                            true,
                            &record.imu) ||
                        !imu_publish_apply_timing(
                            &record.imu, (uint32_t)skipped)) {
                        fprintf(stderr, "Publication evaluation failed.\n");
                        exit_status = 1;
                        break;
                    }

                    /*
                     * The bounded buffer never silently drops a row.
                     * Buffer exhaustion is a fatal validation failure.
                     */
                    record.imu.loggerDrops = 0u;

                    if (stop_requested) {
                        break;
                    }

                    if (!validate_logger_append(&logger, &record)) {
                        perror("trial buffer append");
                        exit_status = 1;
                        break;
                    }

                    skipped_total += skipped;
                    next_gate = due_gate + 1u;
                }
            }
        } else if (mode == VALIDATE_FINALIZING &&
                   validate_logger_done(&logger)) {
            bool archive_failed = validate_logger_failed(&logger);

            if (!validate_logger_reap(&logger)) {
                perror("logger join");
                exit_status = 1;
                break;
            }

            logger_finishing = false;
            logger_open = false;

            if (archive_failed) {
                fprintf(
                    stderr, "CSV finalization failed: %s\n",
                    validate_logger_filename(&logger));
                exit_status = 1;
                break;
            }

            printf(
                "SAVED: %s rows=%zu skipped_gates=%" PRIu64 "\n",
                validate_logger_filename(&logger),
                logger.count,
                skipped_total);

            /*
             * Discard keys received during recording/finalization.
             * They must not become a queued next attempt.
             */
            if (tcflush(STDIN_FILENO, TCIFLUSH) != 0) {
                perror("READY input flush");
                exit_status = 1;
                break;
            }

            mode = VALIDATE_READY;
            last_status = 0u;
        }

        if (last_status == 0u ||
            now - last_status >= VALIDATE_STATUS_NS) {
            print_runtime_status(
                mode, trial, start_ns, now,
                &snapshot, &encoder_state, &logger);
            last_status = now;
        }

        if (!sleep_turn(&next_wake)) {
            perror("service pacing");
            exit_status = 1;
            break;
        }
    }

cleanup:
    /*
     * Freeze a partial trial before cleanup. A file opened immediately
     * before a zero/start failure is retained as aborted_before_start.
     */
    if (logger_open && !logger_finishing) {
        uint64_t end_ns = 0u;
        ValidateOutcome outcome;

        if (!monotonic_ns(&now)) {
            now = start_ns;
            exit_status = 1;
        }

        if (start_ns == 0u) {
            outcome = VALIDATE_OUTCOME_ABORTED_BEFORE_START;
        } else {
            end_ns = now < deadline_ns ? now : deadline_ns;
            if (end_ns < start_ns) {
                end_ns = start_ns;
            }

            outcome =
                exit_status != 0
                    ? VALIDATE_OUTCOME_FAILED
                    : VALIDATE_OUTCOME_INTERRUPTED;
        }

        if (!finish_trial(
                &logger, trial, start_ns, end_ns, now,
                skipped_total, outcome)) {
            perror("partial CSV finalization start");
            exit_status = 1;
        }
        logger_finishing = true;
    }

    if (console_started) {
        imu_console_request_stop(&console);
        if (!imu_console_join(&console)) {
            fprintf(stderr, "Console cleanup join failed.\n");
            exit_status = 1;
        } else {
            console_started = false;
        }
    }

    if (console_initialized && !console_started) {
        imu_console_destroy(&console);
    }

    raw_terminal_close(&terminal);

    /*
     * Do not free the trial buffer until the finalization worker exits.
     * Continue a 500 ms heartbeat during orderly shutdown.
     */
    if (logger_finishing) {
        uint64_t last_wait_status = 0u;

        while (!validate_logger_done(&logger)) {
            struct timespec pause = {0, 10000000L};
            uint64_t wait_now;

            if (session_open) {
                imu_session_service();
            }
            if (encoder != NULL) {
                (void)amt102_poll(encoder, 0);
            }

            if (monotonic_ns(&wait_now) &&
                (last_wait_status == 0u ||
                 wait_now - last_wait_status >= VALIDATE_STATUS_NS)) {
                printf(
                    "FINALIZING: %s\n",
                    validate_logger_filename(&logger));
                (void)fflush(stdout);
                last_wait_status = wait_now;
            }

            (void)nanosleep(&pause, NULL);
        }

        if (validate_logger_failed(&logger)) {
            fprintf(
                stderr, "CSV write/flush/sync/close failure: %s\n",
                validate_logger_filename(&logger));
            exit_status = 1;
        }

        if (!validate_logger_reap(&logger)) {
            perror("final logger join");
            exit_status = 1;
        } else {
            logger_finishing = false;
            printf(
                "CLOSED: %s rows=%zu\n",
                validate_logger_filename(&logger),
                logger.count);
        }
    }

    if (session_open) {
        imu_session_close();
    }

    amt102_close(encoder);

    if (logger_initialized && !logger_finishing) {
        validate_logger_destroy(&logger);
    }

    printf(
        "Validation stopped: status=%d signal=%d trials=%" PRIu64 "\n",
        exit_status, (int)stop_signal, trial);

    return exit_status;
}
