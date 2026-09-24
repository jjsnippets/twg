/* bno/app/imu_console.c */
#define _POSIX_C_SOURCE 200809L

#include "app/imu_console.h"

#include <errno.h>
#include <poll.h>
#include <sched.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define INPUT_POLL_MS 100

static ImuConsoleInput_t classify_payload(const char *payload)
{
    if (strcmp(payload, "q") == 0) {
        return IMU_CONSOLE_INPUT_Q;
    }
    if (strcmp(payload, "CLEAR") == 0) {
        return IMU_CONSOLE_INPUT_CLEAR;
    }
    if (strcmp(payload, "y") == 0) {
        return IMU_CONSOLE_INPUT_Y;
    }
    if (payload[0] == '\0') {
        return IMU_CONSOLE_INPUT_ENTER;
    }
    return IMU_CONSOLE_INPUT_INVALID;
}

static bool enqueue(ImuConsole_t *console, ImuConsoleInput_t input)
{
    bool accepted = false;

    if (pthread_mutex_lock(&console->mutex) != 0) {
        atomic_store(&console->inputError, true);
        return false;
    }
    if (console->count < IMU_CONSOLE_QUEUE_CAPACITY) {
        unsigned tail = (console->head + console->count) %
                        IMU_CONSOLE_QUEUE_CAPACITY;
        console->items[tail] = input;
        ++console->count;
        accepted = true;
    } else {
        atomic_fetch_add(&console->droppedLines, 1u);
    }
    (void)pthread_mutex_unlock(&console->mutex);
    return accepted;
}

bool imu_console_offer_line(ImuConsole_t *console, const char *payload)
{
    if (console == NULL || !console->initialized || payload == NULL) {
        return false;
    }
    /* The worker enforces this bound before calling offer_line(). */
    if (strlen(payload) >= IMU_CONSOLE_LINE_CAPACITY) {
        return enqueue(console, IMU_CONSOLE_INPUT_INVALID);
    }
    return enqueue(console, classify_payload(payload));
}

static void *input_worker(void *argument)
{
    ImuConsole_t *console = argument;
    char line[IMU_CONSOLE_LINE_CAPACITY];
    size_t length = 0u;
    bool discarding = false;

    atomic_store(&console->workerIsSchedOther,
                 sched_getscheduler(0) == SCHED_OTHER);

    while (!atomic_load(&console->stopRequested)) {
        struct pollfd fd;
        char bytes[64];
        ssize_t n;
        int ready;
        ssize_t i;

        fd.fd = console->inputFd;
        fd.events = POLLIN | POLLHUP;
        fd.revents = 0;
        ready = poll(&fd, 1, INPUT_POLL_MS);
        if (ready == 0) {
            continue;
        }
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            atomic_store(&console->inputError, true);
            break;
        }
        if ((fd.revents & (POLLERR | POLLNVAL)) != 0) {
            atomic_store(&console->inputError, true);
            break;
        }
        if ((fd.revents & (POLLIN | POLLHUP)) == 0) {
            continue;
        }

        n = read(console->inputFd, bytes, sizeof(bytes));
        if (n == 0) {
            /* EOF does not mean PROCESS_STOP and does not confirm a
             * partial line lacking its canonical line ending. */
            atomic_store(&console->eofObserved, true);
            break;
        }
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) {
                continue;
            }
            atomic_store(&console->inputError, true);
            break;
        }

        for (i = 0; i < n && !atomic_load(&console->stopRequested); ++i) {
            char c = bytes[i];

            if (c == '\n') {
                if (discarding) {
                    (void)enqueue(console, IMU_CONSOLE_INPUT_INVALID);
                } else {
                    /* Remove only the CR immediately preceding LF. */
                    if (length != 0u && line[length - 1u] == '\r') {
                        --length;
                    }
                    line[length] = '\0';
                    (void)imu_console_offer_line(console, line);
                }
                length = 0u;
                discarding = false;
            } else if (!discarding) {
                if (length + 1u < sizeof(line)) {
                    line[length++] = c;
                } else {
                    discarding = true;
                }
            }
        }
    }
    return NULL;
}

bool imu_console_init(ImuConsole_t *console, int inputFd)
{
    if (console == NULL || inputFd < 0) {
        return false;
    }
    memset(console, 0, sizeof(*console));
    console->inputFd = inputFd;
    if (pthread_mutex_init(&console->mutex, NULL) != 0) {
        return false;
    }
    atomic_init(&console->stopRequested, false);
    atomic_init(&console->eofObserved, false);
    atomic_init(&console->inputError, false);
    atomic_init(&console->workerIsSchedOther, false);
    atomic_init(&console->droppedLines, 0u);
    console->initialized = true;
    return true;
}

bool imu_console_start(ImuConsole_t *console)
{
    pthread_attr_t attributes;
    struct sched_param priority;
    int rc;

    if (console == NULL || !console->initialized ||
        console->workerStarted) {
        return false;
    }

    memset(&priority, 0, sizeof(priority));
    if (pthread_attr_init(&attributes) != 0) {
        return false;
    }
    rc = pthread_attr_setinheritsched(&attributes,
                                      PTHREAD_EXPLICIT_SCHED);
    if (rc == 0) {
        rc = pthread_attr_setschedpolicy(&attributes, SCHED_OTHER);
    }
    if (rc == 0) {
        rc = pthread_attr_setschedparam(&attributes, &priority);
    }
    if (rc == 0) {
        rc = pthread_create(&console->worker, &attributes,
                            input_worker, console);
    }
    (void)pthread_attr_destroy(&attributes);
    if (rc != 0) {
        return false;
    }
    console->workerStarted = true;
    return true;
}

void imu_console_request_stop(ImuConsole_t *console)
{
    if (console != NULL && console->initialized) {
        atomic_store(&console->stopRequested, true);
    }
}

bool imu_console_join(ImuConsole_t *console)
{
    if (console == NULL || !console->initialized) {
        return false;
    }
    if (!console->workerStarted) {
        return true;
    }
    if (pthread_join(console->worker, NULL) != 0) {
        return false;
    }
    console->workerStarted = false;
    return true;
}

void imu_console_destroy(ImuConsole_t *console)
{
    if (console == NULL || !console->initialized) {
        return;
    }
    imu_console_request_stop(console);
    if (!imu_console_join(console)) {
        /* Do not destroy a mutex that may still belong to the worker. */
        return;
    }
    (void)pthread_mutex_destroy(&console->mutex);
    console->initialized = false;
}

bool imu_console_dequeue(ImuConsole_t *console, ImuConsoleInput_t *out)
{
    bool haveItem = false;

    if (console == NULL || !console->initialized || out == NULL ||
        pthread_mutex_trylock(&console->mutex) != 0) {
        return false;
    }
    if (console->count != 0u) {
        *out = console->items[console->head];
        console->head = (console->head + 1u) %
                        IMU_CONSOLE_QUEUE_CAPACITY;
        --console->count;
        haveItem = true;
    }
    (void)pthread_mutex_unlock(&console->mutex);
    return haveItem;
}

bool imu_console_observe(const ImuConsole_t *console,
                         ImuConsoleObservation_t *out)
{
    if (console == NULL || !console->initialized || out == NULL) {
        return false;
    }
    out->eofObserved = atomic_load(&console->eofObserved);
    out->inputError = atomic_load(&console->inputError);
    out->workerIsSchedOther = atomic_load(&console->workerIsSchedOther);
    out->droppedLines = atomic_load(&console->droppedLines);
    return true;
}

static bool command_active(ImuCmdIdentity_t identity)
{
    return identity >= IMU_CMD_ID_CALIBRATION &&
           identity <= IMU_CMD_ID_PROBE;
}

bool imu_console_translate(ImuConsoleInput_t input,
                           const ImuCmdProgress_t *progress,
                           ImuCmdEvent_t *out)
{
    ImuCmdEventType_t eventType;

    if (progress == NULL || out == NULL ||
        !command_active(progress->active)) {
        return false;
    }

    switch (input) {
    case IMU_CONSOLE_INPUT_Q:
        if (progress->requiredAction == IMU_CMD_ACTION_NONE) {
            return false;
        }
        eventType = IMU_CMD_EVENT_OPERATOR_Q;
        break;
    case IMU_CONSOLE_INPUT_CLEAR:
        if ((progress->active != IMU_CMD_ID_DCD_CLEAR &&
             progress->active != IMU_CMD_ID_TARE_CLEAR) ||
            progress->requiredAction != IMU_CMD_ACTION_CONFIRM) {
            return false;
        }
        eventType = IMU_CMD_EVENT_OPERATOR_CONFIRM;
        break;
    case IMU_CONSOLE_INPUT_Y:
        if (progress->active != IMU_CMD_ID_TARE ||
            progress->requiredAction !=
                IMU_CMD_ACTION_ALIGN_AND_CONFIRM) {
            return false;
        }
        eventType = IMU_CMD_EVENT_OPERATOR_CONFIRM;
        break;
    case IMU_CONSOLE_INPUT_ENTER:
        if (progress->active != IMU_CMD_ID_CALIBRATION ||
            progress->requiredAction !=
                IMU_CMD_ACTION_PRESS_Q_TO_END ||
            progress->cal.deadlineNs != 0ull ||
            (progress->cal.phase != IMU_CMD_CAL_PHASE_ACCEL &&
             progress->cal.phase != IMU_CMD_CAL_PHASE_GYRO &&
             progress->cal.phase != IMU_CMD_CAL_PHASE_MAG &&
             progress->cal.phase != IMU_CMD_CAL_PHASE_VERIFY)) {
            return false;
        }
        eventType = IMU_CMD_EVENT_OPERATOR_CONFIRM;
        break;
    case IMU_CONSOLE_INPUT_NONE:
    case IMU_CONSOLE_INPUT_INVALID:
    default:
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->type = eventType;
    return true;
}

static bool appendf(char *buffer, size_t capacity, size_t *used,
                    const char *format, ...)
{
    va_list arguments;
    int written;

    if (*used >= capacity) {
        return false;
    }
    va_start(arguments, format);
    written = vsnprintf(buffer + *used, capacity - *used,
                        format, arguments);
    va_end(arguments);
    if (written < 0 || (size_t)written >= capacity - *used) {
        return false;
    }
    *used += (size_t)written;
    return true;
}

static const char *identity_name(ImuCmdIdentity_t identity)
{
    switch (identity) {
    case IMU_CMD_ID_CALIBRATION: return "CALIBRATION";
    case IMU_CMD_ID_DCD_CLEAR:   return "DCD_CLEAR";
    case IMU_CMD_ID_TARE:        return "TARE";
    case IMU_CMD_ID_TARE_CLEAR:  return "TARE_CLEAR";
    case IMU_CMD_ID_TARE_CHECK:  return "TARE_CHECK";
    case IMU_CMD_ID_CHECK:       return "CHECK";
    case IMU_CMD_ID_PROBE:       return "PROBE";
    case IMU_CMD_ID_SETTLE:      return "SETTLE";
    case IMU_CMD_ID_ACQUISITION: return "ACQUISITION";
    case IMU_CMD_ID_NONE:
    default:                     return "NONE";
    }
}

static const char *state_name(ImuCmdResultState_t state)
{
    switch (state) {
    case IMU_CMD_STATE_NOT_REQUESTED:  return "NOT_REQUESTED";
    case IMU_CMD_STATE_RUNNING:        return "RUNNING";
    case IMU_CMD_STATE_SUCCEEDED:      return "SUCCEEDED";
    case IMU_CMD_STATE_FAILED:         return "FAILED";
    case IMU_CMD_STATE_CANCELLED:      return "CANCELLED";
    case IMU_CMD_STATE_TIMED_OUT:      return "TIMED_OUT";
    case IMU_CMD_STATE_RECOVERY_FAILED:return "RECOVERY_FAILED";
    case IMU_CMD_STATE_ABANDONED:      return "ABANDONED";
    default:                           return "UNKNOWN";
    }
}

static const char *reason_name(ImuCmdReason_t reason)
{
    switch (reason) {
    case IMU_CMD_REASON_NONE:            return "NONE";
    case IMU_CMD_REASON_OK:              return "OK";
    case IMU_CMD_REASON_OPERATOR_Q:      return "OPERATOR_Q";
    case IMU_CMD_REASON_DCD_CLEAR_FAILED:return "DCD_CLEAR_FAILED";
    case IMU_CMD_REASON_PROBE_DEADLINE:  return "PROBE_DEADLINE";
    case IMU_CMD_REASON_CONFIG_FAILED:   return "CONFIG_FAILED";
    case IMU_CMD_REASON_SESSION_UNUSABLE:return "SESSION_UNUSABLE";
    case IMU_CMD_REASON_PROCESS_STOP:    return "PROCESS_STOP";
    default:                             return "OTHER";
    }
}

bool imu_console_render_progress(const ImuCmdProgress_t *progress,
                                 char *buffer, size_t capacity)
{
    size_t used = 0u;

    if (progress == NULL || buffer == NULL || capacity == 0u ||
        progress->version != IMU_CMD_PROGRESS_VERSION) {
        return false;
    }
    buffer[0] = '\0';

    if (!appendf(buffer, capacity, &used,
                 "R8 version=%u identity=%s action=%d"
                 " calPhase=%d tarePhase=%d checkPhase=%d\n",
                 progress->version, identity_name(progress->active),
                 (int)progress->requiredAction,
                 (int)progress->cal.phase,
                 (int)progress->tare.phase,
                 (int)progress->check.phase)) {
        return false;
    }
    if (progress->active == IMU_CMD_ID_CALIBRATION ||
        progress->active == IMU_CMD_ID_DCD_CLEAR) {
        return appendf(buffer, capacity, &used,
                       "cal pose=%u/%u accelRound=%u magRound=%u"
                       " elapsedFromStateNs=%llu deadlineNs=%llu"
                       " remainingNs=%llu sustainedGoodNs=%llu"
                       " accelStatus=%u gyroStatus=%u magStatus=%u"
                       " rvStatus=%u gatePassingNow=%u\n",
                       progress->cal.currentPoseIndex,
                       progress->cal.totalPoseCount,
                       progress->cal.accelRound,
                       progress->cal.magRound,
                       (unsigned long long)progress->cal.stateEntryNs,
                       (unsigned long long)progress->cal.deadlineNs,
                       (unsigned long long)progress->cal.remainingNs,
                       (unsigned long long)progress->cal.sustainedGoodNs,
                       progress->cal.accelAccuracy,
                       progress->cal.gyroAccuracy,
                       progress->cal.magAccuracy,
                       progress->cal.rvAccuracy,
                       progress->cal.gatePassingNow ? 1u : 0u);
    }
    if (progress->active == IMU_CMD_ID_TARE ||
        progress->active == IMU_CMD_ID_TARE_CLEAR ||
        progress->active == IMU_CMD_ID_TARE_CHECK) {
        return appendf(buffer, capacity, &used,
                       "tare axes=%d confirmPending=%u"
                       " attitudeValid=%u epochMatched=%u"
                       " deadlineNs=%llu remainingNs=%llu"
                       " yawRad=%.4f pitchRad=%.4f rollRad=%.4f"
                       " verificationEvidence=%u\n",
                       (int)progress->tare.requestedAxes,
                       progress->tare.confirmationPending ? 1u : 0u,
                       progress->tare.attitudeValid ? 1u : 0u,
                       progress->tare.attitudeEpochMatched ? 1u : 0u,
                       (unsigned long long)progress->tare.deadlineNs,
                       (unsigned long long)progress->tare.remainingNs,
                       (double)progress->tare.yawRad,
                       (double)progress->tare.pitchRad,
                       (double)progress->tare.rollRad,
                       progress->tare.verificationEvidence ? 1u : 0u);
    }
    if (progress->active == IMU_CMD_ID_CHECK ||
        progress->active == IMU_CMD_ID_PROBE) {
        return appendf(buffer, capacity, &used,
                       "check requestedMaskValid=%u requestedMask=0x%02x"
                       " effectiveMask=0x%02x actualMaskValid=%u"
                       " actualMask=0x%02x factsEpoch=%u"
                       " factsEpochMatched=%u deadlineValid=%u"
                       " deadlineNs=%llu remainingNs=%llu"
                       " sustainedGoodNs=%llu"
                       " haveAccel=%u accelStatus=%u"
                       " haveGyro=%u gyroStatus=%u"
                       " haveMag=%u magStatus=%u"
                       " haveRv=%u rvStatus=%u"
                       " gatePassingNow=%u gateReached=%u\n",
                       progress->check.requestedMaskValid ? 1u : 0u,
                       progress->check.requestedMask,
                       progress->check.effectiveMask,
                       progress->check.actualMaskValid ? 1u : 0u,
                       progress->check.actualMask,
                       progress->check.factsEpoch,
                       progress->check.factsEpochMatched ? 1u : 0u,
                       progress->check.deadlineValid ? 1u : 0u,
                       (unsigned long long)progress->check.deadlineNs,
                       (unsigned long long)progress->check.remainingNs,
                       (unsigned long long)progress->check.sustainedGoodNs,
                       progress->check.haveAccel ? 1u : 0u,
                       progress->check.accelStatus,
                       progress->check.haveGyro ? 1u : 0u,
                       progress->check.gyroStatus,
                       progress->check.haveMag ? 1u : 0u,
                       progress->check.magStatus,
                       progress->check.haveRv ? 1u : 0u,
                       progress->check.rvStatus,
                       progress->check.gatePassingNow ? 1u : 0u,
                       progress->check.gateReached ? 1u : 0u);
    }
    return true;
}

bool imu_console_render_result(const ImuCmdResult_t *result,
                               bool doNotAcquire,
                               char *buffer, size_t capacity)
{
    size_t used = 0u;

    if (result == NULL || buffer == NULL || capacity == 0u ||
        result->version != IMU_CMD_RESULT_VERSION) {
        return false;
    }
    buffer[0] = '\0';

    if (!appendf(buffer, capacity, &used,
                 "R7 version=%u identity=%s state=%s"
                 " reason=%s reasonCode=%d warningRequired=%u"
                 " doNotAcquire=%u startedNs=%llu endedNs=%llu"
                 " epochBefore=%u epochAfter=%u"
                 " restoredProduction=%u dcdSaved=%u verified=%u\n",
                 result->version, identity_name(result->identity),
                 state_name(result->state),
                 reason_name(result->reason), (int)result->reason,
                 result->warningRequired ? 1u : 0u,
                 doNotAcquire ? 1u : 0u,
                 (unsigned long long)result->startedNs,
                 (unsigned long long)result->endedNs,
                 result->epochBefore, result->epochAfter,
                 result->restoredProduction ? 1u : 0u,
                 result->dcdSaved ? 1u : 0u,
                 result->verified ? 1u : 0u)) {
        return false;
    }
    if (result->identity == IMU_CMD_ID_CHECK ||
        result->identity == IMU_CMD_ID_PROBE) {
        return appendf(buffer, capacity, &used,
                       "check requestedMaskValid=%u requestedMask=0x%02x"
                       " actualMaskValid=%u actualMask=0x%02x"
                       " gatePassingNow=%u gateReached=%u"
                       " probeTimedOut=%u operatorEndedEarly=%u"
                       " probeReachedGate=%u\n",
                       result->probeMaskRequestedValid ? 1u : 0u,
                       result->probeMaskRequested,
                       result->sub.probeMaskActualValid ? 1u : 0u,
                       result->sub.probeMaskActual,
                       result->terminalProgress.check.gatePassingNow
                           ? 1u : 0u,
                       result->terminalProgress.check.gateReached
                           ? 1u : 0u,
                       result->sub.probeTimedOut ? 1u : 0u,
                       result->sub.probeOperatorEndedEarly ? 1u : 0u,
                       result->sub.probeReachedGate ? 1u : 0u);
    }
    if (result->identity == IMU_CMD_ID_TARE ||
        result->identity == IMU_CMD_ID_TARE_CLEAR ||
        result->identity == IMU_CMD_ID_TARE_CHECK) {
        return appendf(buffer, capacity, &used,
                       "tare tareNow=%d persist=%d"
                       " clearActive=%d clearSaved=%d"
                       " requestedAxes=%d verificationEvidence=%u\n",
                       (int)result->sub.tareNow,
                       (int)result->sub.persist,
                       (int)result->sub.clearActive,
                       (int)result->sub.clearSaved,
                       (int)result->requestedTareAxes,
                       result->terminalProgress.tare.verificationEvidence
                           ? 1u : 0u);
    }
    return true;
}
