/* bno/app/imu_console.h */
#ifndef IMU_CONSOLE_H
#define IMU_CONSOLE_H

#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app/imu_cmd.h"

#define IMU_CONSOLE_QUEUE_CAPACITY 8u
#define IMU_CONSOLE_LINE_CAPACITY 64u

typedef enum {
    IMU_CONSOLE_INPUT_NONE = 0,
    IMU_CONSOLE_INPUT_Q,
    IMU_CONSOLE_INPUT_CLEAR,
    IMU_CONSOLE_INPUT_Y,
    IMU_CONSOLE_INPUT_ENTER,
    IMU_CONSOLE_INPUT_INVALID
} ImuConsoleInput_t;

typedef struct {
    ImuConsoleInput_t items[IMU_CONSOLE_QUEUE_CAPACITY];
    unsigned head;
    unsigned count;
    pthread_mutex_t mutex;
    pthread_t worker;
    int inputFd;                 /* Borrowed; this adapter never closes it. */
    bool initialized;
    bool workerStarted;
    _Atomic bool stopRequested;
    _Atomic bool eofObserved;
    _Atomic bool inputError;
    _Atomic bool workerIsSchedOther;
    _Atomic uint64_t droppedLines;
} ImuConsole_t;

typedef struct {
    bool eofObserved;
    bool inputError;
    bool workerIsSchedOther;
    uint64_t droppedLines;
} ImuConsoleObservation_t;

/* Initialize before owner-thread RT promotion. fd must remain open to join. */
bool imu_console_init(ImuConsole_t *console, int inputFd);
bool imu_console_start(ImuConsole_t *console);
void imu_console_request_stop(ImuConsole_t *console);
bool imu_console_join(ImuConsole_t *console);
void imu_console_destroy(ImuConsole_t *console);

/*
 * Producer-side/test seam: exact payload WITHOUT its line ending.
 * Returns false if the bounded queue is full or input is invalid.
 * Full-queue rejection increments droppedLines; no accepted item is replaced.
 */
bool imu_console_offer_line(ImuConsole_t *console, const char *payload);

/* Owner side: try-lock only; false means no item or temporarily contended. */
bool imu_console_dequeue(ImuConsole_t *console, ImuConsoleInput_t *out);
bool imu_console_observe(const ImuConsole_t *console,
                         ImuConsoleObservation_t *out);

/* Pure, context-dependent mapping. False means ignore; out is untouched. */
bool imu_console_translate(ImuConsoleInput_t input,
                           const ImuCmdProgress_t *progress,
                           ImuCmdEvent_t *out);

/* Pure R8 and R7 formatters: no terminal write or clock ownership. */
bool imu_console_render_progress(const ImuCmdProgress_t *progress,
                                 char *buffer, size_t capacity);
bool imu_console_render_result(const ImuCmdResult_t *result,
                               bool doNotAcquire,
                               char *buffer, size_t capacity);

#endif /* IMU_CONSOLE_H */
