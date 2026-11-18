#ifndef VALIDATE_CONTRACT_H
#define VALIDATE_CONTRACT_H

#include <stdint.h>

#include "app/imu_publish.h"
#include "validation/amt102.h"

#define VALIDATE_SCHEMA_VERSION 2u

#define VALIDATE_SERVICE_NS UINT64_C(1000000)
#define VALIDATE_PUBLICATION_NS UINT64_C(10000000)
#define VALIDATE_STATUS_NS UINT64_C(500000000)
#define VALIDATE_WINDOW_NS UINT64_C(20000000000)

/*
 * At most one publication per distinct 10 ms gate.
 * 2048 records leave headroom above a 20-second, 100 Hz trial.
 */
#define VALIDATE_RECORD_CAPACITY 2048u
#define VALIDATE_FILENAME_CAPACITY 64u

typedef struct {
    uint64_t trial;
    uint64_t elapsed_ns;
    amt102_state_t encoder;
    ImuPublicationRecord_t imu;
} ValidateRecord;

typedef enum {
    VALIDATE_OUTCOME_COMPLETE = 0,
    VALIDATE_OUTCOME_INTERRUPTED,
    VALIDATE_OUTCOME_FAILED,
    VALIDATE_OUTCOME_ABORTED_BEFORE_START
} ValidateOutcome;

typedef struct {
    uint64_t trial;
    uint64_t start_ns;
    uint64_t end_ns;
    uint64_t observed_stop_ns;
    uint64_t gates_skipped;
    int signal_number;
    ValidateOutcome outcome;
} ValidateEnd;

#endif /* VALIDATE_CONTRACT_H */
