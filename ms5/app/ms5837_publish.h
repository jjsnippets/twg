#ifndef MS5837_PUBLISH_H
#define MS5837_PUBLISH_H

#include <stdbool.h>
#include <stdint.h>

#include "ms5837_contract.h"

#define MS5837_PUBLICATION_CONTRACT_VERSION 1U
#define MS5837_RUN_STATS_CONTRACT_VERSION 1U

#define MS5837_PUBLISH_SERVICE_PERIOD_NS UINT64_C(1000000)
#define MS5837_PUBLISH_GATE_TICKS UINT32_C(10)
#define MS5837_PUBLISH_T_LATE_NS UINT64_C(1000000)

typedef struct {
    uint64_t previous_measurement_sequence;
    uint64_t next_publication_sequence;
    bool previous_valid;
    bool window_open;
} Ms5837PublisherState_t;

typedef struct {
    uint32_t version;
    uint32_t reserved;
    uint64_t service_ticks;
    uint64_t service_overruns;
    uint64_t clock_read_failures;
    uint64_t publications_attempted;
    uint64_t publications_enqueued;
    uint64_t publications_not_ready;
    uint64_t publications_fresh;
    uint64_t publications_stale;
    uint64_t deadline_misses;
    uint64_t gates_skipped;
    uint64_t logger_drops;
    uint64_t maximum_loop_body_ns;
} Ms5837RunStats_t;

typedef struct {
    uint32_t version;
    uint32_t reserved;
    uint64_t publication_sequence;
    uint64_t scheduled_ns;
    uint64_t actual_ns;
    uint64_t age_ns;
    uint32_t gates_skipped_before;
    int32_t driver_state;
    uint8_t scheduled_valid;
    uint8_t actual_valid;
    uint8_t deadline_missed;
    uint8_t sample_ready;
    uint8_t not_ready;
    uint8_t sample_fresh;
    uint8_t sample_stale;
    uint8_t age_valid;
    BaroSample_t sample;
} Ms5837PublicationRecord_t;

typedef enum {
    MS5837_PUBLISH_GATE_IDLE = 0,
    MS5837_PUBLISH_GATE_DUE,
    MS5837_PUBLISH_GATE_COMPLETE,
    MS5837_PUBLISH_GATE_ERROR
} Ms5837PublishGateStatus_t;

typedef struct {
    uint64_t gate_index;
    uint64_t scheduled_ns;
    uint32_t gates_skipped_before;
    bool scheduled_valid;
} Ms5837PublishGateDecision_t;

typedef struct {
    uint64_t tick1_ns;
    uint64_t tick_index;
    uint64_t last_gate;
    uint64_t max_gate;
    uint64_t gates_skipped_total;
    bool tick1_valid;
    bool active;
} Ms5837PublishGate_t;

bool ms5837_publish_begin_window(
    Ms5837PublisherState_t *state,
    const BaroSample_t *baseline,
    bool baseline_ready);

bool ms5837_publish_evaluate(
    Ms5837PublisherState_t *state,
    const BaroSample_t *sample,
    bool sample_available,
    int32_t driver_state,
    uint64_t scheduled_ns,
    bool scheduled_valid,
    uint64_t actual_ns,
    bool actual_valid,
    uint32_t gates_skipped_before,
    Ms5837PublicationRecord_t *out);

bool ms5837_publish_gate_begin(
    Ms5837PublishGate_t *gate,
    uint64_t tick1_ns,
    bool tick1_valid,
    uint64_t window_ns);

Ms5837PublishGateStatus_t ms5837_publish_gate_advance(
    Ms5837PublishGate_t *gate,
    uint64_t ticks,
    Ms5837PublishGateDecision_t *out);

bool ms5837_publish_gate_window_done(
    const Ms5837PublishGate_t *gate);

#endif /* MS5837_PUBLISH_H */
