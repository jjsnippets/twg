#include "ms5837_publish.h"

#include <limits.h>
#include <math.h>
#include <string.h>

static bool sample_contract_valid(const BaroSample_t *sample)
{
    return (sample != NULL) &&
           (sample->contract_version == BARO_SAMPLE_CONTRACT_VERSION) &&
           ((sample->status_flags & BARO_STATUS_RESERVED_MASK) == 0U);
}

static bool sample_ready(
    const BaroSample_t *sample,
    bool available)
{
    return available &&
           sample_contract_valid(sample) &&
           (sample->measurement_sequence != 0U) &&
           ((sample->status_flags & BARO_STATUS_NOT_READY) == 0U);
}

bool ms5837_publish_begin_window(
    Ms5837PublisherState_t *state,
    const BaroSample_t *baseline,
    bool baseline_ready)
{
    if (state == NULL) {
        return false;
    }

    memset(state, 0, sizeof(*state));
    state->next_publication_sequence = 1U;
    state->window_open = true;

    if (sample_ready(baseline, baseline_ready)) {
        state->previous_measurement_sequence =
            baseline->measurement_sequence;
        state->previous_valid = true;
    }

    return true;
}

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
    Ms5837PublicationRecord_t *out)
{
    bool ready;
    bool late;

    if ((state == NULL) ||
        !state->window_open ||
        (out == NULL)) {
        return false;
    }

    memset(out, 0, sizeof(*out));

    out->version = MS5837_PUBLICATION_CONTRACT_VERSION;
    out->publication_sequence =
        state->next_publication_sequence++;
    out->scheduled_ns =
        scheduled_valid ? scheduled_ns : 0U;
    out->actual_ns =
        actual_valid ? actual_ns : 0U;
    out->gates_skipped_before =
        gates_skipped_before;
    out->driver_state = driver_state;
    out->scheduled_valid =
        scheduled_valid ? 1U : 0U;
    out->actual_valid =
        actual_valid ? 1U : 0U;

    late =
        !scheduled_valid ||
        !actual_valid ||
        (gates_skipped_before != 0U) ||
        ((actual_ns > scheduled_ns) &&
         ((actual_ns - scheduled_ns) >
          MS5837_PUBLISH_T_LATE_NS));

    out->deadline_missed = late ? 1U : 0U;

    ready = sample_ready(sample, sample_available);
    out->sample_ready = ready ? 1U : 0U;
    out->not_ready = ready ? 0U : 1U;

    if (!ready) {
        out->sample.contract_version =
            BARO_SAMPLE_CONTRACT_VERSION;
        out->sample.status_flags =
            BARO_STATUS_NOT_READY;
        out->sample.pressure_mbar = NAN;
        out->sample.temperature_c = NAN;
        return true;
    }

    out->sample = *sample;

    if (!state->previous_valid ||
        (sample->measurement_sequence >
         state->previous_measurement_sequence)) {
        out->sample_fresh = 1U;
    } else {
        out->sample_stale = 1U;
    }

    state->previous_measurement_sequence =
        sample->measurement_sequence;
    state->previous_valid = true;

    if (actual_valid &&
        (sample->measurement_complete_time_ns != 0U) &&
        (actual_ns >= sample->measurement_complete_time_ns)) {
        out->age_valid = 1U;
        out->age_ns =
            actual_ns -
            sample->measurement_complete_time_ns;
    }

    return true;
}

bool ms5837_publish_gate_begin(
    Ms5837PublishGate_t *gate,
    uint64_t tick1_ns,
    bool tick1_valid,
    uint64_t window_ns)
{
    uint64_t whole_ticks;

    if ((gate == NULL) ||
        (window_ns == 0U) ||
        (window_ns >
         UINT64_MAX -
         (MS5837_PUBLISH_SERVICE_PERIOD_NS - 1U))) {
        return false;
    }

    /*
     * Service tick k starts inside the window when:
     *
     *     (k - 1) * T_service < window_ns
     */
    whole_ticks =
        (window_ns +
         MS5837_PUBLISH_SERVICE_PERIOD_NS - 1U) /
        MS5837_PUBLISH_SERVICE_PERIOD_NS;

    memset(gate, 0, sizeof(*gate));

    gate->max_gate =
        whole_ticks / MS5837_PUBLISH_GATE_TICKS;

    if (gate->max_gate == 0U) {
        return false;
    }

    gate->tick1_ns =
        tick1_valid ? tick1_ns : 0U;
    gate->tick1_valid = tick1_valid;
    gate->active = true;

    return true;
}

Ms5837PublishGateStatus_t ms5837_publish_gate_advance(
    Ms5837PublishGate_t *gate,
    uint64_t ticks,
    Ms5837PublishGateDecision_t *out)
{
    uint64_t current;
    uint64_t skipped;
    uint64_t offset_ticks;

    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }

    if ((gate == NULL) ||
        (out == NULL) ||
        !gate->active ||
        (ticks == 0U) ||
        (gate->tick_index > UINT64_MAX - ticks)) {
        return MS5837_PUBLISH_GATE_ERROR;
    }

    if (gate->last_gate >= gate->max_gate) {
        return MS5837_PUBLISH_GATE_COMPLETE;
    }

    gate->tick_index += ticks;

    current =
        gate->tick_index / MS5837_PUBLISH_GATE_TICKS;

    if (current > gate->max_gate) {
        current = gate->max_gate;
    }

    if (current <= gate->last_gate) {
        return MS5837_PUBLISH_GATE_IDLE;
    }

    skipped =
        current - gate->last_gate - 1U;

    if (skipped > UINT32_MAX) {
        return MS5837_PUBLISH_GATE_ERROR;
    }

    out->gate_index = current;
    out->gates_skipped_before =
        (uint32_t)skipped;

    offset_ticks =
        current * MS5837_PUBLISH_GATE_TICKS - 1U;

    if (gate->tick1_valid &&
        (offset_ticks <=
         (UINT64_MAX - gate->tick1_ns) /
         MS5837_PUBLISH_SERVICE_PERIOD_NS)) {
        out->scheduled_ns =
            gate->tick1_ns +
            offset_ticks *
            MS5837_PUBLISH_SERVICE_PERIOD_NS;
        out->scheduled_valid = true;
    }

    gate->last_gate = current;
    gate->gates_skipped_total += skipped;

    return MS5837_PUBLISH_GATE_DUE;
}

bool ms5837_publish_gate_window_done(
    const Ms5837PublishGate_t *gate)
{
    return (gate != NULL) &&
           gate->active &&
           (gate->last_gate >= gate->max_gate);
}
