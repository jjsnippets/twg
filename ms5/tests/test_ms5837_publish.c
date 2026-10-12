#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ms5837_publish.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf( \
            stderr, \
            "FAIL %s:%d: %s\n", \
            __FILE__, \
            __LINE__, \
            #condition); \
        return 1; \
    } \
} while (0)

static BaroSample_t make_sample(
    uint64_t sequence,
    uint64_t complete_ns)
{
    BaroSample_t value;

    memset(&value, 0, sizeof(value));

    value.contract_version =
        BARO_SAMPLE_CONTRACT_VERSION;
    value.measurement_sequence = sequence;
    value.measurement_complete_time_ns =
        complete_ns;
    value.raw_pressure_d1 = 6465444U;
    value.raw_temperature_d2 = 8077636U;
    value.pressure_mbar = 1000.25;
    value.temperature_c = 20.0;

    return value;
}

static int test_fresh_stale_and_not_ready(void)
{
    Ms5837PublisherState_t state;
    Ms5837PublicationRecord_t record;
    BaroSample_t baseline =
        make_sample(10U, 900U);
    BaroSample_t next =
        make_sample(11U, 1500U);

    CHECK(ms5837_publish_begin_window(
        &state,
        &baseline,
        true));

    CHECK(ms5837_publish_evaluate(
        &state,
        &next,
        true,
        1,
        2000U,
        true,
        2500U,
        true,
        0U,
        &record));

    CHECK(
        record.version ==
        MS5837_PUBLICATION_CONTRACT_VERSION);
    CHECK(record.publication_sequence == 1U);
    CHECK(record.sample_ready == 1U);
    CHECK(record.not_ready == 0U);
    CHECK(record.sample_fresh == 1U);
    CHECK(record.sample_stale == 0U);
    CHECK(record.age_valid == 1U);
    CHECK(record.age_ns == 1000U);
    CHECK(record.deadline_missed == 0U);

    CHECK(ms5837_publish_evaluate(
        &state,
        &next,
        true,
        1,
        3000U,
        true,
        3500U,
        true,
        0U,
        &record));

    CHECK(record.publication_sequence == 2U);
    CHECK(record.sample_fresh == 0U);
    CHECK(record.sample_stale == 1U);
    CHECK(record.age_ns == 2000U);

    CHECK(ms5837_publish_evaluate(
        &state,
        NULL,
        false,
        5,
        4000U,
        true,
        4500U,
        true,
        0U,
        &record));

    CHECK(record.publication_sequence == 3U);
    CHECK(record.sample_ready == 0U);
    CHECK(record.not_ready == 1U);
    CHECK(
        record.sample.status_flags ==
        BARO_STATUS_NOT_READY);
    CHECK(isnan(record.sample.pressure_mbar));

    return 0;
}

static int test_age_and_deadline_guards(void)
{
    Ms5837PublisherState_t state;
    Ms5837PublicationRecord_t record;
    BaroSample_t value =
        make_sample(
            1U,
            UINT64_C(5000000));

    CHECK(ms5837_publish_begin_window(
        &state,
        NULL,
        false));

    /*
     * The sample completion time is in the future relative to actual_ns.
     * Age must be invalid instead of wrapping an unsigned subtraction.
     */
    CHECK(ms5837_publish_evaluate(
        &state,
        &value,
        true,
        1,
        UINT64_C(1000000),
        true,
        UINT64_C(3000001),
        true,
        0U,
        &record));

    CHECK(record.age_valid == 0U);
    CHECK(record.age_ns == 0U);
    CHECK(record.deadline_missed == 1U);

    CHECK(ms5837_publish_evaluate(
        &state,
        &value,
        true,
        1,
        0U,
        false,
        0U,
        false,
        0U,
        &record));

    CHECK(record.age_valid == 0U);
    CHECK(record.deadline_missed == 1U);

    CHECK(ms5837_publish_evaluate(
        &state,
        &value,
        true,
        1,
        UINT64_C(6000000),
        true,
        UINT64_C(6000000),
        true,
        1U,
        &record));

    CHECK(record.deadline_missed == 1U);
    CHECK(record.gates_skipped_before == 1U);

    return 0;
}

static int test_contract_rejection(void)
{
    Ms5837PublisherState_t state;
    Ms5837PublicationRecord_t record;
    BaroSample_t value =
        make_sample(1U, 1U);

    CHECK(ms5837_publish_begin_window(
        &state,
        NULL,
        false));

    value.contract_version = 999U;

    CHECK(ms5837_publish_evaluate(
        &state,
        &value,
        true,
        1,
        10U,
        true,
        10U,
        true,
        0U,
        &record));

    CHECK(record.not_ready == 1U);

    value = make_sample(2U, 2U);
    value.status_flags =
        BARO_STATUS_RESERVED_MASK;

    CHECK(ms5837_publish_evaluate(
        &state,
        &value,
        true,
        1,
        20U,
        true,
        20U,
        true,
        0U,
        &record));

    CHECK(record.not_ready == 1U);

    return 0;
}

static int test_gate_grid_and_skips(void)
{
    Ms5837PublishGate_t gate;
    Ms5837PublishGateDecision_t decision;
    Ms5837PublishGateStatus_t status;
    unsigned int index;

    CHECK(ms5837_publish_gate_begin(
        &gate,
        UINT64_C(1000000),
        true,
        UINT64_C(30000000)));

    for (index = 0U; index < 9U; ++index) {
        status = ms5837_publish_gate_advance(
            &gate,
            1U,
            &decision);

        CHECK(
            status ==
            MS5837_PUBLISH_GATE_IDLE);
    }

    status = ms5837_publish_gate_advance(
        &gate,
        1U,
        &decision);

    CHECK(
        status ==
        MS5837_PUBLISH_GATE_DUE);
    CHECK(decision.gate_index == 1U);
    CHECK(decision.scheduled_valid);
    CHECK(
        decision.scheduled_ns ==
        UINT64_C(10000000));
    CHECK(
        decision.gates_skipped_before == 0U);

    /*
     * Advance across two boundaries. Gate 2 is skipped and gate 3 is
     * emitted. No backfill row is created.
     */
    status = ms5837_publish_gate_advance(
        &gate,
        20U,
        &decision);

    CHECK(
        status ==
        MS5837_PUBLISH_GATE_DUE);
    CHECK(decision.gate_index == 3U);
    CHECK(
        decision.gates_skipped_before == 1U);
    CHECK(gate.gates_skipped_total == 1U);
    CHECK(
        ms5837_publish_gate_window_done(
            &gate));

    CHECK(
        ms5837_publish_gate_advance(
            &gate,
            1U,
            &decision) ==
        MS5837_PUBLISH_GATE_COMPLETE);

    return 0;
}

int main(void)
{
    CHECK(
        test_fresh_stale_and_not_ready() == 0);
    CHECK(
        test_age_and_deadline_guards() == 0);
    CHECK(
        test_contract_rejection() == 0);
    CHECK(
        test_gate_grid_and_skips() == 0);

    puts(
        "PASS: MS5837 pure publisher and 100 Hz gate");

    return 0;
}
