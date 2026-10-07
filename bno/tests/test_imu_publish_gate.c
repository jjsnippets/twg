#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_publish.h"

static int s_failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++s_failures;                                                    \
        }                                                                    \
    } while (0)

#define T 1000000ull
#define ANCHOR 5000000000ull

static ImuPublishGateDecision_t step(ImuPublishGate_t *g, uint64_t ticks,
                                     ImuPublishGateStatus_t expect)
{
    ImuPublishGateDecision_t d;
    ImuPublishGateStatus_t st = imu_publish_gate_advance(g, ticks, &d);

    CHECK(st == expect);
    return d;
}

static void test_begin_validation(void)
{
    ImuPublishGate_t g;

    CHECK(!imu_publish_gate_begin(NULL, ANCHOR, true, 10000000000ull));
    CHECK(!imu_publish_gate_begin(&g, ANCHOR, true, 0u));
    CHECK(!imu_publish_gate_begin(&g, ANCHOR, true, 9u * T));
    CHECK(!imu_publish_gate_begin(&g, ANCHOR, true, UINT64_MAX));

    CHECK(imu_publish_gate_begin(&g, ANCHOR, true, 10000000000ull));
    CHECK(g.maxGate == 1000u);
    CHECK(imu_publish_gate_begin(&g, ANCHOR, true, 10000000001ull));
    CHECK(g.maxGate == 1000u);
    CHECK(imu_publish_gate_begin(&g, ANCHOR, true, 10010000000ull));
    CHECK(g.maxGate == 1001u);
    CHECK(imu_publish_gate_begin(&g, ANCHOR, true, 10u * T));
    CHECK(g.maxGate == 1u);
    CHECK(!imu_publish_gate_window_done(&g));
}

static void test_normal_cadence_full_window(void)
{
    ImuPublishGate_t g;
    ImuPublishGateDecision_t d;
    uint64_t due = 0u;
    uint64_t lastScheduled = 0u;
    uint64_t k;

    CHECK(imu_publish_gate_begin(&g, ANCHOR, true, 10000000000ull));

    for (k = 1u; k <= 10000u; ++k) {
        ImuPublishGateStatus_t st = imu_publish_gate_advance(&g, 1u, &d);

        if (k % 10u != 0u) {
            CHECK(st == IMU_PUBLISH_GATE_IDLE);
            continue;
        }
        CHECK(st == IMU_PUBLISH_GATE_DUE);
        CHECK(d.gateIndex == k / 10u);
        CHECK(d.gatesSkippedBefore == 0u);
        CHECK(d.scheduledValid == 1u);
        CHECK(d.scheduledNs == ANCHOR + (k - 1u) * T);
        ++due;
        lastScheduled = d.scheduledNs;
    }

    CHECK(due == 1000u);
    CHECK(lastScheduled == ANCHOR + 9999u * T);
    CHECK(lastScheduled < ANCHOR + 10000000000ull);
    CHECK(g.gatesSkippedTotal == 0u);
    CHECK(imu_publish_gate_window_done(&g));
    (void)step(&g, 1u, IMU_PUBLISH_GATE_COMPLETE);
}

static void test_first_gate_and_late_turn(void)
{
    ImuPublishGate_t g;
    ImuPublishGateDecision_t d;
    ImuPublicationRecord_t r;

    CHECK(imu_publish_gate_begin(&g, ANCHOR, true, 10000000000ull));
    (void)step(&g, 8u, IMU_PUBLISH_GATE_IDLE);

    d = step(&g, 3u, IMU_PUBLISH_GATE_DUE);
    CHECK(d.gateIndex == 1u);
    CHECK(d.gatesSkippedBefore == 0u);
    CHECK(d.scheduledNs == ANCHOR + 9u * T);

    memset(&r, 0, sizeof(r));
    r.version = IMU_PUBLICATION_CONTRACT_VERSION;
    r.scheduledValid = r.actualValid = 1u;
    r.scheduledNs = d.scheduledNs;
    r.actualNs = d.scheduledNs + 2u * T;
    CHECK(imu_publish_apply_timing(&r, d.gatesSkippedBefore));
    CHECK(r.deadlineMissed == 1u);
    CHECK(r.gatesSkippedBefore == 0u);
}

static void test_skipped_gates_counted_not_backfilled(void)
{
    ImuPublishGate_t g;
    ImuPublishGateDecision_t d;
    ImuPublicationRecord_t r;
    uint64_t k;

    CHECK(imu_publish_gate_begin(&g, ANCHOR, true, 10000000000ull));
    for (k = 1u; k < 100u; ++k) {
        (void)step(&g, 1u, (k % 10u == 0u) ? IMU_PUBLISH_GATE_DUE
                                           : IMU_PUBLISH_GATE_IDLE);
    }
    d = step(&g, 1u, IMU_PUBLISH_GATE_DUE);
    CHECK(d.gateIndex == 10u && d.gatesSkippedBefore == 0u);

    d = step(&g, 25u, IMU_PUBLISH_GATE_DUE);
    CHECK(d.gateIndex == 12u);
    CHECK(d.gatesSkippedBefore == 1u);
    CHECK(d.scheduledNs == ANCHOR + 119u * T);
    CHECK(g.gatesSkippedTotal == 1u);

    memset(&r, 0, sizeof(r));
    r.version = IMU_PUBLICATION_CONTRACT_VERSION;
    r.scheduledValid = r.actualValid = 1u;
    r.scheduledNs = d.scheduledNs;
    r.actualNs = d.scheduledNs;
    CHECK(imu_publish_apply_timing(&r, d.gatesSkippedBefore));
    CHECK(r.gatesSkippedBefore == 1u);
    CHECK(r.deadlineMissed == 1u);

    (void)step(&g, 4u, IMU_PUBLISH_GATE_IDLE);
    d = step(&g, 1u, IMU_PUBLISH_GATE_DUE);
    CHECK(d.gateIndex == 13u && d.gatesSkippedBefore == 0u);

    d = step(&g, 120u, IMU_PUBLISH_GATE_DUE);
    CHECK(d.gateIndex == 25u);
    CHECK(d.gatesSkippedBefore == 11u);
    CHECK(g.gatesSkippedTotal == 12u);
}

static void test_lateness_boundary(void)
{
    ImuPublicationRecord_t r;

    memset(&r, 0, sizeof(r));
    r.version = IMU_PUBLICATION_CONTRACT_VERSION;
    r.scheduledValid = r.actualValid = 1u;
    r.scheduledNs = 1000u * T;

    r.actualNs = r.scheduledNs - 5u;
    CHECK(imu_publish_apply_timing(&r, 0u) && r.deadlineMissed == 0u);
    r.actualNs = r.scheduledNs;
    CHECK(imu_publish_apply_timing(&r, 0u) && r.deadlineMissed == 0u);
    r.actualNs = r.scheduledNs + IMU_PUBLISH_T_LATE_NS;
    CHECK(imu_publish_apply_timing(&r, 0u) && r.deadlineMissed == 0u);
    r.actualNs = r.scheduledNs + IMU_PUBLISH_T_LATE_NS + 1u;
    CHECK(imu_publish_apply_timing(&r, 0u) && r.deadlineMissed == 1u);

    r.actualNs = r.scheduledNs;
    r.actualValid = 0u;
    CHECK(imu_publish_apply_timing(&r, 0u) && r.deadlineMissed == 1u);
    r.actualValid = 1u;
    r.scheduledValid = 0u;
    CHECK(imu_publish_apply_timing(&r, 0u) && r.deadlineMissed == 1u);

    CHECK(!imu_publish_apply_timing(NULL, 0u));
    r.version = 0u;
    CHECK(!imu_publish_apply_timing(&r, 0u));
}

static void test_window_end_clamp_and_early_stop(void)
{
    ImuPublishGate_t g;
    ImuPublishGateDecision_t d;
    uint64_t k;
    unsigned rows = 0u;

    CHECK(imu_publish_gate_begin(&g, ANCHOR, true, 1000u * T));
    CHECK(g.maxGate == 100u);
    d = step(&g, 5000u, IMU_PUBLISH_GATE_DUE);
    CHECK(d.gateIndex == 100u);
    CHECK(d.gatesSkippedBefore == 99u);
    CHECK(d.scheduledNs == ANCHOR + 999u * T);
    CHECK(d.scheduledNs < ANCHOR + 1000u * T);
    CHECK(imu_publish_gate_window_done(&g));
    (void)step(&g, 1u, IMU_PUBLISH_GATE_COMPLETE);

    CHECK(imu_publish_gate_begin(&g, ANCHOR, true, 10000000000ull));
    for (k = 1u; k <= 95u; ++k) {
        ImuPublishGateStatus_t st = imu_publish_gate_advance(&g, 1u, &d);

        if (st == IMU_PUBLISH_GATE_DUE) {
            ++rows;
        }
    }
    CHECK(rows == 9u);
    CHECK(g.gatesSkippedTotal == 0u);
    CHECK(!imu_publish_gate_window_done(&g));
}

static void test_invalid_anchor_and_errors(void)
{
    ImuPublishGate_t g;
    ImuPublishGateDecision_t d;
    ImuPublicationRecord_t r;

    CHECK(imu_publish_gate_begin(&g, 123u, false, 10000000000ull));
    (void)step(&g, 9u, IMU_PUBLISH_GATE_IDLE);
    d = step(&g, 1u, IMU_PUBLISH_GATE_DUE);
    CHECK(d.gateIndex == 1u);
    CHECK(d.scheduledValid == 0u && d.scheduledNs == 0u);

    memset(&r, 0, sizeof(r));
    r.version = IMU_PUBLICATION_CONTRACT_VERSION;
    r.actualValid = 1u;
    r.actualNs = 7u;
    CHECK(imu_publish_apply_timing(&r, d.gatesSkippedBefore));
    CHECK(r.deadlineMissed == 1u);

    CHECK(imu_publish_gate_begin(&g, UINT64_MAX - 25u * T, true,
                                 10000000000ull));
    (void)step(&g, 9u, IMU_PUBLISH_GATE_IDLE);
    d = step(&g, 1u, IMU_PUBLISH_GATE_DUE);
    CHECK(d.scheduledValid == 1u);
    d = step(&g, 10u, IMU_PUBLISH_GATE_DUE);
    CHECK(d.scheduledValid == 1u);
    d = step(&g, 10u, IMU_PUBLISH_GATE_DUE);
    CHECK(d.gateIndex == 3u);
    CHECK(d.scheduledValid == 0u && d.scheduledNs == 0u);

    CHECK(imu_publish_gate_begin(&g, ANCHOR, true, 10000000000ull));
    CHECK(imu_publish_gate_advance(&g, 0u, &d) == IMU_PUBLISH_GATE_ERROR);
    CHECK(imu_publish_gate_advance(&g, 1u, NULL) == IMU_PUBLISH_GATE_ERROR);
    CHECK(imu_publish_gate_advance(NULL, 1u, &d) == IMU_PUBLISH_GATE_ERROR);
    g.tickIndex = UINT64_MAX - 1u;
    CHECK(imu_publish_gate_advance(&g, 5u, &d) == IMU_PUBLISH_GATE_ERROR);
    memset(&g, 0, sizeof(g));
    CHECK(imu_publish_gate_advance(&g, 1u, &d) == IMU_PUBLISH_GATE_ERROR);
}

static ImuSampleSnapshot_t snap(uint32_t epoch, ImuReaderState_t state,
                                uint8_t mask, uint64_t rv, uint64_t acc,
                                uint64_t gyr)
{
    ImuSampleSnapshot_t s;
    ImuGroupMeta_t *m[3];
    uint64_t seq[3];
    unsigned i;

    memset(&s, 0, sizeof(s));
    s.version = IMU_SAMPLE_CONTRACT_VERSION;
    s.configurationEpoch = epoch;
    s.readerState = state;
    s.validMask = mask;
    m[0] = &s.rotationMeta; m[1] = &s.accelMeta; m[2] = &s.gyroMeta;
    seq[0] = rv; seq[1] = acc; seq[2] = gyr;
    for (i = 0u; i < 3u; ++i) {
        m[i]->version = IMU_METADATA_CONTRACT_VERSION;
        m[i]->configurationEpoch = epoch;
        m[i]->groupEventSeq = seq[i];
        m[i]->hostDecodeNs = (mask & (1u << i)) ? 1000u + i : 0u;
    }
    return s;
}

static void test_publisher_history_survives_gaps(void)
{
    ImuPublisherState_t p;
    ImuPublishGate_t g;
    ImuPublishGateDecision_t d;
    ImuSampleSnapshot_t base = snap(3u, IMU_READER_STATE_OPERATIONAL,
                                    IMU_GROUP_MASK_REQUIRED, 10u, 20u, 30u);
    ImuSampleSnapshot_t s;
    ImuPublicationRecord_t r;

    CHECK(imu_publish_begin_window(&p, &base));
    CHECK(imu_publish_gate_begin(&g, ANCHOR, true, 10000000000ull));

    (void)step(&g, 9u, IMU_PUBLISH_GATE_IDLE);
    d = step(&g, 1u, IMU_PUBLISH_GATE_DUE);
    s = snap(3u, IMU_READER_STATE_OPERATIONAL, IMU_GROUP_MASK_REQUIRED,
             11u, 21u, 31u);
    CHECK(imu_publish_evaluate_snapshot(&p, &s, d.scheduledNs, true,
                                        d.scheduledNs, true, &r));
    CHECK(imu_publish_apply_timing(&r, d.gatesSkippedBefore));
    CHECK(r.publicationSeq == 1u && r.freshMask == IMU_GROUP_MASK_REQUIRED);

    d = step(&g, 31u, IMU_PUBLISH_GATE_DUE);
    CHECK(d.gateIndex == 4u && d.gatesSkippedBefore == 2u);
    s = snap(3u, IMU_READER_STATE_RECOVERING, 0u, 0u, 0u, 0u);
    CHECK(imu_publish_evaluate_snapshot(&p, &s, d.scheduledNs, true,
                                        d.scheduledNs, true, &r));
    CHECK(imu_publish_apply_timing(&r, d.gatesSkippedBefore));
    CHECK(r.publicationSeq == 2u);
    CHECK(r.gatesSkippedBefore == 2u && r.deadlineMissed == 1u);
    CHECK(r.missingMask == IMU_GROUP_MASK_REQUIRED);
    CHECK(r.sampleReady == 0u && r.notReady == 1u);

    d = step(&g, 10u, IMU_PUBLISH_GATE_DUE);
    CHECK(d.gateIndex == 5u && d.gatesSkippedBefore == 0u);
    s = snap(3u, IMU_READER_STATE_OPERATIONAL, IMU_GROUP_MASK_REQUIRED,
             14u, 21u, 32u);
    CHECK(imu_publish_evaluate_snapshot(&p, &s, d.scheduledNs, true,
                                        d.scheduledNs, true, &r));
    CHECK(r.publicationSeq == 3u);
    CHECK(r.freshMask == IMU_GROUP_MASK_REQUIRED);
    CHECK(r.staleMask == 0u);
    CHECK(r.multiUpdateMask == 0u);
    CHECK(r.gatesSkippedBefore == 0u);
}

int main(void)
{
    test_begin_validation();
    test_normal_cadence_full_window();
    test_first_gate_and_late_turn();
    test_skipped_gates_counted_not_backfilled();
    test_lateness_boundary();
    test_window_end_clamp_and_early_stop();
    test_invalid_anchor_and_errors();
    test_publisher_history_survives_gaps();

    if (s_failures != 0) {
        fprintf(stderr, "test_imu_publish_gate: %d failure(s)\n", s_failures);
        return 1;
    }

    puts("test_imu_publish_gate: PASS");
    return 0;
}
