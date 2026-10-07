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

static ImuGroupMeta_t *meta_for(ImuSampleSnapshot_t *s, unsigned index)
{
    switch (index) {
    case IMU_PUBLISH_GROUP_ROTATION:
        return &s->rotationMeta;
    case IMU_PUBLISH_GROUP_ACCEL:
        return &s->accelMeta;
    default:
        return &s->gyroMeta;
    }
}

static ImuSampleSnapshot_t snapshot(uint32_t epoch,
                                    ImuReaderState_t readerState,
                                    uint8_t validMask,
                                    uint64_t rvSeq,
                                    uint64_t accSeq,
                                    uint64_t gyroSeq,
                                    uint64_t hostNs)
{
    ImuSampleSnapshot_t s;
    uint64_t seqs[IMU_PUBLISH_GROUP_COUNT] = {
        rvSeq, accSeq, gyroSeq
    };
    unsigned i;

    memset(&s, 0, sizeof(s));
    s.version = IMU_SAMPLE_CONTRACT_VERSION;
    s.configurationEpoch = epoch;
    s.readerState = readerState;
    s.validMask = validMask;

    for (i = 0u; i < IMU_PUBLISH_GROUP_COUNT; ++i) {
        ImuGroupMeta_t *meta = meta_for(&s, i);
        uint8_t bit = (uint8_t)IMU_PUBLISH_GROUP_BIT(i);

        meta->version = IMU_METADATA_CONTRACT_VERSION;
        meta->configurationEpoch = epoch;
        meta->groupEventSeq = seqs[i];
        meta->hostDecodeNs =
            (validMask & bit) != 0u ? hostNs + i : 0u;
        meta->sensorTimeUs = 100u + i;
        meta->deviceReportSeq = (uint8_t)(10u + i);
        meta->rawStatus = (uint8_t)(2u + i);
    }

    s.qw = 1.0f;
    s.ax = 2.0f;
    s.gx = 3.0f;
    return s;
}

static ImuPublicationRecord_t evaluate(ImuPublisherState_t *publisher,
                                       const ImuSampleSnapshot_t *s,
                                       uint64_t actualNs)
{
    ImuPublicationRecord_t r;

    CHECK(imu_publish_evaluate_snapshot(
        publisher, s, actualNs - 100u, true, actualNs, true, &r));

    return r;
}

static void check_partition(const ImuPublicationRecord_t *r)
{
    unsigned i;

    CHECK((r->validMask & ~IMU_GROUP_MASK_REQUIRED) == 0u);
    CHECK((r->freshMask & ~IMU_GROUP_MASK_REQUIRED) == 0u);
    CHECK((r->staleMask & ~IMU_GROUP_MASK_REQUIRED) == 0u);
    CHECK((r->missingMask & ~IMU_GROUP_MASK_REQUIRED) == 0u);
    CHECK((r->multiUpdateMask & ~IMU_GROUP_MASK_REQUIRED) == 0u);
    CHECK((r->multiUpdateMask & ~r->freshMask) == 0u);

    for (i = 0u; i < IMU_PUBLISH_GROUP_COUNT; ++i) {
        uint8_t bit = (uint8_t)IMU_PUBLISH_GROUP_BIT(i);
        unsigned classes =
            ((r->freshMask & bit) != 0u) +
            ((r->staleMask & bit) != 0u) +
            ((r->missingMask & bit) != 0u);

        CHECK(classes == 1u);
        CHECK(((r->validMask & bit) != 0u) ==
              (((r->freshMask | r->staleMask) & bit) != 0u));
    }
}

static void test_baseline_and_advances(void)
{
    ImuPublisherState_t p;
    ImuSampleSnapshot_t base =
        snapshot(7u, IMU_READER_STATE_OPERATIONAL,
                 IMU_GROUP_MASK_REQUIRED,
                 10u, 20u, 30u, 1000u);
    ImuSampleSnapshot_t s;
    ImuPublicationRecord_t r;

    CHECK(imu_publish_begin_window(&p, &base));

    r = evaluate(&p, &base, 2000u);
    CHECK(r.publicationSeq == 1u);
    CHECK(r.validMask == IMU_GROUP_MASK_REQUIRED);
    CHECK(r.freshMask == 0u);
    CHECK(r.staleMask == IMU_GROUP_MASK_REQUIRED);
    CHECK(r.missingMask == 0u);
    CHECK(r.multiUpdateMask == 0u);
    CHECK(r.sampleReady == 1u && r.notReady == 0u);
    check_partition(&r);

    s = snapshot(7u, IMU_READER_STATE_OPERATIONAL,
                 IMU_GROUP_MASK_REQUIRED,
                 11u, 21u, 31u, 2100u);

    r = evaluate(&p, &s, 3000u);
    CHECK(r.publicationSeq == 2u);
    CHECK(r.freshMask == IMU_GROUP_MASK_REQUIRED);
    CHECK(r.staleMask == 0u);
    CHECK(r.multiUpdateMask == 0u);
    CHECK(r.rotation.ageValid == 1u);
    CHECK(r.rotation.ageNs == 900u);
    CHECK(r.accel.ageValid == 1u);
    CHECK(r.accel.ageNs == 899u);
    CHECK(r.gyro.ageValid == 1u);
    CHECK(r.gyro.ageNs == 898u);
    check_partition(&r);

    s = snapshot(7u, IMU_READER_STATE_OPERATIONAL,
                 IMU_GROUP_MASK_REQUIRED,
                 14u, 22u, 31u, 3100u);

    r = evaluate(&p, &s, 4000u);
    CHECK(r.freshMask ==
          (IMU_GROUP_BIT_ROTATION | IMU_GROUP_BIT_ACCEL));
    CHECK(r.staleMask == IMU_GROUP_BIT_GYRO);
    CHECK(r.multiUpdateMask == IMU_GROUP_BIT_ROTATION);
    check_partition(&r);
}

static void test_missing_and_first_decode(void)
{
    ImuPublisherState_t p;
    ImuSampleSnapshot_t base =
        snapshot(8u, IMU_READER_STATE_OPERATIONAL,
                 IMU_GROUP_BIT_ROTATION,
                 50u, 0u, 0u, 1000u);
    ImuSampleSnapshot_t s;
    ImuPublicationRecord_t r;

    CHECK(imu_publish_begin_window(&p, &base));

    r = evaluate(&p, &base, 2000u);
    CHECK(r.validMask == IMU_GROUP_BIT_ROTATION);
    CHECK(r.staleMask == IMU_GROUP_BIT_ROTATION);
    CHECK(r.missingMask ==
          (IMU_GROUP_BIT_ACCEL | IMU_GROUP_BIT_GYRO));
    CHECK(r.accel.identityPresent == 0u);
    CHECK(r.accel.ageValid == 0u);
    CHECK(r.accel.ageNs == 0u);
    CHECK(r.notReady == 1u);
    check_partition(&r);

    s = snapshot(8u, IMU_READER_STATE_OPERATIONAL,
                 IMU_GROUP_BIT_ROTATION | IMU_GROUP_BIT_ACCEL,
                 50u, 75u, 0u, 2100u);

    r = evaluate(&p, &s, 3000u);
    CHECK(r.freshMask == IMU_GROUP_BIT_ACCEL);
    CHECK(r.multiUpdateMask == 0u);
    CHECK(r.missingMask == IMU_GROUP_BIT_GYRO);
    check_partition(&r);
}

static void test_epoch_change_invalidates_previous(void)
{
    ImuPublisherState_t p;
    ImuSampleSnapshot_t base =
        snapshot(10u, IMU_READER_STATE_OPERATIONAL,
                 IMU_GROUP_MASK_REQUIRED,
                 100u, 200u, 300u, 1000u);
    ImuSampleSnapshot_t changed =
        snapshot(11u, IMU_READER_STATE_RECOVERING,
                 IMU_GROUP_MASK_REQUIRED,
                 101u, 201u, 301u, 2000u);
    ImuSampleSnapshot_t sameNewEpoch;
    ImuSampleSnapshot_t decoded;
    ImuPublicationRecord_t r;

    CHECK(imu_publish_begin_window(&p, &base));

    r = evaluate(&p, &changed, 3000u);
    CHECK(r.validMask == 0u);
    CHECK(r.freshMask == 0u);
    CHECK(r.staleMask == 0u);
    CHECK(r.missingMask == IMU_GROUP_MASK_REQUIRED);
    CHECK(r.sampleReady == 0u);
    CHECK(r.notReady == 1u);
    check_partition(&r);

    sameNewEpoch = changed;
    sameNewEpoch.readerState = IMU_READER_STATE_OPERATIONAL;

    r = evaluate(&p, &sameNewEpoch, 4000u);
    CHECK(r.freshMask == 0u);
    CHECK(r.staleMask == IMU_GROUP_MASK_REQUIRED);
    CHECK(r.multiUpdateMask == 0u);
    check_partition(&r);

    decoded = snapshot(11u, IMU_READER_STATE_OPERATIONAL,
                       IMU_GROUP_MASK_REQUIRED,
                       102u, 202u, 302u, 4100u);

    r = evaluate(&p, &decoded, 5000u);
    CHECK(r.freshMask == IMU_GROUP_MASK_REQUIRED);
    CHECK(r.multiUpdateMask == 0u);
    check_partition(&r);
}

static void test_ineligible_reader_states(void)
{
    static const ImuReaderState_t states[] = {
        IMU_READER_STATE_SETTLING,
        IMU_READER_STATE_CONFIGURING,
        IMU_READER_STATE_RECOVERING,
        IMU_READER_STATE_FAULTED
    };
    unsigned i;

    for (i = 0u; i < sizeof(states) / sizeof(states[0]); ++i) {
        ImuPublisherState_t p;
        ImuSampleSnapshot_t base =
            snapshot(12u, IMU_READER_STATE_OPERATIONAL,
                     IMU_GROUP_MASK_REQUIRED,
                     1u, 1u, 1u, 1000u);
        ImuSampleSnapshot_t s =
            snapshot(12u, states[i],
                     IMU_GROUP_MASK_REQUIRED,
                     2u, 2u, 2u, 2000u);
        ImuPublicationRecord_t r;

        CHECK(imu_publish_begin_window(&p, &base));

        r = evaluate(&p, &s, 3000u);
        CHECK(r.readerState == states[i]);
        CHECK(r.sampleReady == 0u);
        CHECK(r.notReady == 1u);
        CHECK(r.freshMask == IMU_GROUP_MASK_REQUIRED);
        check_partition(&r);
    }
}

static void test_age_edges_and_invalid_inputs(void)
{
    ImuPublisherState_t p;
    ImuSampleSnapshot_t base =
        snapshot(13u, IMU_READER_STATE_OPERATIONAL,
                 IMU_GROUP_MASK_REQUIRED,
                 1u, 1u, 1u, 1000u);
    ImuSampleSnapshot_t s =
        snapshot(13u, IMU_READER_STATE_OPERATIONAL,
                 IMU_GROUP_MASK_REQUIRED,
                 2u, 2u, 2u, 4000u);
    ImuPublicationRecord_t r;

    CHECK(!imu_publish_begin_window(NULL, &base));
    CHECK(!imu_publish_begin_window(&p, NULL));

    base.configurationEpoch = IMU_EPOCH_NONE;
    CHECK(!imu_publish_begin_window(&p, &base));

    base.configurationEpoch = 13u;
    base.readerState = IMU_READER_STATE_SETTLING;
    CHECK(!imu_publish_begin_window(&p, &base));

    base.readerState = IMU_READER_STATE_OPERATIONAL;
    CHECK(imu_publish_begin_window(&p, &base));

    meta_for(&s, IMU_PUBLISH_GROUP_ROTATION)->hostDecodeNs = 5001u;
    meta_for(&s, IMU_PUBLISH_GROUP_ACCEL)->hostDecodeNs = 0u;
    meta_for(&s, IMU_PUBLISH_GROUP_GYRO)->hostDecodeNs = 5000u;

    r = evaluate(&p, &s, 5000u);
    CHECK(r.rotation.ageValid == 0u);
    CHECK(r.rotation.ageNs == 0u);
    CHECK(r.accel.ageValid == 0u);
    CHECK(r.accel.ageNs == 0u);
    CHECK(r.gyro.ageValid == 1u);
    CHECK(r.gyro.ageNs == 0u);
    CHECK(r.deadlineMissed == 0u);
    CHECK(r.gatesSkippedBefore == 0u);
    check_partition(&r);

    CHECK(imu_publish_evaluate_snapshot(
        &p, &s, 999u, false, 888u, false, &r));

    CHECK(r.scheduledValid == 0u);
    CHECK(r.scheduledNs == 0u);
    CHECK(r.actualValid == 0u);
    CHECK(r.actualNs == 0u);
    CHECK(r.rotation.ageValid == 0u);
    CHECK(r.rotation.ageNs == 0u);
}

static void test_invariant_sweep(void)
{
    unsigned baselineMask;
    unsigned rowMask;

    for (baselineMask = 0u;
         baselineMask <= IMU_GROUP_MASK_REQUIRED;
         ++baselineMask) {
        for (rowMask = 0u;
             rowMask <= IMU_GROUP_MASK_REQUIRED;
             ++rowMask) {
            ImuPublisherState_t p;
            ImuSampleSnapshot_t base =
                snapshot(
                    20u,
                    IMU_READER_STATE_OPERATIONAL,
                    (uint8_t)baselineMask,
                    (baselineMask & 1u) ? 10u : 0u,
                    (baselineMask & 2u) ? 10u : 0u,
                    (baselineMask & 4u) ? 10u : 0u,
                    1000u);
            ImuSampleSnapshot_t s =
                snapshot(
                    20u,
                    IMU_READER_STATE_OPERATIONAL,
                    (uint8_t)rowMask,
                    (rowMask & 1u) ? 12u : 0u,
                    (rowMask & 2u) ? 11u : 0u,
                    (rowMask & 4u) ? 10u : 0u,
                    2000u);
            ImuPublicationRecord_t r;

            CHECK(imu_publish_begin_window(&p, &base));
            r = evaluate(&p, &s, 3000u);
            check_partition(&r);
        }
    }
}

int main(void)
{
    test_baseline_and_advances();
    test_missing_and_first_decode();
    test_epoch_change_invalidates_previous();
    test_ineligible_reader_states();
    test_age_edges_and_invalid_inputs();
    test_invariant_sweep();

    if (s_failures != 0) {
        fprintf(stderr,
                "test_imu_publish: %d failure(s)\n",
                s_failures);
        return 1;
    }

    puts("test_imu_publish: PASS");
    return 0;
}
