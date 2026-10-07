#include "app/imu_publish.h"

#include <string.h>

static bool snapshot_contract_valid(const ImuSampleSnapshot_t *snapshot)
{
    return snapshot != NULL &&
           snapshot->version == IMU_SAMPLE_CONTRACT_VERSION;
}

static const ImuGroupMeta_t *snapshot_group_meta(
    const ImuSampleSnapshot_t *snapshot, unsigned index)
{
    switch (index) {
    case IMU_PUBLISH_GROUP_ROTATION:
        return &snapshot->rotationMeta;
    case IMU_PUBLISH_GROUP_ACCEL:
        return &snapshot->accelMeta;
    case IMU_PUBLISH_GROUP_GYRO:
        return &snapshot->gyroMeta;
    default:
        return NULL;
    }
}

static ImuPublicationGroup_t *publication_group(
    ImuPublicationRecord_t *record, unsigned index)
{
    switch (index) {
    case IMU_PUBLISH_GROUP_ROTATION:
        return &record->rotation;
    case IMU_PUBLISH_GROUP_ACCEL:
        return &record->accel;
    case IMU_PUBLISH_GROUP_GYRO:
        return &record->gyro;
    default:
        return NULL;
    }
}

static bool group_valid_in_snapshot(const ImuSampleSnapshot_t *snapshot,
                                    const ImuGroupMeta_t *meta, uint8_t bit)
{
    return (snapshot->validMask & bit) != 0u &&
           snapshot->configurationEpoch != IMU_EPOCH_NONE &&
           meta->version == IMU_METADATA_CONTRACT_VERSION &&
           meta->configurationEpoch == snapshot->configurationEpoch &&
           meta->groupEventSeq != IMU_GROUP_EVENT_SEQ_NONE;
}

static void save_previous(ImuPublisherPreviousGroup_t *previous,
                          const ImuGroupMeta_t *meta, bool valid)
{
    memset(previous, 0, sizeof(*previous));
    if (valid) {
        previous->configurationEpoch = meta->configurationEpoch;
        previous->groupEventSeq = meta->groupEventSeq;
        previous->previousValid = 1u;
    }
}

static void copy_group(ImuPublicationGroup_t *out,
                       const ImuGroupMeta_t *meta,
                       uint64_t actualNs, bool actualValid)
{
    out->groupEventSeq = meta->groupEventSeq;
    out->hostDecodeNs = meta->hostDecodeNs;
    out->sensorTimeUs = meta->sensorTimeUs;
    out->identityPresent = 1u;
    out->deviceReportSeq = meta->deviceReportSeq;
    out->rawStatus = meta->rawStatus;

    if (actualValid && meta->hostDecodeNs != 0u &&
        actualNs >= meta->hostDecodeNs) {
        out->ageValid = 1u;
        out->ageNs = actualNs - meta->hostDecodeNs;
    }
}

static void copy_values(ImuPublicationRecord_t *out,
                        const ImuSampleSnapshot_t *snapshot, uint8_t validMask)
{
    if ((validMask & IMU_GROUP_BIT_ROTATION) != 0u) {
        out->qw = snapshot->qw;
        out->qx = snapshot->qi;
        out->qy = snapshot->qj;
        out->qz = snapshot->qk;
        out->yawRad = snapshot->yaw;
        out->pitchRad = snapshot->pitch;
        out->rollRad = snapshot->roll;
        out->orientationErrRad = snapshot->orientationErrRad;
    }

    if ((validMask & IMU_GROUP_BIT_ACCEL) != 0u) {
        out->ax = snapshot->ax;
        out->ay = snapshot->ay;
        out->az = snapshot->az;
    }

    if ((validMask & IMU_GROUP_BIT_GYRO) != 0u) {
        out->gx = snapshot->gx;
        out->gy = snapshot->gy;
        out->gz = snapshot->gz;
    }
}

bool imu_publish_begin_window(ImuPublisherState_t *state,
                              const ImuSampleSnapshot_t *baseline)
{
    unsigned i;

    if (state == NULL || !snapshot_contract_valid(baseline) ||
        baseline->configurationEpoch == IMU_EPOCH_NONE ||
        baseline->readerState != IMU_READER_STATE_OPERATIONAL) {
        return false;
    }

    memset(state, 0, sizeof(*state));
    state->comparisonEpoch = baseline->configurationEpoch;
    state->nextPublicationSeq = 1u;
    state->windowOpen = 1u;

    for (i = 0u; i < IMU_PUBLISH_GROUP_COUNT; ++i) {
        const ImuGroupMeta_t *meta = snapshot_group_meta(baseline, i);
        uint8_t bit = (uint8_t)IMU_PUBLISH_GROUP_BIT(i);
        bool valid = group_valid_in_snapshot(baseline, meta, bit);

        save_previous(&state->previous[i], meta, valid);
    }

    return true;
}

bool imu_publish_evaluate_snapshot(ImuPublisherState_t *state,
                                   const ImuSampleSnapshot_t *snapshot,
                                   uint64_t scheduledNs,
                                   bool scheduledValid,
                                   uint64_t actualNs,
                                   bool actualValid,
                                   ImuPublicationRecord_t *out)
{
    unsigned i;
    bool epochChanged;
    bool dataEligible;

    if (state == NULL || state->windowOpen == 0u ||
        !snapshot_contract_valid(snapshot) || out == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->version = IMU_PUBLICATION_CONTRACT_VERSION;
    out->configurationEpoch = snapshot->configurationEpoch;
    out->publicationSeq = state->nextPublicationSeq++;
    out->scheduledNs = scheduledValid ? scheduledNs : 0u;
    out->actualNs = actualValid ? actualNs : 0u;
    out->scheduledValid = scheduledValid ? 1u : 0u;
    out->actualValid = actualValid ? 1u : 0u;
    out->readerState = snapshot->readerState;
    out->readerFlags =
        snapshot->readerStatusFlags & IMU_CSV_READER_FLAGS_MASK;

    epochChanged =
        snapshot->configurationEpoch != state->comparisonEpoch;

    dataEligible =
        snapshot->readerState == IMU_READER_STATE_OPERATIONAL &&
        snapshot->configurationEpoch != IMU_EPOCH_NONE &&
        !epochChanged;

    for (i = 0u; i < IMU_PUBLISH_GROUP_COUNT; ++i) {
        const ImuGroupMeta_t *meta = snapshot_group_meta(snapshot, i);
        ImuPublicationGroup_t *group = publication_group(out, i);
        ImuPublisherPreviousGroup_t *previous = &state->previous[i];
        uint8_t bit = (uint8_t)IMU_PUBLISH_GROUP_BIT(i);
        bool valid = group_valid_in_snapshot(snapshot, meta, bit);
        bool comparable =
            valid &&
            previous->previousValid != 0u &&
            previous->configurationEpoch == snapshot->configurationEpoch;

        if (epochChanged || !valid) {
            out->missingMask |= bit;
        } else {
            out->validMask |= bit;
            copy_group(group, meta, actualNs, actualValid);

            if (!comparable ||
                meta->groupEventSeq > previous->groupEventSeq) {
                out->freshMask |= bit;

                if (comparable &&
                    meta->groupEventSeq - previous->groupEventSeq > 1u) {
                    out->multiUpdateMask |= bit;
                }
            } else {
                out->staleMask |= bit;
            }
        }

        save_previous(previous, meta, valid);
    }

    if (epochChanged) {
        state->comparisonEpoch = snapshot->configurationEpoch;
    }

    copy_values(out, snapshot, out->validMask);

    out->sampleReady =
        dataEligible && out->validMask != 0u ? 1u : 0u;

    out->notReady =
        !dataEligible ||
        out->validMask != IMU_GROUP_MASK_REQUIRED ? 1u : 0u;

    return true;
}

bool imu_publish_gate_begin(ImuPublishGate_t *gate, uint64_t tick1Ns,
                            bool tick1Valid, uint64_t windowNs)
{
    uint64_t wholeTicks;

    if (gate == NULL || windowNs == 0u ||
        windowNs > UINT64_MAX - (IMU_PUBLISH_SERVICE_PERIOD_NS - 1u)) {
        return false;
    }

    /* Tick k starts inside the window iff (k - 1) * T_service < windowNs. */
    wholeTicks = (windowNs + IMU_PUBLISH_SERVICE_PERIOD_NS - 1u) /
                 IMU_PUBLISH_SERVICE_PERIOD_NS;

    memset(gate, 0, sizeof(*gate));
    gate->maxGate = wholeTicks / IMU_PUBLISH_GATE_TICKS;
    if (gate->maxGate == 0u) {
        memset(gate, 0, sizeof(*gate));
        return false;
    }
    gate->tick1Ns = tick1Valid ? tick1Ns : 0u;
    gate->tick1Valid = tick1Valid ? 1u : 0u;
    gate->active = 1u;
    return true;
}

bool imu_publish_gate_window_done(const ImuPublishGate_t *gate)
{
    return gate != NULL && gate->active != 0u &&
           gate->lastGate >= gate->maxGate;
}

ImuPublishGateStatus_t imu_publish_gate_advance(
    ImuPublishGate_t *gate, uint64_t ticks, ImuPublishGateDecision_t *out)
{
    uint64_t current;
    uint64_t skipped;
    uint64_t offsetTicks;

    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }
    if (gate == NULL || out == NULL || gate->active == 0u || ticks == 0u ||
        gate->tickIndex > UINT64_MAX - ticks) {
        return IMU_PUBLISH_GATE_ERROR;
    }
    if (gate->lastGate >= gate->maxGate) {
        return IMU_PUBLISH_GATE_COMPLETE;
    }

    gate->tickIndex += ticks;
    current = gate->tickIndex / IMU_PUBLISH_GATE_TICKS;
    if (current > gate->maxGate) {
        current = gate->maxGate;
    }
    if (current <= gate->lastGate) {
        return IMU_PUBLISH_GATE_IDLE;
    }

    skipped = current - gate->lastGate - 1u;
    if (skipped > UINT32_MAX) {
        return IMU_PUBLISH_GATE_ERROR;
    }

    out->gateIndex = current;
    out->gatesSkippedBefore = (uint32_t)skipped;
    offsetTicks = current * IMU_PUBLISH_GATE_TICKS - 1u;
    if (gate->tick1Valid != 0u &&
        offsetTicks <= (UINT64_MAX - gate->tick1Ns) /
                           IMU_PUBLISH_SERVICE_PERIOD_NS) {
        out->scheduledNs = gate->tick1Ns +
                           offsetTicks * IMU_PUBLISH_SERVICE_PERIOD_NS;
        out->scheduledValid = 1u;
    }

    gate->lastGate = current;
    gate->gatesSkippedTotal += skipped;
    return IMU_PUBLISH_GATE_DUE;
}

bool imu_publish_apply_timing(ImuPublicationRecord_t *record,
                              uint32_t gatesSkippedBefore)
{
    bool late;

    if (record == NULL || record->version != IMU_PUBLICATION_CONTRACT_VERSION) {
        return false;
    }

    late = record->scheduledValid == 0u || record->actualValid == 0u ||
           gatesSkippedBefore != 0u ||
           (record->actualNs > record->scheduledNs &&
            record->actualNs - record->scheduledNs > IMU_PUBLISH_T_LATE_NS);

    record->gatesSkippedBefore = gatesSkippedBefore;
    record->deadlineMissed = late ? 1u : 0u;
    return true;
}
