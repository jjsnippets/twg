#define _POSIX_C_SOURCE 200809L

#include "demo_imu.h"

#include <math.h>
#include <stddef.h>
#include <string.h>
#include <time.h>

#include "app/imu_contract.h"
#include "app/imu_session.h"

#define NS_PER_MS              UINT64_C(1000000)
#define NS_PER_SEC             UINT64_C(1000000000)

#define PRODUCTION_SETTLE_NS   (UINT64_C(300) * NS_PER_MS)
#define TARE_SETTLE_NS         (UINT64_C(2) * NS_PER_SEC)
#define TARE_POST_NS           (UINT64_C(500) * NS_PER_MS)

#define STARTUP_TIMEOUT_NS     (UINT64_C(5) * NS_PER_SEC)
#define TARE_TIMEOUT_NS        (UINT64_C(5) * NS_PER_SEC)
#define POST_TIMEOUT_NS        (UINT64_C(3) * NS_PER_SEC)
#define RESTORE_TIMEOUT_NS     (UINT64_C(5) * NS_PER_SEC)

#define FRESH_NS               (UINT64_C(250) * NS_PER_MS)
#define FLIGHT_CAL_MASK        0u

uint64_t demo_now_ns(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0;
    }

    return (uint64_t)ts.tv_sec * NS_PER_SEC + (uint64_t)ts.tv_nsec;
}

static bool recent(uint64_t now, uint64_t stamp)
{
    return stamp != 0 && now >= stamp && now - stamp <= FRESH_NS;
}

static bool normalize(float w, float x, float y, float z, float out[4])
{
    double n2;
    double scale;

    if (!isfinite(w) || !isfinite(x) ||
        !isfinite(y) || !isfinite(z)) {
        return false;
    }

    n2 = (double)w * w + (double)x * x +
         (double)y * y + (double)z * z;

    if (n2 < 0.25 || n2 > 2.25) {
        return false;
    }

    scale = 1.0 / sqrt(n2);
    out[0] = (float)(w * scale);
    out[1] = (float)(x * scale);
    out[2] = (float)(y * scale);
    out[3] = (float)(z * scale);
    return true;
}

static void update_epoch(DemoImu *imu)
{
    ImuSampleSnapshot_t sample;

    if (imu_session_get_snapshot(&sample)) {
        imu->output.epoch = sample.configurationEpoch;
    }
}

static void fault(DemoImu *imu, DemoError error)
{
    imu->output.phase = DEMO_PHASE_FAULT;
    imu->output.error = error;
    imu->output.valid = false;

    if (imu->output.tare_result == 1) {
        imu->output.tare_result = -1;
    }

    update_epoch(imu);
}

/*
 * Read only current-epoch, recently decoded rotation data.
 * Calibration accuracy is reported, not used as a hard display gate.
 */
static bool production_sample(uint64_t now, uint64_t not_before,
                              ImuSampleSnapshot_t *sample, float q[4])
{
    if (!imu_session_get_snapshot(sample)) {
        return false;
    }

    if ((sample->validMask & IMU_GROUP_BIT_ROTATION) == 0 ||
        sample->configurationEpoch == IMU_EPOCH_NONE ||
        sample->rotationMeta.configurationEpoch !=
            sample->configurationEpoch ||
        sample->rotationMeta.groupEventSeq == 0 ||
        sample->rotationMeta.hostDecodeNs < not_before ||
        !recent(now, sample->rotationMeta.hostDecodeNs)) {
        return false;
    }

    return normalize(sample->qw, sample->qi,
                     sample->qj, sample->qk, q);
}

static void adopt(DemoImu *imu, const ImuSampleSnapshot_t *sample,
                  const float q[4])
{
    imu->output.epoch = sample->configurationEpoch;
    imu->output.sequence = sample->rotationMeta.groupEventSeq;
    imu->output.sample_ns = sample->rotationMeta.hostDecodeNs;
    imu->output.accuracy = sample->rotationMeta.rawStatus & 3u;
    imu->output.valid = true;
    memcpy(imu->output.q, q, sizeof(imu->output.q));
}

static bool tare_sample(const DemoImu *imu, uint64_t now,
                        uint64_t not_before)
{
    ImuTareFacts_t facts;
    float q[4];

    if (!imu_session_get_tare_facts(&facts)) {
        return false;
    }

    return facts.valid &&
           facts.configurationEpoch == imu->output.epoch &&
           facts.rotationEventSequence != 0 &&
           facts.hostDecodeNs >= not_before &&
           recent(now, facts.hostDecodeNs) &&
           normalize(facts.quatReal, facts.quatI,
                     facts.quatJ, facts.quatK, q);
}

static void begin_restore(DemoImu *imu)
{
    uint64_t now;

    imu->output.phase = DEMO_PHASE_RESTORING;
    imu->output.valid = false;

    if (!imu_session_restore_production(FLIGHT_CAL_MASK) ||
        !imu_session_begin_settle()) {
        fault(imu, DEMO_ERROR_RESTORE);
        return;
    }

    now = demo_now_ns();
    imu->minimum_ns = now + PRODUCTION_SETTLE_NS;
    imu->deadline_ns = now + RESTORE_TIMEOUT_NS;
    update_epoch(imu);
}

bool demo_imu_open(DemoImu *imu)
{
    uint64_t now;

    if (imu == NULL) {
        return false;
    }

    memset(imu, 0, sizeof(*imu));
    imu->output.reference = 1u;
    imu->output.q[0] = 1.0f;
    imu->output.phase = DEMO_PHASE_STARTING;

    if (!imu_session_open()) {
        imu_session_close();
        fault(imu, DEMO_ERROR_STARTUP);
        return false;
    }

    imu->opened = true;

    if (!imu_session_configure_production(FLIGHT_CAL_MASK) ||
        !imu_session_begin_settle()) {
        fault(imu, DEMO_ERROR_STARTUP);
        demo_imu_close(imu);
        return false;
    }

    now = demo_now_ns();
    imu->minimum_ns = now + PRODUCTION_SETTLE_NS;
    imu->deadline_ns = now + STARTUP_TIMEOUT_NS;
    update_epoch(imu);
    return true;
}

bool demo_imu_request_tare(DemoImu *imu, uint32_t request_id)
{
    uint64_t now;

    if (imu == NULL || !imu->opened || request_id == 0 ||
        imu->output.phase != DEMO_PHASE_TRACKING) {
        return false;
    }

    imu->output.tare_id = request_id;
    imu->output.tare_result = 1;
    imu->output.error = DEMO_ERROR_NONE;
    imu->output.valid = false;
    imu->output.phase = DEMO_PHASE_TARE_SETTLE;
    imu->reference_dirty = false;

    if (!imu_session_return_to_configuring() ||
        !imu_session_configure_tare()) {
        fault(imu, DEMO_ERROR_TARE_CONFIG);
        return true;
    }

    now = demo_now_ns();
    imu->minimum_ns = now + TARE_SETTLE_NS;
    imu->deadline_ns = now + TARE_TIMEOUT_NS;
    update_epoch(imu);
    return true;
}

void demo_imu_service(DemoImu *imu)
{
    ImuSampleSnapshot_t sample;
    float q[4];
    uint64_t now;

    if (imu == NULL || !imu->opened) {
        return;
    }

    imu_session_service();
    now = demo_now_ns();
    imu->output.serial += 1u;

    if (!imu_session_get_snapshot(&sample)) {
        fault(imu, DEMO_ERROR_SESSION);
        return;
    }

    if (sample.readerState == IMU_READER_STATE_RECOVERING) {
        fault(imu, DEMO_ERROR_SENSOR_RESET);
        return;
    }

    if (sample.readerState == IMU_READER_STATE_FAULTED) {
        fault(imu, DEMO_ERROR_SESSION);
        return;
    }

    switch (imu->output.phase) {
    case DEMO_PHASE_STARTING:
    case DEMO_PHASE_RESTORING:
        if (now >= imu->minimum_ns &&
            production_sample(now, imu->minimum_ns, &sample, q)) {
            if (!imu_session_mark_operational()) {
                fault(imu, DEMO_ERROR_SESSION);
                return;
            }

            adopt(imu, &sample, q);

            if (imu->reference_dirty) {
                imu->output.reference += 1u;
                if (imu->output.reference == 0) {
                    imu->output.reference = 1u;
                }
                imu->reference_dirty = false;
            }

            if (imu->output.tare_result == 1) {
                imu->output.tare_result =
                    imu->output.error == DEMO_ERROR_NONE ? 2 : -1;
            }

            imu->output.phase = DEMO_PHASE_TRACKING;
        } else if (now >= imu->deadline_ns) {
            fault(imu, imu->output.phase == DEMO_PHASE_STARTING
                       ? DEMO_ERROR_STARTUP_DATA
                       : DEMO_ERROR_RESTORE);
        }
        break;

    case DEMO_PHASE_TRACKING:
        if (production_sample(now, 0, &sample, q)) {
            adopt(imu, &sample, q);
        } else {
            imu->output.valid = false;
        }
        break;

    case DEMO_PHASE_TARE_SETTLE:
        if (now >= imu->minimum_ns &&
            tare_sample(imu, now, imu->minimum_ns)) {
            /*
             * A failed command return does not prove that the device
             * reference was untouched. Re-reference after restoration
             * whenever the tare command was attempted.
             */
            imu->reference_dirty = true;

            if (!imu_session_tare_now(IMU_SESSION_TARE_AXES_Z)) {
                imu->output.error = DEMO_ERROR_TARE_COMMAND;
                begin_restore(imu);
                break;
            }

            now = demo_now_ns();
            update_epoch(imu);
            imu->output.phase = DEMO_PHASE_TARE_POST;
            imu->minimum_ns = now + TARE_POST_NS;
            imu->deadline_ns = now + POST_TIMEOUT_NS;
        } else if (now >= imu->deadline_ns) {
            imu->output.error = DEMO_ERROR_TARE_DATA;
            begin_restore(imu);
        }
        break;

    case DEMO_PHASE_TARE_POST:
        if (now >= imu->minimum_ns &&
            tare_sample(imu, now, imu->minimum_ns)) {
            begin_restore(imu);
        } else if (now >= imu->deadline_ns) {
            imu->output.error = DEMO_ERROR_TARE_DATA;
            begin_restore(imu);
        }
        break;

    case DEMO_PHASE_FAULT:
        imu->output.valid = false;
        break;

    default:
        fault(imu, DEMO_ERROR_SESSION);
        break;
    }
}

void demo_imu_snapshot(const DemoImu *imu, DemoImuSnapshot *out)
{
    if (imu != NULL && out != NULL) {
        *out = imu->output;
    }
}

void demo_imu_close(DemoImu *imu)
{
    if (imu == NULL) {
        return;
    }

    if (imu->opened) {
        imu_session_close();
        imu->opened = false;
    }

    imu->output.valid = false;
}
