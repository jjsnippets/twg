#include "app/imu_check_adapter.h"

#include <stdint.h>
#include <string.h>

#include "app/imu_check.h"
#include "app/imu_session.h"

static bool get_epoch(uint32_t *out)
{
    ImuSampleSnapshot_t snapshot;

    if (out == NULL || !imu_session_get_snapshot(&snapshot) ||
        snapshot.version != IMU_SAMPLE_CONTRACT_VERSION ||
        snapshot.configurationEpoch == IMU_EPOCH_NONE) {
        return false;
    }
    *out = snapshot.configurationEpoch;
    return true;
}

static bool forward_facts(void)
{
    ImuCheckFacts_t facts;

    if (!imu_session_get_check_facts(&facts)) {
        return true; /* No constructed mailbox yet is not a facts error. */
    }
    if (facts.version != IMU_CHECK_FACTS_VERSION) {
        return false;
    }
    return imu_check_feed_facts(&facts);
}

static bool execute_request(const ImuCheckPendingRequest_t *request,
                            ImuCheckSessionResult_t *result)
{
    ImuSessionCheckConfigResult_t configured;
    ImuSessionCheckMode_t mode;
    uint8_t policyMask;

    if (request == NULL || result == NULL) {
        return false;
    }
    memset(result, 0, sizeof(*result));
    result->type = request->type;

    switch (request->type) {
    case IMU_CHECK_REQ_CONFIGURE_CHECK:
        if (request->identity == IMU_CMD_ID_CHECK) {
            if (request->effectiveMask != 0u) {
                return false;
            }
            mode = IMU_SESSION_CHECK_MODE_CHECK;
        } else if (request->identity == IMU_CMD_ID_PROBE) {
            mode = IMU_SESSION_CHECK_MODE_PROBE;
        } else {
            return false;
        }

        memset(&configured, 0, sizeof(configured));
        result->success = imu_session_configure_check(
            mode, request->effectiveMask, &configured);
        /*
         * Preserve readable evidence even on a mismatch (success false).
         * Do not synthesize a value when the readback was unavailable.
         */
        result->actualMaskValid = configured.actualMaskValid;
        result->actualMask = configured.actualMaskValid
                           ? configured.actualMask : 0u;
        return true;

    case IMU_CHECK_REQ_RESTORE_PRODUCTION:
        result->success =
            imu_session_restore_production(request->flightCalMask);
        if (result->success) {
            /*
             * Follow the existing cal/tare adapter convention: require the
             * session's flight-policy getter to remain available after
             * restoration. That getter may expose a cached policy, not an
             * independently observed hardware mask, so do not set
             * actualMaskValid here.
             */
            result->success = imu_session_get_cal_policy(&policyMask);
        }
        return true;

    case IMU_CHECK_REQ_NONE:
    default:
        return false;
    }
}

bool imu_check_adapter_pump(void)
{
    ImuCheckPendingRequest_t request;
    ImuCheckSessionResult_t result;
    ImuCheckEvent_t event;
    uint32_t epochBefore;
    uint32_t epochAfter;

    if (!forward_facts()) {
        return false;
    }

    request = imu_check_pending_request();
    if (request.type == IMU_CHECK_REQ_NONE) {
        return true;
    }
    if (request.type != IMU_CHECK_REQ_CONFIGURE_CHECK &&
        request.type != IMU_CHECK_REQ_RESTORE_PRODUCTION) {
        return false;
    }
    if (!get_epoch(&epochBefore)) {
        return false;
    }
    if (!execute_request(&request, &result)) {
        return false;
    }
    if (!get_epoch(&epochAfter)) {
        return false;
    }

    result.epochBefore = epochBefore;
    result.epochAfter = epochAfter;
    memset(&event, 0, sizeof(event));
    event.type = IMU_CHECK_EVENT_SESSION_RESULT;
    event.session = result;
    return imu_check_post(&event);
}
