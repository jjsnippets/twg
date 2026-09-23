#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app/imu_session.h"
#include "app/imu_tare.h"
#include "app/imu_tare_adapter.h"

static bool get_epoch(uint32_t *outEpoch)
{
    ImuSampleSnapshot_t snapshot;

    if (outEpoch == NULL || !imu_session_get_snapshot(&snapshot)) {
        return false;
    }
    *outEpoch = snapshot.configurationEpoch;
    return true;
}

static bool forward_tare_facts(void)
{
    ImuTareFacts_t facts;

    if (!imu_session_get_tare_facts(&facts)) {
        return true;
    }
    if (facts.version != IMU_TARE_FACTS_VERSION) {
        return false;
    }
    return imu_tare_feed_facts(&facts);
}

static ImuCmdSubResult_t map_clear_sub(ImuSessionTareSubResult_t sub)
{
    if (sub == IMU_SESSION_TARE_SUB_SUCCEEDED) {
        return IMU_CMD_SUB_SUCCEEDED;
    }
    if (sub == IMU_SESSION_TARE_SUB_FAILED) {
        return IMU_CMD_SUB_FAILED;
    }
    return IMU_CMD_SUB_NOT_ATTEMPTED;
}

static bool map_axes(ImuCmdTareAxes_t in, ImuSessionTareAxes_t *out)
{
    if (out == NULL) {
        return false;
    }
    if (in == IMU_CMD_TARE_AXES_Z) {
        *out = IMU_SESSION_TARE_AXES_Z;
        return true;
    }
    if (in == IMU_CMD_TARE_AXES_FULL) {
        *out = IMU_SESSION_TARE_AXES_FULL;
        return true;
    }
    return false;
}

static bool execute_request(const ImuTarePendingRequest_t *request,
                            ImuTareSessionResult_t *result)
{
    uint8_t policyMask = 0u;
    ImuSessionTareAxes_t axes;
    ImuSessionClearTareResult_t clear;

    if (request == NULL || result == NULL) {
        return false;
    }

    memset(result, 0, sizeof(*result));
    result->type = request->type;

    switch (request->type) {
    case IMU_TARE_REQ_CONFIGURE:
        result->success = imu_session_configure_tare();
        if (result->success) {
            result->policyMaskValid = imu_session_get_cal_policy(&policyMask);
            result->policyMask = policyMask;
            result->success = result->policyMaskValid;
        }
        return true;

    case IMU_TARE_REQ_TARE_NOW:
        if (!map_axes(request->tareAxes, &axes)) {
            return false;
        }
        result->success = imu_session_tare_now(axes);
        return true;

    case IMU_TARE_REQ_PERSIST:
        result->success = imu_session_persist_tare();
        return true;

    case IMU_TARE_REQ_CLEAR:
        memset(&clear, 0, sizeof(clear));
        result->success = imu_session_clear_tare(&clear);
        result->clearActive = map_clear_sub(clear.clearActive);
        result->clearSaved = map_clear_sub(clear.clearSaved);
        return true;

    case IMU_TARE_REQ_RESTORE_PRODUCTION:
        result->success =
            imu_session_restore_production(request->flightCalMask);
        if (result->success) {
            result->policyMaskValid = imu_session_get_cal_policy(&policyMask);
            result->policyMask = policyMask;
            result->success = result->policyMaskValid;
        }
        return true;

    case IMU_TARE_REQ_NONE:
    default:
        return false;
    }
}

bool imu_tare_adapter_pump(void)
{
    ImuTarePendingRequest_t request;
    ImuTareSessionResult_t result;
    ImuTareEvent_t event;
    uint32_t epochBefore;
    uint32_t epochAfter;

    if (!forward_tare_facts()) {
        return false;
    }

    request = imu_tare_pending_request();
    if (request.type == IMU_TARE_REQ_NONE) {
        return true;
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
    event.type = IMU_TARE_EVENT_SESSION_RESULT;
    event.session = result;
    return imu_tare_post(&event);
}
