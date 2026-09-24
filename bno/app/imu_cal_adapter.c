#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app/imu_cal.h"
#include "app/imu_cal_adapter.h"
#include "app/imu_session.h"

static bool get_epoch(uint32_t *outEpoch)
{
    ImuSampleSnapshot_t snapshot;

    if (outEpoch == NULL || !imu_session_get_snapshot(&snapshot)) {
        return false;
    }

    *outEpoch = snapshot.configurationEpoch;
    return true;
}

static bool forward_cal_facts(void)
{
    ImuCalFacts_t facts;

    ImuCmdProgress_t progress;

    if (!imu_cal_get_progress(&progress)) {
        return false;
    }
    if (progress.active == IMU_CMD_ID_DCD_CLEAR) {
        return true;
    }

    if (!imu_session_get_cal_facts(&facts)) {
        return true;
    }

    if (facts.version != IMU_CAL_FACTS_VERSION) {
        return false;
    }

    return imu_cal_feed_facts(&facts);
}

static bool execute_request(const ImuCalPendingRequest_t *request,
                            ImuCalSessionResult_t *result)
{
    uint8_t policyMask = 0u;

    if (request == NULL || result == NULL) {
        return false;
    }

    memset(result, 0, sizeof(*result));
    result->type = request->type;

    switch (request->type) {
    case IMU_CAL_REQ_CLEAR_DCD:
        result->success = imu_session_clear_dcd();
        return true;
    case IMU_CAL_REQ_CONFIGURE_CALIBRATION:
        result->success =
            imu_session_configure_calibration(request->calMask);
        if (result->success) {
            result->success = imu_session_get_cal_policy(&policyMask);
            result->policyMask = policyMask;
        }
        return true;

    case IMU_CAL_REQ_SAVE_DCD:
        result->success = imu_session_save_dcd();
        return true;

    case IMU_CAL_REQ_VERIFY_REOPEN:
        result->success = imu_session_begin_verification_reopen();
        return true;

    case IMU_CAL_REQ_RESTORE_PRODUCTION:
        result->success =
            imu_session_restore_production(request->calMask);
        if (result->success) {
            result->success = imu_session_get_cal_policy(&policyMask);
            result->policyMask = policyMask;
        }
        return true;

    case IMU_CAL_REQ_NONE:
    default:
        return false;
    }
}

bool imu_cal_adapter_pump(void)
{
    ImuCalPendingRequest_t request;
    ImuCalSessionResult_t result;
    ImuCalEvent_t event;
    uint32_t epochBefore;
    uint32_t epochAfter;

    if (imu_cal_complete()) {
        return imu_cal_pending_request().type == IMU_CAL_REQ_NONE;
    }

    if (!forward_cal_facts()) {
        return false;
    }

    request = imu_cal_pending_request();
    if (request.type == IMU_CAL_REQ_NONE) {
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

    {
        ImuSampleSnapshot_t snapshot;

        if (!imu_session_get_snapshot(&snapshot)) {
            return false;
        }
        result.sessionUsable =
            snapshot.readerState != IMU_READER_STATE_FAULTED &&
            snapshot.readerState != IMU_READER_STATE_CLOSED &&
            snapshot.readerState != IMU_READER_STATE_OPENING &&
            snapshot.readerState != IMU_READER_STATE_RECOVERING;
    }

    memset(&event, 0, sizeof(event));
    event.type = IMU_CAL_EVENT_SESSION_RESULT;
    event.session = result;
    return imu_cal_post(&event);
}
