#ifndef IMU_CAL_H
#define IMU_CAL_H

#include "app/imu_cmd.h"
#include "app/imu_session.h"

#define IMU_CAL_ENABLE_MASK 0x07u
#define IMU_CAL_ACCURACY_GOAL 2u
#define IMU_CAL_MAX_HEADING_ERR_RAD 0.35f
#define IMU_CAL_SUSTAINED_GOOD_NS 3000000000ull
#define IMU_CAL_ACCEL_FACE_WINDOW_NS 2000000000ull
#define IMU_CAL_ACCEL_GATE_TIMEOUT_NS 10000000000ull
#define IMU_CAL_GYRO_GATE_TIMEOUT_NS 15000000000ull
#define IMU_CAL_MAG_MOTION_NS 8000000000ull
#define IMU_CAL_MAG_GATE_TIMEOUT_NS 30000000000ull
#define IMU_CAL_HOLD_WINDOW_NS 10000000000ull
#define IMU_CAL_HOLD_GATE_TIMEOUT_NS 5000000000ull
#define IMU_CAL_SAVE_RETRY_HOLD_NS 5000000000ull
#define IMU_CAL_VERIFY_MOTION_NS 10000000000ull
#define IMU_CAL_VERIFY_GATE_TIMEOUT_NS 20000000000ull
#define IMU_CAL_ACCEL_POSE_COUNT 6u
#define IMU_CAL_ACCEL_MAX_ROUNDS 3u
#define IMU_CAL_MAG_MAX_ROUNDS 5u
#define IMU_CAL_MAX_SAVE_ATTEMPTS 3u

typedef enum {
    IMU_CAL_REQ_NONE = 0,
    IMU_CAL_REQ_CONFIGURE_CALIBRATION,
    IMU_CAL_REQ_SAVE_DCD,
    IMU_CAL_REQ_VERIFY_REOPEN,
    IMU_CAL_REQ_RESTORE_PRODUCTION,
    IMU_CAL_REQ_CLEAR_DCD
} ImuCalRequestType_t;

typedef struct {
    ImuCalRequestType_t type;
    uint8_t calMask;
} ImuCalPendingRequest_t;

typedef struct {
    ImuCalRequestType_t type;
    bool success;
    /* False only if the action left the sole session non-actionable. */
    bool sessionUsable;
    uint8_t policyMask;
    uint32_t epochBefore;
    uint32_t epochAfter;
} ImuCalSessionResult_t;

typedef enum {
    IMU_CAL_EVENT_TICK = 0,
    IMU_CAL_EVENT_OPERATOR_Q,
    IMU_CAL_EVENT_OPERATOR_CONFIRM,
    IMU_CAL_EVENT_SESSION_RESULT
} ImuCalEventType_t;

typedef struct {
    ImuCalEventType_t type;
    uint64_t monotonicNs;
    ImuCalSessionResult_t session;
} ImuCalEvent_t;

bool imu_cal_init(uint8_t flightCalMask, uint64_t nowNs);
/* Destructive calibration-family alternative; emits no request until confirm. */
bool imu_cal_init_dcd_clear(uint8_t flightCalMask, uint64_t nowNs);
bool imu_cal_post(const ImuCalEvent_t *event);
void imu_cal_service(void);
bool imu_cal_feed_facts(const ImuCalFacts_t *facts);
bool imu_cal_get_progress(ImuCmdProgress_t *out);
bool imu_cal_get_result(ImuCmdResult_t *out);
ImuCalPendingRequest_t imu_cal_pending_request(void);
bool imu_cal_complete(void);

#endif /* IMU_CAL_H */
