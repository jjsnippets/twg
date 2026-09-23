#ifndef IMU_CHECK_H
#define IMU_CHECK_H

#include "app/imu_cmd.h"

#define IMU_CHECK_FACTS_VERSION 1u
#define IMU_CHECK_GATE_NS 3000000000ull
#define IMU_CHECK_PROBE_NS 10000000000ull
#define IMU_CHECK_ACCURACY_GOAL 2u

/*
 * Check-only, epoch-scoped mailbox snapshot. The session owns construction
 * and decoding in Step 7.2; this machine accepts copies as plain host data.
 * Each have* flag is independent and sticky only within its source epoch.
 */
typedef struct {
    uint8_t version;
    uint32_t configurationEpoch;
    bool valid;
    bool haveAccel;
    bool haveGyro;
    bool haveMag;
    bool haveRv;
    uint64_t accelHostDecodeNs;
    uint64_t gyroHostDecodeNs;
    uint64_t magHostDecodeNs;
    uint64_t rvHostDecodeNs;
    uint8_t accelStatus;
    uint8_t gyroStatus;
    uint8_t magStatus;
    uint8_t rvStatus;
    float rvErrRad;
    float magXuT;
    float magYuT;
    float magZuT;
} ImuCheckFacts_t;

typedef enum {
    IMU_CHECK_REQ_NONE = 0,
    IMU_CHECK_REQ_CONFIGURE_CHECK,
    IMU_CHECK_REQ_RESTORE_PRODUCTION
} ImuCheckRequestType_t;

typedef struct {
    ImuCheckRequestType_t type;
    ImuCmdIdentity_t identity;
    uint8_t effectiveMask;
    uint8_t flightCalMask;
} ImuCheckPendingRequest_t;

typedef struct {
    ImuCheckRequestType_t type;
    bool success;
    bool actualMaskValid;
    uint8_t actualMask;
    uint32_t epochBefore;
    uint32_t epochAfter;
} ImuCheckSessionResult_t;

typedef enum {
    IMU_CHECK_EVENT_TICK = 0,
    IMU_CHECK_EVENT_OPERATOR_Q,
    IMU_CHECK_EVENT_SESSION_RESULT
} ImuCheckEventType_t;

typedef struct {
    ImuCheckEventType_t type;
    uint64_t monotonicNs;
    ImuCheckSessionResult_t session;
} ImuCheckEvent_t;

/*
 * CHECK requires probeMaskPresent == false and applies 0x00.
 * PROBE requires probeMaskPresent == true; 0x00 remains a valid probe mask.
 * Time comes only from init/TICK. PROCESS_STOP belongs to imu_cmd.
 */
bool imu_check_init(ImuCmdIdentity_t identity, bool probeMaskPresent,
                    uint8_t probeMask, uint8_t flightCalMask,
                    uint64_t nowNs);
bool imu_check_post(const ImuCheckEvent_t *event);
bool imu_check_feed_facts(const ImuCheckFacts_t *facts);
void imu_check_service(void);
ImuCheckPendingRequest_t imu_check_pending_request(void);
bool imu_check_get_progress(ImuCmdProgress_t *out);
bool imu_check_get_result(ImuCmdResult_t *out);
bool imu_check_complete(void);

#endif /* IMU_CHECK_H */
