#ifndef IMU_TARE_H
#define IMU_TARE_H

/*
 * Host-only tare-family machine boundary (Phase 6.1).
 *
 * Types and API only. Does not own the SH-2 session, sleep, print,
 * read stdin, or call exit. Does not include sh2 or imu_session.
 * bno_app does not link this module.
 *
 * imu_tare produces a complete ImuCmdResult_t. It must not
 * down-convert through ImuCmdStageTerminal_t. PROCESS_STOP stays
 * coordinator-owned and is not an imu_tare event.
 *
 * Init rejects illegal combinations:
 * - identity other than TARE, TARE_CLEAR, or TARE_CHECK
 * - TARE without Z or FULL axes
 * - TARE_CLEAR or TARE_CHECK with axes other than NONE
 *
 * At most one pending request. A SESSION_RESULT whose type does not
 * match the current pending request is rejected.
 */

#include "app/imu_cmd.h"
#include "app/imu_session.h"

#define IMU_TARE_REPORT_HZ     100u
#define IMU_TARE_SETTLE_NS     2000000000ull
#define IMU_TARE_VERIFY_NS     500000000ull

typedef enum {
    IMU_TARE_REQ_NONE = 0,
    IMU_TARE_REQ_CONFIGURE,
    IMU_TARE_REQ_TARE_NOW,
    IMU_TARE_REQ_PERSIST,
    IMU_TARE_REQ_CLEAR,
    IMU_TARE_REQ_RESTORE_PRODUCTION
} ImuTareRequestType_t;

typedef struct {
    ImuTareRequestType_t type;
    ImuCmdTareAxes_t tareAxes;
    uint8_t flightCalMask;
} ImuTarePendingRequest_t;

typedef struct {
    ImuTareRequestType_t type;
    bool success;
    uint32_t epochBefore;
    uint32_t epochAfter;
    ImuCmdSubResult_t clearActive;
    ImuCmdSubResult_t clearSaved;
    bool policyMaskValid;
    uint8_t policyMask;
} ImuTareSessionResult_t;

typedef enum {
    IMU_TARE_EVENT_TICK = 0,
    IMU_TARE_EVENT_OPERATOR_CONFIRM,
    IMU_TARE_EVENT_OPERATOR_Q,
    IMU_TARE_EVENT_FACTS,
    IMU_TARE_EVENT_SESSION_RESULT
} ImuTareEventType_t;

typedef struct {
    ImuTareEventType_t type;
    uint64_t monotonicNs;
    ImuTareFacts_t facts;
    ImuTareSessionResult_t session;
} ImuTareEvent_t;

bool imu_tare_init(ImuCmdIdentity_t identity,
                   ImuCmdTareAxes_t axes,
                   uint8_t flightCalMask,
                   uint64_t nowNs);
bool imu_tare_post(const ImuTareEvent_t *event);
void imu_tare_service(void);
bool imu_tare_feed_facts(const ImuTareFacts_t *facts);
bool imu_tare_get_progress(ImuCmdProgress_t *out);
bool imu_tare_get_result(ImuCmdResult_t *out);
ImuTarePendingRequest_t imu_tare_pending_request(void);
bool imu_tare_complete(void);

#endif /* IMU_TARE_H */