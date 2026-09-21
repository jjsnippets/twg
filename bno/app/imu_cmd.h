#ifndef IMU_CMD_H
#define IMU_CMD_H

/*
 * Host-only command coordinator (Phase 4).
 *
 * Immutable R6 plan in, injected events in, R7/R8 and next-stage
 * requests out. Does not open hardware, parse CLI, sleep, print, or
 * call exit. Does not include imu_session or imu_contract.
 *
 * bno_app does not link this module in Phase 4.
 */

#include <stdbool.h>
#include <stdint.h>

#define IMU_CMD_PLAN_VERSION           1u
#define IMU_CMD_RESULT_VERSION         1u

#define IMU_CMD_PROBE_DEADLINE_S       10u
#define IMU_CMD_ACQUIRE_DEFAULT_S      10u
#define IMU_CMD_ACQUIRE_MIN_S          1u
#define IMU_CMD_ACQUIRE_MAX_S          3600u
#define IMU_CMD_SETTLE_DEFAULT_MS      300u
#define IMU_CMD_SERVICE_PERIOD_NS      1000000ull
#define IMU_CMD_PUB_PERIOD_NS          10000000ull

typedef enum {
    IMU_CMD_ID_NONE = 0,
    IMU_CMD_ID_CALIBRATION,
    IMU_CMD_ID_DCD_CLEAR,
    IMU_CMD_ID_TARE,
    IMU_CMD_ID_TARE_CLEAR,
    IMU_CMD_ID_TARE_CHECK,
    IMU_CMD_ID_CHECK,
    IMU_CMD_ID_PROBE,
    IMU_CMD_ID_SETTLE,
    IMU_CMD_ID_ACQUISITION,
    IMU_CMD_ID_COUNT
} ImuCmdIdentity_t;

typedef enum {
    IMU_CMD_STATE_NOT_REQUESTED = 0,
    IMU_CMD_STATE_RUNNING,
    IMU_CMD_STATE_SUCCEEDED,
    IMU_CMD_STATE_FAILED,
    IMU_CMD_STATE_CANCELLED,
    IMU_CMD_STATE_TIMED_OUT,
    IMU_CMD_STATE_RECOVERY_FAILED,
    IMU_CMD_STATE_ABANDONED
} ImuCmdResultState_t;

typedef enum {
    IMU_CMD_REASON_NONE = 0,
    IMU_CMD_REASON_OK,
    IMU_CMD_REASON_OPERATOR_Q,
    IMU_CMD_REASON_OPERATOR_CONFIRM_TIMEOUT,
    IMU_CMD_REASON_GATE_NOT_REACHED,
    IMU_CMD_REASON_DCD_SAVE_FAILED,
    IMU_CMD_REASON_DCD_CLEAR_FAILED,
    IMU_CMD_REASON_TARE_NOW_FAILED,
    IMU_CMD_REASON_TARE_PERSIST_FAILED,
    IMU_CMD_REASON_TARE_CLEAR_PARTIAL,
    IMU_CMD_REASON_TARE_CLEAR_FAILED,
    IMU_CMD_REASON_TARE_VERIFY_FAILED,
    IMU_CMD_REASON_CONFIG_FAILED,
    IMU_CMD_REASON_PROBE_DEADLINE,
    IMU_CMD_REASON_SESSION_UNUSABLE,
    IMU_CMD_REASON_PROCESS_STOP,
    IMU_CMD_REASON_ILLEGAL_PLAN
} ImuCmdReason_t;

typedef enum {
    IMU_CMD_TARE_AXES_NONE = 0,
    IMU_CMD_TARE_AXES_Z,
    IMU_CMD_TARE_AXES_FULL
} ImuCmdTareAxes_t;

typedef enum {
    IMU_CMD_SUB_NOT_ATTEMPTED = 0,
    IMU_CMD_SUB_SUCCEEDED,
    IMU_CMD_SUB_FAILED
} ImuCmdSubResult_t;

typedef enum {
    IMU_CMD_ACTION_NONE = 0,
    IMU_CMD_ACTION_CONFIRM,
    IMU_CMD_ACTION_PRESS_Q_TO_END,
    IMU_CMD_ACTION_ALIGN_AND_CONFIRM
} ImuCmdOperatorAction_t;

typedef enum {
    IMU_CMD_REQ_NONE = 0,
    IMU_CMD_REQ_RUN_COMMAND,
    IMU_CMD_REQ_SETTLE,
    IMU_CMD_REQ_ACQUIRE,
    IMU_CMD_REQ_STOP_PLAN
} ImuCmdRequest_t;

typedef enum {
    IMU_CMD_EVENT_OPERATOR_Q = 0,
    IMU_CMD_EVENT_OPERATOR_CONFIRM,
    IMU_CMD_EVENT_PROCESS_STOP,
    IMU_CMD_EVENT_TICK,
    IMU_CMD_EVENT_STAGE_TERMINAL,
    IMU_CMD_EVENT_SETTLE_DONE,
    IMU_CMD_EVENT_SESSION_USABLE,
    IMU_CMD_EVENT_SESSION_UNRESTORABLE
} ImuCmdEventType_t;

typedef struct {
    ImuCmdSubResult_t tareNow;
    ImuCmdSubResult_t persist;
    ImuCmdSubResult_t clearActive;
    ImuCmdSubResult_t clearSaved;
    bool probeReachedGate;
    bool probeTimedOut;
    bool probeOperatorEndedEarly;
    bool probeMaskActualValid;
    uint8_t probeMaskActual;
} ImuCmdSubResults_t;

typedef struct {
    uint32_t version;
    ImuCmdIdentity_t slot1;
    ImuCmdIdentity_t slot2;
    ImuCmdIdentity_t slot3;
    ImuCmdTareAxes_t tareAxes;
    bool persistTare;
    bool probeMaskPresent;
    uint8_t probeMask;
    uint32_t probeDeadlineS;
    uint32_t acquisitionDurationS;
    uint32_t settleDurationMs;
    uint64_t servicePeriodNs;
    uint64_t publicationPeriodNs;
    bool confirmDcdClear;
    bool confirmTare;
    bool confirmTareClear;
} ImuCmdPlan_t;

typedef struct {
    uint32_t version;
    ImuCmdIdentity_t identity;
    ImuCmdResultState_t state;
    ImuCmdReason_t reason;
    bool warningRequired;
    uint64_t startedNs;
    uint64_t endedNs;
    ImuCmdTareAxes_t requestedTareAxes;
    uint8_t probeMaskRequested;
    bool probeMaskRequestedValid;
    ImuCmdSubResults_t sub;
} ImuCmdResult_t;

typedef struct {
    ImuCmdIdentity_t active;
    ImuCmdOperatorAction_t requiredAction;
    bool probeTimeValid;
    uint64_t probeRemainingNs;
} ImuCmdProgress_t;

typedef struct {
    ImuCmdIdentity_t identity;
    ImuCmdResultState_t state;
    ImuCmdReason_t reason;
    bool warningRequired;
    ImuCmdSubResults_t sub;
} ImuCmdStageTerminal_t;

typedef struct {
    ImuCmdEventType_t type;
    uint64_t monotonicNs;
    ImuCmdStageTerminal_t terminal;
} ImuCmdEvent_t;

void imu_cmd_plan_clear(ImuCmdPlan_t *plan);
bool imu_cmd_plan_acquire_default(ImuCmdPlan_t *plan);

bool imu_cmd_init(const ImuCmdPlan_t *plan);
bool imu_cmd_post(const ImuCmdEvent_t *event);
void imu_cmd_service(void);

bool imu_cmd_get_result(ImuCmdIdentity_t id, ImuCmdResult_t *out);
bool imu_cmd_get_progress(ImuCmdProgress_t *out);
ImuCmdRequest_t imu_cmd_request(void);
ImuCmdIdentity_t imu_cmd_request_identity(void);

bool imu_cmd_do_not_acquire(void);
bool imu_cmd_process_stop_seen(void);
bool imu_cmd_warning_required(void);
bool imu_cmd_plan_complete(void);
ImuCmdReason_t imu_cmd_init_reason(void);

#endif /* IMU_CMD_H */