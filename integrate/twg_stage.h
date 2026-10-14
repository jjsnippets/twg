#ifndef TWG_STAGE_H
#define TWG_STAGE_H

#include <stdbool.h>
#include <stdint.h>

#include "app/imu_cmd.h"
#include "twg_term.h"

#define TWG_STAGE_SETTLE_NS UINT64_C(300000000)

typedef enum {
    TWG_STAGE_MODE_ACQUISITION = 0,
    TWG_STAGE_MODE_CALIBRATION,
    TWG_STAGE_MODE_TARE_Z,
    TWG_STAGE_MODE_TARE_FULL,
    TWG_STAGE_MODE_PRESSURE_ZERO,
    TWG_STAGE_MODE_CLEAR_CONFIRM,
    TWG_STAGE_MODE_CLEAR_ALL,
    TWG_STAGE_MODE_RESTORE,
    TWG_STAGE_MODE_STOPPED,
    TWG_STAGE_MODE_ERROR
} TwgStageMode;

typedef enum {
    TWG_STAGE_OK = 0,
    TWG_STAGE_PROCESS_STOP,
    TWG_STAGE_ERROR
} TwgStageStatus;

typedef struct {
    uint8_t flightCalMask;

    TwgStageMode mode;
    TwgStageMode requestedMode;

    uint64_t settleDeadlineNs;
    uint64_t inputFlushTicket;

    bool initialized;
    bool imuPlanActive;
    bool settleStarted;
    bool clearConfirmed;
    bool pressureClearRequested;
    bool pressureZeroRequested;
    bool pressureZeroCancelled;
    bool processStopPosted;
    bool productionConfigured;
    bool resultPrinted[IMU_CMD_ID_COUNT];

    uint32_t promptKey;
    uint64_t lastStatusNs;

    const char *lastStage;
    const char *lastResult;
    const char *lastReason;
    bool lastWarning;
    bool resultEvent;

    ImuCmdIdentity_t lastActive;
} TwgStage;

typedef struct {
    const char *active;
    const char *lastStage;
    const char *lastResult;
    const char *lastReason;
    bool lastWarning;
    bool resultEvent;
    bool calValid;
    ImuCmdCalProgress_t cal;
} TwgStageInfo;

bool twg_stage_init(
    TwgStage *stage,
    uint8_t flightCalMask);

TwgStageStatus twg_stage_turn(
    TwgStage *stage,
    TwgTerm *term,
    uint64_t nowNs,
    bool processStop);

TwgStageMode twg_stage_mode(
    const TwgStage *stage);

bool twg_stage_take_pressure_zero_request(
    TwgStage *stage);

bool twg_stage_take_pressure_zero_cancel(
    TwgStage *stage);

bool twg_stage_take_pressure_clear(
    TwgStage *stage);
    
void twg_stage_pressure_zero_finished(
    TwgStage *stage,
    bool ok);

void twg_stage_get_info(
    TwgStage *stage,
    TwgStageInfo *info);

bool twg_stage_acquisition_active(
    const TwgStage *stage);

#endif /* TWG_STAGE_H */
