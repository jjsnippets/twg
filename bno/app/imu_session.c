#define _POSIX_C_SOURCE 200809L

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "sh2.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"
#include "euler.h"

#include "app/imu_contract.h"
#include "app/imu_session.h"

extern sh2_Hal_t *sh2_hal_rpi_init(void);

#define SENSOR_RATE_HZ      100U
#define SENSOR_INTERVAL_US  (1000000U / SENSOR_RATE_HZ)
#define GROUP_COUNT         3

static sh2_Hal_t *sHal;
static ImuSampleSnapshot_t sSnapshot;
static bool sHasMailbox;
static bool sOpenAttempted;
static ImuReaderState_t sState;
static uint32_t sEpoch;
static uint8_t sFlightCalMask;
static bool sHardwareConfigured;
static bool sRecoveryEpochTaken;
static uint64_t sAdoptedSeq[GROUP_COUNT];
static bool sHaveAdopted[GROUP_COUNT];
static uint64_t sAssignedSeq[GROUP_COUNT];
static void asyncEventCallback(void *cookie, sh2_AsyncEvent_t *pEvent);
static void sensorCallback(void *cookie, sh2_SensorEvent_t *pEvent);

static int group_index(uint8_t groupBit)
{
    if (groupBit == IMU_GROUP_BIT_ROTATION) {
        return 0;
    }
    if (groupBit == IMU_GROUP_BIT_ACCEL) {
        return 1;
    }
    if (groupBit == IMU_GROUP_BIT_GYRO) {
        return 2;
    }
    return -1;
}

static uint64_t monotonic_ns(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0;
    }

    return ((uint64_t)ts.tv_sec * 1000000000ull) + (uint64_t)ts.tv_nsec;
}

static void sync_header(void)
{
    sSnapshot.version = IMU_SAMPLE_CONTRACT_VERSION;
    sSnapshot.configurationEpoch = sEpoch;
    sSnapshot.readerState = sState;
    sSnapshot.readerStatusFlags = 0;
    sHasMailbox = true;
}

static void clear_validity(void)
{
    sSnapshot.validMask = 0;
    sSnapshot.rotationMeta.epochUpdateCount = 0;
    sSnapshot.accelMeta.epochUpdateCount = 0;
    sSnapshot.gyroMeta.epochUpdateCount = 0;
}

static void increment_epoch(void)
{
    sEpoch += 1u;
    clear_validity();
    sync_header();
}

static void enter_faulted(void)
{
    sState = IMU_READER_STATE_FAULTED;
    sync_header();
}

static void zero_mailbox(void)
{
    memset(&sSnapshot, 0, sizeof(sSnapshot));
    sSnapshot.rotationMeta.version = IMU_METADATA_CONTRACT_VERSION;
    sSnapshot.accelMeta.version = IMU_METADATA_CONTRACT_VERSION;
    sSnapshot.gyroMeta.version = IMU_METADATA_CONTRACT_VERSION;
    sync_header();
}

static bool configure_sensor(sh2_SensorId_t sensorId)
{
    sh2_SensorConfig_t cfg;

    memset(&cfg, 0, sizeof(cfg));
    cfg.reportInterval_us = SENSOR_INTERVAL_US;
    cfg.batchInterval_us = 0;

    return sh2_setSensorConfig(sensorId, &cfg) == SH2_OK;
}

static bool apply_hardware_production(uint8_t flightCalMask)
{
    if (!sHal) {
        return true;
    }

    if (!configure_sensor(SH2_ROTATION_VECTOR) ||
        !configure_sensor(SH2_LINEAR_ACCELERATION) ||
        !configure_sensor(SH2_GYROSCOPE_CALIBRATED)) {
        return false;
    }

    return sh2_setCalConfig(flightCalMask) == SH2_OK;
}

static bool reopen_hardware(void)
{
    if (!sHal) {
        return true;
    }

    sh2_close();
    sHal = sh2_hal_rpi_init();
    if (!sHal) {
        return false;
    }

    if (sh2_open(sHal, asyncEventCallback, NULL) != SH2_OK) {
        sHal = NULL;
        return false;
    }

    if (sh2_setSensorCallback(sensorCallback, NULL) != SH2_OK) {
        sh2_close();
        sHal = NULL;
        return false;
    }

    return true;
}

static bool should_adopt(int idx, uint64_t seq, uint64_t hostDecodeNs)
{
    if (sEpoch == IMU_EPOCH_NONE) {
        return false;
    }
    if (seq == IMU_GROUP_EVENT_SEQ_NONE) {
        return false;
    }
    if (hostDecodeNs == 0) {
        return false;
    }
    if (sHaveAdopted[idx] && seq <= sAdoptedSeq[idx]) {
        return false;
    }
    return true;
}

static ImuGroupMeta_t *meta_for_index(int idx)
{
    if (idx == 0) {
        return &sSnapshot.rotationMeta;
    }
    if (idx == 1) {
        return &sSnapshot.accelMeta;
    }
    return &sSnapshot.gyroMeta;
}

static void adopt_meta(int idx, uint8_t groupBit, uint64_t seq,
                       uint64_t hostDecodeNs, uint64_t sensorTimeUs,
                       uint8_t deviceReportSeq, uint8_t rawStatus)
{
    ImuGroupMeta_t *meta = meta_for_index(idx);

    meta->version = IMU_METADATA_CONTRACT_VERSION;
    meta->configurationEpoch = sEpoch;
    meta->groupEventSeq = seq;
    meta->hostDecodeNs = hostDecodeNs;
    meta->sensorTimeUs = sensorTimeUs;
    meta->deviceReportSeq = deviceReportSeq;
    meta->epochUpdateCount += 1u;
    meta->rawStatus = rawStatus;

    sHaveAdopted[idx] = true;
    sAdoptedSeq[idx] = seq;
    if (seq > sAssignedSeq[idx]) {
        sAssignedSeq[idx] = seq;
    }

    sSnapshot.processDecodeCount += 1ull;
    sSnapshot.newestHostUpdateNs = hostDecodeNs;
    sSnapshot.validMask |= groupBit;
    sync_header();
}

static void apply_group(const ImuSessionTestGroupEvent_t *event)
{
    int idx;

    if (!event) {
        return;
    }

    idx = group_index(event->groupBit);
    if (idx < 0) {
        return;
    }

    if (!should_adopt(idx, event->groupEventSeq, event->hostDecodeNs)) {
        return;
    }

    if (event->groupBit == IMU_GROUP_BIT_ROTATION) {
        sSnapshot.qw = event->qw;
        sSnapshot.qi = event->qi;
        sSnapshot.qj = event->qj;
        sSnapshot.qk = event->qk;
        sSnapshot.yaw = event->yaw;
        sSnapshot.pitch = event->pitch;
        sSnapshot.roll = event->roll;
        sSnapshot.orientationErrRad = event->orientationErrRad;
    } else if (event->groupBit == IMU_GROUP_BIT_ACCEL) {
        sSnapshot.ax = event->ax;
        sSnapshot.ay = event->ay;
        sSnapshot.az = event->az;
    } else {
        sSnapshot.gx = event->gx;
        sSnapshot.gy = event->gy;
        sSnapshot.gz = event->gz;
    }

    adopt_meta(idx, event->groupBit, event->groupEventSeq,
               event->hostDecodeNs, event->sensorTimeUs,
               event->deviceReportSeq, event->rawStatus);
}

static void handle_reset(void)
{
    if (sState == IMU_READER_STATE_CLOSED ||
        sState == IMU_READER_STATE_OPENING) {
        return;
    }

    if (!sRecoveryEpochTaken) {
        increment_epoch();
        sRecoveryEpochTaken = true;
    }

    sState = IMU_READER_STATE_RECOVERING;
    sync_header();
}

static void asyncEventCallback(void *cookie, sh2_AsyncEvent_t *pEvent)
{
    (void)cookie;

    if (pEvent && pEvent->eventId == SH2_RESET) {
        handle_reset();
    }
}

static void sensorCallback(void *cookie, sh2_SensorEvent_t *pEvent)
{
    sh2_SensorValue_t value;
    ImuSessionTestGroupEvent_t event;
    uint64_t hostNs;
    int idx;

    (void)cookie;

    if (!pEvent) {
        return;
    }

    memset(&value, 0, sizeof(value));
    if (sh2_decodeSensorEvent(&value, pEvent) != SH2_OK) {
        return;
    }

    memset(&event, 0, sizeof(event));
    hostNs = monotonic_ns();

    switch (value.sensorId) {
        case SH2_ROTATION_VECTOR: {
            const sh2_RotationVectorWAcc_t *rv = &value.un.rotationVector;

            event.groupBit = IMU_GROUP_BIT_ROTATION;
            event.qw = rv->real;
            event.qi = rv->i;
            event.qj = rv->j;
            event.qk = rv->k;
            event.yaw = (float)q_to_yaw(rv->real, rv->i, rv->j, rv->k);
            event.pitch = (float)q_to_pitch(rv->real, rv->i, rv->j, rv->k);
            event.roll = (float)q_to_roll(rv->real, rv->i, rv->j, rv->k);
            event.orientationErrRad = rv->accuracy;
            idx = 0;
            break;
        }

        case SH2_LINEAR_ACCELERATION:
            event.groupBit = IMU_GROUP_BIT_ACCEL;
            event.ax = value.un.linearAcceleration.x;
            event.ay = value.un.linearAcceleration.y;
            event.az = value.un.linearAcceleration.z;
            idx = 1;
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            event.groupBit = IMU_GROUP_BIT_GYRO;
            event.gx = value.un.gyroscope.x;
            event.gy = value.un.gyroscope.y;
            event.gz = value.un.gyroscope.z;
            idx = 2;
            break;

        default:
            return;
    }

    sAssignedSeq[idx] += 1ull;
    event.groupEventSeq = sAssignedSeq[idx];
    event.hostDecodeNs = hostNs;
    event.sensorTimeUs = pEvent->timestamp_uS;
    event.deviceReportSeq = value.sequence;
    event.rawStatus = value.status;
    apply_group(&event);
}

static bool finish_open(bool success)
{
    sOpenAttempted = true;
    sHasMailbox = true;

    if (!success) {
        sEpoch = IMU_EPOCH_NONE;
        enter_faulted();
        return false;
    }

    increment_epoch();
    sState = IMU_READER_STATE_CONFIGURING;
    sRecoveryEpochTaken = false;
    sHardwareConfigured = false;
    sync_header();
    return true;
}

bool imu_session_open(void)
{
    if (sOpenAttempted || sState != IMU_READER_STATE_CLOSED) {
        return false;
    }

    sState = IMU_READER_STATE_OPENING;
    zero_mailbox();

    sHal = sh2_hal_rpi_init();
    if (!sHal) {
        return finish_open(false);
    }

    if (sh2_open(sHal, asyncEventCallback, NULL) != SH2_OK) {
        sHal = NULL;
        return finish_open(false);
    }

    if (sh2_setSensorCallback(sensorCallback, NULL) != SH2_OK) {
        sh2_close();
        sHal = NULL;
        return finish_open(false);
    }

    return finish_open(true);
}

bool imu_session_configure_production(uint8_t flightCalMask)
{
    bool policyChanged;

    if (sState != IMU_READER_STATE_CONFIGURING) {
        return false;
    }

    if (!apply_hardware_production(flightCalMask)) {
        enter_faulted();
        return false;
    }

    policyChanged = sHardwareConfigured && (flightCalMask != sFlightCalMask);
    if (policyChanged) {
        increment_epoch();
        sState = IMU_READER_STATE_CONFIGURING;
    }

    sFlightCalMask = flightCalMask;
    sHardwareConfigured = true;
    sync_header();
    return true;
}

bool imu_session_begin_settle(void)
{
    if (sState != IMU_READER_STATE_CONFIGURING) {
        return false;
    }

    sState = IMU_READER_STATE_SETTLING;
    sync_header();
    return true;
}

bool imu_session_mark_operational(void)
{
    if (sState != IMU_READER_STATE_SETTLING) {
        return false;
    }

    sState = IMU_READER_STATE_OPERATIONAL;
    sync_header();
    return true;
}

void imu_session_service(void)
{
    if (sHal) {
        sh2_service();
    }
}

bool imu_session_get_snapshot(ImuSampleSnapshot_t *out)
{
    if (!out || !sHasMailbox) {
        return false;
    }

    *out = sSnapshot;
    return true;
}

bool imu_session_begin_recovery(void)
{
    if (sState == IMU_READER_STATE_CLOSED ||
        sState == IMU_READER_STATE_OPENING) {
        return false;
    }

    if (!sRecoveryEpochTaken) {
        increment_epoch();
        sRecoveryEpochTaken = true;
        sState = IMU_READER_STATE_RECOVERING;
        sync_header();
    }

    if (!reopen_hardware()) {
        enter_faulted();
        return false;
    }

    sState = IMU_READER_STATE_CONFIGURING;
    sRecoveryEpochTaken = false;
    sync_header();
    return true;
}

void imu_session_close(void)
{
    if (sHal) {
        sh2_close();
        sHal = NULL;
    }

    sState = IMU_READER_STATE_CLOSED;
    sEpoch = IMU_EPOCH_NONE;
    clear_validity();
    sync_header();
}

void imu_session_test_reset(void)
{
    if (sHal) {
        sh2_close();
        sHal = NULL;
    }

    sHasMailbox = false;
    sOpenAttempted = false;
    sState = IMU_READER_STATE_CLOSED;
    sEpoch = IMU_EPOCH_NONE;
    sFlightCalMask = 0;
    sHardwareConfigured = false;
    sRecoveryEpochTaken = false;
    memset(sAdoptedSeq, 0, sizeof(sAdoptedSeq));
    memset(sHaveAdopted, 0, sizeof(sHaveAdopted));
    memset(sAssignedSeq, 0, sizeof(sAssignedSeq));
    zero_mailbox();
    sHasMailbox = false;
    sSnapshot.readerState = IMU_READER_STATE_CLOSED;
}

bool imu_session_test_open(bool success)
{
    if (sOpenAttempted || sState != IMU_READER_STATE_CLOSED) {
        return false;
    }

    sState = IMU_READER_STATE_OPENING;
    zero_mailbox();
    return finish_open(success);
}

void imu_session_test_inject_reset(void)
{
    handle_reset();
}

bool imu_session_test_force_state(ImuReaderState_t state)
{
    if ((int)state < IMU_READER_STATE_CLOSED ||
        state > IMU_READER_STATE_FAULTED) {
        return false;
    }

    sState = state;
    sync_header();
    return true;
}

void imu_session_test_inject_group(const ImuSessionTestGroupEvent_t *event)
{
    apply_group(event);
}