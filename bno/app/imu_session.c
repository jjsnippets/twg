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
#define FRS_RECORD_DCD      0x1F1Fu

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
static ImuCalFacts_t s_calFacts;
static uint8_t s_calMask;
static bool s_calFactsValid;
static bool s_testSaveDcdOk = true;
static bool s_testReopenOk = true;
static bool s_testDcdFlashDeleteOk = true;
static bool s_testDcdClearResetOk = true;
static uint32_t s_dcdFlashDeleteAttempts;
static uint32_t s_dcdClearResetAttempts;
static ImuTareFacts_t s_tareFacts;
static ImuCheckFacts_t s_checkFacts;
static bool s_testConfigureCheckOk = true;
static bool s_testCheckReadbackAvailable = true;
static uint8_t s_testCheckReadbackMask;
static bool s_testCheckReadbackOverride;
static ImuSessionTestReportConfig_t
s_reportConfig[IMU_SESSION_TEST_REPORT_COUNT];
static bool s_testConfigureTareOk = true;
static bool s_testTareNowOk = true;
static bool s_testPersistTareOk = true;
static bool s_testClearTareOk = true;
static uint8_t s_lastTareAxes;
static uint8_t s_lastTareBasis;
static bool s_haveLastTareNow;
static uint64_t s_tareEventSeq;
static bool s_testProductionOk = true;
static bool s_recoveryObserved;
static uint32_t s_recoveryAttempts;
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

static void clear_cal_facts(void)
{
    memset(&s_calFacts, 0, sizeof(s_calFacts));
    s_calFacts.version = IMU_CAL_FACTS_VERSION;
    s_calFacts.configurationEpoch = sEpoch;
    s_calFactsValid = false;
}

static void clear_tare_facts(void)
{
    uint64_t seq = s_tareEventSeq;

    memset(&s_tareFacts, 0, sizeof(s_tareFacts));
    s_tareFacts.version = IMU_TARE_FACTS_VERSION;
    s_tareFacts.configurationEpoch = sEpoch;
    s_tareEventSeq = seq;
}

static void clear_check_facts(void)
{
    memset(&s_checkFacts, 0, sizeof(s_checkFacts));
    s_checkFacts.version = IMU_CHECK_FACTS_VERSION;
    s_checkFacts.configurationEpoch = sEpoch;
}

static void increment_epoch(void)
{
    sEpoch += 1u;
    clear_validity();
    clear_cal_facts();
    clear_tare_facts();
    clear_check_facts();
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

    /* This is an attempted report configuration, not receipt evidence. */
    if (sensorId == SH2_ROTATION_VECTOR) {
        s_reportConfig[IMU_SESSION_TEST_REPORT_RV].attempted = true;
        s_reportConfig[IMU_SESSION_TEST_REPORT_RV].reportIntervalUs =
            SENSOR_INTERVAL_US;
        s_reportConfig[IMU_SESSION_TEST_REPORT_RV].batchIntervalUs = 0u;
    } else if (sensorId == SH2_LINEAR_ACCELERATION) {
        s_reportConfig[IMU_SESSION_TEST_REPORT_LINEAR].attempted = true;
        s_reportConfig[IMU_SESSION_TEST_REPORT_LINEAR].reportIntervalUs =
            SENSOR_INTERVAL_US;
        s_reportConfig[IMU_SESSION_TEST_REPORT_LINEAR].batchIntervalUs = 0u;
    } else if (sensorId == SH2_GYROSCOPE_CALIBRATED) {
        s_reportConfig[IMU_SESSION_TEST_REPORT_GYRO].attempted = true;
        s_reportConfig[IMU_SESSION_TEST_REPORT_GYRO].reportIntervalUs =
            SENSOR_INTERVAL_US;
        s_reportConfig[IMU_SESSION_TEST_REPORT_GYRO].batchIntervalUs = 0u;
    }

    return sh2_setSensorConfig(sensorId, &cfg) == SH2_OK;
}

static void note_check_report(ImuSessionTestReportId_t report,
                              uint32_t intervalUs)
{
    s_reportConfig[report].attempted = true;
    s_reportConfig[report].reportIntervalUs = intervalUs;
    s_reportConfig[report].batchIntervalUs = 0u;
}

/* Zero interval is the SH-2 disable request; batch stays zero. */
static bool set_check_report(sh2_SensorId_t sensorId,
                             ImuSessionTestReportId_t report,
                             uint32_t intervalUs)
{
    sh2_SensorConfig_t cfg;

    note_check_report(report, intervalUs);
    if (!sHal) {
        return true;
    }
    memset(&cfg, 0, sizeof(cfg));
    cfg.reportInterval_us = intervalUs;
    cfg.batchInterval_us = 0u;
    return sh2_setSensorConfig(sensorId, &cfg) == SH2_OK;
}

/*
 * Configure the diagnostic set explicitly; do not reuse CALIBRATION mode.
 * Stage the hardware first and publish the new reader epoch only after the
 * report set and sh2_setCalConfig have succeeded.
 */
static bool apply_hardware_check(uint8_t mask,
                                 ImuSessionCheckConfigResult_t *out)
{
    uint8_t readback = 0u;

    if (!s_testConfigureCheckOk) {
        return false;
    }
    if (!set_check_report(SH2_LINEAR_ACCELERATION,
                          IMU_SESSION_TEST_REPORT_LINEAR, 0u) ||
        !set_check_report(SH2_MAGNETIC_FIELD_CALIBRATED,
                          IMU_SESSION_TEST_REPORT_MAG,
                          IMU_CHECK_MAG_INTERVAL_US) ||
        !set_check_report(SH2_ACCELEROMETER,
                          IMU_SESSION_TEST_REPORT_ACCEL,
                          IMU_CHECK_SLOW_INTERVAL_US) ||
        !set_check_report(SH2_GYROSCOPE_CALIBRATED,
                          IMU_SESSION_TEST_REPORT_GYRO,
                          IMU_CHECK_SLOW_INTERVAL_US) ||
        !set_check_report(SH2_ROTATION_VECTOR,
                          IMU_SESSION_TEST_REPORT_RV,
                          IMU_CHECK_SLOW_INTERVAL_US)) {
        return false;
    }

    if (sHal) {
        if (sh2_setCalConfig(mask) != SH2_OK) {
            return false;
        }
        out->actualMaskValid = sh2_getCalConfig(&readback) == SH2_OK;
    } else {
        out->actualMaskValid = s_testCheckReadbackAvailable;
        readback = s_testCheckReadbackOverride
                 ? s_testCheckReadbackMask : mask;
    }

    if (out->actualMaskValid) {
        out->actualMask = readback;
        s_calMask = readback;
    } else {
        /* Existing policy getter may hold an applied/requested estimate.
         * Only out->actualMaskValid may assert observed readback. */
        s_calMask = mask;
    }
    return true;
}

static bool configure_sensor_interval(sh2_SensorId_t sensorId, uint32_t intervalUs)
{
    sh2_SensorConfig_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.reportInterval_us = intervalUs;
    cfg.batchInterval_us = 0;
    return sh2_setSensorConfig(sensorId, &cfg) == SH2_OK;
}

static bool apply_hardware_calibration(uint8_t calMask)
{
    uint8_t readback = calMask;
    if (!sHal) {
      s_calMask = calMask;
      return true;
    }
    
    if (!configure_sensor_interval(SH2_MAGNETIC_FIELD_CALIBRATED, IMU_CAL_MAG_INTERVAL_US) ||
        !configure_sensor_interval(SH2_ACCELEROMETER, IMU_CAL_SLOW_INTERVAL_US) ||
        !configure_sensor_interval(SH2_GYROSCOPE_CALIBRATED, IMU_CAL_SLOW_INTERVAL_US) ||
        !configure_sensor_interval(SH2_ROTATION_VECTOR, IMU_CAL_SLOW_INTERVAL_US)) {
        return false;
    }

    if (sh2_setCalConfig(calMask) != SH2_OK) {
        return false;
    }
    
    if (sh2_getCalConfig(&readback) == SH2_OK) {
        s_calMask = readback;
    } else {
        s_calMask = calMask;
    }
    
    return true;
}

static void adopt_cal_facts_from_event(const sh2_SensorValue_t *value, uint64_t hostNs)
{
    if (value == NULL) {
        return;
    }
    
    s_calFacts.version = IMU_CAL_FACTS_VERSION;
    s_calFacts.configurationEpoch = sEpoch;
    s_calFacts.hostDecodeNs = hostNs;
    s_calFacts.valid = true;
    s_calFactsValid = true;
    
    switch (value->sensorId) {
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        s_calFacts.magAccuracy = value->status;
        s_calFacts.haveMag = true;
        s_calFacts.magXuT = value->un.magneticField.x;
        s_calFacts.magYuT = value->un.magneticField.y;
        s_calFacts.magZuT = value->un.magneticField.z;
        break;
      case SH2_ACCELEROMETER:
        s_calFacts.accelAccuracy = value->status;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        s_calFacts.gyroAccuracy = value->status;
        break;
      case SH2_ROTATION_VECTOR:
        s_calFacts.rvAccuracy = value->status;
        s_calFacts.rvErrRad = value->un.rotationVector.accuracy;
        break;
      default:
        break;
    }
}

static bool apply_hardware_production(uint8_t flightCalMask)
{
    uint8_t readback = flightCalMask;
    
    if (!sHal) {
        if (!s_testProductionOk) {
            return false;
        }
        /* Model the exact production report set in the host trace. */
        note_check_report(IMU_SESSION_TEST_REPORT_MAG, 0u);
        note_check_report(IMU_SESSION_TEST_REPORT_ACCEL, 0u);
        note_check_report(IMU_SESSION_TEST_REPORT_RV, SENSOR_INTERVAL_US);
        note_check_report(IMU_SESSION_TEST_REPORT_LINEAR,
                          SENSOR_INTERVAL_US);
        note_check_report(IMU_SESSION_TEST_REPORT_GYRO,
                          SENSOR_INTERVAL_US);
        s_calMask = flightCalMask;
        return true;
    }

    /* Disable diagnostic-only reports before restoring the 100 Hz set. */
    if (!set_check_report(SH2_ACCELEROMETER,
                          IMU_SESSION_TEST_REPORT_ACCEL, 0u) ||
        !set_check_report(SH2_MAGNETIC_FIELD_CALIBRATED,
                          IMU_SESSION_TEST_REPORT_MAG, 0u) ||
        !configure_sensor(SH2_ROTATION_VECTOR) ||
        !configure_sensor(SH2_LINEAR_ACCELERATION) ||
        !configure_sensor(SH2_GYROSCOPE_CALIBRATED) ||
        sh2_setCalConfig(flightCalMask) != SH2_OK) {
        return false;
    }

    if (sh2_getCalConfig(&readback) == SH2_OK) {
        s_calMask = readback;
    } else {
        s_calMask = flightCalMask;
    }

    return true;
}

static void adopt_check_facts_from_event(const sh2_SensorValue_t *value,
                                         uint64_t hostNs)
{
    if (value == NULL || sEpoch == IMU_EPOCH_NONE) {
        return;
    }

    s_checkFacts.version = IMU_CHECK_FACTS_VERSION;
    s_checkFacts.configurationEpoch = sEpoch;
    switch (value->sensorId) {
    case SH2_ACCELEROMETER:
        s_checkFacts.haveAccel = true;
        s_checkFacts.accelHostDecodeNs = hostNs;
        s_checkFacts.accelStatus = value->status;
        break;
    case SH2_GYROSCOPE_CALIBRATED:
        s_checkFacts.haveGyro = true;
        s_checkFacts.gyroHostDecodeNs = hostNs;
        s_checkFacts.gyroStatus = value->status;
        break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
        s_checkFacts.haveMag = true;
        s_checkFacts.magHostDecodeNs = hostNs;
        s_checkFacts.magStatus = value->status;
        s_checkFacts.magXuT = value->un.magneticField.x;
        s_checkFacts.magYuT = value->un.magneticField.y;
        s_checkFacts.magZuT = value->un.magneticField.z;
        break;
    case SH2_ROTATION_VECTOR:
        s_checkFacts.haveRv = true;
        s_checkFacts.rvHostDecodeNs = hostNs;
        s_checkFacts.rvStatus = value->status;
        s_checkFacts.rvErrRad = value->un.rotationVector.accuracy;
        break;
    default:
        return;
    }
    s_checkFacts.valid = true;
}

static bool reopen_hardware(void)
{
    if (!sHal) {
        return s_testReopenOk;
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

static bool apply_hardware_tare(void)
{
    sh2_SensorConfig_t off;
    uint8_t readback = 0u;

    memset(&off, 0, sizeof(off));
    off.reportInterval_us = 0u;

    if (!sHal) {
        if (!s_testConfigureTareOk) {
            return false;
        }
        s_calMask = 0u;
        return true;
    }

    if (!configure_sensor(SH2_ROTATION_VECTOR) ||
        sh2_setSensorConfig(SH2_LINEAR_ACCELERATION, &off) != SH2_OK ||
        sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &off) != SH2_OK ||
        sh2_setSensorConfig(SH2_MAGNETIC_FIELD_CALIBRATED, &off) != SH2_OK ||
        sh2_setCalConfig(0u) != SH2_OK) {
        return false;
    }

    if (sh2_getCalConfig(&readback) == SH2_OK) {
        s_calMask = readback;
    } else {
        s_calMask = 0u;
    }
    return true;
}

static bool map_tare_axes(ImuSessionTareAxes_t axes, uint8_t *out)
{
    if (out == NULL) {
        return false;
    }
    if (axes == IMU_SESSION_TARE_AXES_Z) {
        *out = SH2_TARE_Z;
        return true;
    }
    if (axes == IMU_SESSION_TARE_AXES_FULL) {
        *out = (uint8_t)(SH2_TARE_X | SH2_TARE_Y | SH2_TARE_Z);
        return true;
    }
    return false;
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

    if (sState == IMU_READER_STATE_CHECK ||
        sState == IMU_READER_STATE_PROBE) {
        adopt_check_facts_from_event(&value, hostNs);
        return; /* Diagnostic data must not enter production R2. */
    }

    if (sState == IMU_READER_STATE_CALIBRATION) {
      switch (value.sensorId) {
        case SH2_MAGNETIC_FIELD_CALIBRATED:
        case SH2_ACCELEROMETER:
        case SH2_GYROSCOPE_CALIBRATED:
        case SH2_ROTATION_VECTOR:
            adopt_cal_facts_from_event(&value, hostNs);
            return;
        default:
            return;
      }
    }

    if (sState == IMU_READER_STATE_TARE) {
        if (value.sensorId == SH2_ROTATION_VECTOR) {
            s_tareEventSeq += 1ull;
            s_tareFacts.version = IMU_TARE_FACTS_VERSION;
            s_tareFacts.configurationEpoch = sEpoch;
            s_tareFacts.hostDecodeNs = hostNs;
            s_tareFacts.rotationEventSequence = s_tareEventSeq;
            s_tareFacts.rotationStatus = value.status;
            s_tareFacts.quatI = value.un.rotationVector.i;
            s_tareFacts.quatJ = value.un.rotationVector.j;
            s_tareFacts.quatK = value.un.rotationVector.k;
            s_tareFacts.quatReal = value.un.rotationVector.real;
            s_tareFacts.oriErrRad = value.un.rotationVector.accuracy;
            q_to_ypr(s_tareFacts.quatReal, s_tareFacts.quatI, 
                     s_tareFacts.quatJ, s_tareFacts.quatK, 
                     &s_tareFacts.yawRad, &s_tareFacts.pitchRad,
                     &s_tareFacts.rollRad);
            s_tareFacts.valid = true;
        }
        return;
    }

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

bool imu_session_configure_check(ImuSessionCheckMode_t mode, uint8_t mask,
                                  ImuSessionCheckConfigResult_t *out)
{
    ImuSessionCheckConfigResult_t observed;
    ImuReaderState_t target;

    if (out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    if (sState != IMU_READER_STATE_CONFIGURING ||
        (mode != IMU_SESSION_CHECK_MODE_CHECK &&
         mode != IMU_SESSION_CHECK_MODE_PROBE) ||
        (mode == IMU_SESSION_CHECK_MODE_CHECK && mask != 0u)) {
        return false;
    }

    memset(&observed, 0, sizeof(observed));
    if (!apply_hardware_check(mask, &observed)) {
        /* SH-2 calls may already have changed reports: not an actionable
         * diagnostic mode, and no successful entry epoch is claimed. */
        enter_faulted();
        return false;
    }

    target = mode == IMU_SESSION_CHECK_MODE_CHECK
           ? IMU_READER_STATE_CHECK : IMU_READER_STATE_PROBE;
    increment_epoch();
    sHardwareConfigured = true;
    sState = target;
    sync_header();
    *out = observed;

    /* Readable mismatch is failure, but the configured session remains
     * actionable: the machine can request production restoration. */
    return !observed.actualMaskValid || observed.actualMask == mask;
}

bool imu_session_get_check_facts(ImuCheckFacts_t *out)
{
    if (out == NULL || !sHasMailbox) {
        return false;
    }
    *out = s_checkFacts;
    return true;
}

bool imu_session_restore_production(uint8_t flightCalMask)
{
    bool hadConfiguredReports;

    if (sState != IMU_READER_STATE_CALIBRATION &&
        sState != IMU_READER_STATE_TARE &&
        sState != IMU_READER_STATE_CHECK &&
        sState != IMU_READER_STATE_PROBE &&
        sState != IMU_READER_STATE_CONFIGURING) {
        return false;
    }

    hadConfiguredReports = sHardwareConfigured;
    if (!apply_hardware_production(flightCalMask)) {
        enter_faulted();
        return false;
    }

    if (hadConfiguredReports) {
        increment_epoch();
    }

    sFlightCalMask = flightCalMask;
    sHardwareConfigured = true;
    sState = IMU_READER_STATE_CONFIGURING;
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
    
    s_recoveryObserved = true;
    s_recoveryAttempts += 1u;

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
    s_calMask = 0;
    s_calFactsValid = false;
    s_testSaveDcdOk = true;
    s_testReopenOk = true;
    s_testDcdFlashDeleteOk = true;
    s_testDcdClearResetOk = true;
    s_dcdFlashDeleteAttempts = 0u;
    s_dcdClearResetAttempts = 0u;
    s_testProductionOk = true;
    s_recoveryObserved = false;
    s_recoveryAttempts = 0u;
    memset(&s_calFacts, 0, sizeof(s_calFacts));
    s_calFacts.version = IMU_CAL_FACTS_VERSION;
    s_testConfigureTareOk = true;
    s_testConfigureCheckOk = true;
    s_testCheckReadbackAvailable = true;
    s_testCheckReadbackMask = 0u;
    s_testCheckReadbackOverride = false;
    memset(s_reportConfig, 0, sizeof(s_reportConfig));
    s_testTareNowOk = true;
    s_testPersistTareOk = true;
    s_testClearTareOk = true;
    s_lastTareAxes = 0u;
    s_lastTareBasis = 0u;
    s_haveLastTareNow = false;
    s_tareEventSeq = 0ull;
    zero_mailbox();
    clear_tare_facts();
    clear_check_facts();
    sHasMailbox = false;
    sSnapshot.readerState = IMU_READER_STATE_CLOSED;
}

void imu_session_test_set_configure_check_result(bool success)
{
    s_testConfigureCheckOk = success;
}

void imu_session_test_set_check_readback(bool available, uint8_t actualMask)
{
    s_testCheckReadbackAvailable = available;
    s_testCheckReadbackMask = actualMask;
    s_testCheckReadbackOverride = true;
}

void imu_session_test_inject_check_report(ImuSessionTestReportId_t report,
                                          uint8_t status,
                                          uint64_t hostDecodeNs,
                                          float rvErrRad,
                                          float magX, float magY, float magZ)
{
    if (sState != IMU_READER_STATE_CHECK &&
        sState != IMU_READER_STATE_PROBE) {
        return;
    }
    s_checkFacts.version = IMU_CHECK_FACTS_VERSION;
    s_checkFacts.configurationEpoch = sEpoch;
    switch (report) {
    case IMU_SESSION_TEST_REPORT_ACCEL:
        s_checkFacts.haveAccel = true;
        s_checkFacts.accelHostDecodeNs = hostDecodeNs;
        s_checkFacts.accelStatus = status;
        break;
    case IMU_SESSION_TEST_REPORT_GYRO:
        s_checkFacts.haveGyro = true;
        s_checkFacts.gyroHostDecodeNs = hostDecodeNs;
        s_checkFacts.gyroStatus = status;
        break;
    case IMU_SESSION_TEST_REPORT_MAG:
        s_checkFacts.haveMag = true;
        s_checkFacts.magHostDecodeNs = hostDecodeNs;
        s_checkFacts.magStatus = status;
        s_checkFacts.magXuT = magX;
        s_checkFacts.magYuT = magY;
        s_checkFacts.magZuT = magZ;
        break;
    case IMU_SESSION_TEST_REPORT_RV:
        s_checkFacts.haveRv = true;
        s_checkFacts.rvHostDecodeNs = hostDecodeNs;
        s_checkFacts.rvStatus = status;
        s_checkFacts.rvErrRad = rvErrRad;
        break;
    default:
        return;
    }
    s_checkFacts.valid = true;
}

void imu_session_test_inject_check_facts(const ImuCheckFacts_t *facts)
{
    if (facts == NULL ||
        (sState != IMU_READER_STATE_CHECK &&
         sState != IMU_READER_STATE_PROBE)) {
        return;
    }
    /* Deliberately do not fix version or epoch: adapter negative tests
     * must be able to observe and reject malformed mailbox snapshots. */
    s_checkFacts = *facts;
}

bool imu_session_test_get_report_config(ImuSessionTestReportId_t report,
                                        ImuSessionTestReportConfig_t *out)
{
    if (out == NULL || (unsigned)report >=
                       (unsigned)IMU_SESSION_TEST_REPORT_COUNT) {
        return false;
    }
    *out = s_reportConfig[report];
    return true;
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


bool imu_session_configure_calibration(uint8_t calMask)
{
    if (sState != IMU_READER_STATE_CONFIGURING) {
        return false;
    }
    if (!apply_hardware_calibration(calMask)) {
        enter_faulted();
        return false;
    }
    increment_epoch();
    sHardwareConfigured = true;
    sState = IMU_READER_STATE_CALIBRATION;
    sync_header();
    return true;
}

bool imu_session_get_cal_policy(uint8_t *outMask)
{
    if (outMask == NULL || sState == IMU_READER_STATE_CLOSED ||
        sState == IMU_READER_STATE_OPENING) {
        return false;
    }
    *outMask = s_calMask;
    return true;
}

bool imu_session_save_dcd(void)
{
    if (sState == IMU_READER_STATE_CLOSED || sState == IMU_READER_STATE_OPENING) {
        return false;
    }
    if (!sHal) {
        return s_testSaveDcdOk;
    }
    return sh2_saveDcdNow() == SH2_OK;
}

bool imu_session_clear_dcd(void)
{
    uint32_t dummy = 0u;

    if (sState != IMU_READER_STATE_CONFIGURING) {
        return false;
    }

    ++s_dcdFlashDeleteAttempts;
    if (sHal) {
        if (sh2_setFrs(FRS_RECORD_DCD, &dummy, 0) != SH2_OK) {
            return false;
        }
    } else if (!s_testDcdFlashDeleteOk) {
        return false;
    }

    ++s_dcdClearResetAttempts;
    if (sHal) {
        if (sh2_clearDcdAndReset() != SH2_OK) {
            return false;
       }
    } else if (!s_testDcdClearResetOk) {
        return false;
    }

    /*
     * The command has no ordinary response and resets the chip. Reopen
     * using the existing owner/recovery path, never imu_session_open().
     * If SH2_RESET already took this epoch, begin_recovery coalesces it.
     */
    return imu_session_begin_recovery();
}

bool imu_session_begin_verification_reopen(void)
{
    bool ok;

    if (sState == IMU_READER_STATE_CLOSED || sState == IMU_READER_STATE_OPENING) {
        return false;
    }
    increment_epoch();
    ok = (sHal == NULL) ? s_testReopenOk : reopen_hardware();
    if (!ok) {
        s_recoveryObserved = true;
        s_recoveryAttempts += 1u;
        sState = IMU_READER_STATE_RECOVERING;
        enter_faulted();
        return false;
    }
    sState = IMU_READER_STATE_CONFIGURING;
    sRecoveryEpochTaken = false;
    sync_header();
    return true;
}

bool imu_session_get_cal_facts(ImuCalFacts_t *out)
{
    if (out == NULL || !sHasMailbox) {
        return false;
    }
    *out = s_calFacts;
    return true;
}

void imu_session_test_inject_cal_facts(const ImuCalFacts_t *facts)
{
    if (facts == NULL) {
        return;
    }
    s_calFacts = *facts;
    s_calFacts.version = IMU_CAL_FACTS_VERSION;
    s_calFacts.configurationEpoch = sEpoch;
    s_calFactsValid = facts->valid;
}

void imu_session_test_set_save_dcd_result(bool success)
{
    s_testSaveDcdOk = success;
}

void imu_session_test_set_clear_dcd_results(bool flashDeleteOk,
                                            bool clearResetOk)
{
    s_testDcdFlashDeleteOk = flashDeleteOk;
    s_testDcdClearResetOk = clearResetOk;
}

uint32_t imu_session_test_dcd_flash_delete_attempts(void)
{
    return s_dcdFlashDeleteAttempts;
}

uint32_t imu_session_test_dcd_clear_reset_attempts(void)
{
    return s_dcdClearResetAttempts;
}

void imu_session_test_set_reopen_result(bool success)
{
    s_testReopenOk = success;
}

void imu_session_test_set_production_result(bool success)
{
    s_testProductionOk = success;
}

bool imu_session_test_recovery_observed(void)
{
    return s_recoveryObserved;
}

uint32_t imu_session_test_recovery_attempt_count(void)
{
    return s_recoveryAttempts;
}

bool imu_session_configure_tare(void)
{
    if (sState != IMU_READER_STATE_CONFIGURING) {
        return false;
    }
    if (!apply_hardware_tare()) {
        enter_faulted();
        return false;
    }
    increment_epoch();
    sHardwareConfigured = true;
    sState = IMU_READER_STATE_TARE;
    sync_header();
    return true;
}

bool imu_session_tare_now(ImuSessionTareAxes_t axes)
{
    uint8_t mapped = 0u;

    if (sState != IMU_READER_STATE_TARE || !map_tare_axes(axes, &mapped)) {
        return false;
    }

    s_lastTareAxes = mapped;
    s_lastTareBasis = (uint8_t)SH2_TARE_BASIS_ROTATION_VECTOR;
    s_haveLastTareNow = true;

    if (sHal) {
        if (sh2_setTareNow(mapped, SH2_TARE_BASIS_ROTATION_VECTOR) != SH2_OK) {
            return false;
        }
    } else if (!s_testTareNowOk) {
        return false;
    }

    increment_epoch();
    sState = IMU_READER_STATE_TARE;
    sync_header();
    return true;
}

bool imu_session_persist_tare(void)
{
    if (sState != IMU_READER_STATE_TARE) {
        return false;
    }
    if (!sHal) {
        return s_testPersistTareOk;
    }
    return sh2_persistTare() == SH2_OK;
}

bool imu_session_clear_tare(ImuSessionClearTareResult_t *out)
{
    bool ok;

    if (out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->clearActive = IMU_SESSION_TARE_SUB_NOT_ATTEMPTED;
    out->clearSaved = IMU_SESSION_TARE_SUB_NOT_ATTEMPTED;

    if (sState != IMU_READER_STATE_TARE) {
        return false;
    }

    ok = sHal ? (sh2_clearTare() == SH2_OK) : s_testClearTareOk;
    out->success = ok;
    out->clearActive = ok ? IMU_SESSION_TARE_SUB_SUCCEEDED
                          : IMU_SESSION_TARE_SUB_FAILED;
    out->clearSaved = out->clearActive;
    if (ok) {
        increment_epoch();
        sState = IMU_READER_STATE_TARE;
        sync_header();
    }
    return ok;
}

bool imu_session_get_tare_facts(ImuTareFacts_t *out)
{
    if (out == NULL || !sHasMailbox) {
        return false;
    }
    *out = s_tareFacts;
    return true;
}

void imu_session_test_inject_tare_facts(const ImuTareFacts_t *facts)
{
    uint64_t seq;

    if (facts == NULL) {
        return;
    }
    seq = s_tareEventSeq;
    s_tareFacts = *facts;
    s_tareFacts.version = IMU_TARE_FACTS_VERSION;
    s_tareFacts.configurationEpoch = sEpoch;
    s_tareEventSeq = seq;
}

void imu_session_test_set_configure_tare_result(bool success)
{
    s_testConfigureTareOk = success;
}

void imu_session_test_set_tare_now_result(bool success)
{
    s_testTareNowOk = success;
}

void imu_session_test_set_persist_tare_result(bool success)
{
    s_testPersistTareOk = success;
}

void imu_session_test_set_clear_tare_result(bool success)
{
    s_testClearTareOk = success;
}

uint8_t imu_session_test_last_tare_axes(void)
{
    return s_lastTareAxes;
}

uint8_t imu_session_test_last_tare_basis(void)
{
    return s_lastTareBasis;
}

bool imu_session_test_have_last_tare_now(void)
{
    return s_haveLastTareNow;
}
