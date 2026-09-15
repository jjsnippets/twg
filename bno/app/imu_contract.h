#ifndef IMU_CONTRACT_H
#define IMU_CONTRACT_H

#include <stdint.h>

/*
 * Phase 2 public BNO contract generation 1.
 *
 * Incompatible with IMU_SAMPLE_STRUCT_VERSION 3 / ImuSample_t.
 * This header is facts only: R0 bits, R1 metadata, minimal R2 snapshot, R3
 * state. It must not grow publisher freshness, publication sequence, ages,
 * command records, or runtime counters.
 */

#define IMU_METADATA_CONTRACT_VERSION  1u
#define IMU_SAMPLE_CONTRACT_VERSION    1u

#define IMU_GROUP_BIT_ROTATION         (1u << 0)  /* SH2_ROTATION_VECTOR */
#define IMU_GROUP_BIT_ACCEL            (1u << 1)  /* SH2_LINEAR_ACCELERATION */
#define IMU_GROUP_BIT_GYRO             (1u << 2)  /* SH2_GYROSCOPE_CALIBRATED */
#define IMU_GROUP_MASK_REQUIRED        0x07u

#define IMU_EPOCH_NONE                 0u
#define IMU_GROUP_EVENT_SEQ_NONE       0ull

typedef enum {
    IMU_READER_STATE_CLOSED       = 0,
    IMU_READER_STATE_OPENING      = 1,
    IMU_READER_STATE_CONFIGURING  = 2,
    IMU_READER_STATE_SETTLING     = 3,
    IMU_READER_STATE_OPERATIONAL  = 4,
    IMU_READER_STATE_CALIBRATION  = 5,
    IMU_READER_STATE_TARE         = 6,
    IMU_READER_STATE_CHECK        = 7,
    IMU_READER_STATE_PROBE        = 8,
    IMU_READER_STATE_RECOVERING   = 9,
    IMU_READER_STATE_FAULTED      = 10
} ImuReaderState_t;

/*
 * R1 per-group metadata. Comparable identity is
 * (configurationEpoch, groupEventSeq) only.
 *
 * configurationEpoch 0 and groupEventSeq 0 are reserved. The first real
 * group event of a process is groupEventSeq 1. Sequence does not wrap and
 * must not be reset on epoch, settle, or a consumer "row 1" request.
 */
typedef struct {
    uint8_t  version;
    uint32_t configurationEpoch;
    uint64_t groupEventSeq;
    uint64_t hostDecodeNs;
    uint64_t sensorTimeUs;
    uint8_t  deviceReportSeq;
    uint32_t epochUpdateCount;
    uint8_t  rawStatus;
} ImuGroupMeta_t;

/*
 * Minimal R2 mailbox. Reader facts only.
 *
 * readerStatusFlags exists for later R4 work and must read as 0 in Phase 2.
 * validMask is sticky seen-in-this-epoch state using IMU_GROUP_BIT_*.
 */
typedef struct {
    uint8_t          version;
    uint32_t         configurationEpoch;
    ImuReaderState_t readerState;
    uint32_t         readerStatusFlags;
    uint64_t         newestHostUpdateNs;
    uint64_t         processDecodeCount;

    ImuGroupMeta_t   rotationMeta;
    float            qw;
    float            qi;
    float            qj;
    float            qk;
    float            yaw;
    float            pitch;
    float            roll;
    float            orientationErrRad;

    ImuGroupMeta_t   accelMeta;
    float            ax;
    float            ay;
    float            az;

    ImuGroupMeta_t   gyroMeta;
    float            gx;
    float            gy;
    float            gz;

    uint8_t          validMask;
} ImuSampleSnapshot_t;

#endif /* IMU_CONTRACT_H */