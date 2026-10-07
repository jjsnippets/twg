#ifndef IMU_PUBLISH_H
#define IMU_PUBLISH_H

/*
 * Phase 9 publication, statistics, termination, and metadata contracts.
 *
 * Contract generation 1 of R9-R13 plus the pure publisher baseline and
 * classification API. No clock reads, scheduling, I/O, or allocation.
 *
 * R0 group bits and IMU_GROUP_MASK_REQUIRED stay owned by imu_contract.h.
 * R1/R2 stay reader facts only; nothing here is added to them.
 *
 * Skipped publication gates follow Phase 1G section 7: they are counted
 * (gatesSkippedBefore, R10 gatesSkipped) and never produce a row. There is
 * no "not evaluated" row kind in CSV schema 1.
 */

#include <stdbool.h>
#include <stdint.h>

#include "app/imu_contract.h"

#define IMU_PUBLICATION_CONTRACT_VERSION   1u   /* R9  */
#define IMU_RUN_STATS_CONTRACT_VERSION     1u   /* R10 */
#define IMU_LOGGER_STATS_CONTRACT_VERSION  1u   /* R11 */
#define IMU_TERMINATION_CONTRACT_VERSION   1u   /* R12 */
#define IMU_RUN_METADATA_CONTRACT_VERSION  1u   /* R13 in-memory record */

/* Companion JSON meta_schema_ver. 2 recorded the approved filename
 * amendment (bno_acq_YYYYMMDD_HHMMSS); Phase 1H defined 1. 3 renames the
 * JSON key filename_utc to filename_local: the stamp is system local
 * wall-clock time with no offset or zone. */
#define IMU_META_SCHEMA_VERSION            3u
#define IMU_CSV_SCHEMA_VERSION             1u
#define IMU_CSV_COLUMN_COUNT               52u
#define IMU_CSV_MISSING_TOKEN              "NaN"

/* Group slot order for R9 per-group blocks. Slot i is R0 bit (1u << i). */
typedef enum {
    IMU_PUBLISH_GROUP_ROTATION = 0,
    IMU_PUBLISH_GROUP_ACCEL    = 1,
    IMU_PUBLISH_GROUP_GYRO     = 2,
    IMU_PUBLISH_GROUP_COUNT    = 3
} ImuPublishGroupIndex_t;

/*
 * Pure publisher-owned comparison state. previousValid means the stored
 * identity came from a valid group in comparisonEpoch. An epoch change
 * invalidates comparability before the changed-epoch row is classified.
 */
typedef struct {
    uint32_t configurationEpoch;
    uint64_t groupEventSeq;
    uint8_t  previousValid;
    uint8_t  reserved[7];
} ImuPublisherPreviousGroup_t;

typedef struct {
    uint32_t comparisonEpoch;
    uint32_t reserved;
    uint64_t nextPublicationSeq;
    ImuPublisherPreviousGroup_t previous[IMU_PUBLISH_GROUP_COUNT];
    uint8_t windowOpen;
    uint8_t reserved8[7];
} ImuPublisherState_t;

#define IMU_PUBLISH_GROUP_BIT(index)       (1u << (unsigned)(index))

/* Phase 1H section 9 serialization-only bit map for reader_flags. */
#define IMU_CSV_FLAG_RESET_OBSERVED              (1u << 0)
#define IMU_CSV_FLAG_DECODE_ERROR_OBSERVED       (1u << 1)
#define IMU_CSV_FLAG_UNEXPECTED_REPORT_OBSERVED  (1u << 2)
#define IMU_CSV_FLAG_REPORT_SEQUENCE_GAP_OBSERVED (1u << 3)
#define IMU_CSV_FLAG_CONFIGURATION_FAILED        (1u << 4)
#define IMU_CSV_FLAG_SESSION_RECOVERY_OCCURRED   (1u << 5)
#define IMU_CSV_READER_FLAGS_MASK                0x3Fu  /* bits 6+ are zero */

/*
 * R9 per-group block. Copied from R1 for CSV; ageNs is meaningful only when
 * ageValid is 1 (Phase 1G section 5.2: invalid age is ageValid 0, ageNs 0).
 * identityPresent is 0 for a group that is missing on this row.
 */
typedef struct {
    uint64_t groupEventSeq;
    uint64_t hostDecodeNs;
    uint64_t sensorTimeUs;
    uint64_t ageNs;
    uint8_t  identityPresent;
    uint8_t  deviceReportSeq;
    uint8_t  rawStatus;
    uint8_t  ageValid;
    uint32_t reserved;          /* must be zero */
} ImuPublicationGroup_t;

/*
 * R9 publication record. One fixed record per emitted tenth-tick gate.
 * Masks use R0 bits only. Flags are 0 or 1. Reserved fields must be zero.
 * Field order, not packing, is the contract; the record is copied by value.
 */
typedef struct {
    uint32_t version;                 /* IMU_PUBLICATION_CONTRACT_VERSION */
    uint32_t configurationEpoch;      /* snapshot epoch; 0 only if no session */
    uint64_t publicationSeq;          /* >= 1, +1 per emitted row, no jumps */
    uint64_t scheduledNs;             /* CLOCK_MONOTONIC, if scheduledValid */
    uint64_t actualNs;                /* CLOCK_MONOTONIC, if actualValid */
    uint64_t loggerDrops;             /* R11 drops before this enqueue attempt */
    uint32_t gatesSkippedBefore;      /* skipped gates since previous row */
    uint32_t readerFlags;             /* R4 copy, IMU_CSV_FLAG_* bit map */
    ImuReaderState_t readerState;     /* R3 copy */

    uint8_t  scheduledValid;
    uint8_t  actualValid;
    uint8_t  deadlineMissed;
    uint8_t  sampleReady;
    uint8_t  notReady;
    uint8_t  validMask;
    uint8_t  freshMask;
    uint8_t  staleMask;
    uint8_t  missingMask;
    uint8_t  multiUpdateMask;
    uint8_t  reserved8[2];            /* must be zero */

    ImuPublicationGroup_t rotation;
    ImuPublicationGroup_t accel;
    ImuPublicationGroup_t gyro;

    /* Rotation vector. qx/qy/qz carry ImuSampleSnapshot_t qi/qj/qk. */
    float qw, qx, qy, qz;
    float yawRad, pitchRad, rollRad, orientationErrRad;
    /* Linear acceleration, m/s^2. */
    float ax, ay, az;
    /* Calibrated gyroscope, rad/s. */
    float gx, gy, gz;
} ImuPublicationRecord_t;

/* R10 run statistics. Publisher/main owned; never part of R2. */
typedef struct {
    uint32_t version;                 /* IMU_RUN_STATS_CONTRACT_VERSION */
    uint32_t reserved;                /* must be zero */
    uint64_t serviceTicks;
    uint64_t serviceOverruns;         /* RT_SleepUntil skipped periods */
    uint64_t clockReadFailures;
    uint64_t publicationsAttempted;
    uint64_t publicationsEnqueued;
    uint64_t publicationsAllValid;
    uint64_t publicationsAllFresh;
    uint64_t publicationsPartiallyFresh;
    uint64_t publicationsNotReady;
    uint64_t deadlineMisses;
    uint64_t gatesSkipped;
    uint64_t staleByGroup[IMU_PUBLISH_GROUP_COUNT];
    uint64_t multiUpdateByGroup[IMU_PUBLISH_GROUP_COUNT];
} ImuRunStats_t;

typedef enum {
    IMU_LOGGER_START_NOT_ATTEMPTED = 0,
    IMU_LOGGER_START_OK            = 1,
    IMU_LOGGER_START_FAILED        = 2
} ImuLoggerStartResult_t;

/* R11 logger statistics. Logger-worker owned. */
typedef struct {
    uint32_t version;                 /* IMU_LOGGER_STATS_CONTRACT_VERSION */
    uint32_t queueCapacity;
    uint32_t queueHighWater;
    ImuLoggerStartResult_t startResult;
    uint64_t recordsEnqueued;
    uint64_t recordsWritten;
    uint64_t recordsDropped;
    uint32_t shutdownErrors;
    uint8_t  writeFailed;             /* sticky archival failure */
    uint8_t  drainStarted;
    uint8_t  drainComplete;
    uint8_t  drainTimedOut;
} ImuLoggerStats_t;

typedef enum {
    IMU_TERM_REASON_NONE = 0,
    IMU_TERM_REASON_COMPLETED,           /* duration reached */
    IMU_TERM_REASON_PROCESS_STOP,        /* SIGINT or SIGTERM */
    IMU_TERM_REASON_NO_ACQUISITION,      /* plan ended before the window */
    IMU_TERM_REASON_SESSION_UNRECOVERED,
    IMU_TERM_REASON_LOGGER_FAILURE,
    IMU_TERM_REASON_STARTUP_FAILURE,
    IMU_TERM_REASON_OWNER_FAILURE,
    IMU_TERM_REASON_COUNT
} ImuTerminationReason_t;

/* R12 termination. Assembled by main after drain and close. */
typedef struct {
    uint32_t version;                 /* IMU_TERMINATION_CONTRACT_VERSION */
    ImuTerminationReason_t reason;
    int32_t  signalNumber;            /* 0 if none, else raw signal number */
    int32_t  exitStatus;
    ImuReaderState_t finalReaderState;
    uint8_t  sessionUnrecovered;
    uint8_t  acquisitionWindowClosed;
    uint8_t  reserved[2];             /* must be zero */
} ImuTermination_t;

#define IMU_META_BASE_NAME_CAPACITY        64u
#define IMU_META_UTC_STAMP_CAPACITY        16u  /* "YYYYMMDD_HHMMSS" + NUL */
#define IMU_META_BUILD_IDENTITY_CAPACITY   64u

typedef enum {
    IMU_META_COMPLETION_RUNNING    = 0,
    IMU_META_COMPLETION_COMPLETE   = 1,
    IMU_META_COMPLETION_INCOMPLETE = 2,
    IMU_META_COMPLETION_NO_CSV     = 3
} ImuMetaCompletion_t;

/*
 * R13 companion metadata, in-memory form. Plan (R5/R6) and per-stage R7
 * results are supplied separately to the JSON builder from imu_cmd.h; they
 * are not embedded here so this header stays independent of imu_cmd.h.
 * pinned_git_sha_expected is always JSON null and has no field.
 */
typedef struct {
    uint32_t version;                 /* IMU_RUN_METADATA_CONTRACT_VERSION */
    uint32_t metaSchemaVersion;       /* IMU_META_SCHEMA_VERSION */
    uint32_t csvSchemaVersion;        /* IMU_CSV_SCHEMA_VERSION */
    ImuMetaCompletion_t completion;
    uint64_t tLateNs;
    uint64_t servicePeriodNs;
    uint64_t publicationPeriodNs;
    uint64_t settleDurationNs;
    uint8_t  buildIdentityPresent;
    uint8_t  terminationPresent;
    uint8_t  reserved[2];             /* must be zero */
    char     baseName[IMU_META_BASE_NAME_CAPACITY];
    char     filenameUtc[IMU_META_UTC_STAMP_CAPACITY];
    char     buildIdentity[IMU_META_BUILD_IDENTITY_CAPACITY];
    ImuRunStats_t    stats;
    ImuLoggerStats_t logger;
    ImuTermination_t termination;
} ImuRunMetadata_t;

/* CSV schema 1 header tokens, Phase 1H section 6, in frozen order. */
static const char *const IMU_CSV_HEADER_TOKENS[IMU_CSV_COLUMN_COUNT] = {
    "schema_ver", "pub_seq", "sched_ns", "actual_ns", "epoch", "deadline",
    "gates_skipped", "sample_ready", "not_ready", "valid_mask", "fresh_mask",
    "stale_mask", "missing_mask", "multi_mask", "reader_state", "reader_flags",
    "logger_drops",
    "rv_seq", "rv_host_ns", "rv_sensor_us", "rv_report_seq", "rv_status",
    "rv_age_valid", "rv_age_ns", "rv_qw", "rv_qx", "rv_qy", "rv_qz",
    "rv_yaw_rad", "rv_pitch_rad", "rv_roll_rad", "rv_err_rad",
    "acc_seq", "acc_host_ns", "acc_sensor_us", "acc_report_seq", "acc_status",
    "acc_age_valid", "acc_age_ns", "acc_x", "acc_y", "acc_z",
    "gyr_seq", "gyr_host_ns", "gyr_sensor_us", "gyr_report_seq", "gyr_status",
    "gyr_age_valid", "gyr_age_ns", "gyr_x", "gyr_y", "gyr_z"
};

/*
 * Capture the pre-window comparison baseline. No clock is read here.
 * The baseline may contain missing groups; their first later valid decode is
 * fresh, but is not called a multiple update without a comparable identity.
 */
bool imu_publish_begin_window(ImuPublisherState_t *state,
                              const ImuSampleSnapshot_t *baseline);

/*
 * Build one R9 record without allocation, I/O, or clock access. scheduledNs
 * and actualNs are caller-provided CLOCK_MONOTONIC readings. Deadline and
 * skipped-gate policy are added by Phase 9.3; this step leaves them zero.
 */
bool imu_publish_evaluate_snapshot(ImuPublisherState_t *state,
                                   const ImuSampleSnapshot_t *snapshot,
                                   uint64_t scheduledNs,
                                   bool scheduledValid,
                                   uint64_t actualNs,
                                   bool actualValid,
                                   ImuPublicationRecord_t *out);

/* ---- Phase 9.3 schedule and skipped-gate accounting ---- */

#define IMU_PUBLISH_SERVICE_PERIOD_NS  1000000ull   /* T_service, plan R6 */
#define IMU_PUBLISH_GATE_TICKS         10u          /* tenth-tick gate */
#define IMU_PUBLISH_T_LATE_NS          1000000ull   /* Phase 10 may replace */

typedef enum {
    IMU_PUBLISH_GATE_IDLE     = 0,   /* no new gate boundary reached */
    IMU_PUBLISH_GATE_DUE      = 1,   /* evaluate and emit one row now */
    IMU_PUBLISH_GATE_COMPLETE = 2,   /* every gate in the window is done */
    IMU_PUBLISH_GATE_ERROR    = 3
} ImuPublishGateStatus_t;

typedef struct {
    uint64_t gateIndex;              /* 1-based gate being emitted */
    uint64_t scheduledNs;            /* meaningful only if scheduledValid */
    uint32_t gatesSkippedBefore;
    uint8_t  scheduledValid;
    uint8_t  reserved[3];            /* must be zero */
} ImuPublishGateDecision_t;

/*
 * Pure grid counter. tick k = 1 is the first service tick after baseline.
 * Gate g is due at tick k = 10 * g and is scheduled at the intended start
 * of that tick: tick1Ns + (10 * g - 1) * T_service. Gates are never reset
 * by reader state; the grid keeps running through recovery.
 */
typedef struct {
    uint64_t tick1Ns;
    uint64_t tickIndex;
    uint64_t lastGate;               /* newest gate emitted or skipped */
    uint64_t maxGate;                /* last gate that starts inside window */
    uint64_t gatesSkippedTotal;      /* R10 gates skipped so far */
    uint8_t  tick1Valid;
    uint8_t  active;
    uint8_t  reserved[6];
} ImuPublishGate_t;

/* windowNs is the acquisition duration. A gate belongs to the window only
 * if its scheduled time is strictly before tick1Ns + windowNs. */
bool imu_publish_gate_begin(ImuPublishGate_t *gate, uint64_t tick1Ns,
                            bool tick1Valid, uint64_t windowNs);

/* ticks is 1 plus the service periods RT_SleepUntil reported skipped since
 * the previous turn. At most one row is due per call; skipped boundaries
 * are counted, never backfilled. */
ImuPublishGateStatus_t imu_publish_gate_advance(
    ImuPublishGate_t *gate, uint64_t ticks, ImuPublishGateDecision_t *out);

bool imu_publish_gate_window_done(const ImuPublishGate_t *gate);

/* Finish R9 timing facts after evaluate: gatesSkippedBefore and the
 * deadline flag (skipped gate, invalid clock, or lateness above T_late). */
bool imu_publish_apply_timing(ImuPublicationRecord_t *record,
                              uint32_t gatesSkippedBefore);

#endif /* IMU_PUBLISH_H */
