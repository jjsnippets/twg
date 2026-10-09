#ifndef IMU_META_H
#define IMU_META_H

/*
 * Companion metadata and filename generation. Pure and bounded:
 * no I/O, no clock, no allocation, no globals. The caller reads the wall
 * clock once, converts it to broken-down system local time (localtime_r
 * after tzset), and passes the fields in. The fields are local wall-clock
 * values; no offset or zone is recorded. The ImuMetaUtc_t type and the
 * *_utc_* names are historical and do not imply UTC. File creation,
 * collision handling, fsync, and rename belong to the logger and main.
 *
 * Filename: bno_acq_YYYYMMDD_HHMMSS in local time (meta_schema_ver 3).
 * The CSV is <base>.csv and the companion is <base>.json; a temporary
 * companion name is <base>.json.tmp via imu_meta_file_name().
 *
 * JSON: one compact UTF-8 object plus a single trailing '\n'. Integers are
 * plain decimal; enums are lowercase strings except reader state, which is
 * numeric like the CSV. The local stamp is emitted as "filename_local".
 * Unavailable values are JSON null. R5/R6 plan and R7 stage results come
 * from imu_cmd.h and are passed in separately.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app/imu_cmd.h"
#include "app/imu_publish.h"

#define IMU_META_JSON_CAPACITY  16384u
#define IMU_META_FILE_NAME_CAPACITY  96u

/* Broken-down calendar fields in system local time (name is historical). */
typedef struct {
    uint16_t year;   /* 1970..9999 */
    uint8_t  month;    /* 1..12 */
    uint8_t  day;      /* 1..31, checked against the month */
    uint8_t  hour;     /* 0..23 */
    uint8_t  minute;   /* 0..59 */
    uint8_t  second;   /* 0..59 */
    uint8_t  reserved[1];
} ImuMetaUtc_t;

/* "YYYYMMDD_HHMMSS" (local time) into out[IMU_META_UTC_STAMP_CAPACITY]. */
bool imu_meta_utc_stamp(const ImuMetaUtc_t *utc, char *out, size_t capacity);

/* "bno_acq_" + stamp. The stamp is validated as 8 digits, '_', 6 digits. */
bool imu_meta_base_name(const char *stamp, char *out, size_t capacity);

/* base + "." + ext. ext is [a-z0-9.]+ without a leading dot. */
bool imu_meta_file_name(const char *base, const char *ext,
                        char *out, size_t capacity);

/* Fill metadata->filenameUtc (the local stamp, serialized as
 * "filename_local") and metadata->baseName from the broken-down fields. */
bool imu_meta_fill_names(ImuRunMetadata_t *metadata, const ImuMetaUtc_t *utc);

/*
 * results[i] is the R7 result for ImuCmdIdentity_t i, or NULL if that stage
 * has no result. results may itself be NULL (no stage results). Returns the
 * length excluding the NUL, or 0 on failure with out[0] = '\0'.
 */
size_t imu_meta_format_json(const ImuRunMetadata_t *metadata,
                            const ImuCmdPlan_t *plan,
                            const ImuCmdResult_t *const results[IMU_CMD_ID_COUNT],
                            char *out, size_t capacity);

#endif /* IMU_META_H */
