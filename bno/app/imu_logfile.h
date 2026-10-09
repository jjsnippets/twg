#ifndef IMU_LOGFILE_H
#define IMU_LOGFILE_H

/*
 * CSV/companion file lifecycle. No clock, no threads, no globals.
 * Called from startup/shutdown only, never from the 1 kHz path.
 *
 * Files for base bno_acq_YYYYMMDD_HHMMSS[_N]:
 *   .csv.tmp   -> .csv | .csv.partial   (link+unlink, never overwrites)
 *   .json.tmp  -> .json                 (first publish never overwrites;
 *                                        running -> final replaces our own)
 * A CSV plus JSON pair is not an atomic transaction. The JSON status carries
 * completeness. Partial artifacts are preserved, never deleted.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app/imu_logger.h"

#define IMU_LOGFILE_PATH_CAPACITY 320u
#define IMU_LOGFILE_DIR_CAPACITY  192u
#define IMU_LOGFILE_BASE_CAPACITY 48u
#define IMU_LOGFILE_MAX_SUFFIX    999u

typedef enum {
    IMU_LOGFILE_OK = 0,
    IMU_LOGFILE_ERR_ARG,
    IMU_LOGFILE_ERR_NAME,
    IMU_LOGFILE_ERR_EXHAUSTED,
    IMU_LOGFILE_ERR_OPEN,
    IMU_LOGFILE_ERR_WRITE,
    IMU_LOGFILE_ERR_EXISTS,
    IMU_LOGFILE_ERR_RENAME,
    IMU_LOGFILE_ERR_STATE
} ImuLogfileResult_t;

typedef enum {
    IMU_LOGFILE_CSV_NONE = 0,
    IMU_LOGFILE_CSV_COMPLETE,
    IMU_LOGFILE_CSV_PARTIAL
} ImuLogfileCsvOutcome_t;

typedef struct {
    int      csvFd;
    int      lastErrno;
    uint32_t suffix;
    uint8_t  wantCsv;
    uint8_t  csvOpen;
    uint8_t  csvFinalized;
    uint8_t  jsonPublished;
    uint8_t  jsonCommitted;
    uint8_t  csvOutcome;
    uint8_t  reserved[2];
    char     base[IMU_LOGFILE_BASE_CAPACITY];
    char     csvTmp[IMU_LOGFILE_PATH_CAPACITY];
    char     csvPath[IMU_LOGFILE_PATH_CAPACITY];
    char     csvPartial[IMU_LOGFILE_PATH_CAPACITY];
    char     jsonTmp[IMU_LOGFILE_PATH_CAPACITY];
    char     jsonPath[IMU_LOGFILE_PATH_CAPACITY];
} ImuLogfile_t;

/*
 * Reserve a collision-free base in dir (NULL or "" = process cwd). stamp is
 * "YYYYMMDD_HHMMSS". Creates .json.tmp, and .csv.tmp when wantCsv, with
 * O_EXCL. Failure leaves no file behind. Call before the acquisition window.
 */
ImuLogfileResult_t imu_logfile_open(ImuLogfile_t *lf, const char *dir,
                                    const char *stamp, bool wantCsv);

/* Final base name including any collision suffix, for R13 baseName. */
const char *imu_logfile_base_name(const ImuLogfile_t *lf);

/* Sink for imu_logger_start(); fails if no CSV was requested or it is closed. */
bool imu_logfile_sink(ImuLogfile_t *lf, ImuLoggerSink_t *sink);

/* Publish the status-running companion. Never overwrites an existing name. */
ImuLogfileResult_t imu_logfile_write_running(ImuLogfile_t *lf,
                                             const char *json, size_t length);

/*
 * Finish the CSV after imu_logger_drain(). workerJoined=false (drain timeout)
 * keeps the fd open and forces .csv.partial. complete=false also forces
 * .csv.partial. No-op success when no CSV was requested.
 */
ImuLogfileResult_t imu_logfile_close_csv(ImuLogfile_t *lf, bool complete,
                                         bool workerJoined);

/* Publish the final companion (replaces the running one). Once only. */
ImuLogfileResult_t imu_logfile_commit_json(ImuLogfile_t *lf,
                                           const char *json, size_t length);

ImuLogfileCsvOutcome_t imu_logfile_csv_outcome(const ImuLogfile_t *lf);
int imu_logfile_last_errno(const ImuLogfile_t *lf);

#endif /* IMU_LOGFILE_H */
