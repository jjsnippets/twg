#define _POSIX_C_SOURCE 200809L

#include "app/imu_logfile.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static bool stamp_valid(const char *stamp)
{
    unsigned i;

    if (stamp == NULL || strlen(stamp) != 15u) {
        return false;
    }
    for (i = 0u; i < 15u; ++i) {
        if (i == 8u) {
            if (stamp[i] != '_') {
                return false;
            }
        } else if (stamp[i] < '0' || stamp[i] > '9') {
            return false;
        }
    }
    return true;
}

static bool path_exists(const char *path)
{
    struct stat st;

    if (lstat(path, &st) == 0) {
        return true;
    }
    return errno != ENOENT;
}

static bool join_path(char *out, const char *dir, const char *base,
                      const char *ext)
{
    int n;

    if (dir[0] != '\0') {
        n = snprintf(out, IMU_LOGFILE_PATH_CAPACITY, "%s/%s.%s", dir, base, ext);
    } else {
        n = snprintf(out, IMU_LOGFILE_PATH_CAPACITY, "%s.%s", base, ext);
    }
    return n > 0 && (size_t)n < IMU_LOGFILE_PATH_CAPACITY;
}

static bool build_paths(ImuLogfile_t *lf, const char *dir)
{
    return join_path(lf->csvTmp, dir, lf->base, "csv.tmp") &&
           join_path(lf->csvPath, dir, lf->base, "csv") &&
           join_path(lf->csvPartial, dir, lf->base, "csv.partial") &&
           join_path(lf->jsonTmp, dir, lf->base, "json.tmp") &&
           join_path(lf->jsonPath, dir, lf->base, "json");
}

static bool write_all(int fd, const char *data, size_t length)
{
    while (length > 0u) {
        ssize_t w = write(fd, data, length);

        if (w < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        data += w;
        length -= (size_t)w;
    }
    return true;
}

ImuLogfileResult_t imu_logfile_open(ImuLogfile_t *lf, const char *dir,
                                    const char *stamp, bool wantCsv)
{
    char dirCopy[IMU_LOGFILE_DIR_CAPACITY];
    uint32_t suffix;

    if (lf == NULL || stamp == NULL) {
        return IMU_LOGFILE_ERR_ARG;
    }
    memset(lf, 0, sizeof(*lf));
    lf->csvFd = -1;

    if (!stamp_valid(stamp)) {
        return IMU_LOGFILE_ERR_NAME;
    }
    dirCopy[0] = '\0';
    if (dir != NULL && dir[0] != '\0') {
        if (strlen(dir) >= sizeof(dirCopy)) {
            return IMU_LOGFILE_ERR_ARG;
        }
        strcpy(dirCopy, dir);
    }

    for (suffix = 0u; suffix <= IMU_LOGFILE_MAX_SUFFIX; ++suffix) {
        int jfd;
        int n;

        if (suffix == 0u) {
            n = snprintf(lf->base, sizeof(lf->base), "bno_acq_%s", stamp);
        } else {
            n = snprintf(lf->base, sizeof(lf->base), "bno_acq_%s_%u", stamp,
                         (unsigned)suffix);
        }
        if (n <= 0 || (size_t)n >= sizeof(lf->base) ||
            !build_paths(lf, dirCopy)) {
            return IMU_LOGFILE_ERR_NAME;
        }

        if (path_exists(lf->csvTmp) || path_exists(lf->csvPath) ||
            path_exists(lf->csvPartial) || path_exists(lf->jsonTmp) ||
            path_exists(lf->jsonPath)) {
            continue;
        }

        jfd = open(lf->jsonTmp, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
        if (jfd < 0) {
            if (errno == EEXIST) {
                continue;
            }
            lf->lastErrno = errno;
            return IMU_LOGFILE_ERR_OPEN;
        }
        close(jfd);

        if (wantCsv) {
            int cfd = open(lf->csvTmp,
                           O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);

            if (cfd < 0) {
                int e = errno;

                unlink(lf->jsonTmp);
                if (e == EEXIST) {
                    continue;
                }
                lf->lastErrno = e;
                return IMU_LOGFILE_ERR_OPEN;
            }
            lf->csvFd = cfd;
            lf->csvOpen = 1u;
        }
        lf->wantCsv = wantCsv ? 1u : 0u;
        lf->suffix = suffix;
        return IMU_LOGFILE_OK;
    }

    return IMU_LOGFILE_ERR_EXHAUSTED;
}

const char *imu_logfile_base_name(const ImuLogfile_t *lf)
{
    return lf != NULL ? lf->base : NULL;
}

bool imu_logfile_sink(ImuLogfile_t *lf, ImuLoggerSink_t *sink)
{
    if (lf == NULL || sink == NULL || lf->csvOpen == 0u || lf->csvFd < 0) {
        return false;
    }
    return imu_logger_sink_fd(sink, &lf->csvFd);
}

static ImuLogfileResult_t write_json_tmp(ImuLogfile_t *lf, const char *json,
                                         size_t length)
{
    int fd = open(lf->jsonTmp, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);

    if (fd < 0) {
        lf->lastErrno = errno;
        return IMU_LOGFILE_ERR_OPEN;
    }
    if (!write_all(fd, json, length) || fsync(fd) != 0) {
        lf->lastErrno = errno;
        close(fd);
        return IMU_LOGFILE_ERR_WRITE;
    }
    if (close(fd) != 0) {
        lf->lastErrno = errno;
        return IMU_LOGFILE_ERR_WRITE;
    }
    return IMU_LOGFILE_OK;
}

static ImuLogfileResult_t publish_json(ImuLogfile_t *lf, const char *json,
                                       size_t length)
{
    ImuLogfileResult_t r;

    if (json == NULL || length == 0u) {
        return IMU_LOGFILE_ERR_ARG;
    }
    r = write_json_tmp(lf, json, length);
    if (r != IMU_LOGFILE_OK) {
        return r;
    }

    if (lf->jsonPublished == 0u) {
        if (link(lf->jsonTmp, lf->jsonPath) != 0) {
            lf->lastErrno = errno;
            return errno == EEXIST ? IMU_LOGFILE_ERR_EXISTS
                                   : IMU_LOGFILE_ERR_RENAME;
        }
        unlink(lf->jsonTmp);
        lf->jsonPublished = 1u;
    } else if (rename(lf->jsonTmp, lf->jsonPath) != 0) {
        lf->lastErrno = errno;
        return IMU_LOGFILE_ERR_RENAME;
    }
    return IMU_LOGFILE_OK;
}

ImuLogfileResult_t imu_logfile_write_running(ImuLogfile_t *lf,
                                             const char *json, size_t length)
{
    if (lf == NULL) {
        return IMU_LOGFILE_ERR_ARG;
    }
    if (lf->jsonPublished != 0u || lf->jsonCommitted != 0u) {
        return IMU_LOGFILE_ERR_STATE;
    }
    return publish_json(lf, json, length);
}

ImuLogfileResult_t imu_logfile_commit_json(ImuLogfile_t *lf, const char *json,
                                           size_t length)
{
    ImuLogfileResult_t r;

    if (lf == NULL) {
        return IMU_LOGFILE_ERR_ARG;
    }
    if (lf->jsonCommitted != 0u) {
        return IMU_LOGFILE_ERR_STATE;
    }
    r = publish_json(lf, json, length);
    if (r == IMU_LOGFILE_OK) {
        lf->jsonCommitted = 1u;
    }
    return r;
}

ImuLogfileResult_t imu_logfile_close_csv(ImuLogfile_t *lf, bool complete,
                                         bool workerJoined)
{
    ImuLogfileResult_t result = IMU_LOGFILE_OK;
    bool effectiveComplete;
    const char *target;

    if (lf == NULL) {
        return IMU_LOGFILE_ERR_ARG;
    }
    if (lf->wantCsv == 0u) {
        return IMU_LOGFILE_OK;
    }
    if (lf->csvFinalized != 0u || lf->csvOpen == 0u) {
        return IMU_LOGFILE_ERR_STATE;
    }

    effectiveComplete = complete && workerJoined;

    if (workerJoined) {
        if (close(lf->csvFd) != 0 && errno != EINTR) {
            lf->lastErrno = errno;
            effectiveComplete = false;
            result = IMU_LOGFILE_ERR_WRITE;
        }
        lf->csvFd = -1;
        lf->csvOpen = 0u;
    }

    target = effectiveComplete ? lf->csvPath : lf->csvPartial;
    if (link(lf->csvTmp, target) != 0) {
        lf->lastErrno = errno;
        return errno == EEXIST ? IMU_LOGFILE_ERR_EXISTS
                               : IMU_LOGFILE_ERR_RENAME;
    }
    unlink(lf->csvTmp);

    lf->csvFinalized = 1u;
    lf->csvOutcome = effectiveComplete ? IMU_LOGFILE_CSV_COMPLETE
                                       : IMU_LOGFILE_CSV_PARTIAL;
    return result;
}

ImuLogfileCsvOutcome_t imu_logfile_csv_outcome(const ImuLogfile_t *lf)
{
    return lf != NULL ? (ImuLogfileCsvOutcome_t)lf->csvOutcome
                      : IMU_LOGFILE_CSV_NONE;
}

int imu_logfile_last_errno(const ImuLogfile_t *lf)
{
    return lf != NULL ? lf->lastErrno : 0;
}
