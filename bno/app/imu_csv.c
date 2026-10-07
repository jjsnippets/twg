#include "app/imu_csv.h"

#include <inttypes.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

typedef struct {
    char    *buf;
    size_t   capacity;
    size_t   used;
    unsigned fields;
    int      ok;
} CsvWriter_t;

static void csv_append(CsvWriter_t *w, const char *fmt, ...)
{
    va_list ap;
    int n;

    if (!w->ok) {
        return;
    }
    va_start(ap, fmt);
    n = vsnprintf(w->buf + w->used, w->capacity - w->used, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= w->capacity - w->used) {
        w->ok = 0;
        return;
    }
    w->used += (size_t)n;
}

static void csv_sep(CsvWriter_t *w)
{
    if (w->fields++ != 0u) {
        csv_append(w, ",");
    }
}

static void csv_u64(CsvWriter_t *w, uint64_t v)
{
    csv_sep(w);
    csv_append(w, "%" PRIu64, v);
}

static void csv_missing(CsvWriter_t *w)
{
    csv_sep(w);
    csv_append(w, "%s", IMU_CSV_MISSING_TOKEN);
}

static void csv_u64_if(CsvWriter_t *w, int present, uint64_t v)
{
    if (present) {
        csv_u64(w, v);
    } else {
        csv_missing(w);
    }
}

static void csv_float_if(CsvWriter_t *w, int present, float v)
{
    if (!present || !isfinite(v)) {
        csv_missing(w);
        return;
    }
    csv_sep(w);
    csv_append(w, "%.9g", (double)v);
}

static void csv_group_identity(CsvWriter_t *w, int present,
                               const ImuPublicationGroup_t *g)
{
    int age = present && g->ageValid != 0u;

    csv_u64_if(w, present, g->groupEventSeq);
    csv_u64_if(w, present, g->hostDecodeNs);
    csv_u64_if(w, present, g->sensorTimeUs);
    csv_u64_if(w, present, g->deviceReportSeq);
    csv_u64_if(w, present, g->rawStatus);
    csv_u64(w, age ? 1u : 0u);
    csv_u64_if(w, age, g->ageNs);
}

static int group_present(const ImuPublicationRecord_t *r, unsigned bit,
                         const ImuPublicationGroup_t *g)
{
    return (r->validMask & bit) != 0u && g->identityPresent != 0u;
}

static size_t csv_finish(CsvWriter_t *w)
{
    csv_append(w, "\n");
    if (!w->ok) {
        if (w->capacity != 0u) {
            w->buf[0] = '\0';
        }
        return 0u;
    }
    return w->used;
}

static int csv_begin(CsvWriter_t *w, char *out, size_t capacity)
{
    w->buf = out;
    w->capacity = capacity;
    w->used = 0u;
    w->fields = 0u;
    w->ok = out != NULL && capacity != 0u;
    if (w->ok) {
        out[0] = '\0';
    }
    return w->ok;
}

size_t imu_csv_format_header(char *out, size_t capacity)
{
    CsvWriter_t w;
    unsigned i;

    if (!csv_begin(&w, out, capacity)) {
        return 0u;
    }
    for (i = 0u; i < IMU_CSV_COLUMN_COUNT; ++i) {
        csv_sep(&w);
        csv_append(&w, "%s", IMU_CSV_HEADER_TOKENS[i]);
    }
    return csv_finish(&w);
}

size_t imu_csv_format_row(const ImuPublicationRecord_t *r,
                          char *out, size_t capacity)
{
    CsvWriter_t w;
    int rv;
    int acc;
    int gyr;

    if (r == NULL || !csv_begin(&w, out, capacity) ||
        r->version != IMU_PUBLICATION_CONTRACT_VERSION) {
        if (out != NULL && capacity != 0u) {
            out[0] = '\0';
        }
        return 0u;
    }

    rv  = group_present(r, IMU_GROUP_BIT_ROTATION, &r->rotation);
    acc = group_present(r, IMU_GROUP_BIT_ACCEL, &r->accel);
    gyr = group_present(r, IMU_GROUP_BIT_GYRO, &r->gyro);

    csv_u64(&w, IMU_CSV_SCHEMA_VERSION);
    csv_u64(&w, r->publicationSeq);
    csv_u64_if(&w, r->scheduledValid != 0u, r->scheduledNs);
    csv_u64_if(&w, r->actualValid != 0u, r->actualNs);
    csv_u64(&w, r->configurationEpoch);
    csv_u64(&w, r->deadlineMissed != 0u ? 1u : 0u);
    csv_u64(&w, r->gatesSkippedBefore);
    csv_u64(&w, r->sampleReady != 0u ? 1u : 0u);
    csv_u64(&w, r->notReady != 0u ? 1u : 0u);
    csv_u64(&w, r->validMask);
    csv_u64(&w, r->freshMask);
    csv_u64(&w, r->staleMask);
    csv_u64(&w, r->missingMask);
    csv_u64(&w, r->multiUpdateMask);
    csv_u64(&w, (uint64_t)r->readerState);
    csv_u64(&w, r->readerFlags);
    csv_u64(&w, r->loggerDrops);

    csv_group_identity(&w, rv, &r->rotation);
    csv_float_if(&w, rv, r->qw);
    csv_float_if(&w, rv, r->qx);
    csv_float_if(&w, rv, r->qy);
    csv_float_if(&w, rv, r->qz);
    csv_float_if(&w, rv, r->yawRad);
    csv_float_if(&w, rv, r->pitchRad);
    csv_float_if(&w, rv, r->rollRad);
    csv_float_if(&w, rv, r->orientationErrRad);

    csv_group_identity(&w, acc, &r->accel);
    csv_float_if(&w, acc, r->ax);
    csv_float_if(&w, acc, r->ay);
    csv_float_if(&w, acc, r->az);

    csv_group_identity(&w, gyr, &r->gyro);
    csv_float_if(&w, gyr, r->gx);
    csv_float_if(&w, gyr, r->gy);
    csv_float_if(&w, gyr, r->gz);

    if (w.fields != IMU_CSV_COLUMN_COUNT) {
        w.ok = 0;
    }
    return csv_finish(&w);
}
