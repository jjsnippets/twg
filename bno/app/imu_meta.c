#include "app/imu_meta.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ---- filenames ---- */

static bool is_digit(char c)
{
    return c >= '0' && c <= '9';
}

static bool leap_year(unsigned y)
{
    return (y % 4u == 0u && y % 100u != 0u) || y % 400u == 0u;
}

static unsigned days_in_month(unsigned y, unsigned m)
{
    static const uint8_t days[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

    return (m == 2u && leap_year(y)) ? 29u : days[m - 1u];
}

bool imu_meta_utc_stamp(const ImuMetaUtc_t *utc, char *out, size_t capacity)
{
    int n;

    if (out != NULL && capacity != 0u) {
        out[0] = '\0';
    }
    if (utc == NULL || out == NULL ||
        capacity < IMU_META_UTC_STAMP_CAPACITY ||
        utc->year < 1970u || utc->year > 9999u ||
        utc->month < 1u || utc->month > 12u ||
        utc->day < 1u || utc->day > days_in_month(utc->year, utc->month) ||
        utc->hour > 23u || utc->minute > 59u || utc->second > 59u) {
        return false;
    }
    n = snprintf(out, capacity, "%04u%02u%02u_%02u%02u%02u",
                 (unsigned)utc->year, (unsigned)utc->month,
                 (unsigned)utc->day, (unsigned)utc->hour,
                 (unsigned)utc->minute, (unsigned)utc->second);
    if (n != 15) {
        out[0] = '\0';
        return false;
    }
    return true;
}

static bool stamp_valid(const char *s)
{
    unsigned i;

    if (s == NULL || strlen(s) != 15u || s[8] != '_') {
        return false;
    }
    for (i = 0u; i < 15u; ++i) {
        if (i != 8u && !is_digit(s[i])) {
            return false;
        }
    }
    return true;
}

bool imu_meta_base_name(const char *stamp, char *out, size_t capacity)
{
    int n;

    if (out != NULL && capacity != 0u) {
        out[0] = '\0';
    }
    if (out == NULL || !stamp_valid(stamp)) {
        return false;
    }
    n = snprintf(out, capacity, "bno_acq_%s", stamp);
    if (n < 0 || (size_t)n >= capacity) {
        out[0] = '\0';
        return false;
    }
    return true;
}

bool imu_meta_file_name(const char *base, const char *ext,
                        char *out, size_t capacity)
{
    const char *p;
    int n;

    if (out != NULL && capacity != 0u) {
        out[0] = '\0';
    }
    if (out == NULL || base == NULL || ext == NULL || base[0] == '\0' ||
        ext[0] == '\0' || ext[0] == '.') {
        return false;
    }
    for (p = ext; *p != '\0'; ++p) {
        if (!((*p >= 'a' && *p <= 'z') || is_digit(*p) || *p == '.')) {
            return false;
        }
    }
    n = snprintf(out, capacity, "%s.%s", base, ext);
    if (n < 0 || (size_t)n >= capacity) {
        out[0] = '\0';
        return false;
    }
    return true;
}

bool imu_meta_fill_names(ImuRunMetadata_t *metadata, const ImuMetaUtc_t *utc)
{
    char stamp[IMU_META_UTC_STAMP_CAPACITY];
    char base[IMU_META_BASE_NAME_CAPACITY];

    if (metadata == NULL ||
        !imu_meta_utc_stamp(utc, stamp, sizeof(stamp)) ||
        !imu_meta_base_name(stamp, base, sizeof(base))) {
        return false;
    }
    memcpy(metadata->filenameUtc, stamp, sizeof(stamp));
    memcpy(metadata->baseName, base, sizeof(base));
    return true;
}

/* ---- enum names ---- */

#define COUNT_OF(a) (sizeof(a) / sizeof((a)[0]))

static const char *name_of(const char *const *table, size_t count,
                           unsigned long index)
{
    return index < count ? table[index] : "unknown";
}

static const char *const k_identity[] = {
    "none", "calibration", "dcd_clear", "tare", "tare_clear", "tare_check",
    "check", "probe", "settle", "acquisition"
};
static const char *const k_state[] = {
    "not_requested", "running", "succeeded", "failed", "cancelled",
    "timed_out", "recovery_failed", "abandoned"
};
static const char *const k_reason[] = {
    "none", "ok", "operator_q", "operator_confirm_timeout",
    "gate_not_reached", "dcd_save_failed", "dcd_clear_failed",
    "tare_now_failed", "tare_persist_failed", "tare_clear_partial",
    "tare_clear_failed", "tare_verify_failed", "cal_config_failed",
    "cal_mag_exhausted", "cal_hold_degraded", "cal_save_failed",
    "cal_reopen_failed", "cal_verify_config_failed",
    "cal_verify_gate_failed", "cal_restore_failed", "cal_session_unusable",
    "config_failed", "probe_deadline", "session_unusable", "process_stop",
    "illegal_plan"
};
static const char *const k_axes[] = { "none", "z", "full" };
static const char *const k_sub[] = { "not_attempted", "succeeded", "failed" };
static const char *const k_completion[] = {
    "running", "complete", "incomplete", "no_csv"
};
static const char *const k_logger_start[] = {
    "not_attempted", "ok", "failed"
};
static const char *const k_term[] = {
    "none", "completed", "process_stop", "no_acquisition",
    "session_unrecovered", "logger_failure", "startup_failure",
    "owner_failure"
};

/* ---- JSON writer ---- */

typedef struct {
    char   *buf;
    size_t  capacity;
    size_t  used;
    int     ok;
    int     comma;
} Jw_t;

static void jw_raw(Jw_t *w, const char *fmt, ...)
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

static void jw_string(Jw_t *w, const char *s, size_t maxLen)
{
    size_t i;

    if (memchr(s, '\0', maxLen) == NULL) {
        w->ok = 0;
        return;
    }
    jw_raw(w, "\"");
    for (i = 0u; s[i] != '\0'; ++i) {
        unsigned char c = (unsigned char)s[i];

        if (c == '"') {
            jw_raw(w, "\\\"");
        } else if (c == '\\') {
            jw_raw(w, "\\\\");
        } else if (c < 0x20u) {
            jw_raw(w, "\\u%04x", (unsigned)c);
        } else {
            jw_raw(w, "%c", (int)c);
        }
    }
    jw_raw(w, "\"");
}

static void jw_key(Jw_t *w, const char *key)
{
    if (w->comma) {
        jw_raw(w, ",");
    }
    if (key != NULL) {
        jw_raw(w, "\"%s\":", key);
    }
    w->comma = 0;
}

static void jw_begin(Jw_t *w, const char *key, char open)
{
    jw_key(w, key);
    jw_raw(w, "%c", open);
    w->comma = 0;
}

static void jw_end(Jw_t *w, char close)
{
    jw_raw(w, "%c", close);
    w->comma = 1;
}

static void jw_u64(Jw_t *w, const char *key, uint64_t v)
{
    jw_key(w, key);
    jw_raw(w, "%" PRIu64, v);
    w->comma = 1;
}

static void jw_i64(Jw_t *w, const char *key, int64_t v)
{
    jw_key(w, key);
    jw_raw(w, "%" PRId64, v);
    w->comma = 1;
}

static void jw_bool(Jw_t *w, const char *key, int v)
{
    jw_key(w, key);
    jw_raw(w, v ? "true" : "false");
    w->comma = 1;
}

static void jw_null(Jw_t *w, const char *key)
{
    jw_key(w, key);
    jw_raw(w, "null");
    w->comma = 1;
}

static void jw_str(Jw_t *w, const char *key, const char *s)
{
    jw_key(w, key);
    jw_string(w, s, strlen(s) + 1u);
    w->comma = 1;
}

static void jw_cstr(Jw_t *w, const char *key, const char *s, size_t capacity)
{
    jw_key(w, key);
    jw_string(w, s, capacity);
    w->comma = 1;
}

static void jw_u64_array(Jw_t *w, const char *key, const uint64_t *v,
                         size_t count)
{
    size_t i;

    jw_begin(w, key, '[');
    for (i = 0u; i < count; ++i) {
        jw_u64(w, NULL, v[i]);
    }
    jw_end(w, ']');
}

/* ---- sections ---- */

static void put_plan(Jw_t *w, const ImuCmdPlan_t *p)
{
    jw_begin(w, "plan", '{');
    jw_u64(w, "plan_version", p->version);
    jw_begin(w, "slots", '[');
    jw_str(w, NULL, name_of(k_identity, COUNT_OF(k_identity), p->slot1));
    jw_str(w, NULL, name_of(k_identity, COUNT_OF(k_identity), p->slot2));
    jw_str(w, NULL, name_of(k_identity, COUNT_OF(k_identity), p->slot3));
    jw_end(w, ']');
    jw_str(w, "tare_axes", name_of(k_axes, COUNT_OF(k_axes), p->tareAxes));
    jw_bool(w, "persist_tare", p->persistTare);
    if (p->probeMaskPresent) {
        jw_u64(w, "probe_mask", p->probeMask);
    } else {
        jw_null(w, "probe_mask");
    }
    jw_u64(w, "flight_cal_mask", p->flightCalMask);
    jw_u64(w, "probe_deadline_s", p->probeDeadlineS);
    jw_u64(w, "acquisition_duration_s", p->acquisitionDurationS);
    jw_u64(w, "settle_duration_ms", p->settleDurationMs);
    jw_u64(w, "service_period_ns", p->servicePeriodNs);
    jw_u64(w, "publication_period_ns", p->publicationPeriodNs);
    jw_bool(w, "confirm_dcd_clear", p->confirmDcdClear);
    jw_bool(w, "confirm_tare", p->confirmTare);
    jw_bool(w, "confirm_tare_clear", p->confirmTareClear);
    jw_end(w, '}');
}

static void put_stage(Jw_t *w, const ImuCmdResult_t *r)
{
    jw_begin(w, NULL, '{');
    jw_str(w, "identity", name_of(k_identity, COUNT_OF(k_identity), r->identity));
    jw_str(w, "state", name_of(k_state, COUNT_OF(k_state), r->state));
    jw_str(w, "reason", name_of(k_reason, COUNT_OF(k_reason), r->reason));
    jw_bool(w, "warning_required", r->warningRequired);
    jw_u64(w, "started_ns", r->startedNs);
    jw_u64(w, "ended_ns", r->endedNs);
    jw_u64(w, "epoch_before", r->epochBefore);
    jw_u64(w, "epoch_after", r->epochAfter);
    jw_str(w, "requested_tare_axes",
           name_of(k_axes, COUNT_OF(k_axes), r->requestedTareAxes));
    if (r->probeMaskRequestedValid) {
        jw_u64(w, "probe_mask_requested", r->probeMaskRequested);
    } else {
        jw_null(w, "probe_mask_requested");
    }
    jw_bool(w, "dcd_saved", r->dcdSaved);
    jw_bool(w, "verified", r->verified);
    jw_bool(w, "restored_production", r->restoredProduction);
    jw_begin(w, "sub", '{');
    jw_str(w, "tare_now", name_of(k_sub, COUNT_OF(k_sub), r->sub.tareNow));
    jw_str(w, "persist", name_of(k_sub, COUNT_OF(k_sub), r->sub.persist));
    jw_str(w, "clear_active",
           name_of(k_sub, COUNT_OF(k_sub), r->sub.clearActive));
    jw_str(w, "clear_saved",
           name_of(k_sub, COUNT_OF(k_sub), r->sub.clearSaved));
    jw_bool(w, "probe_reached_gate", r->sub.probeReachedGate);
    jw_bool(w, "probe_timed_out", r->sub.probeTimedOut);
    jw_bool(w, "probe_operator_ended_early", r->sub.probeOperatorEndedEarly);
    if (r->sub.probeMaskActualValid) {
        jw_u64(w, "probe_mask_actual", r->sub.probeMaskActual);
    } else {
        jw_null(w, "probe_mask_actual");
    }
    jw_end(w, '}');
    jw_end(w, '}');
}

static void put_stats(Jw_t *w, const ImuRunStats_t *s)
{
    jw_begin(w, "stats", '{');
    jw_u64(w, "service_ticks", s->serviceTicks);
    jw_u64(w, "service_overruns", s->serviceOverruns);
    jw_u64(w, "clock_read_failures", s->clockReadFailures);
    jw_u64(w, "publications_attempted", s->publicationsAttempted);
    jw_u64(w, "publications_enqueued", s->publicationsEnqueued);
    jw_u64(w, "publications_all_valid", s->publicationsAllValid);
    jw_u64(w, "publications_all_fresh", s->publicationsAllFresh);
    jw_u64(w, "publications_partially_fresh", s->publicationsPartiallyFresh);
    jw_u64(w, "publications_not_ready", s->publicationsNotReady);
    jw_u64(w, "deadline_misses", s->deadlineMisses);
    jw_u64(w, "gates_skipped", s->gatesSkipped);
    jw_u64_array(w, "stale_by_group", s->staleByGroup, IMU_PUBLISH_GROUP_COUNT);
    jw_u64_array(w, "multi_update_by_group", s->multiUpdateByGroup,
                 IMU_PUBLISH_GROUP_COUNT);
    jw_end(w, '}');
}

static void put_logger(Jw_t *w, const ImuLoggerStats_t *l)
{
    jw_begin(w, "logger", '{');
    jw_str(w, "start_result",
           name_of(k_logger_start, COUNT_OF(k_logger_start), l->startResult));
    jw_u64(w, "queue_capacity", l->queueCapacity);
    jw_u64(w, "queue_high_water", l->queueHighWater);
    jw_u64(w, "records_enqueued", l->recordsEnqueued);
    jw_u64(w, "records_written", l->recordsWritten);
    jw_u64(w, "records_dropped", l->recordsDropped);
    jw_u64(w, "shutdown_errors", l->shutdownErrors);
    jw_bool(w, "write_failed", l->writeFailed);
    jw_bool(w, "drain_started", l->drainStarted);
    jw_bool(w, "drain_complete", l->drainComplete);
    jw_bool(w, "drain_timed_out", l->drainTimedOut);
    jw_end(w, '}');
}

static void put_termination(Jw_t *w, const ImuTermination_t *t)
{
    jw_begin(w, "termination", '{');
    jw_str(w, "reason", name_of(k_term, COUNT_OF(k_term), t->reason));
    jw_i64(w, "signal_number", t->signalNumber);
    jw_i64(w, "exit_status", t->exitStatus);
    jw_u64(w, "final_reader_state", (uint64_t)t->finalReaderState);
    jw_bool(w, "session_unrecovered", t->sessionUnrecovered);
    jw_bool(w, "acquisition_window_closed", t->acquisitionWindowClosed);
    jw_end(w, '}');
}

static bool metadata_valid(const ImuRunMetadata_t *m, const ImuCmdPlan_t *p)
{
    return m != NULL && p != NULL &&
           m->version == IMU_RUN_METADATA_CONTRACT_VERSION &&
           m->metaSchemaVersion == IMU_META_SCHEMA_VERSION &&
           m->csvSchemaVersion == IMU_CSV_SCHEMA_VERSION &&
           (unsigned)m->completion <= (unsigned)IMU_META_COMPLETION_NO_CSV &&
           m->stats.version == IMU_RUN_STATS_CONTRACT_VERSION &&
           m->logger.version == IMU_LOGGER_STATS_CONTRACT_VERSION &&
           (m->terminationPresent == 0u ||
            m->termination.version == IMU_TERMINATION_CONTRACT_VERSION) &&
           p->version == IMU_CMD_PLAN_VERSION;
}

size_t imu_meta_format_json(const ImuRunMetadata_t *m,
                            const ImuCmdPlan_t *plan,
                            const ImuCmdResult_t *const results[IMU_CMD_ID_COUNT],
                            char *out, size_t capacity)
{
    Jw_t w;
    char file[IMU_META_FILE_NAME_CAPACITY];
    unsigned i;

    if (out != NULL && capacity != 0u) {
        out[0] = '\0';
    }
    if (out == NULL || capacity == 0u || !metadata_valid(m, plan) ||
        memchr(m->baseName, '\0', sizeof(m->baseName)) == NULL ||
        m->baseName[0] == '\0') {
        return 0u;
    }

    w.buf = out;
    w.capacity = capacity;
    w.used = 0u;
    w.ok = 1;
    w.comma = 0;

    jw_begin(&w, NULL, '{');
    jw_u64(&w, "meta_schema_ver", m->metaSchemaVersion);
    jw_u64(&w, "csv_schema_ver", m->csvSchemaVersion);
    jw_str(&w, "completion",
           name_of(k_completion, COUNT_OF(k_completion), m->completion));
    jw_cstr(&w, "base_name", m->baseName, sizeof(m->baseName));
    jw_cstr(&w, "filename_local", m->filenameUtc, sizeof(m->filenameUtc));

    if (m->completion != IMU_META_COMPLETION_NO_CSV &&
        imu_meta_file_name(m->baseName, "csv", file, sizeof(file))) {
        jw_str(&w, "csv_file", file);
    } else {
        jw_null(&w, "csv_file");
    }
    if (imu_meta_file_name(m->baseName, "json", file, sizeof(file))) {
        jw_str(&w, "meta_file", file);
    } else {
        w.ok = 0;
    }

    if (m->buildIdentityPresent) {
        jw_cstr(&w, "build_identity", m->buildIdentity,
                sizeof(m->buildIdentity));
    } else {
        jw_null(&w, "build_identity");
    }
    jw_null(&w, "pinned_git_sha_expected");

    jw_u64(&w, "t_late_ns", m->tLateNs);
    jw_u64(&w, "service_period_ns", m->servicePeriodNs);
    jw_u64(&w, "publication_period_ns", m->publicationPeriodNs);
    jw_u64(&w, "settle_duration_ns", m->settleDurationNs);

    put_plan(&w, plan);

    jw_begin(&w, "stages", '[');
    for (i = 0u; results != NULL && i < (unsigned)IMU_CMD_ID_COUNT; ++i) {
        if (results[i] != NULL) {
            put_stage(&w, results[i]);
        }
    }
    jw_end(&w, ']');

    put_stats(&w, &m->stats);
    put_logger(&w, &m->logger);
    if (m->terminationPresent) {
        put_termination(&w, &m->termination);
    } else {
        jw_null(&w, "termination");
    }
    jw_end(&w, '}');
    jw_raw(&w, "\n");

    if (!w.ok) {
        out[0] = '\0';
        return 0u;
    }
    return w.used;
}
