#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app/imu_meta.h"

static int s_failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++s_failures;                                                    \
        }                                                                    \
    } while (0)

/* ---- minimal strict JSON validator ---- */

static const char *skip_ws(const char *p)
{
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') {
        ++p;
    }
    return p;
}

static const char *parse_value(const char *p, int depth);

static const char *parse_string(const char *p)
{
    if (*p != '"') {
        return NULL;
    }
    for (++p; *p != '"'; ++p) {
        unsigned char c = (unsigned char)*p;
        unsigned i;

        if (c == '\0' || c < 0x20u) {
            return NULL;
        }
        if (c == '\\') {
            ++p;
            if (*p == 'u') {
                for (i = 0u; i < 4u; ++i) {
                    ++p;
                    if (!((*p >= '0' && *p <= '9') ||
                          (*p >= 'a' && *p <= 'f') ||
                          (*p >= 'A' && *p <= 'F'))) {
                        return NULL;
                    }
                }
            } else if (strchr("\"\\/bfnrt", *p) == NULL || *p == '\0') {
                return NULL;
            }
        }
    }
    return p + 1;
}

static const char *parse_number(const char *p)
{
    const char *start = p;

    if (*p == '-') {
        ++p;
    }
    if (*p < '0' || *p > '9') {
        return NULL;
    }
    if (*p == '0' && p[1] >= '0' && p[1] <= '9') {
        return NULL;
    }
    while (*p >= '0' && *p <= '9') {
        ++p;
    }
    return p == start ? NULL : p;
}

static const char *parse_value(const char *p, int depth)
{
    if (depth > 8) {
        return NULL;
    }
    p = skip_ws(p);
    if (*p == '{') {
        p = skip_ws(p + 1);
        if (*p == '}') {
            return p + 1;
        }
        for (;;) {
            p = parse_string(skip_ws(p));
            if (p == NULL) return NULL;
            p = skip_ws(p);
            if (*p != ':') return NULL;
            p = parse_value(p + 1, depth + 1);
            if (p == NULL) return NULL;
            p = skip_ws(p);
            if (*p == ',') { ++p; continue; }
            return *p == '}' ? p + 1 : NULL;
        }
    }
    if (*p == '[') {
        p = skip_ws(p + 1);
        if (*p == ']') {
            return p + 1;
        }
        for (;;) {
            p = parse_value(p, depth + 1);
            if (p == NULL) return NULL;
            p = skip_ws(p);
            if (*p == ',') { ++p; continue; }
            return *p == ']' ? p + 1 : NULL;
        }
    }
    if (*p == '"') return parse_string(p);
    if (!strncmp(p, "true", 4)) return p + 4;
    if (!strncmp(p, "false", 5)) return p + 5;
    if (!strncmp(p, "null", 4)) return p + 4;
    return parse_number(p);
}

static int json_valid(const char *s)
{
    const char *end = parse_value(s, 0);

    return end != NULL && *skip_ws(end) == '\0';
}

/* ---- fixtures ---- */

static ImuRunMetadata_t make_meta(void)
{
    ImuRunMetadata_t m;
    ImuMetaUtc_t utc = { 2026, 10, 6, 15, 49, 7, {0} };

    memset(&m, 0, sizeof(m));
    m.version = IMU_RUN_METADATA_CONTRACT_VERSION;
    m.metaSchemaVersion = IMU_META_SCHEMA_VERSION;
    m.csvSchemaVersion = IMU_CSV_SCHEMA_VERSION;
    m.completion = IMU_META_COMPLETION_COMPLETE;
    m.tLateNs = IMU_PUBLISH_T_LATE_NS;
    m.servicePeriodNs = 1000000ull;
    m.publicationPeriodNs = 10000000ull;
    m.settleDurationNs = 300000000ull;
    m.stats.version = IMU_RUN_STATS_CONTRACT_VERSION;
    m.stats.serviceTicks = 10000u;
    m.stats.publicationsAttempted = 1000u;
    m.stats.publicationsEnqueued = 1000u;
    m.stats.staleByGroup[1] = 7u;
    m.stats.multiUpdateByGroup[2] = 2u;
    m.logger.version = IMU_LOGGER_STATS_CONTRACT_VERSION;
    m.logger.startResult = IMU_LOGGER_START_OK;
    m.logger.queueCapacity = 2048u;
    m.logger.recordsWritten = 1000u;
    m.termination.version = IMU_TERMINATION_CONTRACT_VERSION;
    m.termination.reason = IMU_TERM_REASON_COMPLETED;
    m.termination.finalReaderState = IMU_READER_STATE_OPERATIONAL;
    m.termination.acquisitionWindowClosed = 1u;
    m.terminationPresent = 1u;
    m.buildIdentityPresent = 1u;
    strcpy(m.buildIdentity, "bno-app 9.5 test");
    CHECK(imu_meta_fill_names(&m, &utc));
    return m;
}

static ImuCmdPlan_t make_plan(void)
{
    ImuCmdPlan_t p;

    memset(&p, 0, sizeof(p));
    p.version = IMU_CMD_PLAN_VERSION;
    p.slot1 = IMU_CMD_ID_PROBE;
    p.slot2 = IMU_CMD_ID_SETTLE;
    p.slot3 = IMU_CMD_ID_ACQUISITION;
    p.tareAxes = IMU_CMD_TARE_AXES_Z;
    p.probeMaskPresent = true;
    p.probeMask = 7u;
    p.probeDeadlineS = 10u;
    p.acquisitionDurationS = 10u;
    p.settleDurationMs = 300u;
    p.servicePeriodNs = 1000000ull;
    p.publicationPeriodNs = 10000000ull;
    return p;
}

static ImuCmdResult_t make_result(ImuCmdIdentity_t id)
{
    ImuCmdResult_t r;

    memset(&r, 0, sizeof(r));
    r.identity = id;
    r.state = IMU_CMD_STATE_SUCCEEDED;
    r.reason = IMU_CMD_REASON_OK;
    r.startedNs = 111u;
    r.endedNs = 222u;
    r.epochBefore = 1u;
    r.epochAfter = 2u;
    return r;
}

static void test_names(void)
{
    ImuMetaUtc_t utc = { 2026, 10, 6, 15, 49, 7, {0} };
    char stamp[IMU_META_UTC_STAMP_CAPACITY];
    char base[IMU_META_BASE_NAME_CAPACITY];
    char file[IMU_META_FILE_NAME_CAPACITY];
    ImuMetaUtc_t bad;
    char tiny[15];

    CHECK(imu_meta_utc_stamp(&utc, stamp, sizeof(stamp)));
    CHECK(!strcmp(stamp, "20261006_154907"));
    CHECK(imu_meta_base_name(stamp, base, sizeof(base)));
    CHECK(!strcmp(base, "bno_acq_20261006_154907"));
    CHECK(imu_meta_file_name(base, "csv", file, sizeof(file)));
    CHECK(!strcmp(file, "bno_acq_20261006_154907.csv"));
    CHECK(imu_meta_file_name(base, "json.tmp", file, sizeof(file)));
    CHECK(!strcmp(file, "bno_acq_20261006_154907.json.tmp"));

    CHECK(!imu_meta_file_name(base, ".csv", file, sizeof(file)));
    CHECK(!imu_meta_file_name(base, "C/V", file, sizeof(file)));
    CHECK(!imu_meta_file_name(base, "", file, sizeof(file)));
    CHECK(!imu_meta_file_name(base, "csv", file, 10u) && file[0] == '\0');
    CHECK(!imu_meta_file_name(NULL, "csv", file, sizeof(file)));

    CHECK(!imu_meta_utc_stamp(&utc, tiny, sizeof(tiny)));
    CHECK(!imu_meta_utc_stamp(NULL, stamp, sizeof(stamp)));
    CHECK(!imu_meta_utc_stamp(&utc, NULL, 16u));

    bad = utc; bad.year = 1969u;  CHECK(!imu_meta_utc_stamp(&bad, stamp, sizeof(stamp)));
    bad = utc; bad.year = 10000u; CHECK(!imu_meta_utc_stamp(&bad, stamp, sizeof(stamp)));
    bad = utc; bad.month = 0u;    CHECK(!imu_meta_utc_stamp(&bad, stamp, sizeof(stamp)));
    bad = utc; bad.month = 13u;   CHECK(!imu_meta_utc_stamp(&bad, stamp, sizeof(stamp)));
    bad = utc; bad.day = 0u;      CHECK(!imu_meta_utc_stamp(&bad, stamp, sizeof(stamp)));
    bad = utc; bad.hour = 24u;    CHECK(!imu_meta_utc_stamp(&bad, stamp, sizeof(stamp)));
    bad = utc; bad.minute = 60u;  CHECK(!imu_meta_utc_stamp(&bad, stamp, sizeof(stamp)));
    bad = utc; bad.second = 60u;  CHECK(!imu_meta_utc_stamp(&bad, stamp, sizeof(stamp)));

    bad = utc; bad.month = 2u; bad.day = 29u; bad.year = 2028u;
    CHECK(imu_meta_utc_stamp(&bad, stamp, sizeof(stamp)));
    CHECK(!strcmp(stamp, "20280229_154907"));
    bad.year = 2027u; CHECK(!imu_meta_utc_stamp(&bad, stamp, sizeof(stamp)));
    bad.year = 2100u; CHECK(!imu_meta_utc_stamp(&bad, stamp, sizeof(stamp)));
    bad.year = 2000u; CHECK(imu_meta_utc_stamp(&bad, stamp, sizeof(stamp)));
    bad = utc; bad.month = 4u; bad.day = 31u;
    CHECK(!imu_meta_utc_stamp(&bad, stamp, sizeof(stamp)));

    CHECK(!imu_meta_base_name("2026100_6154907", base, sizeof(base)));
    CHECK(!imu_meta_base_name("20261006-154907", base, sizeof(base)));
    CHECK(!imu_meta_base_name("20261006_15490", base, sizeof(base)));
    CHECK(!imu_meta_base_name(NULL, base, sizeof(base)));
    CHECK(!imu_meta_base_name("20261006_154907", base, 10u));
}

static void test_json_basic(void)
{
    ImuRunMetadata_t m = make_meta();
    ImuCmdPlan_t p = make_plan();
    ImuCmdResult_t probe = make_result(IMU_CMD_ID_PROBE);
    ImuCmdResult_t acq = make_result(IMU_CMD_ID_ACQUISITION);
    const ImuCmdResult_t *results[IMU_CMD_ID_COUNT] = {0};
    char json[IMU_META_JSON_CAPACITY];
    size_t n;

    probe.sub.probeMaskActualValid = true;
    probe.sub.probeMaskActual = 7u;
    probe.sub.probeReachedGate = true;
    probe.probeMaskRequestedValid = true;
    probe.probeMaskRequested = 7u;
    results[IMU_CMD_ID_PROBE] = &probe;
    results[IMU_CMD_ID_ACQUISITION] = &acq;

    n = imu_meta_format_json(&m, &p, results, json, sizeof(json));
    CHECK(n != 0u && n == strlen(json));
    CHECK(json[n - 1u] == '\n' && strchr(json, '\r') == NULL);
    CHECK(json_valid(json));
    CHECK(strstr(json, "\"meta_schema_ver\":3,") != NULL);
    CHECK(strstr(json, "\"csv_schema_ver\":1,") != NULL);
    CHECK(strstr(json, "\"completion\":\"complete\"") != NULL);
    CHECK(strstr(json, "\"base_name\":\"bno_acq_20261006_154907\"") != NULL);
    CHECK(strstr(json, "\"filename_local\":\"20261006_154907\"") != NULL);
    CHECK(strstr(json, "filename_utc") == NULL);
    CHECK(strstr(json, "\"csv_file\":\"bno_acq_20261006_154907.csv\"") != NULL);
    CHECK(strstr(json, "\"meta_file\":\"bno_acq_20261006_154907.json\"") != NULL);
    CHECK(strstr(json, "\"build_identity\":\"bno-app 9.5 test\"") != NULL);
    CHECK(strstr(json, "\"pinned_git_sha_expected\":null") != NULL);
    CHECK(strstr(json, "\"t_late_ns\":1000000,") != NULL);
    CHECK(strstr(json, "\"slots\":[\"probe\",\"settle\",\"acquisition\"]") != NULL);
    CHECK(strstr(json, "\"probe_mask\":7,") != NULL);
    CHECK(strstr(json, "\"stale_by_group\":[0,7,0]") != NULL);
    CHECK(strstr(json, "\"multi_update_by_group\":[0,0,2]") != NULL);
    CHECK(strstr(json, "\"start_result\":\"ok\"") != NULL);
    CHECK(strstr(json, "\"reason\":\"completed\"") != NULL);
    CHECK(strstr(json, "\"final_reader_state\":4,") != NULL);
    CHECK(strstr(json, "\"probe_mask_actual\":7") != NULL);
    CHECK(strstr(json, "\"identity\":\"probe\"") != NULL);
    CHECK(strstr(json, "\"identity\":\"acquisition\"") != NULL);
    CHECK(strstr(json, "\"identity\":\"settle\",\"state\"") == NULL);
}

static void test_json_optionals(void)
{
    ImuRunMetadata_t m = make_meta();
    ImuCmdPlan_t p = make_plan();
    char json[IMU_META_JSON_CAPACITY];

    m.completion = IMU_META_COMPLETION_NO_CSV;
    m.buildIdentityPresent = 0u;
    m.terminationPresent = 0u;
    p.probeMaskPresent = false;

    CHECK(imu_meta_format_json(&m, &p, NULL, json, sizeof(json)) != 0u);
    CHECK(json_valid(json));
    CHECK(strstr(json, "\"completion\":\"no_csv\"") != NULL);
    CHECK(strstr(json, "\"csv_file\":null") != NULL);
    CHECK(strstr(json, "\"build_identity\":null") != NULL);
    CHECK(strstr(json, "\"probe_mask\":null") != NULL);
    CHECK(strstr(json, "\"stages\":[]") != NULL);
    CHECK(strstr(json, "\"termination\":null") != NULL);

    m.completion = IMU_META_COMPLETION_INCOMPLETE;
    m.termination.reason = IMU_TERM_REASON_PROCESS_STOP;
    m.termination.signalNumber = 15;
    m.termination.exitStatus = -1;
    m.terminationPresent = 1u;
    CHECK(imu_meta_format_json(&m, &p, NULL, json, sizeof(json)) != 0u);
    CHECK(json_valid(json));
    CHECK(strstr(json, "\"completion\":\"incomplete\"") != NULL);
    CHECK(strstr(json, "\"signal_number\":15,\"exit_status\":-1") != NULL);
}

static void test_escaping(void)
{
    ImuRunMetadata_t m = make_meta();
    ImuCmdPlan_t p = make_plan();
    char json[IMU_META_JSON_CAPACITY];

    memset(m.buildIdentity, 0, sizeof(m.buildIdentity));
    strcpy(m.buildIdentity, "a\"b\\c\n\t\x01 end");
    CHECK(imu_meta_format_json(&m, &p, NULL, json, sizeof(json)) != 0u);
    CHECK(json_valid(json));
    CHECK(strstr(json, "a\\\"b\\\\c\\u000a\\u0009\\u0001 end") != NULL);

    memset(m.buildIdentity, 'x', sizeof(m.buildIdentity));
    CHECK(imu_meta_format_json(&m, &p, NULL, json, sizeof(json)) == 0u);
    CHECK(json[0] == '\0');

    m = make_meta();
    memset(m.baseName, 'y', sizeof(m.baseName));
    CHECK(imu_meta_format_json(&m, &p, NULL, json, sizeof(json)) == 0u);
}

static void test_rejects(void)
{
    ImuRunMetadata_t m = make_meta();
    ImuCmdPlan_t p = make_plan();
    char json[IMU_META_JSON_CAPACITY];

    CHECK(imu_meta_format_json(NULL, &p, NULL, json, sizeof(json)) == 0u);
    CHECK(imu_meta_format_json(&m, NULL, NULL, json, sizeof(json)) == 0u);
    CHECK(imu_meta_format_json(&m, &p, NULL, NULL, 100u) == 0u);
    CHECK(imu_meta_format_json(&m, &p, NULL, json, 0u) == 0u);

    m.metaSchemaVersion = 1u;
    CHECK(imu_meta_format_json(&m, &p, NULL, json, sizeof(json)) == 0u);
    m = make_meta();
    m.metaSchemaVersion = 2u;
    CHECK(imu_meta_format_json(&m, &p, NULL, json, sizeof(json)) == 0u);
    m = make_meta(); m.csvSchemaVersion = 2u;
    CHECK(imu_meta_format_json(&m, &p, NULL, json, sizeof(json)) == 0u);
    m = make_meta(); m.version = 9u;
    CHECK(imu_meta_format_json(&m, &p, NULL, json, sizeof(json)) == 0u);
    m = make_meta(); m.completion = (ImuMetaCompletion_t)9;
    CHECK(imu_meta_format_json(&m, &p, NULL, json, sizeof(json)) == 0u);
    m = make_meta(); m.baseName[0] = '\0';
    CHECK(imu_meta_format_json(&m, &p, NULL, json, sizeof(json)) == 0u);
    m = make_meta(); p.version = 1u;
    CHECK(imu_meta_format_json(&m, &p, NULL, json, sizeof(json)) == 0u);
}

static void test_truncation_sweep(void)
{
    ImuRunMetadata_t m = make_meta();
    ImuCmdPlan_t p = make_plan();
    ImuCmdResult_t r = make_result(IMU_CMD_ID_PROBE);
    const ImuCmdResult_t *results[IMU_CMD_ID_COUNT] = {0};
    char full[IMU_META_JSON_CAPACITY];
    char guard[IMU_META_JSON_CAPACITY + 8u];
    size_t n;
    size_t cap;

    results[IMU_CMD_ID_PROBE] = &r;
    n = imu_meta_format_json(&m, &p, results, full, sizeof(full));
    CHECK(n != 0u);

    memset(guard, 0x5A, sizeof(guard));
    CHECK(imu_meta_format_json(&m, &p, results, guard, n + 1u) == n);
    CHECK((unsigned char)guard[n + 1u] == 0x5Au);

    for (cap = 1u; cap <= n; cap += 7u) {
        memset(guard, 0x5A, sizeof(guard));
        CHECK(imu_meta_format_json(&m, &p, results, guard, cap) == 0u);
        CHECK(guard[0] == '\0' && (unsigned char)guard[cap] == 0x5Au);
    }
}

static void test_worst_case(void)
{
    ImuRunMetadata_t m = make_meta();
    ImuCmdPlan_t p = make_plan();
    ImuCmdResult_t r[IMU_CMD_ID_COUNT];
    const ImuCmdResult_t *results[IMU_CMD_ID_COUNT];
    char json[IMU_META_JSON_CAPACITY];
    unsigned i;
    size_t n;

    memset(m.buildIdentity, '"', sizeof(m.buildIdentity) - 1u);
    m.buildIdentity[sizeof(m.buildIdentity) - 1u] = '\0';
    m.stats.serviceTicks = m.stats.serviceOverruns = UINT64_MAX;
    m.stats.publicationsAttempted = m.stats.publicationsEnqueued = UINT64_MAX;
    m.stats.deadlineMisses = m.stats.gatesSkipped = UINT64_MAX;
    for (i = 0u; i < IMU_PUBLISH_GROUP_COUNT; ++i) {
        m.stats.staleByGroup[i] = m.stats.multiUpdateByGroup[i] = UINT64_MAX;
    }
    m.logger.recordsEnqueued = m.logger.recordsWritten = UINT64_MAX;
    m.logger.recordsDropped = UINT64_MAX;
    m.termination.signalNumber = INT32_MIN;
    m.termination.exitStatus = INT32_MIN;
    p.servicePeriodNs = p.publicationPeriodNs = UINT64_MAX;

    for (i = 0u; i < (unsigned)IMU_CMD_ID_COUNT; ++i) {
        r[i] = make_result((ImuCmdIdentity_t)i);
        r[i].state = IMU_CMD_STATE_RECOVERY_FAILED;
        r[i].reason = IMU_CMD_REASON_CAL_VERIFY_CONFIG_FAILED;
        r[i].startedNs = r[i].endedNs = UINT64_MAX;
        r[i].epochBefore = r[i].epochAfter = UINT32_MAX;
        r[i].requestedTareAxes = IMU_CMD_TARE_AXES_FULL;
        r[i].probeMaskRequestedValid = true;
        r[i].probeMaskRequested = 255u;
        r[i].sub.probeMaskActualValid = true;
        r[i].sub.probeMaskActual = 255u;
        r[i].sub.tareNow = r[i].sub.persist = IMU_CMD_SUB_NOT_ATTEMPTED;
        r[i].sub.clearActive = r[i].sub.clearSaved = IMU_CMD_SUB_NOT_ATTEMPTED;
        results[i] = &r[i];
    }

    n = imu_meta_format_json(&m, &p, results, json, sizeof(json));
    CHECK(n != 0u && n < IMU_META_JSON_CAPACITY);
    CHECK(json_valid(json));
    printf("info: worst-case json length=%zu of %u\n", n, IMU_META_JSON_CAPACITY);
}

static void test_out_of_range_enums(void)
{
    ImuRunMetadata_t m = make_meta();
    ImuCmdPlan_t p = make_plan();
    ImuCmdResult_t r = make_result(IMU_CMD_ID_PROBE);
    const ImuCmdResult_t *results[IMU_CMD_ID_COUNT] = {0};
    char json[IMU_META_JSON_CAPACITY];

    r.state = (ImuCmdResultState_t)99;
    r.reason = (ImuCmdReason_t)99;
    p.slot2 = (ImuCmdIdentity_t)99;
    m.logger.startResult = (ImuLoggerStartResult_t)99;
    m.termination.reason = (ImuTerminationReason_t)99;
    results[IMU_CMD_ID_PROBE] = &r;
    CHECK(imu_meta_format_json(&m, &p, results, json, sizeof(json)) != 0u);
    CHECK(json_valid(json));
    CHECK(strstr(json, "\"state\":\"unknown\"") != NULL);
    CHECK(strstr(json, "\"start_result\":\"unknown\"") != NULL);
}

int main(void)
{
    test_names();
    test_json_basic();
    test_json_optionals();
    test_escaping();
    test_rejects();
    test_truncation_sweep();
    test_worst_case();
    test_out_of_range_enums();

    if (s_failures != 0) {
        fprintf(stderr, "test_imu_meta: %d failure(s)\n", s_failures);
        return 1;
    }
    puts("test_imu_meta: PASS");
    return 0;
}
