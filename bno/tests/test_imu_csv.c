#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app/imu_csv.h"

static int s_failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++s_failures;                                                    \
        }                                                                    \
    } while (0)

#define NF IMU_CSV_COLUMN_COUNT

static unsigned split(char *line, char *fields[], unsigned max)
{
    unsigned n = 0u;
    char *p = line;
    size_t len = strlen(line);

    if (len != 0u && line[len - 1u] == '\n') {
        line[len - 1u] = '\0';
    }
    fields[n++] = p;
    for (; *p != '\0'; ++p) {
        if (*p == ',') {
            *p = '\0';
            if (n < max) {
                fields[n] = p + 1;
            }
            ++n;
        }
    }
    return n;
}

static void set_group(ImuPublicationGroup_t *g, uint64_t seq, uint64_t host,
                      uint64_t sensor, uint8_t rep, uint8_t st, uint64_t age)
{
    memset(g, 0, sizeof(*g));
    g->groupEventSeq = seq;
    g->hostDecodeNs = host;
    g->sensorTimeUs = sensor;
    g->deviceReportSeq = rep;
    g->rawStatus = st;
    g->identityPresent = 1u;
    g->ageValid = 1u;
    g->ageNs = age;
}

static ImuPublicationRecord_t full_record(void)
{
    ImuPublicationRecord_t r;

    memset(&r, 0, sizeof(r));
    r.version = IMU_PUBLICATION_CONTRACT_VERSION;
    r.configurationEpoch = 7u;
    r.publicationSeq = 42u;
    r.scheduledNs = 1000000000ull;
    r.actualNs = 1000250000ull;
    r.scheduledValid = r.actualValid = 1u;
    r.loggerDrops = 3u;
    r.gatesSkippedBefore = 2u;
    r.readerFlags = 0x21u;
    r.readerState = IMU_READER_STATE_OPERATIONAL;
    r.deadlineMissed = 1u;
    r.sampleReady = 1u;
    r.validMask = 7u;
    r.freshMask = 5u;
    r.staleMask = 2u;
    r.multiUpdateMask = 1u;
    set_group(&r.rotation, 100u, 900000000ull, 5555u, 9u, 3u, 100000ull);
    set_group(&r.accel, 200u, 900000100ull, 6666u, 10u, 2u, 99900ull);
    set_group(&r.gyro, 300u, 900000200ull, 7777u, 11u, 1u, 99800ull);
    r.qw = 1.0f; r.qx = 0.5f; r.qy = -0.25f; r.qz = 0.125f;
    r.yawRad = 0.1f; r.pitchRad = -0.2f; r.rollRad = 3.0f;
    r.orientationErrRad = 0.017453292f;
    r.ax = 0.0f; r.ay = -9.81f; r.az = 1.0e-7f;
    r.gx = -0.0f; r.gy = 123456.789f; r.gz = 1.0e30f;
    return r;
}

static void test_header(void)
{
    char line[IMU_CSV_LINE_CAPACITY];
    char copy[IMU_CSV_LINE_CAPACITY];
    char *f[NF + 2u];
    size_t n = imu_csv_format_header(line, sizeof(line));
    unsigned i;

    CHECK(n == strlen(line));
    CHECK(n > 0u && line[n - 1u] == '\n' && strchr(line, '\r') == NULL);
    strcpy(copy, line);
    CHECK(split(copy, f, NF + 1u) == NF);
    for (i = 0u; i < NF; ++i) {
        CHECK(strcmp(f[i], IMU_CSV_HEADER_TOKENS[i]) == 0);
    }
    CHECK(strncmp(line, "schema_ver,pub_seq,sched_ns,actual_ns,epoch,", 44) == 0);
    CHECK(imu_csv_format_header(NULL, 10u) == 0u);
    CHECK(imu_csv_format_header(line, 0u) == 0u);
    CHECK(imu_csv_format_header(line, 20u) == 0u && line[0] == '\0');
}

static void test_full_row(void)
{
    ImuPublicationRecord_t r = full_record();
    char line[IMU_CSV_LINE_CAPACITY];
    char *f[NF + 2u];
    size_t n = imu_csv_format_row(&r, line, sizeof(line));

    CHECK(n == strlen(line) && n < IMU_CSV_LINE_CAPACITY / 2u);
    CHECK(line[n - 1u] == '\n' && strchr(line, '\r') == NULL);
    CHECK(split(line, f, NF + 1u) == NF);

    CHECK(!strcmp(f[0], "1"));
    CHECK(!strcmp(f[1], "42"));
    CHECK(!strcmp(f[2], "1000000000"));
    CHECK(!strcmp(f[3], "1000250000"));
    CHECK(!strcmp(f[4], "7"));
    CHECK(!strcmp(f[5], "1"));
    CHECK(!strcmp(f[6], "2"));
    CHECK(!strcmp(f[7], "1") && !strcmp(f[8], "0"));
    CHECK(!strcmp(f[9], "7") && !strcmp(f[10], "5"));
    CHECK(!strcmp(f[11], "2") && !strcmp(f[12], "0") && !strcmp(f[13], "1"));
    CHECK(!strcmp(f[14], "4"));
    CHECK(!strcmp(f[15], "33"));
    CHECK(!strcmp(f[16], "3"));
    CHECK(!strcmp(f[17], "100") && !strcmp(f[18], "900000000"));
    CHECK(!strcmp(f[19], "5555") && !strcmp(f[20], "9") && !strcmp(f[21], "3"));
    CHECK(!strcmp(f[22], "1") && !strcmp(f[23], "100000"));
    CHECK(!strcmp(f[24], "1") && !strcmp(f[25], "0.5") && !strcmp(f[26], "-0.25"));
    CHECK(!strcmp(f[27], "0.125"));
    CHECK(strtof(f[31], NULL) == 0.017453292f);
    CHECK(!strcmp(f[32], "200") && !strcmp(f[38], "99900"));
    CHECK(!strcmp(f[39], "0") && strtof(f[40], NULL) == -9.81f);
    CHECK(strtof(f[41], NULL) == 1.0e-7f);
    CHECK(!strcmp(f[42], "300") && !strcmp(f[48], "99800"));
    CHECK(!strcmp(f[49], "-0"));
    CHECK(strtof(f[50], NULL) == 123456.789f);
    CHECK(strtof(f[51], NULL) == 1.0e30f);
}

static void test_missing_and_invalid(void)
{
    ImuPublicationRecord_t r = full_record();
    char line[IMU_CSV_LINE_CAPACITY];
    char *f[NF + 2u];
    unsigned i;

    memset(&r.accel, 0, sizeof(r.accel));
    r.validMask = 5u;
    r.missingMask = 2u;
    r.staleMask = 0u;
    r.scheduledValid = 0u; r.scheduledNs = 0u;
    r.actualValid = 0u;    r.actualNs = 0u;
    r.gyro.ageValid = 0u;  r.gyro.ageNs = 0u;
    r.qx = NAN;
    r.gz = INFINITY;

    CHECK(imu_csv_format_row(&r, line, sizeof(line)) != 0u);
    CHECK(split(line, f, NF + 1u) == NF);
    CHECK(!strcmp(f[2], "NaN") && !strcmp(f[3], "NaN"));
    CHECK(!strcmp(f[25], "NaN"));
    for (i = 32u; i <= 41u; ++i) {
        CHECK(!strcmp(f[i], i == 37u ? "0" : "NaN"));
    }
    CHECK(!strcmp(f[47], "0") && !strcmp(f[48], "NaN"));
    CHECK(!strcmp(f[51], "NaN"));
    CHECK(!strcmp(f[42], "300"));

    r = full_record();
    r.validMask = 0u;
    r.missingMask = 7u;
    CHECK(imu_csv_format_row(&r, line, sizeof(line)) != 0u);
    CHECK(split(line, f, NF + 1u) == NF);
    for (i = 17u; i < NF; ++i) {
        CHECK(!strcmp(f[i], (i == 22u || i == 37u || i == 47u) ? "0" : "NaN"));
    }
}

static void test_capacity_and_errors(void)
{
    ImuPublicationRecord_t r = full_record();
    char big[IMU_CSV_LINE_CAPACITY];
    char guard[IMU_CSV_LINE_CAPACITY + 8u];
    size_t n = imu_csv_format_row(&r, big, sizeof(big));
    size_t cap;

    CHECK(n != 0u);
    memset(guard, 0x5A, sizeof(guard));
    CHECK(imu_csv_format_row(&r, guard, n + 1u) == n);
    CHECK((unsigned char)guard[n + 1u] == 0x5Au);
    CHECK(strcmp(guard, big) == 0);

    memset(guard, 0x5A, sizeof(guard));
    CHECK(imu_csv_format_row(&r, guard, n) == 0u);
    CHECK(guard[0] == '\0' && (unsigned char)guard[n] == 0x5Au);

    for (cap = 1u; cap < n; cap += 37u) {
        memset(guard, 0x5A, sizeof(guard));
        CHECK(imu_csv_format_row(&r, guard, cap) == 0u);
        CHECK(guard[0] == '\0' && (unsigned char)guard[cap] == 0x5Au);
    }

    CHECK(imu_csv_format_row(NULL, big, sizeof(big)) == 0u);
    CHECK(imu_csv_format_row(&r, NULL, 10u) == 0u);
    CHECK(imu_csv_format_row(&r, big, 0u) == 0u);
    r.version = 2u;
    CHECK(imu_csv_format_row(&r, big, sizeof(big)) == 0u && big[0] == '\0');
}

static void test_worst_case_length(void)
{
    ImuPublicationRecord_t r = full_record();
    char line[IMU_CSV_LINE_CAPACITY];
    size_t n;

    r.publicationSeq = r.scheduledNs = r.actualNs = r.loggerDrops = UINT64_MAX;
    r.configurationEpoch = r.gatesSkippedBefore = UINT32_MAX;
    r.readerFlags = 0x3Fu;
    r.rotation.groupEventSeq = r.accel.groupEventSeq = r.gyro.groupEventSeq = UINT64_MAX;
    r.rotation.hostDecodeNs = r.accel.hostDecodeNs = r.gyro.hostDecodeNs = UINT64_MAX;
    r.rotation.sensorTimeUs = r.accel.sensorTimeUs = r.gyro.sensorTimeUs = UINT64_MAX;
    r.rotation.ageNs = r.accel.ageNs = r.gyro.ageNs = UINT64_MAX;
    r.rotation.deviceReportSeq = r.accel.deviceReportSeq = r.gyro.deviceReportSeq = 255u;
    r.rotation.rawStatus = r.accel.rawStatus = r.gyro.rawStatus = 255u;
    r.qw = r.qx = r.qy = r.qz = -1.23456789e-38f;
    r.yawRad = r.pitchRad = r.rollRad = r.orientationErrRad = -1.23456789e-38f;
    r.ax = r.ay = r.az = r.gx = r.gy = r.gz = -1.23456789e-38f;

    n = imu_csv_format_row(&r, line, sizeof(line));
    CHECK(n != 0u && n < IMU_CSV_LINE_CAPACITY);
    printf("info: worst-case row length=%zu of %u\n", n, IMU_CSV_LINE_CAPACITY);
}

static void test_float_roundtrip(void)
{
    static const float samples[] = {
        0.1f, -0.1f, 3.14159274f, 1.17549435e-38f, 1.0e-45f,
        3.40282347e+38f, 16777217.0f, -9.80665f, 0.0f
    };
    ImuPublicationRecord_t r = full_record();
    char line[IMU_CSV_LINE_CAPACITY];
    char *f[NF + 2u];
    unsigned i;

    for (i = 0u; i < sizeof(samples) / sizeof(samples[0]); ++i) {
        r.yawRad = samples[i];
        CHECK(imu_csv_format_row(&r, line, sizeof(line)) != 0u);
        CHECK(split(line, f, NF + 1u) == NF);
        CHECK(strtof(f[28], NULL) == samples[i]);
    }
}

int main(void)
{
    test_header();
    test_full_row();
    test_missing_and_invalid();
    test_capacity_and_errors();
    test_worst_case_length();
    test_float_roundtrip();

    if (s_failures != 0) {
        fprintf(stderr, "test_imu_csv: %d failure(s)\n", s_failures);
        return 1;
    }
    puts("test_imu_csv: PASS");
    return 0;
}
