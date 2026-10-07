#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "app/imu_csv.h"
#include "app/imu_logfile.h"

static int s_failures;
static char s_dir[64];

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++s_failures;                                                    \
        }                                                                    \
    } while (0)

#define STAMP "20261006_154907"

static void path_of(char *out, size_t cap, const char *base, const char *ext)
{
    snprintf(out, cap, "%s/%s.%s", s_dir, base, ext);
}

static int exists(const char *base, const char *ext)
{
    char p[512];
    struct stat st;

    path_of(p, sizeof(p), base, ext);
    return lstat(p, &st) == 0;
}

static void touch(const char *base, const char *ext, const char *content)
{
    char p[512];
    FILE *f;

    path_of(p, sizeof(p), base, ext);
    f = fopen(p, "wb");
    if (f != NULL) {
        fputs(content, f);
        fclose(f);
    }
}

static int read_file(const char *base, const char *ext, char *buf, size_t cap)
{
    char p[512];
    FILE *f;
    size_t n;

    path_of(p, sizeof(p), base, ext);
    f = fopen(p, "rb");
    if (f == NULL) {
        return -1;
    }
    n = fread(buf, 1u, cap - 1u, f);
    buf[n] = '\0';
    fclose(f);
    return (int)n;
}

static void test_nominal(void)
{
    ImuLogfile_t lf;
    ImuLoggerSink_t sink;
    char buf[256];
    const char *base;

    CHECK(imu_logfile_open(&lf, s_dir, STAMP, true) == IMU_LOGFILE_OK);
    base = imu_logfile_base_name(&lf);
    CHECK(strcmp(base, "bno_acq_20261006_154907") == 0);
    CHECK(exists(base, "csv.tmp") && exists(base, "json.tmp"));
    CHECK(!exists(base, "csv") && !exists(base, "json"));

    CHECK(imu_logfile_sink(&lf, &sink));
    CHECK(sink.write(sink.ctx, "hdr\n", 4u) == 0);

    CHECK(imu_logfile_write_running(&lf, "{\"status\":\"running\"}\n", 20u) ==
          IMU_LOGFILE_OK);
    CHECK(exists(base, "json") && !exists(base, "json.tmp"));
    CHECK(read_file(base, "json", buf, sizeof(buf)) > 0 &&
          strstr(buf, "running") != NULL);

    CHECK(imu_logfile_close_csv(&lf, true, true) == IMU_LOGFILE_OK);
    CHECK(exists(base, "csv") && !exists(base, "csv.tmp"));
    CHECK(read_file(base, "csv", buf, sizeof(buf)) == 4 &&
          strcmp(buf, "hdr\n") == 0);
    CHECK(imu_logfile_csv_outcome(&lf) == IMU_LOGFILE_CSV_COMPLETE);

    CHECK(imu_logfile_commit_json(&lf, "{\"status\":\"final\"}\n", 19u) ==
          IMU_LOGFILE_OK);
    CHECK(read_file(base, "json", buf, sizeof(buf)) > 0 &&
          strstr(buf, "final") != NULL);
    CHECK(!exists(base, "json.tmp"));
    CHECK(imu_logfile_commit_json(&lf, "x", 1u) == IMU_LOGFILE_ERR_STATE);
}

static void test_collision_suffix(void)
{
    ImuLogfile_t a;
    ImuLogfile_t b;
    char buf[64];
    const char *stamp = "20261006_160000";

    touch("bno_acq_20261006_160000", "csv", "keep");

    CHECK(imu_logfile_open(&a, s_dir, stamp, true) == IMU_LOGFILE_OK);
    CHECK(strcmp(imu_logfile_base_name(&a), "bno_acq_20261006_160000_1") == 0);
    CHECK(imu_logfile_open(&b, s_dir, stamp, true) == IMU_LOGFILE_OK);
    CHECK(strcmp(imu_logfile_base_name(&b), "bno_acq_20261006_160000_2") == 0);

    CHECK(read_file("bno_acq_20261006_160000", "csv", buf, sizeof(buf)) == 4 &&
          strcmp(buf, "keep") == 0);

    CHECK(imu_logfile_close_csv(&a, false, true) == IMU_LOGFILE_OK);
    CHECK(imu_logfile_close_csv(&b, false, true) == IMU_LOGFILE_OK);
}

static void test_missing_directory(void)
{
    ImuLogfile_t lf;

    CHECK(imu_logfile_open(&lf, "/nonexistent_imu_logfile_dir", STAMP, true) ==
          IMU_LOGFILE_ERR_OPEN);
    CHECK(lf.csvFd == -1);
    CHECK(imu_logfile_last_errno(&lf) == ENOENT);
    CHECK(imu_logfile_open(&lf, s_dir, "bad-stamp", true) ==
          IMU_LOGFILE_ERR_NAME);
}

static void test_partial_preserved(void)
{
    ImuLogfile_t lf;
    ImuLoggerSink_t sink;
    char buf[64];
    const char *base;

    CHECK(imu_logfile_open(&lf, s_dir, "20261006_161000", true) ==
          IMU_LOGFILE_OK);
    base = imu_logfile_base_name(&lf);
    CHECK(imu_logfile_sink(&lf, &sink));
    CHECK(sink.write(sink.ctx, "rows\n", 5u) == 0);

    CHECK(imu_logfile_close_csv(&lf, false, true) == IMU_LOGFILE_OK);
    CHECK(exists(base, "csv.partial"));
    CHECK(!exists(base, "csv") && !exists(base, "csv.tmp"));
    CHECK(read_file(base, "csv.partial", buf, sizeof(buf)) == 5);
    CHECK(imu_logfile_csv_outcome(&lf) == IMU_LOGFILE_CSV_PARTIAL);
    CHECK(imu_logfile_commit_json(&lf, "{}\n", 3u) == IMU_LOGFILE_OK);
    CHECK(exists(base, "json"));
}

static void test_drain_timeout_keeps_fd(void)
{
    ImuLogfile_t lf;
    const char *base;

    CHECK(imu_logfile_open(&lf, s_dir, "20261006_162000", true) ==
          IMU_LOGFILE_OK);
    base = imu_logfile_base_name(&lf);
    CHECK(imu_logfile_close_csv(&lf, true, false) == IMU_LOGFILE_OK);
    CHECK(lf.csvFd >= 0);
    CHECK(exists(base, "csv.partial") && !exists(base, "csv"));
    CHECK(imu_logfile_csv_outcome(&lf) == IMU_LOGFILE_CSV_PARTIAL);
    close(lf.csvFd);
}

static void test_json_only(void)
{
    ImuLogfile_t lf;
    ImuLoggerSink_t sink;
    const char *base;

    CHECK(imu_logfile_open(&lf, s_dir, "20261006_163000", false) ==
          IMU_LOGFILE_OK);
    base = imu_logfile_base_name(&lf);
    CHECK(!exists(base, "csv.tmp"));
    CHECK(!imu_logfile_sink(&lf, &sink));
    CHECK(imu_logfile_close_csv(&lf, true, true) == IMU_LOGFILE_OK);
    CHECK(imu_logfile_commit_json(&lf, "{\"csv\":null}\n", 13u) ==
          IMU_LOGFILE_OK);
    CHECK(exists(base, "json"));
    CHECK(!exists(base, "csv") && !exists(base, "csv.partial") &&
          !exists(base, "csv.tmp") && !exists(base, "json.tmp"));
    CHECK(imu_logfile_csv_outcome(&lf) == IMU_LOGFILE_CSV_NONE);
}

static void test_no_overwrite_races(void)
{
    ImuLogfile_t lf;
    char buf[64];
    const char *base;

    CHECK(imu_logfile_open(&lf, s_dir, "20261006_164000", true) ==
          IMU_LOGFILE_OK);
    base = imu_logfile_base_name(&lf);

    touch(base, "csv", "other-csv");
    touch(base, "json", "other-json");

    CHECK(imu_logfile_close_csv(&lf, true, true) == IMU_LOGFILE_ERR_EXISTS);
    CHECK(read_file(base, "csv", buf, sizeof(buf)) > 0 &&
          strcmp(buf, "other-csv") == 0);
    CHECK(exists(base, "csv.tmp"));

    CHECK(imu_logfile_commit_json(&lf, "{\"mine\":1}\n", 11u) ==
          IMU_LOGFILE_ERR_EXISTS);
    CHECK(read_file(base, "json", buf, sizeof(buf)) > 0 &&
          strcmp(buf, "other-json") == 0);
    CHECK(exists(base, "json.tmp"));
}

static ImuLogger_t s_logger;

static void test_logger_integration(void)
{
    ImuLogfile_t lf;
    ImuLoggerSink_t sink;
    ImuPublicationRecord_t record;
    char header[IMU_CSV_LINE_CAPACITY];
    char buf[8192];
    const char *base;
    size_t headerLen;
    unsigned lines = 0u;
    size_t i;

    CHECK(imu_logfile_open(&lf, s_dir, "20261006_165000", true) ==
          IMU_LOGFILE_OK);
    base = imu_logfile_base_name(&lf);
    CHECK(imu_logfile_sink(&lf, &sink));

    CHECK(imu_logger_init(&s_logger));
    CHECK(imu_logger_start(&s_logger, &sink));

    memset(&record, 0, sizeof(record));
    record.version = IMU_PUBLICATION_CONTRACT_VERSION;
    record.publicationSeq = 1u;
    CHECK(imu_logger_enqueue(&s_logger, &record));

    CHECK(imu_logger_drain(&s_logger, 5000000000ull));
    CHECK(!imu_logger_write_failed(&s_logger));
    CHECK(imu_logfile_close_csv(&lf, true, true) == IMU_LOGFILE_OK);

    headerLen = imu_csv_format_header(header, sizeof(header));
    CHECK(headerLen > 0u);
    CHECK(read_file(base, "csv", buf, sizeof(buf)) > 0);
    CHECK(strncmp(buf, header, headerLen) == 0);

    for (i = 0u; buf[i] != '\0'; ++i) {
        if (buf[i] == '\n') {
            ++lines;
        }
    }
    CHECK(lines == 2u);
}

int main(void)
{
    snprintf(s_dir, sizeof(s_dir), "/tmp/imu_logfile_XXXXXX");
    if (mkdtemp(s_dir) == NULL) {
        perror("mkdtemp");
        return 1;
    }

    test_nominal();
    test_collision_suffix();
    test_missing_directory();
    test_partial_preserved();
    test_drain_timeout_keeps_fd();
    test_json_only();
    test_no_overwrite_races();
    test_logger_integration();

    if (s_failures != 0) {
        fprintf(stderr, "test_imu_logfile: %d failure(s) (dir kept: %s)\n",
                s_failures, s_dir);
        return 1;
    }

    {
        char cmd[128];

        snprintf(cmd, sizeof(cmd), "rm -rf %s", s_dir);
        if (system(cmd) != 0) {
            fprintf(stderr, "warning: could not remove %s\n", s_dir);
        }
    }
    puts("test_imu_logfile: PASS");
    return 0;
}
