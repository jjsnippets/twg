#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "ms5837_cli.h"
#include "ms5837_logger.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static int test_cli_defaults(void)
{
    char *argv[] = { (char *)"ms5837_capture" };
    Ms5837CliOptions_t options;

    CHECK(ms5837_cli_parse(1, argv, &options) == 0);
    CHECK(options.duration_sec == MS5837_DEFAULT_DURATION_SEC);
    CHECK(options.density_kg_m3 == MS5837_DEFAULT_DENSITY_KG_M3);
    CHECK(!options.zero_requested);
    return 0;
}

static int test_cli_allowed_options(void)
{
    char *argv[] = {
        (char *)"ms5837_capture", (char *)"--zero",
        (char *)"--density", (char *)"1025.5",
        (char *)"--duration", (char *)"3"
    };
    Ms5837CliOptions_t options;

    CHECK(ms5837_cli_parse(6, argv, &options) == 0);
    CHECK(options.zero_requested);
    CHECK(options.duration_sec == 3U);
    CHECK(fabs(options.density_kg_m3 - 1025.5) < 1e-12);
    return 0;
}

static int rejected(int argc, char *argv[])
{
    Ms5837CliOptions_t options;
    return ms5837_cli_parse(argc, argv, &options) < 0;
}

static int test_cli_rejections(void)
{
    char *duration_missing[] = {
        (char *)"ms5837_capture", (char *)"--duration"
    };
    char *duration_zero[] = {
        (char *)"ms5837_capture", (char *)"--duration", (char *)"0"
    };
    char *density_bad[] = {
        (char *)"ms5837_capture", (char *)"--density", (char *)"nan"
    };
    char *old_zero[] = {
        (char *)"ms5837_capture", (char *)"--zero-at-start"
    };
    char *bus[] = {
        (char *)"ms5837_capture", (char *)"--bus", (char *)"/dev/i2c-0"
    };
    char *priority[] = {
        (char *)"ms5837_capture", (char *)"--priority", (char *)"80"
    };
    char *help[] = { (char *)"ms5837_capture", (char *)"--help" };

    CHECK(rejected(2, duration_missing));
    CHECK(rejected(3, duration_zero));
    CHECK(rejected(3, density_bad));
    CHECK(rejected(2, old_zero));
    CHECK(rejected(3, bus));
    CHECK(rejected(3, priority));
    CHECK(rejected(2, help));
    return 0;
}

static BaroLogRecord_t make_record(uint64_t sequence)
{
    BaroLogRecord_t record;

    memset(&record, 0, sizeof(record));
    record.publication_sequence = sequence;
    record.publication_time_ns = sequence * UINT64_C(10000000);
    record.phase = (sequence == 1U)
        ? BARO_CAPTURE_PHASE_ZERO : BARO_CAPTURE_PHASE_RUN;
    record.sample_ready = sequence != 1U;
    record.sample_stale = false;
    record.config.config_version = BARO_RUN_CONFIG_VERSION;
    record.config.fluid_density_kg_m3 = 1000.0;
    record.config.surface_pressure_mbar = sequence == 1U ? NAN : 1000.25;
    record.config.surface_pressure_valid = sequence != 1U;
    record.sample.contract_version = BARO_SAMPLE_CONTRACT_VERSION;
    record.sample.status_flags = sequence == 1U
        ? BARO_STATUS_NOT_READY : BARO_STATUS_NONE;
    record.sample.measurement_sequence = sequence == 1U ? 0U : sequence - 1U;
    record.sample.measurement_complete_time_ns = record.publication_time_ns;
    record.sample.raw_pressure_d1 = sequence == 1U ? 0U : 6465444U;
    record.sample.raw_temperature_d2 = sequence == 1U ? 0U : 8077636U;
    record.sample.pressure_mbar = sequence == 1U ? NAN : 1000.25;
    record.sample.temperature_c = sequence == 1U ? NAN : 20.0;
    record.sample.depth_m = sequence == 1U ? NAN : -0.001;
    return record;
}

static int enqueue_retry(Ms5837Logger_t *logger,
                         const BaroLogRecord_t *record)
{
    struct timespec pause = { 0, 1000000L };
    size_t attempt;

    for (attempt = 0U; attempt < 1000U; ++attempt) {
        int status = ms5837_logger_enqueue(logger, record);
        if (status == 0) {
            return 0;
        }
        if (status != -EAGAIN) {
            return status;
        }
        (void)nanosleep(&pause, NULL);
    }
    return -EAGAIN;
}

static int test_logger_drains_csv(void)
{
    Ms5837Logger_t logger;
    BaroLogRecord_t record;
    char path[MS5837_LOGGER_FILENAME_SIZE];
    char line[512];
    FILE *stream;
    unsigned int lines = 0U;
    bool saw_header = false;
    bool saw_nan = false;
    bool saw_negative_depth = false;
    uint64_t sequence;

    CHECK(snprintf(path, sizeof(path), "test_ms5_%ld.csv", (long)getpid()) > 0);
    (void)unlink(path);
    CHECK(ms5837_logger_start_path(&logger, path) == 0);
    CHECK(strcmp(ms5837_logger_filename(&logger), path) == 0);
    for (sequence = 1U; sequence <= 100U; ++sequence) {
        record = make_record(sequence);
        CHECK(enqueue_retry(&logger, &record) == 0);
    }
    CHECK(ms5837_logger_stop(&logger) == 0);

    stream = fopen(path, "r");
    CHECK(stream != NULL);
    while (fgets(line, sizeof(line), stream) != NULL) {
        ++lines;
        if (strstr(line, "publication_sequence") != NULL) {
            saw_header = true;
        }
        if (strstr(line, ",nan,") != NULL) {
            saw_nan = true;
        }
        if (strstr(line, ",-0.001000") != NULL) {
            saw_negative_depth = true;
        }
    }
    CHECK(fclose(stream) == 0);
    CHECK(unlink(path) == 0);
    CHECK(lines == 101U);
    CHECK(saw_header);
    CHECK(saw_nan);
    CHECK(saw_negative_depth);
    return 0;
}

int main(void)
{
    CHECK(test_cli_defaults() == 0);
    CHECK(test_cli_allowed_options() == 0);
    CHECK(test_cli_rejections() == 0);
    CHECK(test_logger_drains_csv() == 0);
    puts("PASS: Step 4D CLI and asynchronous CSV logger");
    return 0;
}
