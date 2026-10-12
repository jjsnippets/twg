#define _POSIX_C_SOURCE 200809L

#include <errno.h>
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
        fprintf( \
            stderr, \
            "FAIL %s:%d: %s\n", \
            __FILE__, \
            __LINE__, \
            #condition); \
        return 1; \
    } \
} while (0)

static int test_cli_defaults(void)
{
    char *argv[] = {
        (char *)"ms5837_capture"
    };
    Ms5837CliOptions_t options;

    CHECK(
        ms5837_cli_parse(
            1,
            argv,
            &options) == 0);

    CHECK(
        options.duration_sec ==
        MS5837_DEFAULT_DURATION_SEC);

    CHECK(!options.zero_requested);

    return 0;
}

static int test_cli_allowed_options(void)
{
    char *argv[] = {
        (char *)"ms5837_capture",
        (char *)"--zero",
        (char *)"--duration",
        (char *)"3"
    };
    Ms5837CliOptions_t options;

    CHECK(
        ms5837_cli_parse(
            4,
            argv,
            &options) == 0);

    CHECK(options.zero_requested);
    CHECK(options.duration_sec == 3U);

    return 0;
}

static int rejected(
    int argc,
    char *argv[])
{
    Ms5837CliOptions_t options;

    return ms5837_cli_parse(
        argc,
        argv,
        &options) < 0;
}

static int test_cli_rejections(void)
{
    char *duration_missing[] = {
        (char *)"ms5837_capture",
        (char *)"--duration"
    };
    char *duration_zero[] = {
        (char *)"ms5837_capture",
        (char *)"--duration",
        (char *)"0"
    };
    char *duration_duplicate[] = {
        (char *)"ms5837_capture",
        (char *)"--duration",
        (char *)"1",
        (char *)"--duration",
        (char *)"2"
    };
    char *zero_duplicate[] = {
        (char *)"ms5837_capture",
        (char *)"--zero",
        (char *)"--zero"
    };
    char *density[] = {
        (char *)"ms5837_capture",
        (char *)"--density",
        (char *)"1025"
    };
    char *help[] = {
        (char *)"ms5837_capture",
        (char *)"--help"
    };

    CHECK(rejected(2, duration_missing));
    CHECK(rejected(3, duration_zero));
    CHECK(rejected(5, duration_duplicate));
    CHECK(rejected(3, zero_duplicate));
    CHECK(rejected(3, density));
    CHECK(rejected(2, help));

    return 0;
}

static BaroLogRecord_t make_record(
    uint64_t sequence)
{
    BaroLogRecord_t record;
    Ms5837PublicationRecord_t *publication;

    memset(&record, 0, sizeof(record));

    record.schema_version =
        MS5837_CSV_SCHEMA_VERSION;

    record.phase =
        sequence == 1U
        ? BARO_CAPTURE_PHASE_ZERO
        : BARO_CAPTURE_PHASE_RUN;

    publication = &record.publication;

    publication->version =
        MS5837_PUBLICATION_CONTRACT_VERSION;

    publication->publication_sequence =
        sequence;

    publication->scheduled_ns =
        sequence * UINT64_C(10000000);

    publication->actual_ns =
        publication->scheduled_ns +
        UINT64_C(1000);

    publication->scheduled_valid = 1U;
    publication->actual_valid = 1U;
    publication->driver_state = 1;

    publication->sample_ready =
        sequence != 1U;
    publication->not_ready =
        sequence == 1U;
    publication->sample_fresh =
        sequence != 1U;
    publication->age_valid =
        sequence != 1U;
    publication->age_ns =
        sequence != 1U ? 1000U : 0U;

    publication->sample.contract_version =
        BARO_SAMPLE_CONTRACT_VERSION;

    publication->sample.status_flags =
        sequence == 1U
        ? BARO_STATUS_NOT_READY
        : BARO_STATUS_NONE;

    publication->sample.measurement_sequence =
        sequence == 1U
        ? 0U
        : sequence - 1U;

    publication
        ->sample
        .measurement_complete_time_ns =
        sequence == 1U
        ? 0U
        : publication->actual_ns - 1000U;

    publication->sample.raw_pressure_d1 =
        sequence == 1U ? 0U : 6465444U;

    publication->sample.raw_temperature_d2 =
        sequence == 1U ? 0U : 8077636U;

    publication->sample.pressure_mbar =
        sequence == 1U ? NAN : 1000.25;

    publication->sample.temperature_c =
        sequence == 1U ? NAN : 20.0;

    record.zero_requested = true;
    record.zero_valid = sequence != 1U;
    record.zero_valid_samples = sequence - 1U;
    record.zero_rejected_samples = 0U;

    record.surface_pressure_mbar =
        sequence == 1U ? NAN : 1000.25;

    record.fluid_density_kg_m3 = 1000.0;

    record.depth_m =
        sequence == 1U ? NAN : -0.001;

    record.logger_drops_before = 0U;

    return record;
}

static int enqueue_retry(
    Ms5837Logger_t *logger,
    const BaroLogRecord_t *record)
{
    struct timespec pause = {
        0,
        1000000L
    };
    size_t attempt;

    for (attempt = 0U;
         attempt < 1000U;
         ++attempt) {
        int status =
            ms5837_logger_enqueue(
                logger,
                record);

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
    char line[1024];
    FILE *stream;
    unsigned int lines = 0U;
    bool saw_header = false;
    bool saw_zero_row = false;
    bool saw_nan = false;
    bool saw_negative_depth = false;
    uint64_t sequence;

    CHECK(
        snprintf(
            path,
            sizeof(path),
            "test_ms5_%ld.csv",
            (long)getpid()) > 0);

    (void)unlink(path);

    CHECK(
        ms5837_logger_start_path(
            &logger,
            path) == 0);

    CHECK(
        strcmp(
            ms5837_logger_filename(&logger),
            path) == 0);

    for (sequence = 1U;
         sequence <= 100U;
         ++sequence) {
        record = make_record(sequence);

        CHECK(
            enqueue_retry(
                &logger,
                &record) == 0);
    }

    CHECK(ms5837_logger_stop(&logger) == 0);

    stream = fopen(path, "r");
    CHECK(stream != NULL);

    while (fgets(
               line,
               sizeof(line),
               stream) != NULL) {
        ++lines;

        if (strstr(
                line,
                "schema_version,"
                "publication_sequence") != NULL) {
            saw_header = true;
        }

        if (strstr(line, ",zero,") != NULL) {
            saw_zero_row = true;
        }

        if (strstr(line, ",nan,") != NULL) {
            saw_nan = true;
        }

        if (strstr(
                line,
                ",-0.001000,") != NULL) {
            saw_negative_depth = true;
        }
    }

    CHECK(fclose(stream) == 0);
    CHECK(unlink(path) == 0);

    CHECK(lines == 101U);
    CHECK(saw_header);
    CHECK(saw_zero_row);
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

    puts(
        "PASS: MS5837 CLI and versioned "
        "asynchronous CSV logger");

    return 0;
}
