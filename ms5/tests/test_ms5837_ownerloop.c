/* ms5/tests/test_ms5837_ownerloop.c
 *
 * Host proof of the top-level capture owner. app/main.c is compiled into this
 * test through the MS5837_APP_OWNER_TEST seam, so its static helpers run
 * unmodified. Time, sleeping, and RT_SleepUntil are replaced by a fake 1 ms
 * clock; the driver, publisher, and asynchronous CSV logger are the
 * production modules, driven against a mock I2C bus.
 */
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "rt/realtime.h"

#define MS5837_APP_OWNER_TEST

#define TICK_NS UINT64_C(1000000)

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static uint64_t g_now_ns;

/* Defined before the macros below, so it calls the real nanosleep(). */
static void real_pause(void)
{
    struct timespec pause = { 0, 100000L };

    (void)nanosleep(&pause, NULL);
}

static int fake_clock_gettime(clockid_t id, struct timespec *out)
{
    (void)id;

    if (out == NULL) {
        return -1;
    }

    out->tv_sec = (time_t)(g_now_ns / UINT64_C(1000000000));
    out->tv_nsec = (long)(g_now_ns % UINT64_C(1000000000));
    return 0;
}

static int fake_nanosleep(const struct timespec *req, struct timespec *rem)
{
    (void)rem;

    g_now_ns += ((uint64_t)req->tv_sec * UINT64_C(1000000000)) +
                (uint64_t)req->tv_nsec;
    return 0;
}

static int fake_rt_sleep_until(double period_sec)
{
    (void)period_sec;

    g_now_ns += TICK_NS;
    real_pause();
    return 0;
}

#define clock_gettime fake_clock_gettime
#define nanosleep fake_nanosleep
#define RT_SleepUntil fake_rt_sleep_until

/* main.c defines its own feature-test macro before any system header. */
#undef _POSIX_C_SOURCE
#include "../app/main.c"

/* ---------------------------------------------------------------- mock HAL */

typedef struct {
    uint16_t prom[MS5837_PROM_WORD_COUNT];
    uint32_t d1;
    uint32_t d2;
    unsigned int open_count;
    unsigned int close_count;
    unsigned int fail_next_read;
    unsigned int fail_open_count;
    uint8_t selected_command;
    bool last_conversion_is_d1;
    size_t prom_index;
} MockHal_t;

static void load_valid_prom(MockHal_t *mock)
{
    static const uint16_t valid[MS5837_PROM_WORD_COUNT] = {
        0x4BA1U, 0xBA4AU, 0xB779U, 0x7395U,
        0x78A1U, 0x793DU, 0x6ACFU
    };

    memset(mock, 0, sizeof(*mock));
    memcpy(mock->prom, valid, sizeof(valid));
    mock->d1 = 6465444U;
    mock->d2 = 8077636U;
}

static int mock_open(void *opaque)
{
    MockHal_t *mock = (MockHal_t *)opaque;

    ++mock->open_count;

    if (mock->fail_open_count != 0U) {
        --mock->fail_open_count;
        return -EIO;
    }

    return 0;
}

static int mock_close(void *opaque)
{
    MockHal_t *mock = (MockHal_t *)opaque;

    ++mock->close_count;
    return 0;
}

static int mock_write(void *opaque, const uint8_t *data, size_t length)
{
    MockHal_t *mock = (MockHal_t *)opaque;

    if ((data == NULL) || (length != 1U)) {
        return -EINVAL;
    }

    mock->selected_command = data[0];

    if (data[0] == MS5837_COMMAND_RESET) {
        mock->prom_index = 0U;
    } else if (data[0] ==
               (MS5837_COMMAND_CONVERT_D1_BASE | MS5837_OSR_512)) {
        mock->last_conversion_is_d1 = true;
    } else if (data[0] ==
               (MS5837_COMMAND_CONVERT_D2_BASE | MS5837_OSR_512)) {
        mock->last_conversion_is_d1 = false;
    }

    return 0;
}

static int mock_read(void *opaque, uint8_t *data, size_t length)
{
    MockHal_t *mock = (MockHal_t *)opaque;
    uint32_t raw;

    if (data == NULL) {
        return -EINVAL;
    }

    if (mock->fail_next_read != 0U) {
        --mock->fail_next_read;
        return -EIO;
    }

    if (length == 2U) {
        if (mock->prom_index >= MS5837_PROM_WORD_COUNT) {
            return -EPROTO;
        }

        data[0] = (uint8_t)(mock->prom[mock->prom_index] >> 8U);
        data[1] = (uint8_t)(mock->prom[mock->prom_index] & 0xFFU);
        ++mock->prom_index;
        return 0;
    }

    if ((length != 3U) ||
        (mock->selected_command != MS5837_COMMAND_ADC_READ)) {
        return -EPROTO;
    }

    raw = mock->last_conversion_is_d1 ? mock->d1 : mock->d2;
    data[0] = (uint8_t)(raw >> 16U);
    data[1] = (uint8_t)(raw >> 8U);
    data[2] = (uint8_t)raw;
    return 0;
}

static int mock_sleep_ns(void *opaque, uint64_t duration_ns)
{
    (void)opaque;

    g_now_ns += duration_ns;
    return 0;
}

static int mock_time_ns(void *opaque, uint64_t *time_ns)
{
    (void)opaque;

    if (time_ns == NULL) {
        return -EINVAL;
    }

    *time_ns = g_now_ns;
    return 0;
}

static Ms5837Hal_t make_hal(MockHal_t *mock)
{
    Ms5837Hal_t hal;

    hal.context = mock;
    hal.open = mock_open;
    hal.close = mock_close;
    hal.write = mock_write;
    hal.read = mock_read;
    hal.sleep_ns = mock_sleep_ns;
    hal.time_ns = mock_time_ns;
    return hal;
}

/* -------------------------------------------------------------- test rig */

static MockHal_t g_mock;
static Ms5837Hal_t g_hal;
static Ms5837Driver_t g_driver;
static Ms5837Logger_t g_logger;
static Ms5837PublisherState_t g_publisher;
static CapturePolicy_t g_policy;
static BaroDriverStats_t g_driver_stats;
static Ms5837RunStats_t g_run_stats;
static char g_path[MS5837_LOGGER_FILENAME_SIZE];

static int rig_start(void)
{
    BaroSample_t baseline;

    stop_requested = 0;
    g_now_ns = UINT64_C(5000000000);

    load_valid_prom(&g_mock);
    g_hal = make_hal(&g_mock);

    memset(&g_policy, 0, sizeof(g_policy));
    memset(&g_driver_stats, 0, sizeof(g_driver_stats));
    memset(&g_run_stats, 0, sizeof(g_run_stats));

    g_policy.zero_requested = true;
    g_policy.surface_pressure_mbar = NAN;
    g_policy.fluid_density_kg_m3 = CAPTURE_FLUID_DENSITY_KG_M3;
    zero_estimator_begin(&g_policy.zero);
    g_run_stats.version = MS5837_RUN_STATS_CONTRACT_VERSION;

    CHECK(ms5837_driver_init(&g_driver, &g_hal) == MS5837_DRIVER_OK);

    CHECK(snprintf(g_path, sizeof(g_path), "t_own_%ld.csv",
                   (long)getpid()) > 0);
    (void)unlink(g_path);
    CHECK(ms5837_logger_start_path(&g_logger, g_path) == 0);

    CHECK(warm_up_sample(&g_driver, &g_driver_stats, &g_run_stats,
                         &baseline) == 0);
    CHECK(ms5837_publish_begin_window(&g_publisher, &baseline, true));
    return 0;
}

static int rig_finish(void)
{
    CHECK(ms5837_logger_stop(&g_logger) == 0);
    (void)ms5837_driver_shutdown(&g_driver);
    return 0;
}

static int count_rows(unsigned *lines, unsigned *zero_rows,
                      unsigned *run_rows)
{
    char line[1024];
    FILE *stream = fopen(g_path, "r");

    CHECK(stream != NULL);

    *lines = 0U;
    *zero_rows = 0U;
    *run_rows = 0U;

    while (fgets(line, sizeof(line), stream) != NULL) {
        ++(*lines);

        if (strstr(line, ",zero,") != NULL) {
            ++(*zero_rows);
        }

        if (strstr(line, ",run,") != NULL) {
            ++(*run_rows);
        }
    }

    CHECK(fclose(stream) == 0);
    CHECK(unlink(g_path) == 0);
    return 0;
}

static int run_phase(BaroCapturePhase_t phase, uint64_t duration_ns)
{
    return run_capture_phase(&g_driver, &g_logger, &g_publisher, &g_policy,
                             &g_driver_stats, &g_run_stats, phase,
                             duration_ns);
}

static BaroSample_t make_sample(uint64_t sequence, double pressure)
{
    BaroSample_t sample;

    memset(&sample, 0, sizeof(sample));
    sample.contract_version = BARO_SAMPLE_CONTRACT_VERSION;
    sample.measurement_sequence = sequence;
    sample.pressure_mbar = pressure;
    sample.temperature_c = 20.0;
    return sample;
}

/* ----------------------------------------------------------------- tests */

static ZeroEstimator_t g_estimator;

static int test_estimator_rejections_and_capacity(void)
{
    BaroSample_t sample;
    unsigned int index;

    zero_estimator_begin(&g_estimator);

    sample = make_sample(1U, 1000.0);
    CHECK(zero_estimator_add(&g_estimator, &sample));
    CHECK(!zero_estimator_add(&g_estimator, &sample));

    sample = make_sample(0U, 1000.0);
    CHECK(!zero_estimator_add(&g_estimator, &sample));

    sample = make_sample(2U, NAN);
    CHECK(!zero_estimator_add(&g_estimator, &sample));

    sample = make_sample(3U, 1000.0);
    sample.status_flags = BARO_STATUS_NOT_READY;
    CHECK(!zero_estimator_add(&g_estimator, &sample));

    sample = make_sample(4U, 1000.0);
    sample.status_flags = BARO_STATUS_RAW_INVALID;
    CHECK(!zero_estimator_add(&g_estimator, &sample));

    sample = make_sample(5U, 1000.0);
    sample.status_flags = BARO_STATUS_PRESSURE_INVALID;
    CHECK(!zero_estimator_add(&g_estimator, &sample));

    sample = make_sample(10U, 1000.0);
    CHECK(zero_estimator_add(&g_estimator, &sample));
    CHECK(g_estimator.valid_sample_count == 2U);
    CHECK(g_estimator.rejected_sample_count == 6U);

    zero_estimator_begin(&g_estimator);

    for (index = 1U; index <= ZERO_MAX_SAMPLES; ++index) {
        sample = make_sample(index, 1000.25);
        CHECK(zero_estimator_add(&g_estimator, &sample));
    }

    sample = make_sample(ZERO_MAX_SAMPLES + 1U, 1000.25);
    CHECK(!zero_estimator_add(&g_estimator, &sample));
    CHECK(g_estimator.valid_sample_count == ZERO_MAX_SAMPLES);
    CHECK(g_estimator.rejected_sample_count == 1U);
    CHECK(zero_estimator_finalize(&g_estimator));
    CHECK(fabs(g_estimator.surface_pressure_mbar - 1000.25) < 1e-9);
    return 0;
}

static int test_estimator_minimum_and_trim(void)
{
    BaroSample_t sample;
    unsigned int index;

    zero_estimator_begin(&g_estimator);

    for (index = 1U; index < ZERO_MIN_VALID_SAMPLES; ++index) {
        sample = make_sample(index, 1000.0);
        CHECK(zero_estimator_add(&g_estimator, &sample));
    }

    CHECK(!zero_estimator_finalize(&g_estimator));
    CHECK(isnan(g_estimator.surface_pressure_mbar));
    CHECK(!g_estimator.valid);

    /* 350 samples: 35 low and 35 high outliers are trimmed away. */
    zero_estimator_begin(&g_estimator);

    for (index = 0U; index < ZERO_MIN_VALID_SAMPLES; ++index) {
        double pressure = 1000.0;

        if (index < 35U) {
            pressure = -1000.0;
        } else if (index >= 315U) {
            pressure = 5000.0;
        }

        sample = make_sample(index + 1U, pressure);
        CHECK(zero_estimator_add(&g_estimator, &sample));
    }

    CHECK(zero_estimator_finalize(&g_estimator));
    CHECK(g_estimator.valid);
    CHECK(fabs(g_estimator.surface_pressure_mbar - 1000.0) < 1e-9);
    return 0;
}

static int test_capture_depth_policy(void)
{
    Ms5837PublicationRecord_t publication;
    CapturePolicy_t policy;
    double depth;
    double expected;

    memset(&publication, 0, sizeof(publication));
    memset(&policy, 0, sizeof(policy));

    publication.sample_ready = 1U;
    publication.sample.pressure_mbar = 1100.0;

    policy.surface_pressure_valid = true;
    policy.surface_pressure_mbar = 1000.0;
    policy.fluid_density_kg_m3 = 1000.0;

    expected = ms5837_depth_from_reference(1100.0, 1000.0, 1000.0);
    depth = capture_depth(&publication, &policy);

    CHECK(fabs(depth - expected) < 1e-12);
    CHECK((depth > 1.0) && (depth < 1.05));

    publication.sample_ready = 0U;
    CHECK(isnan(capture_depth(&publication, &policy)));

    publication.sample_ready = 1U;
    policy.surface_pressure_valid = false;
    CHECK(isnan(capture_depth(&publication, &policy)));

    CHECK(isnan(capture_depth(NULL, &policy)));
    CHECK(isnan(capture_depth(&publication, NULL)));
    return 0;
}

static int test_startup_retry(void)
{
    MockHal_t mock;
    Ms5837Hal_t hal;
    Ms5837Driver_t driver;
    uint64_t start_ns;

    g_now_ns = UINT64_C(5000000000);
    load_valid_prom(&mock);
    hal = make_hal(&mock);
    mock.fail_open_count = 2U;
    start_ns = g_now_ns;

    CHECK(initialize_with_retry(&driver, &hal) == MS5837_DRIVER_OK);
    CHECK(mock.open_count == 3U);
    CHECK((g_now_ns - start_ns) >= (2U * STARTUP_RETRY_NS));
    CHECK(ms5837_driver_is_initialized(&driver));
    (void)ms5837_driver_shutdown(&driver);

    load_valid_prom(&mock);
    hal = make_hal(&mock);
    mock.fail_open_count = STARTUP_MAX_ATTEMPTS;
    start_ns = g_now_ns;

    CHECK(initialize_with_retry(&driver, &hal) < 0);
    CHECK(mock.open_count == STARTUP_MAX_ATTEMPTS);
    CHECK((g_now_ns - start_ns) >= (2U * STARTUP_RETRY_NS));
    CHECK((g_now_ns - start_ns) < (3U * STARTUP_RETRY_NS));
    return 0;
}

static int test_zero_then_run(void)
{
    BaroSample_t latest;
    unsigned int lines;
    unsigned int zero_rows;
    unsigned int run_rows;
    uint64_t before_ns;

    CHECK(rig_start() == 0);
    before_ns = g_now_ns;

    CHECK(run_phase(BARO_CAPTURE_PHASE_ZERO, ZERO_DURATION_NS) == 0);

    /* 5000 service turns, and no sleep after the final gate. */
    CHECK((g_now_ns - before_ns) == (UINT64_C(4999) * TICK_NS));

    CHECK(g_run_stats.publications_attempted == UINT64_C(500));
    CHECK(g_run_stats.publications_enqueued == UINT64_C(500));
    CHECK(g_run_stats.publications_not_ready == 0U);
    CHECK((g_run_stats.publications_fresh +
           g_run_stats.publications_stale) == UINT64_C(500));
    CHECK(g_run_stats.deadline_misses == 0U);
    CHECK(g_run_stats.gates_skipped == 0U);
    CHECK(g_run_stats.logger_drops == 0U);

    CHECK(g_policy.zero.valid_sample_count >= ZERO_MIN_VALID_SAMPLES);
    CHECK(g_policy.zero.valid_sample_count <= ZERO_MAX_SAMPLES);
    CHECK(!g_policy.surface_pressure_valid);

    CHECK(zero_estimator_finalize(&g_policy.zero));
    g_policy.surface_pressure_mbar = g_policy.zero.surface_pressure_mbar;
    g_policy.surface_pressure_valid = true;

    CHECK(ms5837_driver_latest_sample(&g_driver, &latest));
    CHECK(fabs(g_policy.surface_pressure_mbar - latest.pressure_mbar) <
          1e-9);

    CHECK(run_phase(BARO_CAPTURE_PHASE_RUN, UINT64_C(1000000000)) == 0);

    CHECK(g_run_stats.publications_attempted == UINT64_C(600));
    CHECK(g_run_stats.publications_enqueued == UINT64_C(600));
    CHECK(g_run_stats.publications_not_ready == 0U);
    CHECK(g_run_stats.deadline_misses == 0U);
    CHECK(g_run_stats.gates_skipped == 0U);
    CHECK(g_run_stats.logger_drops == 0U);

    CHECK(rig_finish() == 0);
    CHECK(count_rows(&lines, &zero_rows, &run_rows) == 0);
    CHECK(lines == 601U);
    CHECK(zero_rows == 500U);
    CHECK(run_rows == 100U);
    return 0;
}

static int test_failed_zero_still_runs(void)
{
    unsigned int lines;
    unsigned int zero_rows;
    unsigned int run_rows;

    CHECK(rig_start() == 0);

    /* 0.2 s yields about 20 samples, far below the 350-sample minimum. */
    CHECK(run_phase(BARO_CAPTURE_PHASE_ZERO, UINT64_C(200000000)) == 0);
    CHECK(g_run_stats.publications_attempted == UINT64_C(20));
    CHECK(g_policy.zero.valid_sample_count < ZERO_MIN_VALID_SAMPLES);
    CHECK(!zero_estimator_finalize(&g_policy.zero));
    CHECK(isnan(g_policy.zero.surface_pressure_mbar));
    CHECK(!g_policy.surface_pressure_valid);

    CHECK(run_phase(BARO_CAPTURE_PHASE_RUN, UINT64_C(200000000)) == 0);
    CHECK(g_run_stats.publications_attempted == UINT64_C(40));
    CHECK(g_run_stats.logger_drops == 0U);

    CHECK(rig_finish() == 0);
    CHECK(count_rows(&lines, &zero_rows, &run_rows) == 0);
    CHECK(lines == 41U);
    CHECK(zero_rows == 20U);
    CHECK(run_rows == 20U);
    return 0;
}

static int test_recovery_keeps_loop_and_grid(void)
{
    unsigned int lines;
    unsigned int zero_rows;
    unsigned int run_rows;

    CHECK(rig_start() == 0);

    /* The first ADC read of the run's first pair fails. */
    g_mock.fail_next_read = 1U;

    CHECK(run_phase(BARO_CAPTURE_PHASE_RUN, UINT64_C(1000000000)) == 0);

    CHECK(g_driver_stats.i2c_errors >= 1U);
    CHECK(g_driver_stats.recovery_successes >= 1U);
    CHECK(ms5837_driver_state(&g_driver) != MS5837_STATE_STOPPED);

    /* The 1 kHz loop and 100 Hz grid never stalled during recovery. */
    CHECK(g_run_stats.publications_attempted == UINT64_C(100));
    CHECK(g_run_stats.publications_enqueued == UINT64_C(100));
    CHECK(g_run_stats.deadline_misses == 0U);
    CHECK(g_run_stats.gates_skipped == 0U);
    CHECK(g_run_stats.logger_drops == 0U);
    CHECK(g_run_stats.publications_fresh >= 1U);

    CHECK(rig_finish() == 0);
    CHECK(count_rows(&lines, &zero_rows, &run_rows) == 0);
    CHECK(lines == 101U);
    CHECK(run_rows == 100U);
    return 0;
}

static int test_stop_request(void)
{
    unsigned int lines;
    unsigned int zero_rows;
    unsigned int run_rows;

    CHECK(rig_start() == 0);

    request_stop(SIGINT);
    CHECK(stop_requested != 0);
    CHECK(run_phase(BARO_CAPTURE_PHASE_RUN, UINT64_C(1000000000)) ==
          -EINTR);
    CHECK(g_run_stats.publications_attempted == 0U);

    stop_requested = 0;

    CHECK(rig_finish() == 0);
    CHECK(count_rows(&lines, &zero_rows, &run_rows) == 0);
    CHECK(lines == 1U);
    return 0;
}

int main(void)
{
    CHECK(test_estimator_rejections_and_capacity() == 0);
    CHECK(test_estimator_minimum_and_trim() == 0);
    CHECK(test_capture_depth_policy() == 0);
    CHECK(test_startup_retry() == 0);
    CHECK(test_zero_then_run() == 0);
    CHECK(test_failed_zero_still_runs() == 0);
    CHECK(test_recovery_keeps_loop_and_grid() == 0);
    CHECK(test_stop_request() == 0);

    puts("PASS: MS5837 capture owner loop, zero stage, and recovery");
    return 0;
}
