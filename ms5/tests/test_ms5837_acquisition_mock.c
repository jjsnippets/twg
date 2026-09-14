#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ms5837_driver.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

#define MOCK_WRITE_TIME_NS UINT64_C(25000)
#define MOCK_READ_TIME_NS UINT64_C(30000)

typedef struct {
    uint16_t prom[MS5837_PROM_WORD_COUNT];
    uint32_t d1;
    uint32_t d2;
    uint64_t time_ns;
    unsigned int open_count;
    unsigned int close_count;
    unsigned int conversion_write_count;
    unsigned int adc_read_count;
    unsigned int fail_next_read;
    uint8_t selected_command;
    size_t prom_index;
} MockHal_t;

static void load_valid_prom(MockHal_t *mock)
{
    const uint16_t valid[MS5837_PROM_WORD_COUNT] = {
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
    mock->time_ns += MOCK_WRITE_TIME_NS;
    mock->selected_command = data[0];
    if (data[0] == MS5837_COMMAND_RESET) {
        mock->prom_index = 0U;
    } else if ((data[0] == (MS5837_COMMAND_CONVERT_D1_BASE |
                            MS5837_OSR_512)) ||
               (data[0] == (MS5837_COMMAND_CONVERT_D2_BASE |
                            MS5837_OSR_512))) {
        ++mock->conversion_write_count;
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
    mock->time_ns += MOCK_READ_TIME_NS;
    if (mock->fail_next_read != 0U) {
        mock->fail_next_read = 0U;
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

    raw = ((mock->conversion_write_count & 1U) != 0U)
        ? mock->d1 : mock->d2;
    data[0] = (uint8_t)(raw >> 16U);
    data[1] = (uint8_t)(raw >> 8U);
    data[2] = (uint8_t)raw;
    ++mock->adc_read_count;
    return 0;
}

static int mock_sleep_ns(void *opaque, uint64_t duration_ns)
{
    MockHal_t *mock = (MockHal_t *)opaque;
    mock->time_ns += duration_ns;
    return 0;
}

static int mock_time_ns(void *opaque, uint64_t *time_ns)
{
    MockHal_t *mock = (MockHal_t *)opaque;
    if (time_ns == NULL) {
        return -EINVAL;
    }
    *time_ns = mock->time_ns;
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

static BaroRunConfig_t make_config(void)
{
    BaroRunConfig_t config;
    memset(&config, 0, sizeof(config));
    config.config_version = BARO_RUN_CONFIG_VERSION;
    config.fluid_density_kg_m3 = 997.0;
    config.surface_pressure_mbar = 1100.02;
    config.surface_pressure_valid = true;
    return config;
}

static int initialize(MockHal_t *mock, Ms5837Driver_t *driver)
{
    Ms5837Hal_t hal = make_hal(mock);
    return ms5837_driver_init(driver, &hal);
}

static int complete_pair(MockHal_t *mock, Ms5837Driver_t *driver,
                         const BaroRunConfig_t *config,
                         BaroRuntimeStats_t *stats, BaroSample_t *sample)
{
    int status;
    unsigned int adc_read_count = mock->adc_read_count;

    status = ms5837_driver_service(driver, mock->time_ns, config, stats,
                                    sample);
    CHECK(status == MS5837_SERVICE_PROGRESS);
    CHECK(ms5837_driver_state(driver) == MS5837_STATE_WAIT_D1);
    CHECK(mock->selected_command ==
          (MS5837_COMMAND_CONVERT_D1_BASE | MS5837_OSR_512));

    status = ms5837_driver_service(driver, driver->deadline_ns - 1U,
                                    config, stats, sample);
    CHECK(status == MS5837_SERVICE_PROGRESS);
    CHECK(mock->adc_read_count == adc_read_count);

    mock->time_ns = driver->deadline_ns;
    status = ms5837_driver_service(driver, mock->time_ns, config, stats,
                                    sample);
    CHECK(status == MS5837_SERVICE_PROGRESS);
    CHECK(ms5837_driver_state(driver) == MS5837_STATE_START_D2);

    status = ms5837_driver_service(driver, mock->time_ns, config, stats,
                                    sample);
    CHECK(status == MS5837_SERVICE_PROGRESS);
    CHECK(ms5837_driver_state(driver) == MS5837_STATE_WAIT_D2);
    CHECK(mock->selected_command ==
          (MS5837_COMMAND_CONVERT_D2_BASE | MS5837_OSR_512));

    mock->time_ns = driver->deadline_ns;
    status = ms5837_driver_service(driver, mock->time_ns, config, stats,
                                    sample);
    CHECK(status == MS5837_SERVICE_SAMPLE_READY);
    return 0;
}

static int test_symmetric_osr512_pair(void)
{
    MockHal_t mock;
    Ms5837Driver_t driver;
    BaroRunConfig_t config = make_config();
    BaroRuntimeStats_t stats;
    BaroSample_t sample;
    BaroSample_t latest;
    uint64_t deadline;

    memset(&stats, 0, sizeof(stats));
    load_valid_prom(&mock);
    CHECK(initialize(&mock, &driver) == MS5837_DRIVER_OK);
    mock.conversion_write_count = 0U;
    mock.adc_read_count = 0U;

    CHECK(ms5837_driver_service(&driver, mock.time_ns, &config, &stats,
                                 &sample) == MS5837_SERVICE_PROGRESS);
    deadline = mock.time_ns + MS5837_OSR_512_CONVERSION_NS;
    CHECK(driver.deadline_ns == deadline);
    CHECK(mock.selected_command == 0x42U);

    CHECK(ms5837_driver_service(&driver, deadline - 1U, &config, &stats,
                                 &sample) == MS5837_SERVICE_PROGRESS);
    CHECK(mock.adc_read_count == 0U);
    mock.time_ns = deadline;
    CHECK(ms5837_driver_service(&driver, mock.time_ns, &config, &stats,
                                 &sample) == MS5837_SERVICE_PROGRESS);
    CHECK(mock.adc_read_count == 1U);

    CHECK(ms5837_driver_service(&driver, mock.time_ns, &config, &stats,
                                 &sample) == MS5837_SERVICE_PROGRESS);
    CHECK(mock.selected_command == 0x52U);
    deadline = mock.time_ns + MS5837_OSR_512_CONVERSION_NS;
    CHECK(driver.deadline_ns == deadline);

    mock.time_ns = deadline;
    CHECK(ms5837_driver_service(&driver, mock.time_ns, &config, &stats,
                                 &sample) == MS5837_SERVICE_SAMPLE_READY);
    CHECK(sample.measurement_sequence == 1U);
    CHECK(sample.measurement_complete_time_ns == mock.time_ns);
    CHECK(sample.raw_pressure_d1 == mock.d1);
    CHECK(sample.raw_temperature_d2 == mock.d2);
    CHECK(fabs(sample.pressure_mbar - 1071.05) < 0.01);
    CHECK(fabs(sample.temperature_c - 24.30) < 0.01);
    CHECK(isfinite(sample.depth_m));
    CHECK(stats.measurements_completed == 1U);
    CHECK(ms5837_driver_latest_sample(&driver, &latest));
    CHECK(latest.measurement_sequence == sample.measurement_sequence);

    CHECK(complete_pair(&mock, &driver, &config, &stats, &sample) == 0);
    CHECK(sample.measurement_sequence == 2U);
    CHECK(ms5837_driver_shutdown(&driver) == 0);
    return 0;
}

static int drive_recovery(MockHal_t *mock, Ms5837Driver_t *driver,
                          const BaroRunConfig_t *config,
                          BaroRuntimeStats_t *stats, BaroSample_t *sample)
{
    size_t guard;

    for (guard = 0U; guard < 32U; ++guard) {
        int status;
        if (ms5837_driver_state(driver) == MS5837_STATE_RECOVERY_WAIT_RESET) {
            mock->time_ns = driver->deadline_ns;
        }
        status = ms5837_driver_service(driver, mock->time_ns, config, stats,
                                        sample);
        CHECK(status == MS5837_SERVICE_RECOVERING);
        if (ms5837_driver_state(driver) == MS5837_STATE_START_D1) {
            return 0;
        }
    }
    return 1;
}

static int test_error_aborts_pair_and_recovers(void)
{
    MockHal_t mock;
    Ms5837Driver_t driver;
    BaroRunConfig_t config = make_config();
    BaroRuntimeStats_t stats;
    BaroSample_t sample;

    memset(&stats, 0, sizeof(stats));
    load_valid_prom(&mock);
    CHECK(initialize(&mock, &driver) == MS5837_DRIVER_OK);
    mock.conversion_write_count = 0U;
    mock.adc_read_count = 0U;

    CHECK(ms5837_driver_service(&driver, mock.time_ns, &config, &stats,
                                 &sample) == MS5837_SERVICE_PROGRESS);
    mock.fail_next_read = 1U;
    mock.time_ns = driver.deadline_ns;
    CHECK(ms5837_driver_service(&driver, mock.time_ns, &config, &stats,
                                 &sample) == -EIO);
    CHECK(ms5837_driver_state(&driver) == MS5837_STATE_RECOVERY_OPEN);
    CHECK(!ms5837_driver_is_initialized(&driver));
    CHECK(stats.i2c_errors == 1U);
    CHECK(stats.aborted_pairs == 1U);
    CHECK(stats.measurements_completed == 0U);
    CHECK(mock.close_count == 1U);

    CHECK(drive_recovery(&mock, &driver, &config, &stats, &sample) == 0);
    CHECK(ms5837_driver_is_initialized(&driver));
    CHECK(stats.recovery_attempts == 1U);
    CHECK(stats.recovery_successes == 1U);
    CHECK(mock.open_count == 2U);

    mock.conversion_write_count = 0U;
    mock.adc_read_count = 0U;
    CHECK(complete_pair(&mock, &driver, &config, &stats, &sample) == 0);
    CHECK(sample.measurement_sequence == 1U);
    CHECK(stats.measurements_completed == 1U);
    CHECK(ms5837_driver_shutdown(&driver) == 0);
    return 0;
}

static int test_raw_invalid_still_completes_record(void)
{
    MockHal_t mock;
    Ms5837Driver_t driver;
    BaroRunConfig_t config = make_config();
    BaroRuntimeStats_t stats;
    BaroSample_t sample;

    memset(&stats, 0, sizeof(stats));
    load_valid_prom(&mock);
    mock.d1 = 0U;
    CHECK(initialize(&mock, &driver) == MS5837_DRIVER_OK);
    mock.conversion_write_count = 0U;
    mock.adc_read_count = 0U;

    CHECK(complete_pair(&mock, &driver, &config, &stats, &sample) == 0);
    CHECK((sample.status_flags & BARO_STATUS_RAW_INVALID) != 0U);
    CHECK(isnan(sample.pressure_mbar));
    CHECK(isnan(sample.temperature_c));
    CHECK(isnan(sample.depth_m));
    CHECK(sample.measurement_sequence == 1U);
    CHECK(stats.measurements_completed == 1U);
    CHECK(ms5837_driver_shutdown(&driver) == 0);
    return 0;
}

static int test_depth_reference_flags(void)
{
    MockHal_t mock;
    Ms5837Driver_t driver;
    BaroRunConfig_t config = make_config();
    BaroRuntimeStats_t stats;
    BaroSample_t sample;

    memset(&stats, 0, sizeof(stats));
    load_valid_prom(&mock);
    config.surface_pressure_valid = false;
    config.zero_at_start_requested = true;
    CHECK(initialize(&mock, &driver) == MS5837_DRIVER_OK);
    mock.conversion_write_count = 0U;
    mock.adc_read_count = 0U;
    CHECK(complete_pair(&mock, &driver, &config, &stats, &sample) == 0);
    CHECK((sample.status_flags & BARO_STATUS_DEPTH_REFERENCE_PENDING) != 0U);
    CHECK(isnan(sample.depth_m));
    CHECK(ms5837_driver_shutdown(&driver) == 0);
    return 0;
}

int main(void)
{
    CHECK(test_symmetric_osr512_pair() == 0);
    CHECK(test_error_aborts_pair_and_recovers() == 0);
    CHECK(test_raw_invalid_still_completes_record() == 0);
    CHECK(test_depth_reference_flags() == 0);
    puts("PASS: Step 4C symmetric OSR-512 acquisition and recovery");
    return 0;
}
