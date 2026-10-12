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
    unsigned int fail_open_count;
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

static int initialize(MockHal_t *mock, Ms5837Driver_t *driver)
{
    Ms5837Hal_t hal = make_hal(mock);
    return ms5837_driver_init(driver, &hal);
}

static int complete_pair(MockHal_t *mock, Ms5837Driver_t *driver,
                         BaroDriverStats_t *stats, BaroSample_t *sample)
{
    int status;

    status = ms5837_driver_trigger(driver, stats);
    CHECK(status == MS5837_SERVICE_PROGRESS);
    CHECK(ms5837_driver_state(driver) == MS5837_STATE_WAIT_D1);
    CHECK(mock->selected_command ==
          (MS5837_COMMAND_CONVERT_D1_BASE | MS5837_OSR_512));

    status = ms5837_driver_service(driver, driver->deadline_ns - 1U,
                                    stats, sample);
    CHECK(status == MS5837_SERVICE_PROGRESS);
    mock->time_ns = driver->deadline_ns;
    status = ms5837_driver_service(driver, mock->time_ns, stats, sample);
    CHECK(status == MS5837_SERVICE_PROGRESS);
    CHECK(ms5837_driver_state(driver) == MS5837_STATE_START_D2);

    status = ms5837_driver_service(driver, mock->time_ns, stats, sample);
    CHECK(status == MS5837_SERVICE_PROGRESS);
    CHECK(ms5837_driver_state(driver) == MS5837_STATE_WAIT_D2);
    CHECK(mock->selected_command ==
          (MS5837_COMMAND_CONVERT_D2_BASE | MS5837_OSR_512));

    mock->time_ns = driver->deadline_ns;
    status = ms5837_driver_service(driver, mock->time_ns, stats, sample);
    CHECK(status == MS5837_SERVICE_SAMPLE_READY);
    return 0;
}

static int drive_recovery(MockHal_t *mock, Ms5837Driver_t *driver,
                          BaroDriverStats_t *stats, BaroSample_t *sample)
{
    size_t guard;

    for (guard = 0U; guard < 128U; ++guard) {
        int status;
        if (ms5837_driver_state(driver) == MS5837_STATE_RECOVERY_BACKOFF) {
            mock->time_ns = driver->recovery_retry_deadline_ns;
        } else if (ms5837_driver_state(driver) ==
                   MS5837_STATE_RECOVERY_WAIT_RESET) {
            mock->time_ns = driver->deadline_ns;
        }
        status = ms5837_driver_service(driver, mock->time_ns, stats, sample);
        if (ms5837_driver_state(driver) == MS5837_STATE_IDLE) {
            CHECK(status == MS5837_SERVICE_RECOVERING);
            return 0;
        }
        if (ms5837_driver_state(driver) == MS5837_STATE_STOPPED) {
            return status;
        }
        CHECK((status == MS5837_SERVICE_RECOVERING) || (status < 0));
    }
    return -ETIMEDOUT;
}

static int test_raw_contract_pair(void)
{
    MockHal_t mock;
    Ms5837Driver_t driver;
    BaroDriverStats_t stats;
    BaroSample_t sample;
    BaroSample_t latest;

    memset(&stats, 0, sizeof(stats));
    load_valid_prom(&mock);
    CHECK(initialize(&mock, &driver) == MS5837_DRIVER_OK);
    mock.conversion_write_count = 0U;
    mock.adc_read_count = 0U;

    CHECK(complete_pair(&mock, &driver, &stats, &sample) == 0);
    CHECK(sample.contract_version == 2U);
    CHECK(sample.measurement_sequence == 1U);
    CHECK(sample.raw_pressure_d1 == mock.d1);
    CHECK(sample.raw_temperature_d2 == mock.d2);
    CHECK(fabs(sample.pressure_mbar - 1071.05) < 0.01);
    CHECK(fabs(sample.temperature_c - 24.30) < 0.01);
    CHECK((sample.status_flags & BARO_STATUS_RESERVED_MASK) == 0U);
    CHECK(stats.measurements_completed == 1U);
    CHECK(stats.service_ticks == 4U);
    CHECK(ms5837_driver_latest_sample(&driver, &latest));
    CHECK(latest.measurement_sequence == 1U);
    CHECK(ms5837_driver_shutdown(&driver) == 0);
    return 0;
}

static int test_raw_invalid_record(void)
{
    MockHal_t mock;
    Ms5837Driver_t driver;
    BaroDriverStats_t stats;
    BaroSample_t sample;

    memset(&stats, 0, sizeof(stats));
    load_valid_prom(&mock);
    mock.d1 = 0U;
    CHECK(initialize(&mock, &driver) == MS5837_DRIVER_OK);
    mock.conversion_write_count = 0U;
    CHECK(complete_pair(&mock, &driver, &stats, &sample) == 0);
    CHECK((sample.status_flags & BARO_STATUS_RAW_INVALID) != 0U);
    CHECK((sample.status_flags & BARO_STATUS_RESERVED_MASK) == 0U);
    CHECK(isnan(sample.pressure_mbar));
    CHECK(isnan(sample.temperature_c));
    CHECK(ms5837_driver_shutdown(&driver) == 0);
    return 0;
}

static int test_recovery_success(void)
{
    MockHal_t mock;
    Ms5837Driver_t driver;
    BaroDriverStats_t stats;
    BaroSample_t sample;

    memset(&stats, 0, sizeof(stats));
    load_valid_prom(&mock);
    CHECK(initialize(&mock, &driver) == MS5837_DRIVER_OK);
    mock.conversion_write_count = 0U;
    CHECK(ms5837_driver_trigger(&driver, &stats) ==
          MS5837_SERVICE_PROGRESS);
    mock.fail_next_read = 1U;
    mock.time_ns = driver.deadline_ns;
    CHECK(ms5837_driver_service(&driver, mock.time_ns, &stats, &sample) ==
          -EIO);
    CHECK(ms5837_driver_state(&driver) == MS5837_STATE_RECOVERY_OPEN);
    CHECK(!ms5837_driver_is_initialized(&driver));
    CHECK(stats.recovery_incidents == 1U);
    CHECK(stats.aborted_pairs == 1U);

    CHECK(drive_recovery(&mock, &driver, &stats, &sample) == 0);
    CHECK(ms5837_driver_is_initialized(&driver));
    CHECK(stats.recovery_attempts == 1U);
    CHECK(stats.recovery_successes == 1U);
    CHECK(stats.recovery_exhaustions == 0U);
    CHECK(complete_pair(&mock, &driver, &stats, &sample) == 0);
    CHECK(sample.measurement_sequence == 1U);
    CHECK(ms5837_driver_shutdown(&driver) == 0);
    return 0;
}

static int test_recovery_exhaustion(void)
{
    MockHal_t mock;
    Ms5837Driver_t driver;
    BaroDriverStats_t stats;
    BaroSample_t sample;

    memset(&stats, 0, sizeof(stats));
    load_valid_prom(&mock);
    CHECK(initialize(&mock, &driver) == MS5837_DRIVER_OK);
    mock.conversion_write_count = 0U;
    CHECK(ms5837_driver_trigger(&driver, &stats) ==
          MS5837_SERVICE_PROGRESS);
    mock.fail_next_read = 1U;
    mock.time_ns = driver.deadline_ns;
    CHECK(ms5837_driver_service(&driver, mock.time_ns, &stats, &sample) ==
          -EIO);
    mock.fail_open_count = MS5837_RECOVERY_MAX_ATTEMPTS;
    CHECK(drive_recovery(&mock, &driver, &stats, &sample) == -EIO);
    CHECK(ms5837_driver_state(&driver) == MS5837_STATE_STOPPED);
    CHECK(stats.recovery_incidents == 1U);
    CHECK(stats.recovery_attempts == MS5837_RECOVERY_MAX_ATTEMPTS);
    CHECK(stats.recovery_successes == 0U);
    CHECK(stats.recovery_exhaustions == 1U);
    CHECK(ms5837_driver_trigger(&driver, &stats) ==
          MS5837_DRIVER_ERR_NOT_INITIALIZED);
    CHECK(ms5837_driver_shutdown(&driver) == 0);
    return 0;
}

static int test_recovery_backoff_nonblocking(void)
{
    MockHal_t mock;
    Ms5837Driver_t driver;
    BaroDriverStats_t stats;
    BaroSample_t sample;
    uint64_t failure_time_ns;
    unsigned int open_count;

    memset(&stats, 0, sizeof(stats));
    load_valid_prom(&mock);
    CHECK(initialize(&mock, &driver) == MS5837_DRIVER_OK);
    mock.conversion_write_count = 0U;
    CHECK(ms5837_driver_trigger(&driver, &stats) ==
          MS5837_SERVICE_PROGRESS);
    mock.fail_next_read = 1U;
    mock.time_ns = driver.deadline_ns;
    CHECK(ms5837_driver_service(&driver, mock.time_ns, &stats, &sample) ==
          -EIO);
    mock.fail_open_count = 1U;
    failure_time_ns = mock.time_ns;
    CHECK(ms5837_driver_service(&driver, failure_time_ns, &stats, &sample) ==
          -EIO);
    CHECK(ms5837_driver_state(&driver) == MS5837_STATE_RECOVERY_BACKOFF);
    CHECK(driver.recovery_retry_deadline_ns ==
          failure_time_ns + MS5837_RECOVERY_RETRY_NS);
    CHECK(ms5837_driver_trigger(&driver, &stats) == MS5837_DRIVER_ERR_BUSY);
    CHECK(!ms5837_driver_latest_sample(&driver, &sample));

    open_count = mock.open_count;
    CHECK(ms5837_driver_service(&driver,
                                driver.recovery_retry_deadline_ns - 1U,
                                &stats, &sample) ==
          MS5837_SERVICE_RECOVERING);
    CHECK(mock.open_count == open_count);
    CHECK(ms5837_driver_state(&driver) == MS5837_STATE_RECOVERY_BACKOFF);

    mock.time_ns = driver.recovery_retry_deadline_ns;
    CHECK(drive_recovery(&mock, &driver, &stats, &sample) == 0);
    CHECK(stats.recovery_incidents == 1U);
    CHECK(stats.recovery_attempts == 2U);
    CHECK(stats.recovery_successes == 1U);

    CHECK(ms5837_driver_trigger(&driver, &stats) ==
          MS5837_SERVICE_PROGRESS);
    mock.fail_next_read = 1U;
    mock.time_ns = driver.deadline_ns;
    CHECK(ms5837_driver_service(&driver, mock.time_ns, &stats, &sample) ==
          -EIO);
    CHECK(drive_recovery(&mock, &driver, &stats, &sample) == 0);
    CHECK(stats.recovery_incidents == 2U);
    CHECK(stats.recovery_attempts == 3U);
    CHECK(ms5837_driver_shutdown(&driver) == 0);
    return 0;
}

static int enter_state(MockHal_t *mock, Ms5837Driver_t *driver,
                       BaroDriverStats_t *stats, Ms5837DriverState_t target)
{
    BaroSample_t sample;
    size_t guard;

    if (target == MS5837_STATE_IDLE) {
        return 0;
    }
    CHECK(ms5837_driver_trigger(driver, stats) == MS5837_SERVICE_PROGRESS);
    if (target == MS5837_STATE_WAIT_D1) {
        return 0;
    }
    if ((target == MS5837_STATE_START_D2) ||
        (target == MS5837_STATE_WAIT_D2)) {
        mock->time_ns = driver->deadline_ns;
        CHECK(ms5837_driver_service(driver, mock->time_ns, stats, &sample) ==
              MS5837_SERVICE_PROGRESS);
        if (target == MS5837_STATE_WAIT_D2) {
            CHECK(ms5837_driver_service(driver, mock->time_ns, stats,
                                        &sample) == MS5837_SERVICE_PROGRESS);
        }
        CHECK(ms5837_driver_state(driver) == target);
        return 0;
    }
    if (target == MS5837_STATE_RECOVERY_BACKOFF) {
        mock->fail_open_count = 1U;
    }
    mock->fail_next_read = 1U;
    mock->time_ns = driver->deadline_ns;
    CHECK(ms5837_driver_service(driver, mock->time_ns, stats, &sample) ==
          -EIO);
    for (guard = 0U; guard < 64U; ++guard) {
        if (ms5837_driver_state(driver) == target) {
            return 0;
        }
        if (ms5837_driver_state(driver) == MS5837_STATE_RECOVERY_WAIT_RESET) {
            mock->time_ns = driver->deadline_ns;
        }
        (void)ms5837_driver_service(driver, mock->time_ns, stats, &sample);
    }
    return -ETIMEDOUT;
}

static int test_shutdown_from_every_state(void)
{
    static const Ms5837DriverState_t states[] = {
        MS5837_STATE_IDLE,
        MS5837_STATE_WAIT_D1,
        MS5837_STATE_START_D2,
        MS5837_STATE_WAIT_D2,
        MS5837_STATE_RECOVERY_BACKOFF,
        MS5837_STATE_RECOVERY_OPEN,
        MS5837_STATE_RECOVERY_RESET,
        MS5837_STATE_RECOVERY_WAIT_RESET,
        MS5837_STATE_RECOVERY_PROM,
        MS5837_STATE_RECOVERY_VALIDATE
    };
    size_t index;

    for (index = 0U; index < (sizeof(states) / sizeof(states[0])); ++index) {
        MockHal_t mock;
        Ms5837Driver_t driver;
        BaroDriverStats_t stats;
        BaroSample_t sample;
        unsigned int closes_before;
        unsigned int opens_before;
        bool was_open;

        memset(&stats, 0, sizeof(stats));
        load_valid_prom(&mock);
        CHECK(initialize(&mock, &driver) == MS5837_DRIVER_OK);
        mock.conversion_write_count = 0U;
        CHECK(enter_state(&mock, &driver, &stats, states[index]) == 0);
        CHECK(ms5837_driver_state(&driver) == states[index]);

        was_open = driver.hal_open;
        closes_before = mock.close_count;
        CHECK(ms5837_driver_shutdown(&driver) == 0);
        CHECK(ms5837_driver_state(&driver) == MS5837_STATE_STOPPED);
        CHECK(!ms5837_driver_is_initialized(&driver));
        CHECK(!driver.hal_open);
        CHECK(!driver.recovery_incident_active);
        CHECK(mock.close_count == closes_before + (was_open ? 1U : 0U));

        opens_before = mock.open_count;
        CHECK(ms5837_driver_trigger(&driver, &stats) ==
              MS5837_DRIVER_ERR_NOT_INITIALIZED);
        CHECK(ms5837_driver_service(&driver, mock.time_ns, &stats,
                                    &sample) < 0);
        CHECK(mock.open_count == opens_before);

        closes_before = mock.close_count;
        CHECK(ms5837_driver_shutdown(&driver) == 0);
        CHECK(mock.close_count == closes_before);
    }
    return 0;
}

static int test_recovery_fresh_allowance_per_incident(void)
{
    MockHal_t mock;
    Ms5837Driver_t driver;
    BaroDriverStats_t stats;
    BaroSample_t sample;

    memset(&stats, 0, sizeof(stats));
    load_valid_prom(&mock);
    CHECK(initialize(&mock, &driver) == MS5837_DRIVER_OK);
    mock.conversion_write_count = 0U;

    /* Incident 1 succeeds on its last permitted attempt. */
    CHECK(ms5837_driver_trigger(&driver, &stats) == MS5837_SERVICE_PROGRESS);
    mock.fail_next_read = 1U;
    mock.time_ns = driver.deadline_ns;
    CHECK(ms5837_driver_service(&driver, mock.time_ns, &stats, &sample) ==
          -EIO);
    mock.fail_open_count = MS5837_RECOVERY_MAX_ATTEMPTS - 1U;
    CHECK(drive_recovery(&mock, &driver, &stats, &sample) == 0);
    CHECK(stats.recovery_incidents == 1U);
    CHECK(stats.recovery_attempts == MS5837_RECOVERY_MAX_ATTEMPTS);
    CHECK(stats.recovery_successes == 1U);
    CHECK(stats.recovery_exhaustions == 0U);

    /* Incident 2 must get a full new allowance, then exhaust it exactly. */
    CHECK(ms5837_driver_trigger(&driver, &stats) == MS5837_SERVICE_PROGRESS);
    mock.fail_next_read = 1U;
    mock.time_ns = driver.deadline_ns;
    CHECK(ms5837_driver_service(&driver, mock.time_ns, &stats, &sample) ==
          -EIO);
    mock.fail_open_count = MS5837_RECOVERY_MAX_ATTEMPTS;
    CHECK(drive_recovery(&mock, &driver, &stats, &sample) == -EIO);
    CHECK(ms5837_driver_state(&driver) == MS5837_STATE_STOPPED);
    CHECK(stats.recovery_incidents == 2U);
    CHECK(stats.recovery_attempts == (2U * MS5837_RECOVERY_MAX_ATTEMPTS));
    CHECK(stats.recovery_successes == 1U);
    CHECK(stats.recovery_exhaustions == 1U);
    CHECK(ms5837_driver_shutdown(&driver) == 0);
    return 0;
}

int main(void)
{
    CHECK(test_raw_contract_pair() == 0);
    CHECK(test_raw_invalid_record() == 0);
    CHECK(test_recovery_success() == 0);
    CHECK(test_recovery_exhaustion() == 0);
    CHECK(test_recovery_backoff_nonblocking() == 0);
    CHECK(test_shutdown_from_every_state() == 0);
    CHECK(test_recovery_fresh_allowance_per_incident() == 0);
    puts("PASS: Phase 1 raw contract and bounded recovery");
    return 0;
}
