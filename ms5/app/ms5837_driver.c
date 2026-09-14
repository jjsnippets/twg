#include "ms5837_driver.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static bool hal_contract_valid(const Ms5837Hal_t *hal)
{
    return (hal != NULL) && (hal->context != NULL) &&
           (hal->open != NULL) && (hal->close != NULL) &&
           (hal->write != NULL) && (hal->read != NULL) &&
           (hal->sleep_ns != NULL) && (hal->time_ns != NULL);
}

static bool prom_coefficients_valid(const Ms5837Prom_t *prom)
{
    size_t index;
    bool all_zero = true;
    bool all_ones = true;

    for (index = 1U; index < MS5837_PROM_WORD_COUNT; ++index) {
        if (prom->word[index] != 0U) {
            all_zero = false;
        }
        if (prom->word[index] != 0xFFFFU) {
            all_ones = false;
        }
    }
    return !all_zero && !all_ones;
}

static int fail_and_close(Ms5837Driver_t *driver, int error)
{
    if (driver->hal_open) {
        (void)driver->hal.close(driver->hal.context);
        driver->hal_open = false;
    }
    driver->initialized = false;
    driver->pair_in_progress = false;
    driver->state = MS5837_STATE_STOPPED;
    driver->last_error = error;
    return error;
}

static int enter_recovery(Ms5837Driver_t *driver, int error,
                          BaroRuntimeStats_t *stats, bool transport_error)
{
    if (transport_error) {
        ++stats->i2c_errors;
    }
    if (driver->pair_in_progress) {
        ++stats->aborted_pairs;
    }
    driver->pair_in_progress = false;

    if (driver->hal_open) {
        (void)driver->hal.close(driver->hal.context);
        driver->hal_open = false;
    }
    driver->initialized = false;
    driver->recovery_prom_index = 0U;
    memset(&driver->recovery_prom, 0, sizeof(driver->recovery_prom));
    driver->state = MS5837_STATE_RECOVERY_OPEN;
    driver->last_error = error;
    return error;
}

static int write_command(Ms5837Driver_t *driver, uint8_t command)
{
    return driver->hal.write(driver->hal.context, &command, 1U);
}

static int read_adc(Ms5837Driver_t *driver, uint32_t *value)
{
    uint8_t response[3];
    int status;

    status = write_command(driver, MS5837_COMMAND_ADC_READ);
    if (status < 0) {
        return status;
    }
    status = driver->hal.read(driver->hal.context, response,
                              sizeof(response));
    if (status < 0) {
        return status;
    }
    *value = ((uint32_t)response[0] << 16U) |
             ((uint32_t)response[1] << 8U) |
             (uint32_t)response[2];
    return 0;
}

static void set_depth(BaroSample_t *sample, const BaroRunConfig_t *config)
{
    if ((sample->status_flags & (uint32_t)(BARO_STATUS_RAW_INVALID |
                                           BARO_STATUS_PRESSURE_INVALID)) != 0U) {
        sample->depth_m = NAN;
        return;
    }
    if (!config->surface_pressure_valid) {
        sample->depth_m = NAN;
        sample->status_flags |= config->zero_at_start_requested
            ? (uint32_t)BARO_STATUS_DEPTH_REFERENCE_PENDING
            : (uint32_t)BARO_STATUS_DEPTH_REFERENCE_INVALID;
        return;
    }

    sample->depth_m = ms5837_depth_from_reference(
        sample->pressure_mbar, config->surface_pressure_mbar,
        config->fluid_density_kg_m3);
    if (!isfinite(sample->depth_m)) {
        sample->status_flags |= BARO_STATUS_DEPTH_REFERENCE_INVALID;
    }
}

static void build_sample(Ms5837Driver_t *driver, uint32_t raw_d2,
                         uint64_t complete_time_ns,
                         const BaroRunConfig_t *config,
                         BaroRuntimeStats_t *stats,
                         BaroSample_t *sample)
{
    Ms5837Compensated_t compensated;
    Ms5837MathResult_t math_status;

    memset(sample, 0, sizeof(*sample));
    sample->contract_version = BARO_SAMPLE_CONTRACT_VERSION;
    sample->measurement_sequence = ++driver->measurement_sequence;
    sample->measurement_complete_time_ns = complete_time_ns;
    sample->raw_pressure_d1 = driver->raw_pressure_d1;
    sample->raw_temperature_d2 = raw_d2;
    sample->pressure_mbar = NAN;
    sample->temperature_c = NAN;
    sample->depth_m = NAN;

    math_status = ms5837_compensate(&driver->prom,
                                    driver->raw_pressure_d1,
                                    raw_d2, &compensated);
    if (math_status != MS5837_MATH_OK) {
        sample->status_flags = BARO_STATUS_RAW_INVALID;
    } else {
        sample->pressure_mbar = compensated.pressure_mbar;
        sample->temperature_c = compensated.temperature_c;
        sample->status_flags = ms5837_classify_measurement(
            sample->pressure_mbar, sample->temperature_c);
    }
    set_depth(sample, config);

    ++stats->measurements_completed;
    if ((sample->status_flags & BARO_STATUS_PRESSURE_EXTENDED) != 0U) {
        ++stats->pressure_extended_count;
    }
    if ((sample->status_flags & BARO_STATUS_PRESSURE_INVALID) != 0U) {
        ++stats->pressure_invalid_count;
    }
    if ((sample->status_flags & BARO_STATUS_TEMPERATURE_EXTENDED) != 0U) {
        ++stats->temperature_extended_count;
    }
    driver->latest_sample = *sample;
    driver->has_latest_sample = true;
}

static int start_conversion(Ms5837Driver_t *driver, uint8_t command,
                            Ms5837DriverState_t wait_state,
                            BaroRuntimeStats_t *stats)
{
    uint64_t command_complete_ns;
    int status;

    status = write_command(driver, command);
    if (status < 0) {
        return enter_recovery(driver, status, stats, true);
    }
    status = driver->hal.time_ns(driver->hal.context, &command_complete_ns);
    if (status < 0) {
        return enter_recovery(driver, status, stats, true);
    }
    driver->deadline_ns = command_complete_ns +
                          MS5837_OSR_512_CONVERSION_NS;
    driver->state = wait_state;
    return MS5837_SERVICE_PROGRESS;
}

static int service_recovery(Ms5837Driver_t *driver, uint64_t now_ns,
                            BaroRuntimeStats_t *stats)
{
    uint8_t command;
    uint8_t response[2];
    uint64_t command_complete_ns;
    int status;

    switch (driver->state) {
    case MS5837_STATE_RECOVERY_OPEN:
        ++stats->recovery_attempts;
        status = driver->hal.open(driver->hal.context);
        if (status < 0) {
            driver->last_error = status;
            ++stats->i2c_errors;
            return status;
        }
        driver->hal_open = true;
        driver->state = MS5837_STATE_RECOVERY_RESET;
        return MS5837_SERVICE_RECOVERING;

    case MS5837_STATE_RECOVERY_RESET:
        status = write_command(driver, MS5837_COMMAND_RESET);
        if (status < 0) {
            return enter_recovery(driver, status, stats, true);
        }
        status = driver->hal.time_ns(driver->hal.context,
                                     &command_complete_ns);
        if (status < 0) {
            return enter_recovery(driver, status, stats, true);
        }
        driver->deadline_ns = command_complete_ns + MS5837_RESET_RELOAD_NS;
        driver->state = MS5837_STATE_RECOVERY_WAIT_RESET;
        return MS5837_SERVICE_RECOVERING;

    case MS5837_STATE_RECOVERY_WAIT_RESET:
        if (now_ns >= driver->deadline_ns) {
            driver->recovery_prom_index = 0U;
            driver->state = MS5837_STATE_RECOVERY_PROM;
        }
        return MS5837_SERVICE_RECOVERING;

    case MS5837_STATE_RECOVERY_PROM:
        command = (uint8_t)(MS5837_COMMAND_PROM_READ_BASE +
                            (2U * driver->recovery_prom_index));
        status = write_command(driver, command);
        if (status < 0) {
            return enter_recovery(driver, status, stats, true);
        }
        status = driver->hal.read(driver->hal.context, response,
                                  sizeof(response));
        if (status < 0) {
            return enter_recovery(driver, status, stats, true);
        }
        driver->recovery_prom.word[driver->recovery_prom_index] =
            (uint16_t)(((uint16_t)response[0] << 8U) | response[1]);
        ++driver->recovery_prom_index;
        if (driver->recovery_prom_index >= MS5837_PROM_WORD_COUNT) {
            driver->state = MS5837_STATE_RECOVERY_VALIDATE;
        }
        return MS5837_SERVICE_RECOVERING;

    case MS5837_STATE_RECOVERY_VALIDATE:
        if (!prom_coefficients_valid(&driver->recovery_prom)) {
            return enter_recovery(driver, MS5837_DRIVER_ERR_PROM_INVALID,
                                  stats, false);
        }
        if (!ms5837_prom_crc_valid(&driver->recovery_prom)) {
            return enter_recovery(driver, MS5837_DRIVER_ERR_PROM_CRC,
                                  stats, false);
        }
        driver->prom = driver->recovery_prom;
        driver->initialized = true;
        driver->last_error = MS5837_DRIVER_OK;
        driver->state = MS5837_STATE_START_D1;
        ++stats->recovery_successes;
        return MS5837_SERVICE_RECOVERING;

    default:
        return MS5837_DRIVER_ERR_NOT_INITIALIZED;
    }
}

int ms5837_driver_init(Ms5837Driver_t *driver, const Ms5837Hal_t *hal)
{
    uint8_t command;
    uint8_t response[2];
    size_t index;
    int status;

    if (driver == NULL) {
        return MS5837_DRIVER_ERR_ARGUMENT;
    }

    memset(driver, 0, sizeof(*driver));
    driver->state = MS5837_STATE_STOPPED;
    driver->last_error = MS5837_DRIVER_ERR_NOT_INITIALIZED;

    if (!hal_contract_valid(hal)) {
        driver->last_error = MS5837_DRIVER_ERR_HAL_CONTRACT;
        return driver->last_error;
    }
    driver->hal = *hal;

    status = driver->hal.open(driver->hal.context);
    if (status < 0) {
        driver->last_error = status;
        return status;
    }
    driver->hal_open = true;

    command = MS5837_COMMAND_RESET;
    status = driver->hal.write(driver->hal.context, &command, 1U);
    if (status < 0) {
        return fail_and_close(driver, status);
    }

    status = driver->hal.sleep_ns(driver->hal.context,
                                  MS5837_RESET_RELOAD_NS);
    if (status < 0) {
        return fail_and_close(driver, status);
    }

    for (index = 0U; index < MS5837_PROM_WORD_COUNT; ++index) {
        command = (uint8_t)(MS5837_COMMAND_PROM_READ_BASE + (2U * index));
        status = driver->hal.write(driver->hal.context, &command, 1U);
        if (status < 0) {
            return fail_and_close(driver, status);
        }

        status = driver->hal.read(driver->hal.context, response,
                                  sizeof(response));
        if (status < 0) {
            return fail_and_close(driver, status);
        }
        driver->prom.word[index] =
            (uint16_t)(((uint16_t)response[0] << 8U) | response[1]);
    }

    if (!prom_coefficients_valid(&driver->prom)) {
        return fail_and_close(driver, MS5837_DRIVER_ERR_PROM_INVALID);
    }
    if (!ms5837_prom_crc_valid(&driver->prom)) {
        return fail_and_close(driver, MS5837_DRIVER_ERR_PROM_CRC);
    }

    driver->initialized = true;
    driver->state = MS5837_STATE_START_D1;
    driver->last_error = MS5837_DRIVER_OK;
    return MS5837_DRIVER_OK;
}

int ms5837_driver_shutdown(Ms5837Driver_t *driver)
{
    int status = 0;

    if (driver == NULL) {
        return MS5837_DRIVER_ERR_ARGUMENT;
    }
    if (driver->hal_open && (driver->hal.close != NULL)) {
        status = driver->hal.close(driver->hal.context);
        driver->hal_open = false;
    }
    driver->initialized = false;
    driver->pair_in_progress = false;
    driver->state = MS5837_STATE_STOPPED;
    if (status < 0) {
        driver->last_error = status;
    }
    return status;
}

int ms5837_driver_service(Ms5837Driver_t *driver,
                           uint64_t now_ns,
                           const BaroRunConfig_t *config,
                           BaroRuntimeStats_t *stats,
                           BaroSample_t *completed_sample)
{
    uint32_t raw_d2;
    uint64_t complete_time_ns;
    int status;

    if ((driver == NULL) || (config == NULL) || (stats == NULL) ||
        (completed_sample == NULL)) {
        return MS5837_DRIVER_ERR_ARGUMENT;
    }
    ++stats->service_ticks;

    if ((driver->state >= MS5837_STATE_RECOVERY_OPEN) &&
        (driver->state <= MS5837_STATE_RECOVERY_VALIDATE)) {
        return service_recovery(driver, now_ns, stats);
    }
    if (!driver->initialized || !driver->hal_open) {
        driver->last_error = MS5837_DRIVER_ERR_NOT_INITIALIZED;
        return driver->last_error;
    }

    switch (driver->state) {
    case MS5837_STATE_START_D1:
        status = start_conversion(
            driver,
            (uint8_t)(MS5837_COMMAND_CONVERT_D1_BASE |
                      MS5837_ACQUISITION_OSR),
            MS5837_STATE_WAIT_D1, stats);
        if (status >= 0) {
            driver->pair_in_progress = true;
        }
        return status;

    case MS5837_STATE_WAIT_D1:
        if (now_ns < driver->deadline_ns) {
            return MS5837_SERVICE_PROGRESS;
        }
        status = read_adc(driver, &driver->raw_pressure_d1);
        if (status < 0) {
            return enter_recovery(driver, status, stats, true);
        }
        driver->state = MS5837_STATE_START_D2;
        return MS5837_SERVICE_PROGRESS;

    case MS5837_STATE_START_D2:
        return start_conversion(
            driver,
            (uint8_t)(MS5837_COMMAND_CONVERT_D2_BASE |
                      MS5837_ACQUISITION_OSR),
            MS5837_STATE_WAIT_D2, stats);

    case MS5837_STATE_WAIT_D2:
        if (now_ns < driver->deadline_ns) {
            return MS5837_SERVICE_PROGRESS;
        }
        status = read_adc(driver, &raw_d2);
        if (status < 0) {
            return enter_recovery(driver, status, stats, true);
        }
        status = driver->hal.time_ns(driver->hal.context,
                                     &complete_time_ns);
        if (status < 0) {
            return enter_recovery(driver, status, stats, true);
        }
        build_sample(driver, raw_d2, complete_time_ns, config, stats,
                     completed_sample);
        driver->pair_in_progress = false;
        driver->state = MS5837_STATE_START_D1;
        driver->last_error = MS5837_DRIVER_OK;
        return MS5837_SERVICE_SAMPLE_READY;

    default:
        driver->last_error = MS5837_DRIVER_ERR_NOT_INITIALIZED;
        return driver->last_error;
    }
}

bool ms5837_driver_is_initialized(const Ms5837Driver_t *driver)
{
    return (driver != NULL) && driver->initialized;
}

const Ms5837Prom_t *ms5837_driver_prom(const Ms5837Driver_t *driver)
{
    if ((driver == NULL) || !driver->initialized) {
        return NULL;
    }
    return &driver->prom;
}

bool ms5837_driver_latest_sample(const Ms5837Driver_t *driver,
                                  BaroSample_t *sample)
{
    if ((driver == NULL) || (sample == NULL) || !driver->has_latest_sample) {
        return false;
    }
    *sample = driver->latest_sample;
    return true;
}

Ms5837DriverState_t ms5837_driver_state(const Ms5837Driver_t *driver)
{
    return (driver != NULL) ? driver->state : MS5837_STATE_STOPPED;
}

int ms5837_driver_last_error(const Ms5837Driver_t *driver)
{
    return (driver != NULL) ? driver->last_error
                            : MS5837_DRIVER_ERR_ARGUMENT;
}
