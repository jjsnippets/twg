#include "ms5837_driver.h"

#include <stddef.h>
#include <string.h>

static bool hal_contract_valid(const Ms5837Hal_t *hal)
{
    return (hal != NULL) && (hal->context != NULL) &&
           (hal->open != NULL) && (hal->close != NULL) &&
           (hal->write != NULL) && (hal->read != NULL) &&
           (hal->sleep_ns != NULL);
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
    driver->last_error = error;
    return error;
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
    driver->last_error = MS5837_DRIVER_OK;
    return MS5837_DRIVER_OK;
}

int ms5837_driver_shutdown(Ms5837Driver_t *driver)
{
    int status = 0;

    if (driver == NULL) {
        return MS5837_DRIVER_ERR_ARGUMENT;
    }
    if (driver->hal_open) {
        status = driver->hal.close(driver->hal.context);
        driver->hal_open = false;
    }
    driver->initialized = false;
    driver->last_error = (status < 0) ? status :
                         MS5837_DRIVER_ERR_NOT_INITIALIZED;
    return status;
}

bool ms5837_driver_is_initialized(const Ms5837Driver_t *driver)
{
    return (driver != NULL) && driver->initialized;
}

const Ms5837Prom_t *ms5837_driver_prom(const Ms5837Driver_t *driver)
{
    if (!ms5837_driver_is_initialized(driver)) {
        return NULL;
    }
    return &driver->prom;
}

int ms5837_driver_last_error(const Ms5837Driver_t *driver)
{
    if (driver == NULL) {
        return MS5837_DRIVER_ERR_ARGUMENT;
    }
    return driver->last_error;
}
