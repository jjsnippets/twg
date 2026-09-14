#include <errno.h>
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

typedef struct {
    uint16_t prom[MS5837_PROM_WORD_COUNT];
    size_t prom_index;
    size_t operation_index;
    size_t fail_operation;
    unsigned int open_count;
    unsigned int close_count;
    unsigned int write_count;
    unsigned int read_count;
    unsigned int sleep_count;
    uint8_t selected_prom_command;
} MockHal_t;

static int should_fail(MockHal_t *mock)
{
    ++mock->operation_index;
    return (mock->fail_operation != 0U) &&
           (mock->operation_index == mock->fail_operation);
}

static int mock_open(void *opaque)
{
    MockHal_t *mock = (MockHal_t *)opaque;
    ++mock->open_count;
    return should_fail(mock) ? -EIO : 0;
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

    ++mock->write_count;
    if ((data == NULL) || (length != 1U)) {
        return -EINVAL;
    }
    if (should_fail(mock)) {
        return -EIO;
    }
    if (data[0] == MS5837_COMMAND_RESET) {
        mock->prom_index = 0U;
    } else {
        mock->selected_prom_command = data[0];
    }
    return 0;
}

static int mock_read(void *opaque, uint8_t *data, size_t length)
{
    MockHal_t *mock = (MockHal_t *)opaque;
    uint8_t expected_command;
    uint16_t value;

    ++mock->read_count;
    if ((data == NULL) || (length != 2U) ||
        (mock->prom_index >= MS5837_PROM_WORD_COUNT)) {
        return -EINVAL;
    }
    if (should_fail(mock)) {
        return -EIO;
    }

    expected_command = (uint8_t)(MS5837_COMMAND_PROM_READ_BASE +
                                 (2U * mock->prom_index));
    if (mock->selected_prom_command != expected_command) {
        return -EPROTO;
    }

    value = mock->prom[mock->prom_index];
    data[0] = (uint8_t)(value >> 8U);
    data[1] = (uint8_t)(value & 0xFFU);
    ++mock->prom_index;
    return 0;
}

static int mock_sleep_ns(void *opaque, uint64_t duration_ns)
{
    MockHal_t *mock = (MockHal_t *)opaque;
    ++mock->sleep_count;
    if (duration_ns != MS5837_RESET_RELOAD_NS) {
        return -EINVAL;
    }
    return should_fail(mock) ? -EINTR : 0;
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
    return hal;
}

static void load_valid_prom(MockHal_t *mock)
{
    const uint16_t valid[MS5837_PROM_WORD_COUNT] = {
        0x4BA1U, 0xBA4AU, 0xB779U, 0x7395U,
        0x78A1U, 0x793DU, 0x6ACFU
    };
    memset(mock, 0, sizeof(*mock));
    memcpy(mock->prom, valid, sizeof(valid));
}

static int test_success(void)
{
    MockHal_t mock;
    Ms5837Hal_t hal;
    Ms5837Driver_t driver;
    const Ms5837Prom_t *prom;

    load_valid_prom(&mock);
    hal = make_hal(&mock);
    CHECK(ms5837_driver_init(&driver, &hal) == MS5837_DRIVER_OK);
    CHECK(ms5837_driver_is_initialized(&driver));
    CHECK(mock.open_count == 1U);
    CHECK(mock.close_count == 0U);
    CHECK(mock.write_count == 8U);
    CHECK(mock.read_count == 7U);
    CHECK(mock.sleep_count == 1U);
    prom = ms5837_driver_prom(&driver);
    CHECK(prom != NULL);
    CHECK(prom->word[0] == 0x4BA1U);
    CHECK(prom->word[6] == 0x6ACFU);
    CHECK(ms5837_driver_shutdown(&driver) == 0);
    CHECK(mock.close_count == 1U);
    CHECK(!ms5837_driver_is_initialized(&driver));
    return 0;
}

static int test_each_hal_failure_closes(void)
{
    size_t failure;

    /* 1=open; 2=reset; 3=sleep; then seven command/read pairs. */
    for (failure = 1U; failure <= 17U; ++failure) {
        MockHal_t mock;
        Ms5837Hal_t hal;
        Ms5837Driver_t driver;
        int result;

        load_valid_prom(&mock);
        mock.fail_operation = failure;
        hal = make_hal(&mock);
        result = ms5837_driver_init(&driver, &hal);
        CHECK(result < 0);
        CHECK(!ms5837_driver_is_initialized(&driver));
        CHECK(mock.open_count == 1U);
        CHECK(mock.close_count == ((failure == 1U) ? 0U : 1U));
    }
    return 0;
}

static int test_crc_failure(void)
{
    MockHal_t mock;
    Ms5837Hal_t hal;
    Ms5837Driver_t driver;

    load_valid_prom(&mock);
    mock.prom[3] ^= 1U;
    hal = make_hal(&mock);
    CHECK(ms5837_driver_init(&driver, &hal) ==
          MS5837_DRIVER_ERR_PROM_CRC);
    CHECK(mock.close_count == 1U);
    return 0;
}

static int test_invalid_coefficients(void)
{
    MockHal_t mock;
    Ms5837Hal_t hal;
    Ms5837Driver_t driver;
    size_t index;

    load_valid_prom(&mock);
    for (index = 0U; index < MS5837_PROM_WORD_COUNT; ++index) {
        mock.prom[index] = 0U;
    }
    hal = make_hal(&mock);
    CHECK(ms5837_driver_init(&driver, &hal) ==
          MS5837_DRIVER_ERR_PROM_INVALID);
    CHECK(mock.close_count == 1U);
    return 0;
}

static int test_bad_contract(void)
{
    MockHal_t mock;
    Ms5837Hal_t hal;
    Ms5837Driver_t driver;

    load_valid_prom(&mock);
    hal = make_hal(&mock);
    hal.read = NULL;
    CHECK(ms5837_driver_init(&driver, &hal) ==
          MS5837_DRIVER_ERR_HAL_CONTRACT);
    CHECK(mock.open_count == 0U);
    return 0;
}

int main(void)
{
    CHECK(test_success() == 0);
    CHECK(test_each_hal_failure_closes() == 0);
    CHECK(test_crc_failure() == 0);
    CHECK(test_invalid_coefficients() == 0);
    CHECK(test_bad_contract() == 0);
    puts("PASS: Step 4B mock initialization and cleanup");
    return 0;
}
