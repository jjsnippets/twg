#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ms5837_driver.h"
#include "ms5837_hal_rpi.h"

static const char *driver_error_text(int error)
{
    switch (error) {
    case MS5837_DRIVER_ERR_ARGUMENT:
        return "invalid argument";
    case MS5837_DRIVER_ERR_HAL_CONTRACT:
        return "invalid HAL contract";
    case MS5837_DRIVER_ERR_PROM_CRC:
        return "PROM CRC mismatch";
    case MS5837_DRIVER_ERR_PROM_INVALID:
        return "invalid PROM coefficients";
    case MS5837_DRIVER_ERR_NOT_INITIALIZED:
        return "driver not initialized";
    default:
        return (error < 0 && error > -1000) ? strerror(-error) :
               "unknown driver error";
    }
}

int main(int argc, char **argv)
{
    const char *bus_path = MS5837_DEFAULT_I2C_BUS;
    Ms5837HalRpi_t rpi_context;
    Ms5837Hal_t hal;
    Ms5837Driver_t driver;
    const Ms5837Prom_t *prom;
    size_t index;
    int status;

    if (argc > 2) {
        fprintf(stderr, "Usage: %s [i2c-bus-path]\n", argv[0]);
        return EXIT_FAILURE;
    }
    if (argc == 2) {
        bus_path = argv[1];
    }

    ms5837_hal_rpi_configure(&rpi_context, bus_path,
                             MS5837_I2C_ADDRESS);
    hal = ms5837_hal_rpi_make(&rpi_context);
    status = ms5837_driver_init(&driver, &hal);
    if (status < 0) {
        fprintf(stderr, "FAIL: MS5837 initialization on %s at 0x%02X: "
                "%s (%d)\n", bus_path, MS5837_I2C_ADDRESS,
                driver_error_text(status), status);
        return EXIT_FAILURE;
    }

    prom = ms5837_driver_prom(&driver);
    if (prom == NULL) {
        fprintf(stderr, "FAIL: initialized driver returned no PROM\n");
        (void)ms5837_driver_shutdown(&driver);
        return EXIT_FAILURE;
    }

    printf("MS5837 initialized on %s at 0x%02X\n",
           bus_path, MS5837_I2C_ADDRESS);
    for (index = 0U; index < MS5837_PROM_WORD_COUNT; ++index) {
        printf("PROM[%zu] = 0x%04X", index, prom->word[index]);
        if (index == 0U) {
            printf("  stored_crc=0x%X computed_crc=0x%X",
                   (unsigned int)((prom->word[0] >> 12U) & 0x0FU),
                   (unsigned int)ms5837_crc4(prom->word));
        }
        putchar('\n');
    }
    puts("PASS: reset, PROM read, coefficient check, and CRC validation");

    status = ms5837_driver_shutdown(&driver);
    if (status < 0) {
        fprintf(stderr, "FAIL: HAL close: %s (%d)\n",
                driver_error_text(status), status);
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
