#ifndef MS5837_HAL_RPI_H
#define MS5837_HAL_RPI_H

#include <stdint.h>

#include "ms5837_hal.h"

#define MS5837_DEFAULT_I2C_BUS "/dev/i2c-1"
#define MS5837_I2C_ADDRESS 0x76U

typedef struct {
    const char *bus_path;
    uint8_t address;
    int fd;
} Ms5837HalRpi_t;

void ms5837_hal_rpi_configure(Ms5837HalRpi_t *context,
                              const char *bus_path,
                              uint8_t address);
Ms5837Hal_t ms5837_hal_rpi_make(Ms5837HalRpi_t *context);

#endif
