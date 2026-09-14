#ifndef MS5837_DRIVER_H
#define MS5837_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "ms5837_hal.h"
#include "ms5837_math.h"

#define MS5837_COMMAND_RESET 0x1EU
#define MS5837_COMMAND_PROM_READ_BASE 0xA0U
#define MS5837_RESET_RELOAD_NS UINT64_C(3000000)

typedef enum {
    MS5837_DRIVER_OK = 0,
    MS5837_DRIVER_ERR_ARGUMENT = -1000,
    MS5837_DRIVER_ERR_HAL_CONTRACT = -1001,
    MS5837_DRIVER_ERR_PROM_CRC = -1002,
    MS5837_DRIVER_ERR_PROM_INVALID = -1003,
    MS5837_DRIVER_ERR_NOT_INITIALIZED = -1004
} Ms5837DriverResult_t;

typedef struct {
    Ms5837Hal_t hal;
    Ms5837Prom_t prom;
    bool hal_open;
    bool initialized;
    int last_error;
} Ms5837Driver_t;

int ms5837_driver_init(Ms5837Driver_t *driver, const Ms5837Hal_t *hal);
int ms5837_driver_shutdown(Ms5837Driver_t *driver);
bool ms5837_driver_is_initialized(const Ms5837Driver_t *driver);
const Ms5837Prom_t *ms5837_driver_prom(const Ms5837Driver_t *driver);
int ms5837_driver_last_error(const Ms5837Driver_t *driver);

#endif
