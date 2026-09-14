#ifndef MS5837_DRIVER_H
#define MS5837_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "app_contract.h"
#include "ms5837_hal.h"
#include "ms5837_math.h"

#define MS5837_COMMAND_RESET 0x1EU
#define MS5837_COMMAND_PROM_READ_BASE 0xA0U
#define MS5837_COMMAND_ADC_READ 0x00U
#define MS5837_COMMAND_CONVERT_D1_BASE 0x40U
#define MS5837_COMMAND_CONVERT_D2_BASE 0x50U
#define MS5837_RESET_RELOAD_NS UINT64_C(3000000)

/* Step 4C deliberately supports only symmetric pressure/temperature OSR 512. */
#define MS5837_OSR_512 0x02U
#define MS5837_ACQUISITION_OSR MS5837_OSR_512
#define MS5837_OSR_512_CONVERSION_NS UINT64_C(1100000)

#if MS5837_ACQUISITION_OSR != MS5837_OSR_512
#error "Step 4C supports only symmetric MS5837_OSR_512 acquisition"
#endif

typedef enum {
    MS5837_DRIVER_OK = 0,
    MS5837_DRIVER_ERR_ARGUMENT = -1000,
    MS5837_DRIVER_ERR_HAL_CONTRACT = -1001,
    MS5837_DRIVER_ERR_PROM_CRC = -1002,
    MS5837_DRIVER_ERR_PROM_INVALID = -1003,
    MS5837_DRIVER_ERR_NOT_INITIALIZED = -1004
} Ms5837DriverResult_t;

typedef enum {
    MS5837_SERVICE_PROGRESS = 0,
    MS5837_SERVICE_SAMPLE_READY = 1,
    MS5837_SERVICE_RECOVERING = 2
} Ms5837ServiceResult_t;

typedef enum {
    MS5837_STATE_STOPPED = 0,
    MS5837_STATE_START_D1,
    MS5837_STATE_WAIT_D1,
    MS5837_STATE_START_D2,
    MS5837_STATE_WAIT_D2,
    MS5837_STATE_RECOVERY_OPEN,
    MS5837_STATE_RECOVERY_RESET,
    MS5837_STATE_RECOVERY_WAIT_RESET,
    MS5837_STATE_RECOVERY_PROM,
    MS5837_STATE_RECOVERY_VALIDATE
} Ms5837DriverState_t;

typedef struct {
    Ms5837Hal_t hal;
    Ms5837Prom_t prom;
    Ms5837Prom_t recovery_prom;
    BaroSample_t latest_sample;
    Ms5837DriverState_t state;
    uint64_t deadline_ns;
    uint64_t measurement_sequence;
    uint32_t raw_pressure_d1;
    size_t recovery_prom_index;
    bool hal_open;
    bool initialized;
    bool pair_in_progress;
    bool has_latest_sample;
    int last_error;
} Ms5837Driver_t;

int ms5837_driver_init(Ms5837Driver_t *driver, const Ms5837Hal_t *hal);
int ms5837_driver_shutdown(Ms5837Driver_t *driver);
int ms5837_driver_service(Ms5837Driver_t *driver,
                           uint64_t now_ns,
                           const BaroRunConfig_t *config,
                           BaroRuntimeStats_t *stats,
                           BaroSample_t *completed_sample);
bool ms5837_driver_is_initialized(const Ms5837Driver_t *driver);
const Ms5837Prom_t *ms5837_driver_prom(const Ms5837Driver_t *driver);
bool ms5837_driver_latest_sample(const Ms5837Driver_t *driver,
                                  BaroSample_t *sample);
Ms5837DriverState_t ms5837_driver_state(const Ms5837Driver_t *driver);
int ms5837_driver_last_error(const Ms5837Driver_t *driver);

#endif
