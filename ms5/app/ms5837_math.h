#ifndef MS5837_MATH_H
#define MS5837_MATH_H

#include <stdbool.h>
#include <stdint.h>

#include "app_contract.h"

#define MS5837_PROM_WORD_COUNT 7U
#define MS5837_GRAVITY_M_S2 9.80665
#define MS5837_PRESSURE_MIN_LINEAR_MBAR 10.0
#define MS5837_PRESSURE_MIN_NOMINAL_MBAR 300.0
#define MS5837_PRESSURE_MAX_NOMINAL_MBAR 1200.0
#define MS5837_PRESSURE_MAX_LINEAR_MBAR 2000.0
#define MS5837_TEMPERATURE_MIN_NOMINAL_C (-20.0)
#define MS5837_TEMPERATURE_MAX_NOMINAL_C 85.0

typedef struct {
    uint16_t word[MS5837_PROM_WORD_COUNT];
} Ms5837Prom_t;

typedef struct {
    int32_t temperature_centi_c;
    int32_t pressure_centi_mbar;
    double temperature_c;
    double pressure_mbar;
} Ms5837Compensated_t;

typedef enum {
    MS5837_MATH_OK = 0,
    MS5837_MATH_ERR_ARGUMENT = -1,
    MS5837_MATH_ERR_RAW_INVALID = -2
} Ms5837MathResult_t;

uint8_t ms5837_crc4(const uint16_t prom[MS5837_PROM_WORD_COUNT]);
bool ms5837_prom_crc_valid(const Ms5837Prom_t *prom);
Ms5837MathResult_t ms5837_compensate(const Ms5837Prom_t *prom,
                                     uint32_t raw_pressure_d1,
                                     uint32_t raw_temperature_d2,
                                     Ms5837Compensated_t *result);
uint32_t ms5837_classify_measurement(double pressure_mbar,
                                      double temperature_c);
double ms5837_depth_from_reference(double pressure_mbar,
                                    double surface_pressure_mbar,
                                    double fluid_density_kg_m3);
void ms5837_sample_set_not_ready(BaroSample_t *sample,
                                  bool reference_pending);

#endif
