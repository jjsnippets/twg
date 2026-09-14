#include "ms5837_math.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

uint8_t ms5837_crc4(const uint16_t prom[MS5837_PROM_WORD_COUNT])
{
    uint16_t working[8];
    uint16_t remainder = 0U;
    size_t count;
    int bit;

    if (prom == NULL) {
        return 0xFFU;
    }

    for (count = 0U; count < MS5837_PROM_WORD_COUNT; ++count) {
        working[count] = prom[count];
    }
    working[7] = 0U;
    working[0] &= 0x0FFFU;

    for (count = 0U; count < 16U; ++count) {
        if ((count & 1U) != 0U) {
            remainder ^= working[count >> 1U] & 0x00FFU;
        } else {
            remainder ^= working[count >> 1U] >> 8U;
        }

        for (bit = 0; bit < 8; ++bit) {
            if ((remainder & 0x8000U) != 0U) {
                remainder = (uint16_t)((remainder << 1U) ^ 0x3000U);
            } else {
                remainder = (uint16_t)(remainder << 1U);
            }
        }
    }

    return (uint8_t)((remainder >> 12U) & 0x0FU);
}

bool ms5837_prom_crc_valid(const Ms5837Prom_t *prom)
{
    uint8_t stored_crc;

    if (prom == NULL) {
        return false;
    }

    stored_crc = (uint8_t)((prom->word[0] >> 12U) & 0x0FU);
    return ms5837_crc4(prom->word) == stored_crc;
}

Ms5837MathResult_t ms5837_compensate(const Ms5837Prom_t *prom,
                                     uint32_t raw_pressure_d1,
                                     uint32_t raw_temperature_d2,
                                     Ms5837Compensated_t *result)
{
    int32_t delta_temperature;
    int32_t temperature;
    int32_t compensated_temperature;
    int32_t compensated_pressure;
    int64_t offset;
    int64_t sensitivity;
    int64_t temperature_correction = 0;
    int64_t offset_correction = 0;
    int64_t sensitivity_correction = 0;

    if ((prom == NULL) || (result == NULL)) {
        return MS5837_MATH_ERR_ARGUMENT;
    }

    result->temperature_centi_c = 0;
    result->pressure_centi_mbar = 0;
    result->temperature_c = NAN;
    result->pressure_mbar = NAN;

    if ((raw_pressure_d1 == 0U) || (raw_temperature_d2 == 0U) ||
        (raw_pressure_d1 > 0xFFFFFFU) ||
        (raw_temperature_d2 > 0xFFFFFFU)) {
        return MS5837_MATH_ERR_RAW_INVALID;
    }

    delta_temperature = (int32_t)raw_temperature_d2 -
                        ((int32_t)prom->word[5] << 8);
    temperature = 2000 +
                  (int32_t)(((int64_t)delta_temperature *
                             (int64_t)prom->word[6]) >> 23);
    offset = ((int64_t)prom->word[2] << 17) +
             (((int64_t)prom->word[4] * delta_temperature) >> 6);
    sensitivity = ((int64_t)prom->word[1] << 16) +
                  (((int64_t)prom->word[3] * delta_temperature) >> 7);

    if (temperature < 2000) {
        int64_t temperature_delta = (int64_t)temperature - 2000;

        temperature_correction =
            (11 * (int64_t)delta_temperature * delta_temperature) >> 35;
        offset_correction =
            (31 * temperature_delta * temperature_delta) >> 3;
        sensitivity_correction =
            (63 * temperature_delta * temperature_delta) >> 5;
    }

    offset -= offset_correction;
    sensitivity -= sensitivity_correction;
    compensated_temperature = temperature - (int32_t)temperature_correction;
    compensated_pressure =
        (int32_t)(((((int64_t)raw_pressure_d1 * sensitivity) >> 21) -
                   offset) >> 15);

    result->temperature_centi_c = compensated_temperature;
    result->pressure_centi_mbar = compensated_pressure;
    result->temperature_c = (double)compensated_temperature / 100.0;
    result->pressure_mbar = (double)compensated_pressure / 100.0;
    return MS5837_MATH_OK;
}

uint32_t ms5837_classify_measurement(double pressure_mbar,
                                      double temperature_c)
{
    uint32_t flags = BARO_STATUS_NONE;

    if (!isfinite(pressure_mbar)) {
        flags |= BARO_STATUS_PRESSURE_INVALID;
    } else if ((pressure_mbar < MS5837_PRESSURE_MIN_LINEAR_MBAR) ||
               (pressure_mbar > MS5837_PRESSURE_MAX_LINEAR_MBAR)) {
        flags |= BARO_STATUS_PRESSURE_INVALID;
    } else if ((pressure_mbar < MS5837_PRESSURE_MIN_NOMINAL_MBAR) ||
               (pressure_mbar > MS5837_PRESSURE_MAX_NOMINAL_MBAR)) {
        flags |= BARO_STATUS_PRESSURE_EXTENDED;
    }

    if (!isfinite(temperature_c)) {
        flags |= BARO_STATUS_RAW_INVALID;
    } else if ((temperature_c < MS5837_TEMPERATURE_MIN_NOMINAL_C) ||
               (temperature_c > MS5837_TEMPERATURE_MAX_NOMINAL_C)) {
        flags |= BARO_STATUS_TEMPERATURE_EXTENDED;
    }

    return flags;
}

double ms5837_depth_from_reference(double pressure_mbar,
                                    double surface_pressure_mbar,
                                    double fluid_density_kg_m3)
{
    if (!isfinite(pressure_mbar) || !isfinite(surface_pressure_mbar) ||
        !isfinite(fluid_density_kg_m3) ||
        (fluid_density_kg_m3 <= 0.0)) {
        return NAN;
    }

    return ((pressure_mbar - surface_pressure_mbar) * 100.0) /
           (fluid_density_kg_m3 * MS5837_GRAVITY_M_S2);
}

void ms5837_sample_set_not_ready(BaroSample_t *sample,
                                  bool reference_pending)
{
    if (sample == NULL) {
        return;
    }

    memset(sample, 0, sizeof(*sample));
    sample->contract_version = BARO_SAMPLE_CONTRACT_VERSION;
    sample->status_flags = BARO_STATUS_NOT_READY;
    if (reference_pending) {
        sample->status_flags |= BARO_STATUS_DEPTH_REFERENCE_PENDING;
    } else {
        sample->status_flags |= BARO_STATUS_DEPTH_REFERENCE_INVALID;
    }
    sample->pressure_mbar = NAN;
    sample->temperature_c = NAN;
    sample->depth_m = NAN;
}
