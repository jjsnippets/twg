#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "ms5837_math.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static int test_crc(void)
{
    Ms5837Prom_t live = {
        {0x4BA1U, 0xBA4AU, 0xB779U, 0x7395U,
         0x78A1U, 0x793DU, 0x6ACFU}
    };

    CHECK(ms5837_crc4(live.word) == 0x4U);
    CHECK(ms5837_prom_crc_valid(&live));
    live.word[3] ^= 0x0001U;
    CHECK(!ms5837_prom_crc_valid(&live));
    return 0;
}

static int test_datasheet_vector(void)
{
    Ms5837Prom_t prom = {
        {0U, 46372U, 43981U, 29059U, 27842U, 31553U, 28165U}
    };
    Ms5837Compensated_t result;

    CHECK(ms5837_compensate(&prom, 6465444U, 8077636U, &result) ==
          MS5837_MATH_OK);
    CHECK(result.temperature_centi_c == 2000);
    CHECK(result.pressure_centi_mbar == 110002);
    CHECK(fabs(result.temperature_c - 20.00) < 0.0001);
    CHECK(fabs(result.pressure_mbar - 1100.02) < 0.0001);
    return 0;
}

static int test_low_temperature_compensation(void)
{
    Ms5837Prom_t prom = {
        {0U, 46372U, 43981U, 29059U, 27842U, 31553U, 28165U}
    };
    Ms5837Compensated_t result;

    CHECK(ms5837_compensate(&prom, 6465444U, 7500000U, &result) ==
          MS5837_MATH_OK);
    CHECK(result.temperature_centi_c == -46);
    CHECK(result.pressure_centi_mbar == 105081);
    return 0;
}

static int test_raw_rejection(void)
{
    Ms5837Prom_t prom = {
        {0U, 46372U, 43981U, 29059U, 27842U, 31553U, 28165U}
    };
    Ms5837Compensated_t result;

    CHECK(ms5837_compensate(&prom, 0U, 8077636U, &result) ==
          MS5837_MATH_ERR_RAW_INVALID);
    CHECK(isnan(result.pressure_mbar));
    CHECK(ms5837_compensate(&prom, 6465444U, 0U, &result) ==
          MS5837_MATH_ERR_RAW_INVALID);
    CHECK(isnan(result.temperature_c));
    return 0;
}

static int test_boundaries(void)
{
    CHECK(ms5837_classify_measurement(300.0, -20.0) == BARO_STATUS_NONE);
    CHECK(ms5837_classify_measurement(1200.0, 85.0) == BARO_STATUS_NONE);
    CHECK((ms5837_classify_measurement(10.0, 20.0) &
           BARO_STATUS_PRESSURE_EXTENDED) != 0U);
    CHECK((ms5837_classify_measurement(299.99, 20.0) &
           BARO_STATUS_PRESSURE_EXTENDED) != 0U);
    CHECK((ms5837_classify_measurement(1200.01, 20.0) &
           BARO_STATUS_PRESSURE_EXTENDED) != 0U);
    CHECK((ms5837_classify_measurement(2000.0, 20.0) &
           BARO_STATUS_PRESSURE_EXTENDED) != 0U);
    CHECK((ms5837_classify_measurement(9.99, 20.0) &
           BARO_STATUS_PRESSURE_INVALID) != 0U);
    CHECK((ms5837_classify_measurement(2000.01, 20.0) &
           BARO_STATUS_PRESSURE_INVALID) != 0U);
    CHECK((ms5837_classify_measurement(1000.0, -20.01) &
           BARO_STATUS_TEMPERATURE_EXTENDED) != 0U);
    CHECK((ms5837_classify_measurement(1000.0, 85.01) &
           BARO_STATUS_TEMPERATURE_EXTENDED) != 0U);
    return 0;
}

static int test_depth(void)
{
    double positive = ms5837_depth_from_reference(1100.0, 1000.0, 1000.0);
    double negative = ms5837_depth_from_reference(900.0, 1000.0, 1000.0);

    CHECK(positive > 0.0);
    CHECK(negative < 0.0);
    CHECK(fabs(positive + negative) < 1e-12);
    CHECK(isnan(ms5837_depth_from_reference(1000.0, NAN, 1000.0)));
    CHECK(isnan(ms5837_depth_from_reference(1000.0, 1000.0, 0.0)));
    return 0;
}

static int test_not_ready_sample(void)
{
    BaroSample_t sample;

    ms5837_sample_set_not_ready(&sample, false);
    CHECK(sample.contract_version == BARO_SAMPLE_CONTRACT_VERSION);
    CHECK(sample.measurement_sequence == 0U);
    CHECK((sample.status_flags & BARO_STATUS_NOT_READY) != 0U);
    CHECK((sample.status_flags & BARO_STATUS_DEPTH_REFERENCE_INVALID) != 0U);
    CHECK(isnan(sample.pressure_mbar));
    CHECK(isnan(sample.temperature_c));
    CHECK(isnan(sample.depth_m));

    ms5837_sample_set_not_ready(&sample, true);
    CHECK((sample.status_flags & BARO_STATUS_DEPTH_REFERENCE_PENDING) != 0U);
    return 0;
}

int main(void)
{
    CHECK(test_crc() == 0);
    CHECK(test_datasheet_vector() == 0);
    CHECK(test_low_temperature_compensation() == 0);
    CHECK(test_raw_rejection() == 0);
    CHECK(test_boundaries() == 0);
    CHECK(test_depth() == 0);
    CHECK(test_not_ready_sample() == 0);
    puts("PASS: Step 4A production math contract");
    return 0;
}
