#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "ms5837_reference.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static BaroSample_t make_sample(uint64_t sequence, double pressure,
                                uint32_t flags)
{
    BaroSample_t sample;

    sample.contract_version = BARO_SAMPLE_CONTRACT_VERSION;
    sample.status_flags = flags;
    sample.measurement_sequence = sequence;
    sample.measurement_complete_time_ns = sequence * UINT64_C(10000000);
    sample.raw_pressure_d1 = 1U;
    sample.raw_temperature_d2 = 1U;
    sample.pressure_mbar = pressure;
    sample.temperature_c = 20.0;
    sample.depth_m = NAN;
    return sample;
}

static int test_trimmed_mean(void)
{
    Ms5837Reference_t reference;
    size_t index;

    ms5837_reference_begin(&reference);
    for (index = 0U; index < 500U; ++index) {
        double pressure = 1000.0;
        BaroSample_t sample;

        if (index < 20U) {
            pressure = 900.0;
        } else if (index >= 480U) {
            pressure = 1100.0;
        }
        sample = make_sample((uint64_t)index + 1U, pressure,
                             BARO_STATUS_NONE);
        CHECK(ms5837_reference_add_sample(&reference, &sample));
    }

    CHECK(reference.valid_sample_count == 500U);
    CHECK(ms5837_reference_finalize(&reference));
    CHECK(ms5837_reference_is_valid(&reference));
    CHECK(fabs(ms5837_reference_surface_pressure(&reference) - 1000.0) <
          1e-12);
    return 0;
}

static int test_rejections_and_minimum(void)
{
    Ms5837Reference_t reference;
    BaroSample_t sample;
    size_t index;

    ms5837_reference_begin(&reference);
    for (index = 0U; index < MS5837_ZERO_MIN_VALID_SAMPLES - 1U; ++index) {
        sample = make_sample((uint64_t)index + 1U, 1000.0,
                             BARO_STATUS_NONE);
        CHECK(ms5837_reference_add_sample(&reference, &sample));
    }

    sample = make_sample(1000U, NAN, BARO_STATUS_PRESSURE_INVALID);
    CHECK(!ms5837_reference_add_sample(&reference, &sample));
    CHECK(!ms5837_reference_finalize(&reference));
    CHECK(reference.state == MS5837_REFERENCE_FAILED);
    CHECK(isnan(ms5837_reference_surface_pressure(&reference)));
    return 0;
}

static int test_350_sample_boundary(void)
{
    Ms5837Reference_t reference;
    size_t index;

    CHECK(MS5837_ZERO_MIN_VALID_SAMPLES == 350U);
    ms5837_reference_begin(&reference);
    for (index = 0U; index < MS5837_ZERO_MIN_VALID_SAMPLES; ++index) {
        BaroSample_t sample = make_sample(
            (uint64_t)index + 1U, 1000.0, BARO_STATUS_NONE);
        CHECK(ms5837_reference_add_sample(&reference, &sample));
    }
    CHECK(ms5837_reference_finalize(&reference));
    CHECK(reference.valid_sample_count == 350U);
    CHECK(fabs(ms5837_reference_surface_pressure(&reference) - 1000.0) <
          1e-12);
    return 0;
}

static int test_duplicate_sequence(void)
{
    Ms5837Reference_t reference;
    BaroSample_t sample = make_sample(1U, 1000.0, BARO_STATUS_NONE);

    ms5837_reference_begin(&reference);
    CHECK(ms5837_reference_add_sample(&reference, &sample));
    CHECK(!ms5837_reference_add_sample(&reference, &sample));
    CHECK(reference.valid_sample_count == 1U);
    CHECK(reference.rejected_sample_count == 1U);
    return 0;
}

int main(void)
{
    CHECK(test_trimmed_mean() == 0);
    CHECK(test_rejections_and_minimum() == 0);
    CHECK(test_350_sample_boundary() == 0);
    CHECK(test_duplicate_sequence() == 0);
    puts("PASS: Step 4A surface-reference contract");
    return 0;
}
