#include "ms5837_reference.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static int compare_double(const void *left, const void *right)
{
    const double left_value = *(const double *)left;
    const double right_value = *(const double *)right;

    if (left_value < right_value) {
        return -1;
    }
    if (left_value > right_value) {
        return 1;
    }
    return 0;
}

void ms5837_reference_reset(Ms5837Reference_t *reference)
{
    if (reference == NULL) {
        return;
    }

    memset(reference, 0, sizeof(*reference));
    reference->state = MS5837_REFERENCE_IDLE;
    reference->surface_pressure_mbar = NAN;
}

void ms5837_reference_begin(Ms5837Reference_t *reference)
{
    ms5837_reference_reset(reference);
    if (reference != NULL) {
        reference->state = MS5837_REFERENCE_COLLECTING;
    }
}

bool ms5837_reference_add_sample(Ms5837Reference_t *reference,
                                  const BaroSample_t *sample)
{
    const uint32_t rejection_flags =
        BARO_STATUS_NOT_READY | BARO_STATUS_RAW_INVALID |
        BARO_STATUS_PRESSURE_INVALID;

    if ((reference == NULL) || (sample == NULL) ||
        (reference->state != MS5837_REFERENCE_COLLECTING)) {
        return false;
    }

    if ((sample->measurement_sequence == 0U) ||
        (sample->measurement_sequence == reference->last_measurement_sequence) ||
        ((sample->status_flags & rejection_flags) != 0U) ||
        !isfinite(sample->pressure_mbar) ||
        (reference->valid_sample_count >= MS5837_ZERO_MAX_SAMPLES)) {
        ++reference->rejected_sample_count;
        return false;
    }

    reference->pressure_samples[reference->valid_sample_count] =
        sample->pressure_mbar;
    ++reference->valid_sample_count;
    reference->last_measurement_sequence = sample->measurement_sequence;
    return true;
}

bool ms5837_reference_finalize(Ms5837Reference_t *reference)
{
    size_t trim_count;
    size_t index;
    size_t end_index;
    size_t retained_count;
    double sum = 0.0;

    if ((reference == NULL) ||
        (reference->state != MS5837_REFERENCE_COLLECTING)) {
        return false;
    }

    if (reference->valid_sample_count < MS5837_ZERO_MIN_VALID_SAMPLES) {
        reference->state = MS5837_REFERENCE_FAILED;
        reference->surface_pressure_mbar = NAN;
        return false;
    }

    qsort(reference->pressure_samples, reference->valid_sample_count,
          sizeof(reference->pressure_samples[0]), compare_double);
    trim_count = (size_t)((double)reference->valid_sample_count *
                          MS5837_ZERO_TRIM_FRACTION);
    end_index = reference->valid_sample_count - trim_count;
    retained_count = end_index - trim_count;

    if (retained_count == 0U) {
        reference->state = MS5837_REFERENCE_FAILED;
        reference->surface_pressure_mbar = NAN;
        return false;
    }

    for (index = trim_count; index < end_index; ++index) {
        sum += reference->pressure_samples[index];
    }

    reference->surface_pressure_mbar = sum / (double)retained_count;
    reference->state = MS5837_REFERENCE_VALID;
    return true;
}

bool ms5837_reference_is_valid(const Ms5837Reference_t *reference)
{
    return (reference != NULL) &&
           (reference->state == MS5837_REFERENCE_VALID) &&
           isfinite(reference->surface_pressure_mbar);
}

double ms5837_reference_surface_pressure(const Ms5837Reference_t *reference)
{
    if (!ms5837_reference_is_valid(reference)) {
        return NAN;
    }
    return reference->surface_pressure_mbar;
}
