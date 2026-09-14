#ifndef MS5837_REFERENCE_H
#define MS5837_REFERENCE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_contract.h"

#define MS5837_ZERO_DURATION_NS UINT64_C(5000000000)
#define MS5837_ZERO_TRIM_FRACTION 0.10
#define MS5837_ZERO_MIN_VALID_SAMPLES 400U
#define MS5837_ZERO_MAX_SAMPLES 512U

typedef enum {
    MS5837_REFERENCE_IDLE = 0,
    MS5837_REFERENCE_COLLECTING,
    MS5837_REFERENCE_VALID,
    MS5837_REFERENCE_FAILED
} Ms5837ReferenceState_t;

typedef struct {
    Ms5837ReferenceState_t state;
    double pressure_samples[MS5837_ZERO_MAX_SAMPLES];
    size_t valid_sample_count;
    size_t rejected_sample_count;
    uint64_t last_measurement_sequence;
    double surface_pressure_mbar;
} Ms5837Reference_t;

void ms5837_reference_reset(Ms5837Reference_t *reference);
void ms5837_reference_begin(Ms5837Reference_t *reference);
bool ms5837_reference_add_sample(Ms5837Reference_t *reference,
                                  const BaroSample_t *sample);
bool ms5837_reference_finalize(Ms5837Reference_t *reference);
bool ms5837_reference_is_valid(const Ms5837Reference_t *reference);
double ms5837_reference_surface_pressure(const Ms5837Reference_t *reference);

#endif
