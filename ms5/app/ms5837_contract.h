#ifndef MS5837_CONTRACT_H
#define MS5837_CONTRACT_H

#include <stdint.h>

#define BARO_SAMPLE_CONTRACT_VERSION 2U

/*
 * Per-measurement acquisition facts. Bits 5 and 6 are reserved and producers
 * must leave them zero; baseline validity and depth are consumer policy.
 */
typedef enum {
    BARO_STATUS_NONE                 = 0U,
    BARO_STATUS_NOT_READY            = 1U << 0,
    BARO_STATUS_RAW_INVALID          = 1U << 1,
    BARO_STATUS_PRESSURE_EXTENDED    = 1U << 2,
    BARO_STATUS_PRESSURE_INVALID     = 1U << 3,
    BARO_STATUS_TEMPERATURE_EXTENDED = 1U << 4
} BaroStatusFlag_t;

#define BARO_STATUS_RESERVED_MASK ((UINT32_C(1) << 5) | (UINT32_C(1) << 6))

typedef struct {
    uint32_t contract_version;
    uint32_t status_flags;
    uint64_t measurement_sequence;
    uint64_t measurement_complete_time_ns;
    uint32_t raw_pressure_d1;
    uint32_t raw_temperature_d2;
    double pressure_mbar;
    double temperature_c;
} BaroSample_t;

/* Cumulative acquisition statistics. Only the driver mutates these fields. */
typedef struct {
    uint64_t service_ticks;
    uint64_t measurements_completed;
    uint64_t pressure_extended_count;
    uint64_t pressure_invalid_count;
    uint64_t temperature_extended_count;
    uint64_t i2c_errors;
    uint64_t aborted_pairs;
    uint64_t recovery_incidents;
    uint64_t recovery_attempts;
    uint64_t recovery_successes;
    uint64_t recovery_exhaustions;
    int last_error;
} BaroDriverStats_t;

#endif /* MS5837_CONTRACT_H */
