#ifndef MS5837_APP_CONTRACT_H
#define MS5837_APP_CONTRACT_H

#include <stdbool.h>
#include <stdint.h>

#define BARO_SAMPLE_CONTRACT_VERSION 1U
#define BARO_RUN_CONFIG_VERSION 1U

/* Per-measurement flags. Cumulative runtime health is intentionally separate. */
typedef enum {
    BARO_STATUS_NONE                    = 0U,
    BARO_STATUS_NOT_READY               = 1U << 0,
    BARO_STATUS_RAW_INVALID             = 1U << 1,
    BARO_STATUS_PRESSURE_EXTENDED       = 1U << 2,
    BARO_STATUS_PRESSURE_INVALID        = 1U << 3,
    BARO_STATUS_TEMPERATURE_EXTENDED    = 1U << 4,
    BARO_STATUS_DEPTH_REFERENCE_PENDING = 1U << 5,
    BARO_STATUS_DEPTH_REFERENCE_INVALID = 1U << 6
} BaroStatusFlag_t;

typedef struct {
    uint32_t contract_version;
    uint32_t status_flags;
    uint64_t measurement_sequence;
    uint64_t measurement_complete_time_ns;
    uint32_t raw_pressure_d1;
    uint32_t raw_temperature_d2;
    double pressure_mbar;
    double temperature_c;
    double depth_m;
} BaroSample_t;

/* Run-level metadata; these values are not repeated in BaroSample_t. */
typedef struct {
    uint32_t config_version;
    double fluid_density_kg_m3;
    double surface_pressure_mbar;
    bool surface_pressure_valid;
    bool zero_at_start_requested;
} BaroRunConfig_t;

/* Cumulative statistics are owned outside each measurement sample. */
typedef struct {
    uint64_t service_ticks;
    uint64_t measurements_completed;
    uint64_t publications_attempted;
    uint64_t publications_not_ready;
    uint64_t publications_stale;
    uint64_t pressure_extended_count;
    uint64_t pressure_invalid_count;
    uint64_t temperature_extended_count;
    uint64_t i2c_errors;
    uint64_t aborted_pairs;
    uint64_t recovery_attempts;
    uint64_t recovery_successes;
    uint64_t service_overruns;
    uint64_t publication_deadline_misses;
    uint64_t logger_drops;
    uint64_t maximum_loop_body_ns;
} BaroRuntimeStats_t;

#endif
