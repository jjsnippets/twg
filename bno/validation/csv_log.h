/*
 * csv_log.h — bounded-ring CSV logger for the validation binary.
 *
 * The RT thread never touches the file: it pushes CsvRecord_t values
 * into a fixed-size ring (drop-newest on overflow, never blocks); a
 * dedicated SCHED_OTHER logger thread drains the ring to a stdio-
 * buffered CSV file. SD-card write stalls therefore cannot perturb
 * the 1 ms service cadence.
 *
 * File layout:
 *   - '#'-prefixed metadata block (clocks, units, run notes)
 *   - one plain header row with column names (first non-comment line,
 *     so pandas/read_csv skip-comment='#' reads it directly)
 *   - one row per 100 Hz tick
 *   - '#'-prefixed footer (records, drops, duration)
 *
 * Part of the sensor_validate binary (see validation/Makefile).
 */

#ifndef CSV_LOG_H
#define CSV_LOG_H

#include <stdint.h>

#include "imu_validate.h"

/* One synchronized 100 Hz row: encoder snapshot + latest IMU sample. */
typedef struct {
    uint64_t host_ts_ns;       /* record assembly time, CLOCK_MONOTONIC ns */
    int64_t  enc_count;        /* x4 counts since encoder open */
    double   enc_angle_deg;    /* enc_count * 360/8192, continuous (unwrapped) */
    uint64_t enc_event_ts_ns;  /* kernel stamp of newest encoder event */
    uint64_t enc_x_pulses;     /* index pulses since open */
    uint64_t enc_invalid;      /* illegal quadrature transitions since open */
    uint64_t enc_edges_ab;     /* a_edges + b_edges since open */
    ImuValidateSample_t imu;
    uint64_t drops;            /* cumulative ring drops at push time */
} CsvRecord_t;

typedef struct {
    uint64_t records_written;
    uint64_t records_dropped;
    double   duration_sec;
} CsvLogStats_t;

/*
 * Open path for writing, emit the metadata block and column header, and
 * start the logger thread. Returns 0 on success, -1 on failure (errno).
 */
int csv_log_start(const char *path, double arm_len_m, const char *notes);

/*
 * Queue one record. Returns 0 on success, 1 if the ring was full (the
 * record is dropped and the drop counter incremented — never blocks).
 */
int csv_log_push(const CsvRecord_t *rec);

/* Cumulative ring drops so far (thread-safe). */
uint64_t csv_log_dropped(void);

/*
 * Stop the logger thread, drain, and write the footer. Fills *out with
 * final stats if non-NULL. Safe to call even after a failed start.
 */
void csv_log_stop(CsvLogStats_t *out);

#endif /* CSV_LOG_H */
