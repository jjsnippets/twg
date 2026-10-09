#ifndef IMU_CSV_H
#define IMU_CSV_H

/*
 * CSV schema 1 formatter. Pure and bounded: no I/O, no clock,
 * no allocation, no globals. Intended for the logger worker only; the
 * publisher never formats. One header line and one data line per record,
 * each terminated by a single '\n' (no '\r').
 *
 *  - 52 columns in IMU_CSV_HEADER_TOKENS order.
 *  - Integers are plain decimal. Masks and reader_flags are decimal too.
 *  - Floats use "%.9g" (round-trips IEEE binary32) in the "C" numeric
 *    locale; the process never calls setlocale. Non-finite floats are
 *    written as the missing token.
 *  - A missing field is IMU_CSV_MISSING_TOKEN ("NaN"): invalid
 *    sched_ns/actual_ns, all identity and value columns of a group that
 *    is not valid on this row, and age_ns whenever age_valid is 0.
 *  - *_age_valid is always 0 or 1; it is 0 for a missing group.
 *  - reader_state is the numeric ImuReaderState_t value.
 */

#include <stddef.h>

#include "app/imu_publish.h"

/* Callers size one stack buffer; the worst-case row is far below this. */
#define IMU_CSV_LINE_CAPACITY  1536u

/*
 * Write a NUL-terminated line into out; return its length excluding the
 * NUL. Return 0 on failure (NULL argument, wrong record version, or too
 * little capacity). On failure out[0] is set to '\0' when capacity is
 * nonzero, and no byte at or beyond capacity is written.
 */
size_t imu_csv_format_header(char *out, size_t capacity);
size_t imu_csv_format_row(const ImuPublicationRecord_t *record,
                          char *out, size_t capacity);

#endif /* IMU_CSV_H */
