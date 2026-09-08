/*
 * amt102.c — AMT102-V incremental encoder driver for Raspberry Pi 4B
 *
 * Uses Linux GPIO character device (/dev/gpiochip0) via libgpiod to monitor
 * both rising and falling edges on Channels A, B, and X (Index).
 * Edge events carry kernel CLOCK_MONOTONIC timestamps so encoder events
 * share the exact same clock domain as BNO085 SPI transactions.
 *
 * Part of the sensor_validate harness (see Makefile).
 */

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <gpiod.h>

#include "validation/amt102.h"
#include "validation/quad_decode.h"

#define GPIO_CHIP_NAME "gpiochip0"

/* Wiring matching production validation header */
#define PIN_CH_A 5   /* GPIO 5 (Pin 29) */
#define PIN_CH_B 6   /* GPIO 6 (Pin 31) */
#define PIN_CH_X 13  /* GPIO 13 (Pin 33) */

#define COUNTS_PER_REV 8192.0

static struct gpiod_chip *sChip = NULL;
static struct gpiod_line_bulk sLines;
static quad_decoder_t sDecoder;

static uint64_t sLastEdgeTsNs = 0;
static uint64_t sEdgesAB = 0;
static uint64_t sXPulses = 0;
static int sLastX = 0;

int amt102_init(void)
{
    sChip = gpiod_chip_open_by_name(GPIO_CHIP_NAME);
    if (!sChip) {
        return -1;
    }

    unsigned int offsets[3] = { PIN_CH_A, PIN_CH_B, PIN_CH_X };
    gpiod_line_bulk_init(&sLines);

    if (gpiod_chip_get_lines(sChip, offsets, 3, &sLines) < 0) {
        gpiod_chip_close(sChip);
        sChip = NULL;
        return -1;
    }

    struct gpiod_line_request_config config = {
        .consumer = "amt102_validate",
        .request_type = GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES,
        .flags = 0,
    };

    if (gpiod_line_bulk_request(&sLines, &config, NULL) < 0) {
        gpiod_chip_close(sChip);
        sChip = NULL;
        return -1;
    }

    struct gpiod_line *lineA = gpiod_line_bulk_get_line(&sLines, 0);
    struct gpiod_line *lineB = gpiod_line_bulk_get_line(&sLines, 1);
    struct gpiod_line *lineX = gpiod_line_bulk_get_line(&sLines, 2);

    int valA = gpiod_line_get_value(lineA);
    int valB = gpiod_line_get_value(lineB);
    int valX = gpiod_line_get_value(lineX);

    quad_decode_init(&sDecoder);
    quad_decode_step(&sDecoder, valA > 0, valB > 0);

    sLastX = valX > 0;
    sLastEdgeTsNs = 0;
    sEdgesAB = 0;
    sXPulses = 0;

    return 0;
}

void amt102_poll(void)
{
    if (!sChip) return;

    struct gpiod_line_event event;
    struct gpiod_line_bulk eventLines;

    while (gpiod_line_bulk_event_wait(&sLines, NULL, &eventLines) > 0) {
        unsigned int num_lines = gpiod_line_bulk_num_lines(&eventLines);
        for (unsigned int i = 0; i < num_lines; i++) {
            struct gpiod_line *line = gpiod_line_bulk_get_line(&eventLines, i);
            while (gpiod_line_event_read(line, &event) == 0) {
                unsigned int offset = gpiod_line_offset(line);
                uint64_t ts_ns = ((uint64_t)event.ts.tv_sec * 1000000000ull) +
                                 (uint64_t)event.ts.tv_nsec;
                sLastEdgeTsNs = ts_ns;

                struct gpiod_line *lA = gpiod_line_bulk_get_line(&sLines, 0);
                struct gpiod_line *lB = gpiod_line_bulk_get_line(&sLines, 1);
                struct gpiod_line *lX = gpiod_line_bulk_get_line(&sLines, 2);

                int vA = gpiod_line_get_value(lA);
                int vB = gpiod_line_get_value(lB);
                int vX = gpiod_line_get_value(lX);

                if (offset == PIN_CH_A || offset == PIN_CH_B) {
                    sEdgesAB++;
                    quad_decode_step(&sDecoder, vA > 0, vB > 0);
                } else if (offset == PIN_CH_X) {
                    if (!sLastX && vX > 0) {
                        sXPulses++;
                    }
                    sLastX = vX > 0;
                }
            }
        }
    }
}

void amt102_snapshot(Amt102Snapshot_t *out)
{
    if (!out) return;

    out->count               = sDecoder.count;
    out->angle_deg           = ((double)sDecoder.count * 360.0) / COUNTS_PER_REV;
    out->last_edge_ts_ns     = sLastEdgeTsNs;
    out->x_pulses            = sXPulses;
    out->invalid_transitions = sDecoder.invalid;
    out->edges_ab            = sEdgesAB;
}

void amt102_reset_count(void)
{
    quad_decode_init(&sDecoder);
    sEdgesAB = 0;
    sXPulses = 0;
}

void amt102_close(void)
{
    if (sChip) {
        gpiod_line_bulk_release(&sLines);
        gpiod_chip_close(sChip);
        sChip = NULL;
    }
}
