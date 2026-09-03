/*
 * test_hal_raw.c
 *
 * Stage 1 minimal test harness: exercises sh2_hal_rpi.c directly,
 * completely independent of sh2/ (sh2.c, shtp.c, etc.).
 *
 * Goal: confirm the HAL's hardware-facing contract (open/read/write/
 * getTimeUs, GPIO reset sequencing, INT-gated SPI transfers, WAKE
 * handshake) behaves sanely BEFORE handing control to sh2_lib.
 *
 * This is a throwaway diagnostic program -- not part of the final
 * application (main.c / sensor_app.c / control_loop.c).
 *
 * Build (example):
 *   gcc -o test_hal_raw test_hal_raw.c sh2_hal_rpi.c -I../sh2 -lgpiod
 *
 * Directory layout assumed:
 *   bno/
 *     sh2/        <- CEVA sh2 library (unused by this test)
 *     app/
 *       sh2_hal_rpi.c
 *       sh2_hal_rpi.h  (optional, or rely on sh2_hal_rpi_init() decl)
 *       test_hal_raw.c  <- this file
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "sh2_hal.h"   /* for sh2_Hal_t and SH2_HAL_MAX_TRANSFER_IN/OUT */

/* Declared in sh2_hal_rpi.c; no header strictly required for this test. */
extern sh2_Hal_t *sh2_hal_rpi_init(void);

#define TEST_DURATION_SEC      10
#define READ_BUF_LEN           SH2_HAL_MAX_TRANSFER_IN
#define POLL_INTERVAL_US       1000   /* 1 ms between read() polls */

static void dump_hex(const uint8_t *buf, unsigned len, unsigned maxBytes)
{
    unsigned n = (len < maxBytes) ? len : maxBytes;
    for (unsigned i = 0; i < n; i++) {
        printf("%02X ", buf[i]);
    }
    if (len > maxBytes) {
        printf("...");
    }
    printf("\n");
}

/* Parses just the 4-byte SHTP header for sanity display. */
static void print_shtp_header(const uint8_t *buf, unsigned len)
{
    if (len < 4) {
        printf("    [too short for SHTP header]\n");
        return;
    }
    uint16_t lengthField = (uint16_t)(buf[0] | ((buf[1] & 0x7F) << 8));
    uint8_t continuation = (buf[1] & 0x80) ? 1 : 0;
    uint8_t channel = buf[2];
    uint8_t seq = buf[3];

    printf("    SHTP header: length=%u continuation=%u channel=%u seq=%u\n",
           lengthField, continuation, channel, seq);
}

int main(void)
{
    sh2_Hal_t *hal = sh2_hal_rpi_init();
    if (!hal) {
        fprintf(stderr, "FAIL: sh2_hal_rpi_init() returned NULL\n");
        return 1;
    }

    printf("== Stage 1: raw HAL test (no sh2_lib) ==\n");

    /* --- open() --- */
    int rc = hal->open(hal);
    if (rc != 0) {
        fprintf(stderr, "FAIL: hal->open() returned %d\n", rc);
        return 1;
    }
    printf("PASS: hal->open() succeeded\n");

    /* --- getTimeUs() monotonicity check --- */
    uint32_t t0 = hal->getTimeUs(hal);
    usleep(1000);
    uint32_t t1 = hal->getTimeUs(hal);
    if (t1 <= t0) {
        fprintf(stderr, "FAIL: getTimeUs() not increasing (t0=%u t1=%u)\n", t0, t1);
    } else {
        printf("PASS: getTimeUs() increasing (t0=%u t1=%u, delta=%u us)\n",
               t0, t1, t1 - t0);
    }

    /* --- read() polling loop: expect startup traffic (advertisement, --- */
    /* --- reset-complete, etc.) to appear shortly after open().     --- */
    printf("\n-- Polling read() for %d seconds to observe startup traffic --\n",
           TEST_DURATION_SEC);

    static uint8_t buf[READ_BUF_LEN];
    uint32_t timestamp = 0;
    int messagesSeen = 0;

    time_t startTime = time(NULL);
    while (time(NULL) - startTime < TEST_DURATION_SEC) {
        int n = hal->read(hal, buf, READ_BUF_LEN, &timestamp);

        if (n > 0) {
            messagesSeen++;
            printf("[msg %d] read() returned %d bytes, t_us=%u\n",
                   messagesSeen, n, timestamp);
            print_shtp_header(buf, (unsigned)n);
            printf("    bytes: ");
            dump_hex(buf, (unsigned)n, 32);
        } else if (n < 0) {
            fprintf(stderr, "FAIL: hal->read() returned error %d\n", n);
            break;
        }
        /* n == 0: no data ready, keep polling */

        usleep(POLL_INTERVAL_US);
    }

    if (messagesSeen == 0) {
        fprintf(stderr,
            "FAIL: no messages observed after reset -- check reset timing, "
            "INT wiring/polarity, or SPI mode/speed.\n");
    } else {
        printf("\nPASS: observed %d message(s) after reset "
               "(expect advertisement + reset-complete at minimum)\n",
               messagesSeen);
    }

    /* --- optional write() smoke test: send SH-2 device 'reset' command --- */
    /* Device channel = 0, cargo = single byte 0x01 (reset), per the      */
    /* SH-2 SHTP reference manual's device channel command definitions.  */
    printf("\n-- Attempting a minimal write() (device channel reset command) --\n");

    uint8_t resetCmd[5];
    /* Minimal 4-byte SHTP header + 1 byte cargo. Length field = 5 (total). */
    resetCmd[0] = 5;      /* length LSB */
    resetCmd[1] = 0;      /* length MSB, continuation bit clear */
    resetCmd[2] = 0;      /* channel 0 = device */
    resetCmd[3] = 0;      /* sequence number (test harness, not tracked) */
    resetCmd[4] = 1;      /* cargo: 1 = reset command */

    int wn = hal->write(hal, resetCmd, sizeof(resetCmd));
    if (wn == (int)sizeof(resetCmd)) {
        printf("PASS: hal->write() accepted %d bytes\n", wn);
    } else if (wn == 0) {
        printf("INFO: hal->write() returned 0 (device not ready / wake failed)\n");
    } else {
        fprintf(stderr, "FAIL: hal->write() returned %d\n", wn);
    }

    /* Give the device a moment to respond, then poll read() again briefly. */
    printf("\n-- Polling read() for 3 more seconds after write() --\n");
    time_t writeCheckStart = time(NULL);
    int postWriteMessages = 0;
    while (time(NULL) - writeCheckStart < 3) {
        int n = hal->read(hal, buf, READ_BUF_LEN, &timestamp);
        if (n > 0) {
            postWriteMessages++;
            printf("[post-write msg %d] %d bytes, t_us=%u\n",
                   postWriteMessages, n, timestamp);
            print_shtp_header(buf, (unsigned)n);
        }
        usleep(POLL_INTERVAL_US);
    }
    printf("(observed %d message(s) after write attempt)\n", postWriteMessages);

    /* --- close() --- */
    hal->close(hal);
    printf("\nPASS: hal->close() completed\n");

    printf("\n== Stage 1 test complete ==\n");
    printf("Total startup messages: %d, post-write messages: %d\n",
           messagesSeen, postWriteMessages);

    return 0;
}
