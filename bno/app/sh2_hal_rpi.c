#define _POSIX_C_SOURCE 199309L

/*
 * sh2_hal_rpi.c
 *
 * Raspberry Pi 4B implementation of the sh2_Hal_t interface for the
 * BNO085 over SPI0 (/dev/spidev0.0), using libgpiod v1 for RST, H_INTN,
 * WAKE/PS0, and manual chip-select control.
 *
 * Responsibilities:
 *   - SPI configuration and raw two-phase SHTP read/write transfers
 *   - GPIO ownership for reset, interrupt, wake, and chip select
 *   - Reset sequencing and wake handshaking
 *   - Monotonic microsecond timestamps
 *
 * SHTP framing, fragmentation, sequence validation, and report parsing
 * belong to the CEVA SH-2 library.
 */

#include "sh2_hal.h"

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#include <gpiod.h>

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

#define SPI_DEV_PATH        "/dev/spidev0.0"
#define GPIO_CHIP_NAME      "gpiochip0"

#define GPIO_LINE_RST       13  /* NRST, active-low, physical pin 33 */
#define GPIO_LINE_INT       6   /* H_INTN, active-low, physical pin 31 */
#define GPIO_LINE_WAKE      5   /* PS0/WAKE, active-low, physical pin 29 */
#define GPIO_LINE_CS        25  /* HCSN, active-low, physical pin 22 */

#define SPI_MODE            (SPI_MODE_3 | SPI_NO_CS)
#define SPI_BITS_PER_WORD   8
#define SPI_MAX_SPEED_HZ    3000000U

#define SHTP_HEADER_LEN     4U

#define RESET_LOW_US        (10U * 1000U)
#define RESET_WAIT_US       (120U * 1000U)
#define WAKE_TIMEOUT_US     (200U * 1000U)
#define INT_POLL_STEP_US    500U

/* ------------------------------------------------------------------ */
/* Instance state                                                      */
/* ------------------------------------------------------------------ */

typedef struct sh2_hal_rpi_s {
    sh2_Hal_t hal;

    int spiFd;
    struct gpiod_chip *chip;
    struct gpiod_line *lineRst;
    struct gpiod_line *lineInt;
    struct gpiod_line *lineWake;
    struct gpiod_line *lineCs;

    bool isOpen;
} sh2_hal_rpi_t;

static sh2_hal_rpi_t sHalInstance;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static uint32_t rpi_getTimeUs(sh2_Hal_t *self)
{
    (void)self;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    uint64_t us = ((uint64_t)ts.tv_sec * 1000000ULL) +
                  ((uint64_t)ts.tv_nsec / 1000ULL);

    return (uint32_t)us;
}

static void sleep_us(uint32_t us)
{
    struct timespec ts;

    ts.tv_sec = (time_t)(us / 1000000U);
    ts.tv_nsec = (long)(us % 1000000U) * 1000L;

    nanosleep(&ts, NULL);
}

/*
 * Returns:
 *   1: H_INTN is asserted
 *   0: H_INTN is not asserted
 *  -1: GPIO read error
 */
static int int_is_asserted(sh2_hal_rpi_t *me)
{
    int value = gpiod_line_get_value(me->lineInt);

    if (value < 0) {
        return -1;
    }

    return value == 0;
}

static int cs_set(sh2_hal_rpi_t *me, int level)
{
    if (!me->lineCs) {
        return -1;
    }

    return gpiod_line_set_value(me->lineCs, level);
}

/*
 * SPI wake procedure:
 *   1. Drive WAKE low.
 *   2. Wait until H_INTN asserts.
 *   3. Release WAKE high.
 *
 * Returns:
 *   1: device is ready
 *   0: timeout
 *  -1: GPIO failure
 */
static int wake_sensor(sh2_hal_rpi_t *me)
{
    if (gpiod_line_set_value(me->lineWake, 0) != 0) {
        return -1;
    }

    uint32_t waitedUs = 0;

    while (waitedUs < WAKE_TIMEOUT_US) {
        int asserted = int_is_asserted(me);

        if (asserted < 0) {
            (void)gpiod_line_set_value(me->lineWake, 1);
            return -1;
        }

        if (asserted) {
            (void)gpiod_line_set_value(me->lineWake, 1);
            return 1;
        }

        sleep_us(INT_POLL_STEP_US);
        waitedUs += INT_POLL_STEP_US;
    }

    (void)gpiod_line_set_value(me->lineWake, 1);
    return 0;
}

/*
 * Raw full-duplex SPI transfer of exactly len bytes.
 * Returns 0 on success, -1 on failure.
 */
static int spi_transfer(int fd, const uint8_t *tx, uint8_t *rx, unsigned len)
{
    if (len == 0U) {
        return 0;
    }

    if (!tx && !rx) {
        return -1;
    }

    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));

    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = len;
    tr.speed_hz = SPI_MAX_SPEED_HZ;
    tr.bits_per_word = SPI_BITS_PER_WORD;
    tr.delay_usecs = 0;

    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

    return ret == (int)len ? 0 : -1;
}

/* ------------------------------------------------------------------ */
/* sh2_Hal_t callbacks                                                 */
/* ------------------------------------------------------------------ */

static int rpi_open(sh2_Hal_t *self)
{
    sh2_hal_rpi_t *me = (sh2_hal_rpi_t *)self;

    me->spiFd = -1;
    me->chip = NULL;
    me->lineRst = NULL;
    me->lineInt = NULL;
    me->lineWake = NULL;
    me->lineCs = NULL;
    me->isOpen = false;

    me->spiFd = open(SPI_DEV_PATH, O_RDWR);
    if (me->spiFd < 0) {
        fprintf(stderr, "sh2_hal_rpi: failed to open %s\n", SPI_DEV_PATH);
        return -1;
    }

    uint8_t mode = SPI_MODE;
    uint8_t bits = SPI_BITS_PER_WORD;
    uint32_t speed = SPI_MAX_SPEED_HZ;

    if (ioctl(me->spiFd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(me->spiFd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(me->spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        fprintf(stderr, "sh2_hal_rpi: SPI ioctl configuration failed\n");
        close(me->spiFd);
        me->spiFd = -1;
        return -1;
    }

    me->chip = gpiod_chip_open_by_name(GPIO_CHIP_NAME);
    if (!me->chip) {
        fprintf(stderr, "sh2_hal_rpi: failed to open %s\n", GPIO_CHIP_NAME);
        close(me->spiFd);
        me->spiFd = -1;
        return -1;
    }

    me->lineRst = gpiod_chip_get_line(me->chip, GPIO_LINE_RST);
    me->lineInt = gpiod_chip_get_line(me->chip, GPIO_LINE_INT);
    me->lineWake = gpiod_chip_get_line(me->chip, GPIO_LINE_WAKE);
    me->lineCs = gpiod_chip_get_line(me->chip, GPIO_LINE_CS);

    if (!me->lineRst || !me->lineInt || !me->lineWake || !me->lineCs) {
        fprintf(stderr, "sh2_hal_rpi: failed to get GPIO lines\n");
        goto fail_gpio;
    }

    if (gpiod_line_request_output(me->lineRst, "sh2_hal_rpi", 1) != 0) {
        fprintf(stderr, "sh2_hal_rpi: failed to request RST line\n");
        goto fail_gpio;
    }

    if (gpiod_line_request_input(me->lineInt, "sh2_hal_rpi") != 0) {
        fprintf(stderr, "sh2_hal_rpi: failed to request INT line\n");
        goto fail_gpio;
    }

    if (gpiod_line_request_output(me->lineWake, "sh2_hal_rpi", 1) != 0) {
        fprintf(stderr, "sh2_hal_rpi: failed to request WAKE line\n");
        goto fail_gpio;
    }

    if (gpiod_line_request_output(me->lineCs, "sh2_hal_rpi", 1) != 0) {
        fprintf(stderr, "sh2_hal_rpi: failed to request CS line\n");
        goto fail_gpio;
    }

    /*
     * Select SPI before reset by holding PS1 high externally and PS0/WAKE
     * high here. Then pulse NRST low and wait for BNO085 startup.
     */
    (void)gpiod_line_set_value(me->lineWake, 1);
    (void)gpiod_line_set_value(me->lineRst, 0);
    sleep_us(RESET_LOW_US);
    (void)gpiod_line_set_value(me->lineRst, 1);
    sleep_us(RESET_WAIT_US);

    me->isOpen = true;
    return 0;

fail_gpio:
    if (me->lineCs) {
        (void)gpiod_line_set_value(me->lineCs, 1);
    }

    if (me->lineRst) {
        gpiod_line_release(me->lineRst);
    }

    if (me->lineInt) {
        gpiod_line_release(me->lineInt);
    }

    if (me->lineWake) {
        gpiod_line_release(me->lineWake);
    }

    if (me->lineCs) {
        gpiod_line_release(me->lineCs);
    }

    if (me->chip) {
        gpiod_chip_close(me->chip);
    }

    me->chip = NULL;

    if (me->spiFd >= 0) {
        close(me->spiFd);
        me->spiFd = -1;
    }

    return -1;
}

static void rpi_close(sh2_Hal_t *self)
{
    sh2_hal_rpi_t *me = (sh2_hal_rpi_t *)self;

    if (!me->isOpen) {
        return;
    }

    if (me->lineRst) {
        (void)gpiod_line_set_value(me->lineRst, 0);
    }

    if (me->lineCs) {
        (void)gpiod_line_set_value(me->lineCs, 1);
    }

    if (me->lineRst) {
        gpiod_line_release(me->lineRst);
    }

    if (me->lineInt) {
        gpiod_line_release(me->lineInt);
    }

    if (me->lineWake) {
        gpiod_line_release(me->lineWake);
    }

    if (me->lineCs) {
        gpiod_line_release(me->lineCs);
    }

    if (me->chip) {
        gpiod_chip_close(me->chip);
    }

    me->lineRst = NULL;
    me->lineInt = NULL;
    me->lineWake = NULL;
    me->lineCs = NULL;
    me->chip = NULL;

    if (me->spiFd >= 0) {
        close(me->spiFd);
        me->spiFd = -1;
    }

    me->isOpen = false;
}

/*
 * Non-blocking read. If H_INTN is asserted, read one complete SHTP packet:
 *   1. assert CS;
 *   2. read its 4-byte header;
 *   3. decode the little-endian packet length with bit 15 masked;
 *   4. read exactly the remaining payload bytes while CS stays asserted;
 *   5. deassert CS.
 *
 * The raw packet is returned untouched to the SH-2 SHTP layer.
 */
static int rpi_read(sh2_Hal_t *self,
                    uint8_t *pBuffer,
                    unsigned len,
                    uint32_t *t_us)
{
    sh2_hal_rpi_t *me = (sh2_hal_rpi_t *)self;

    if (!me->isOpen || me->spiFd < 0) {
        return -1;
    }

    int asserted = int_is_asserted(me);

    if (asserted < 0) {
        return -1;
    }

    if (!asserted) {
        return 0;
    }

    uint32_t timestamp = rpi_getTimeUs(self);

    if (!pBuffer ||
        len < SHTP_HEADER_LEN ||
        len > SH2_HAL_MAX_TRANSFER_IN) {
        return -1;
    }

    static const uint8_t zeroTx[SH2_HAL_MAX_TRANSFER_IN] = {0};

    if (cs_set(me, 0) != 0) {
        return -1;
    }

    if (spi_transfer(me->spiFd, zeroTx, pBuffer, SHTP_HEADER_LEN) != 0) {
        (void)cs_set(me, 1);
        return -1;
    }

    uint16_t packetLen =
        (uint16_t)((((uint16_t)pBuffer[1] << 8) | pBuffer[0]) & 0x7fffU);

    if (packetLen < SHTP_HEADER_LEN ||
        packetLen > len ||
        packetLen > SH2_HAL_MAX_TRANSFER_IN) {
        (void)cs_set(me, 1);
        return -1;
    }

    unsigned payloadLen = (unsigned)packetLen - SHTP_HEADER_LEN;

    if (payloadLen > 0U &&
        spi_transfer(me->spiFd,
                     zeroTx,
                     pBuffer + SHTP_HEADER_LEN,
                     payloadLen) != 0) {
        (void)cs_set(me, 1);
        return -1;
    }

    if (cs_set(me, 1) != 0) {
        return -1;
    }

    if (t_us) {
        *t_us = timestamp;
    }

    return (int)packetLen;
}

/*
 * If needed, wake the BNO085, then write one complete outgoing SHTP packet
 * while manual chip select remains asserted.
 */
static int rpi_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    sh2_hal_rpi_t *me = (sh2_hal_rpi_t *)self;

    if (!me->isOpen || me->spiFd < 0 || !pBuffer ||
        len == 0U || len > SH2_HAL_MAX_TRANSFER_OUT) {
        return -1;
    }

    int asserted = int_is_asserted(me);

    if (asserted < 0) {
        return -1;
    }

    if (!asserted) {
        int wokeUp = wake_sensor(me);

        if (wokeUp != 1) {
            return wokeUp < 0 ? -1 : 0;
        }
    }

    static uint8_t dummyRx[SH2_HAL_MAX_TRANSFER_OUT];

    if (cs_set(me, 0) != 0) {
        return -1;
    }

    if (spi_transfer(me->spiFd, pBuffer, dummyRx, len) != 0) {
        (void)cs_set(me, 1);
        return -1;
    }

    if (cs_set(me, 1) != 0) {
        return -1;
    }

    return (int)len;
}

/* ------------------------------------------------------------------ */
/* Public constructor                                                  */
/* ------------------------------------------------------------------ */

sh2_Hal_t *sh2_hal_rpi_init(void)
{
    memset(&sHalInstance, 0, sizeof(sHalInstance));

    sHalInstance.hal.open = rpi_open;
    sHalInstance.hal.close = rpi_close;
    sHalInstance.hal.read = rpi_read;
    sHalInstance.hal.write = rpi_write;
    sHalInstance.hal.getTimeUs = rpi_getTimeUs;

    return &sHalInstance.hal;
}