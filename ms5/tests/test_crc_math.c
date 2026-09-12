#define _POSIX_C_SOURCE 199309L
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <linux/i2c-dev.h>

#define MS5837_DEFAULT_I2C_BUS    "/dev/i2c-1"
#define MS5837_I2C_ADDR           0x76

#define MS5837_CMD_RESET          0x1E
#define MS5837_CMD_PROM_READ_BASE 0xA0
#define MS5837_CMD_CONV_D1_OSR2048 0x46
#define MS5837_CMD_CONV_D2_OSR2048 0x56
#define MS5837_CMD_ADC_READ       0x00

typedef struct {
    uint16_t c[7]; /* c[0]=CRC/factory, c[1..6]=coefficients */
} ms5837_prom_t;

typedef struct {
    int32_t temp_c_hundredths; /* Temperature in 0.01 deg C */
    int32_t press_mbar_hundredths; /* Pressure in 0.01 mbar */
} ms5837_result_t;

static void sleep_ms(long ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

/*
 * Official TE Connectivity 4-bit CRC algorithm (Datasheet page 13).
 */
static uint8_t ms5837_crc4(const uint16_t prom_in[7]) {
    uint16_t n_prom[8];
    for (int i = 0; i < 7; i++) {
        n_prom[i] = prom_in[i];
    }
    n_prom[7] = 0;

    /* Replace CRC nibble with 0 for computation */
    n_prom[0] = (n_prom[0] & 0x0FFF);

    uint16_t n_rem = 0;
    for (int cnt = 0; cnt < 16; cnt++) {
        if (cnt % 2 == 1) {
            n_rem ^= (n_prom[cnt >> 1] & 0x00FF);
        } else {
            n_rem ^= (n_prom[cnt >> 1] >> 8);
        }

        for (int n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem = (n_rem << 1);
            }
        }
    }
    n_rem = ((n_rem >> 12) & 0x000F);
    return (uint8_t)(n_rem ^ 0x00);
}

/*
 * First & second-order compensation algorithm (Datasheet pages 7-8).
 */
static ms5837_result_t ms5837_calculate(const ms5837_prom_t *prom, uint32_t d1, uint32_t d2) {
    ms5837_result_t res;

    /* Step 1: Difference between actual and reference temperature */
    int32_t dt = (int32_t)d2 - ((int32_t)prom->c[5] << 8);

    /* Step 2: 1st order temperature (-40 to 85 deg C, resolution 0.01 deg C) */
    int32_t temp = 2000 + (int32_t)(((int64_t)dt * (int64_t)prom->c[6]) >> 23);

    /* Step 3: Offset and Sensitivity at actual temperature */
    int64_t off = ((int64_t)prom->c[2] << 17) + (((int64_t)prom->c[4] * (int64_t)dt) >> 6);
    int64_t sens = ((int64_t)prom->c[1] << 16) + (((int64_t)prom->c[3] * (int64_t)dt) >> 7);

    /* Step 4: 2nd order temperature compensation */
    int64_t ti = 0;
    int64_t offi = 0;
    int64_t sensi = 0;

    if (temp < 2000) {
        /* Low temperature (< 20.00 deg C) */
        int64_t t_diff = (int64_t)(temp - 2000);
        ti = (11 * ((int64_t)dt * (int64_t)dt)) >> 35;
        offi = (31 * (t_diff * t_diff)) >> 3;
        sensi = (63 * (t_diff * t_diff)) >> 5;
    }

    int64_t off2 = off - offi;
    int64_t sens2 = sens - sensi;
    int32_t temp2 = temp - (int32_t)ti;

    /* Pressure calculation: (10 to 1200 mbar, resolution 0.01 mbar) */
    int32_t p2 = (int32_t)(((((int64_t)d1 * sens2) >> 21) - off2) >> 15);

    res.temp_c_hundredths = temp2;
    res.press_mbar_hundredths = p2;
    return res;
}

/* Run deterministic datasheet example vectors */
static int test_datasheet_example_vector(void) {
    ms5837_prom_t ref_prom = {
        .c = {0, 46372, 43981, 29059, 27842, 31553, 28165}
    };
    uint32_t ref_d1 = 6465444;
    uint32_t ref_d2 = 8077636;

    ms5837_result_t res = ms5837_calculate(&ref_prom, ref_d1, ref_d2);

    printf("--- Datasheet Reference Verification ---\n");
    printf("Expected: Temp = 20.00 deg C, Press = 1100.02 mbar\n");
    printf("Computed: Temp = %.2f deg C, Press = %.2f mbar\n",
           res.temp_c_hundredths / 100.0, res.press_mbar_hundredths / 100.0);

    if (res.temp_c_hundredths == 2000 && res.press_mbar_hundredths == 110002) {
        printf("[OK] Datasheet example compensation matched perfectly.\n\n");
        return 0;
    } else {
        printf("[FAIL] Math mismatch with datasheet worked example!\n\n");
        return -1;
    }
}

int main(int argc, char *argv[]) {
    const char *bus_path = (argc > 1) ? argv[1] : MS5837_DEFAULT_I2C_BUS;

    printf("====================================================\n");
    printf(" MS5837-02BA Step 2: Math & Live Reading Test\n");
    printf("====================================================\n\n");

    /* Part 1: Offline Math Unit Test */
    if (test_datasheet_example_vector() != 0) {
        return 1;
    }

    /* Part 2: Hardware Acquisition & Live Ambient Reading */
    int fd = open(bus_path, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "[FAIL] Cannot open %s: %s\n", bus_path, strerror(errno));
        return 1;
    }

    if (ioctl(fd, I2C_SLAVE, MS5837_I2C_ADDR) < 0) {
        fprintf(stderr, "[FAIL] ioctl(I2C_SLAVE) failed: %s\n", strerror(errno));
        close(fd);
        return 1;
    }

    /* Read PROM */
    ms5837_prom_t prom;
    for (int i = 0; i < 7; i++) {
        uint8_t cmd = MS5837_CMD_PROM_READ_BASE + (i * 2);
        uint8_t buf[2];
        write(fd, &cmd, 1);
        read(fd, buf, 2);
        prom.c[i] = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    }

    /* CRC verification */
    uint8_t expected_crc = (prom.c[0] >> 12) & 0x0F;
    uint8_t computed_crc = ms5837_crc4(prom.c);
    printf("--- Sensor PROM CRC-4 Integrity ---\n");
    printf("Stored CRC: 0x%X | Computed CRC: 0x%X\n", expected_crc, computed_crc);
    if (expected_crc != computed_crc) {
        fprintf(stderr, "[FAIL] CRC mismatch! Corrupted PROM.\n");
        close(fd);
        return 1;
    }
    printf("[OK] PROM CRC-4 matches.\n\n");

    /* Acquire 5 live test samples */
    printf("--- Live Ambient Conversion (OSR 2048) ---\n");
    for (int sample = 1; sample <= 5; sample++) {
        uint8_t cmd;
        uint8_t buf[3];
        uint32_t d1 = 0;
        uint32_t d2 = 0;

        /* Convert D1 (Pressure, OSR 2048: max 4.32 ms) */
        cmd = MS5837_CMD_CONV_D1_OSR2048;
        write(fd, &cmd, 1);
        sleep_ms(6); /* Conservative blocking wait for bring-up test */
        cmd = MS5837_CMD_ADC_READ;
        write(fd, &cmd, 1);
        read(fd, buf, 3);
        d1 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];

        /* Convert D2 (Temperature, OSR 2048: max 4.32 ms) */
        cmd = MS5837_CMD_CONV_D2_OSR2048;
        write(fd, &cmd, 1);
        sleep_ms(6); /* Conservative blocking wait for bring-up test */
        cmd = MS5837_CMD_ADC_READ;
        write(fd, &cmd, 1);
        read(fd, buf, 3);
        d2 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];

        ms5837_result_t res = ms5837_calculate(&prom, d1, d2);

        printf("Sample #%d: D1=%-8u D2=%-8u => Temp = %6.2f deg C | Press = %7.2f mbar (%.4f atm)\n",
               sample, d1, d2,
               res.temp_c_hundredths / 100.0,
               res.press_mbar_hundredths / 100.0,
               (res.press_mbar_hundredths / 100.0) / 1013.25);

        sleep_ms(200);
    }

    close(fd);
    printf("\n====================================================\n");
    printf(" STATUS: STEP 2 COMPLETE AND VERIFIED.\n");
    printf("====================================================\n");
    return 0;
}