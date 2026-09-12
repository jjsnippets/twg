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

#include "../../bno/app/realtime.h"

#define MS5837_DEFAULT_I2C_BUS      "/dev/i2c-1"
#define MS5837_I2C_ADDR             0x76

#define MS5837_CMD_RESET            0x1E
#define MS5837_CMD_PROM_READ_BASE   0xA0
#define MS5837_CMD_ADC_READ         0x00

/* OSR Commands from Datasheet Page 9 */
#define MS5837_CMD_CONV_D1_OSR256   0x40
#define MS5837_CMD_CONV_D2_OSR256   0x50
#define MS5837_CMD_CONV_D1_OSR512   0x42
#define MS5837_CMD_CONV_D2_OSR512   0x52
#define MS5837_CMD_CONV_D1_OSR1024  0x44
#define MS5837_CMD_CONV_D2_OSR1024  0x54
#define MS5837_CMD_CONV_D1_OSR2048  0x46
#define MS5837_CMD_CONV_D2_OSR2048  0x56

typedef enum {
    STATE_START_D1 = 0,
    STATE_WAIT_D1,
    STATE_START_D2,
    STATE_WAIT_D2
} bench_state_t;

typedef struct {
    uint16_t c[7];
} ms5837_prom_t;

static inline int64_t get_time_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void sleep_ms(long ms) {
    struct timespec req;
    req.tv_sec = ms / 1000;
    req.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&req, NULL);
}

static void ms5837_calc(const ms5837_prom_t *prom, uint32_t d1, uint32_t d2,
                        int32_t *out_temp, int32_t *out_press) {
    int32_t dt = (int32_t)d2 - ((int32_t)prom->c[5] << 8);
    int32_t temp = 2000 + (int32_t)(((int64_t)dt * (int64_t)prom->c[6]) >> 23);
    int64_t off = ((int64_t)prom->c[2] << 17) + (((int64_t)prom->c[4] * (int64_t)dt) >> 6);
    int64_t sens = ((int64_t)prom->c[1] << 16) + (((int64_t)prom->c[3] * (int64_t)dt) >> 7);

    int64_t ti = 0, offi = 0, sensi = 0;
    if (temp < 2000) {
        int64_t t_diff = (int64_t)(temp - 2000);
        ti = (11 * ((int64_t)dt * (int64_t)dt)) >> 35;
        offi = (31 * (t_diff * t_diff)) >> 3;
        sensi = (63 * (t_diff * t_diff)) >> 5;
    }
    int64_t off2 = off - offi;
    int64_t sens2 = sens - sensi;
    *out_temp = temp - (int32_t)ti;
    *out_press = (int32_t)(((((int64_t)d1 * sens2) >> 21) - off2) >> 15);
}

static void run_benchmark(int fd, const ms5837_prom_t *prom, const char *label,
                          uint8_t cmd_d1, int64_t d1_delay_ns,
                          uint8_t cmd_d2, int64_t d2_delay_ns,
                          uint32_t temp_every_n, double duration_sec) {
    printf("----------------------------------------------------\n");
    printf(" Running Benchmark: %s\n", label);
    printf(" Ratio: 1 Temp every %u Pressure | D1 Delay: %.2f ms, D2 Delay: %.2f ms\n",
           temp_every_n, d1_delay_ns / 1e6, d2_delay_ns / 1e6);
    printf(" Duration: %.1f seconds | Service Loop: 1000 Hz\n", duration_sec);
    printf("----------------------------------------------------\n");

    bench_state_t state = STATE_START_D1;
    int64_t deadline_ns = 0;
    uint32_t d1 = 0, d2 = 0;
    int32_t temp = 0, press = 0;

    uint32_t pressure_count = 0;
    uint32_t completed_pressure_samples = 0;
    uint32_t loop_ticks = 0;
    uint32_t stale_publications = 0;
    int fresh_sample_available = 0;

    int64_t total_i2c_time_ns = 0;
    uint32_t i2c_tx_count = 0;

    const double loop_dt = 0.001;
    if (StartRT(80, loop_dt) != 0) {
        fprintf(stderr, "Warning: StartRT failed (run with sudo?)\n");
    }

    int64_t start_time_ns = get_time_ns();
    int64_t end_time_ns = start_time_ns + (int64_t)(duration_sec * 1e9);

    while (get_time_ns() < end_time_ns) {
        int64_t now_ns = get_time_ns();
        loop_ticks++;

        switch (state) {
            case STATE_START_D1: {
                int64_t t0 = get_time_ns();
                write(fd, &cmd_d1, 1);
                total_i2c_time_ns += (get_time_ns() - t0);
                i2c_tx_count++;

                deadline_ns = now_ns + d1_delay_ns;
                state = STATE_WAIT_D1;
                break;
            }

            case STATE_WAIT_D1: {
                if (now_ns >= deadline_ns) {
                    uint8_t buf[3] = {0};
                    uint8_t cmd_read = MS5837_CMD_ADC_READ;

                    int64_t t0 = get_time_ns();
                    write(fd, &cmd_read, 1);
                    read(fd, buf, 3);
                    total_i2c_time_ns += (get_time_ns() - t0);
                    i2c_tx_count += 2;

                    d1 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];
                    pressure_count++;
                    completed_pressure_samples++;

                    if (d2 != 0) {
                        ms5837_calc(prom, d1, d2, &temp, &press);
                        fresh_sample_available = 1;
                    }

                    if (pressure_count >= temp_every_n) {
                        pressure_count = 0;
                        state = STATE_START_D2;
                    } else {
                        state = STATE_START_D1;
                    }
                }
                break;
            }

            case STATE_START_D2: {
                int64_t t0 = get_time_ns();
                write(fd, &cmd_d2, 1);
                total_i2c_time_ns += (get_time_ns() - t0);
                i2c_tx_count++;

                deadline_ns = now_ns + d2_delay_ns;
                state = STATE_WAIT_D2;
                break;
            }

            case STATE_WAIT_D2: {
                if (now_ns >= deadline_ns) {
                    uint8_t buf[3] = {0};
                    uint8_t cmd_read = MS5837_CMD_ADC_READ;

                    int64_t t0 = get_time_ns();
                    write(fd, &cmd_read, 1);
                    read(fd, buf, 3);
                    total_i2c_time_ns += (get_time_ns() - t0);
                    i2c_tx_count += 2;

                    d2 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];

                    if (d1 != 0) {
                        ms5837_calc(prom, d1, d2, &temp, &press);
                    }

                    state = STATE_START_D1;
                }
                break;
            }
        }

        /* 100 Hz Publication Check (every 10 ticks = 10 ms) */
        if ((loop_ticks % 10) == 0) {
            if (!fresh_sample_available) {
                stale_publications++;
            }
            fresh_sample_available = 0;
        }

        RT_SleepUntil(loop_dt);
    }

    int64_t elapsed_ns = get_time_ns() - start_time_ns;
    double elapsed_sec = elapsed_ns / 1e9;
    double samples_per_sec = completed_pressure_samples / elapsed_sec;
    double avg_i2c_us = (i2c_tx_count > 0) ? ((double)total_i2c_time_ns / i2c_tx_count) / 1e3 : 0.0;

    printf("RESULTS for %s:\n", label);
    printf("  Elapsed Time          : %.3f s\n", elapsed_sec);
    printf("  1 kHz Service Ticks   : %u\n", loop_ticks);
    printf("  Completed Pressure    : %u (%.2f Hz)\n", completed_pressure_samples, samples_per_sec);
    printf("  100 Hz Publish Frames : %u\n", loop_ticks / 10);
    printf("  Stale 100 Hz Frames   : %u (%.2f %%)\n",
           stale_publications, (100.0 * stale_publications) / (loop_ticks / 10));
    printf("  Average I2C Tx Time   : %.2f us\n", avg_i2c_us);
    printf("  Last Raw Values       : D1=%u, D2=%u\n", d1, d2);
    printf("  Latest Value          : %.2f deg C | %.2f mbar\n\n",
           temp / 100.0, press / 100.0);
}

int main(int argc, char *argv[]) {
    const char *bus_path = (argc > 1) ? argv[1] : MS5837_DEFAULT_I2C_BUS;

    printf("====================================================\n");
    printf(" MS5837-02BA Step 3: Full Rate & OSR Benchmark\n");
    printf("====================================================\n\n");

    int fd = open(bus_path, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Failed to open %s: %s\n", bus_path, strerror(errno));
        return 1;
    }
    if (ioctl(fd, I2C_SLAVE, MS5837_I2C_ADDR) < 0) {
        fprintf(stderr, "Failed to set slave addr: %s\n", strerror(errno));
        close(fd);
        return 1;
    }

    /* Soft reset */
    uint8_t reset_cmd = MS5837_CMD_RESET;
    write(fd, &reset_cmd, 1);
    sleep_ms(10);

    /* Read PROM */
    ms5837_prom_t prom;
    for (int i = 0; i < 7; i++) {
        uint8_t cmd = MS5837_CMD_PROM_READ_BASE + (i * 2);
        uint8_t buf[2];
        write(fd, &cmd, 1);
        read(fd, buf, 2);
        prom.c[i] = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    }

    /* Prime initial D2 temperature */
    {
        uint8_t cmd_d2 = MS5837_CMD_CONV_D2_OSR2048;
        uint8_t cmd_read = MS5837_CMD_ADC_READ;
        uint8_t buf[3];
        write(fd, &cmd_d2, 1);
        sleep_ms(6);
        write(fd, &cmd_read, 1);
        read(fd, buf, 3);
    }

    const double test_duration = 10.0; /* 10 seconds per test */

    /* ================================================================
     * SET A: Pressure @ OSR 2048 (4.40 ms) + Temperature @ OSR 2048 (4.40 ms)
     * ================================================================ */
    printf("\n############################################################\n");
    printf(" SET A: Pressure OSR 2048 + Temperature OSR 2048\n");
    printf("############################################################\n\n");

    for (uint32_t ratio = 1; ratio <= 5; ratio++) {
        char label[128];
        snprintf(label, sizeof(label), "Press OSR 2048 + Temp OSR 2048 (1 Temp : %u Press)", ratio);
        run_benchmark(fd, &prom, label,
                      MS5837_CMD_CONV_D1_OSR2048, 4400000LL,
                      MS5837_CMD_CONV_D2_OSR2048, 4400000LL,
                      ratio, test_duration);
    }

    /* ================================================================
     * SET B: Pressure @ OSR 2048 (4.40 ms) + Temperature @ OSR 1024 (2.25 ms)
     * ================================================================ */
    printf("\n############################################################\n");
    printf(" SET B: Pressure OSR 2048 + Temperature OSR 1024\n");
    printf("############################################################\n\n");

    for (uint32_t ratio = 1; ratio <= 5; ratio++) {
        char label[128];
        snprintf(label, sizeof(label), "Press OSR 2048 + Temp OSR 1024 (1 Temp : %u Press)", ratio);
        run_benchmark(fd, &prom, label,
                      MS5837_CMD_CONV_D1_OSR2048, 4400000LL,
                      MS5837_CMD_CONV_D2_OSR1024, 2250000LL,
                      ratio, test_duration);
    }

    /* ================================================================
     * BASELINE COMPARISONS: Paired 1:1 at OSR 1024, 512, and 256
     * ================================================================ */
    printf("\n############################################################\n");
    printf(" BASELINE COMPARISONS: Paired 1:1 at OSR 1024, 512, and 256\n");
    printf("############################################################\n\n");

    /* Paired OSR 1024: max 2.17 ms -> 2.25 ms */
    run_benchmark(fd, &prom, "BASELINE Paired OSR 1024 (1:1)",
                  MS5837_CMD_CONV_D1_OSR1024, 2250000LL,
                  MS5837_CMD_CONV_D2_OSR1024, 2250000LL,
                  1, test_duration);

    /* Paired OSR 512: max 1.10 ms -> 1.15 ms */
    run_benchmark(fd, &prom, "BASELINE Paired OSR 512 (1:1)",
                  MS5837_CMD_CONV_D1_OSR512, 1150000LL,
                  MS5837_CMD_CONV_D2_OSR512, 1150000LL,
                  1, test_duration);

    /* Paired OSR 256: max 0.56 ms -> 0.60 ms */
    run_benchmark(fd, &prom, "BASELINE Paired OSR 256 (1:1)",
                  MS5837_CMD_CONV_D1_OSR256, 600000LL,
                  MS5837_CMD_CONV_D2_OSR256, 600000LL,
                  1, test_duration);

    close(fd);
    return 0;
}