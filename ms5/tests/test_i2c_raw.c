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

#define MS5837_DEFAULT_I2C_BUS   "/dev/i2c-1"
#define MS5837_I2C_ADDR          0x76

/* Commands from MS5837-02BA01 Datasheet */
#define MS5837_CMD_RESET         0x1E
#define MS5837_CMD_PROM_READ_BASE 0xA0
#define MS5837_PROM_WORD_COUNT   7

static void sleep_ms(long ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

int main(int argc, char *argv[]) {
    const char *bus_path = (argc > 1) ? argv[1] : MS5837_DEFAULT_I2C_BUS;
    int fd;
    uint8_t cmd;
    uint16_t prom[MS5837_PROM_WORD_COUNT] = {0};
    int pass = 1;

    printf("====================================================\n");
    printf(" MS5837-02BA Raw I2C Bring-up & PROM Dump\n");
    printf(" Bus: %s | Target 7-bit Address: 0x%02X\n", bus_path, MS5837_I2C_ADDR);
    printf("====================================================\n\n");

    /* Step 1: Open I2C device node */
    fd = open(bus_path, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "[FAIL] Cannot open %s: %s (errno %d)\n", bus_path, strerror(errno), errno);
        return 1;
    }
    printf("[OK] Opened %s (fd=%d)\n", bus_path, fd);

    /* Step 2: Set slave address */
    if (ioctl(fd, I2C_SLAVE, MS5837_I2C_ADDR) < 0) {
        fprintf(stderr, "[FAIL] ioctl(I2C_SLAVE, 0x%02X) failed: %s (errno %d)\n",
                MS5837_I2C_ADDR, strerror(errno), errno);
        close(fd);
        return 1;
    }
    printf("[OK] Set I2C slave address to 0x%02X\n", MS5837_I2C_ADDR);

    /* Step 3: Issue Soft Reset Command (0x1E) */
    cmd = MS5837_CMD_RESET;
    printf("[INFO] Sending Soft Reset (0x1E)...\n");
    if (write(fd, &cmd, 1) != 1) {
        fprintf(stderr, "[FAIL] Failed to write reset command: %s (errno %d)\n", strerror(errno), errno);
        fprintf(stderr, "       Check power (3.3V/GND) and bus lines (SDA Pin 3, SCL Pin 5).\n");
        close(fd);
        return 1;
    }
    printf("[OK] Reset command ACK received.\n");

    /* Datasheet specifies 2.8 ms reload time after reset; wait 10 ms for safety */
    sleep_ms(10);

    /* Step 4: Read 7 PROM Words (0xA0 to 0xAE) */
    printf("\n--- Reading Calibration PROM (7 words x 16-bit) ---\n");
    for (int i = 0; i < MS5837_PROM_WORD_COUNT; i++) {
        uint8_t prom_cmd = MS5837_CMD_PROM_READ_BASE + (i * 2);
        uint8_t rx_buf[2] = {0};

        if (write(fd, &prom_cmd, 1) != 1) {
            fprintf(stderr, "[FAIL] Failed to send PROM read command 0x%02X for word %d: %s\n",
                    prom_cmd, i, strerror(errno));
            pass = 0;
            break;
        }

        if (read(fd, rx_buf, 2) != 2) {
            fprintf(stderr, "[FAIL] Failed to read 2 bytes for PROM word %d: %s\n", i, strerror(errno));
            pass = 0;
            break;
        }

        prom[i] = ((uint16_t)rx_buf[0] << 8) | (uint16_t)rx_buf[1];
    }

    if (!pass) {
        close(fd);
        return 1;
    }

    /* Step 5: Display and validate raw coefficients */
    printf("Word 0 (Reserved / CRC-4) : 0x%04X (CRC nibble: 0x%X)\n", prom[0], (prom[0] >> 12) & 0x0F);
    printf("Word 1 (C1: Pressure Sens): %u (0x%04X)\n", prom[1], prom[1]);
    printf("Word 2 (C2: Pressure Off) : %u (0x%04X)\n", prom[2], prom[2]);
    printf("Word 3 (C3: Temp Coeff TCS): %u (0x%04X)\n", prom[3], prom[3]);
    printf("Word 4 (C4: Temp Coeff TCO): %u (0x%04X)\n", prom[4], prom[4]);
    printf("Word 5 (C5: Ref Temp TREF): %u (0x%04X)\n", prom[5], prom[5]);
    printf("Word 6 (C6: Temp Coeff TEMPSENS): %u (0x%04X)\n\n", prom[6], prom[6]);

    /* Basic sanity checks: Coefficients C1-C6 must not be 0x0000 or 0xFFFF */
    for (int i = 1; i <= 6; i++) {
        if (prom[i] == 0x0000 || prom[i] == 0xFFFF) {
            fprintf(stderr, "[WARN] Coefficient C%d is invalid (0x%04X)! Possible communication fault.\n",
                    i, prom[i]);
            pass = 0;
        }
    }

    if (pass) {
        printf("====================================================\n");
        printf(" STATUS: SUCCESS. Electrical & I2C communication verified!\n");
        printf("====================================================\n");
    } else {
        printf("====================================================\n");
        printf(" STATUS: FAILURE. PROM coefficients appear corrupted.\n");
        printf("====================================================\n");
    }

    close(fd);
    return pass ? 0 : 1;
}