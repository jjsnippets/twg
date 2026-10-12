#define _POSIX_C_SOURCE 200809L

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "ms5837_driver.h"
#include "ms5837_hal_rpi.h"

static uint64_t monotonic_ns(void)
{
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) < 0) {
        return 0U;
    }
    return ((uint64_t)now.tv_sec * UINT64_C(1000000000)) +
           (uint64_t)now.tv_nsec;
}

int main(void)
{
    Ms5837HalRpi_t rpi;
    Ms5837Hal_t hal;
    Ms5837Driver_t driver;
    BaroDriverStats_t stats;
    BaroSample_t sample;
    int status;

    memset(&stats, 0, sizeof(stats));
    ms5837_hal_rpi_configure(&rpi, MS5837_DEFAULT_I2C_BUS,
                             MS5837_I2C_ADDRESS);
    hal = ms5837_hal_rpi_make(&rpi);
    status = ms5837_driver_init(&driver, &hal);
    if (status < 0) {
        fprintf(stderr, "FAIL: initialization: %d\n", status);
        return EXIT_FAILURE;
    }
    status = ms5837_driver_trigger(&driver, &stats);
    while ((status >= 0) &&
           (status != MS5837_SERVICE_SAMPLE_READY)) {
        struct timespec pause = {0, 100000L};
        (void)nanosleep(&pause, NULL);
        status = ms5837_driver_service(&driver, monotonic_ns(),
                                        &stats, &sample);
    }
    if (status != MS5837_SERVICE_SAMPLE_READY) {
        fprintf(stderr, "FAIL: acquisition: %d\n", status);
        (void)ms5837_driver_shutdown(&driver);
        return EXIT_FAILURE;
    }
    printf("PASS: seq=%" PRIu64 " pressure=%.2f temperature=%.2f\n",
           sample.measurement_sequence, sample.pressure_mbar,
           sample.temperature_c);
    (void)ms5837_driver_shutdown(&driver);
    return EXIT_SUCCESS;
}
