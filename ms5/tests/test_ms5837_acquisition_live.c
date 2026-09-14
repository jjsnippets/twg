#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "ms5837_driver.h"
#include "ms5837_hal_rpi.h"

#define DIAGNOSTIC_DURATION_NS UINT64_C(10000000000)
#define SERVICE_PERIOD_NS UINT64_C(1000000)

static uint64_t monotonic_ns(void)
{
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) < 0) {
        return 0U;
    }
    return ((uint64_t)now.tv_sec * UINT64_C(1000000000)) +
           (uint64_t)now.tv_nsec;
}

static void add_ns(struct timespec *time, uint64_t nanoseconds)
{
    time->tv_sec += (time_t)(nanoseconds / UINT64_C(1000000000));
    time->tv_nsec += (long)(nanoseconds % UINT64_C(1000000000));
    if (time->tv_nsec >= 1000000000L) {
        time->tv_nsec -= 1000000000L;
        ++time->tv_sec;
    }
}

int main(void)
{
    Ms5837HalRpi_t rpi;
    Ms5837Hal_t hal;
    Ms5837Driver_t driver;
    BaroRunConfig_t config;
    BaroRuntimeStats_t stats;
    BaroSample_t sample;
    struct timespec next_tick;
    uint64_t start_ns;
    uint64_t now_ns;
    uint64_t last_printed_sequence = 0U;
    int status;

    memset(&config, 0, sizeof(config));
    memset(&stats, 0, sizeof(stats));
    config.config_version = BARO_RUN_CONFIG_VERSION;
    config.fluid_density_kg_m3 = 997.0;
    config.surface_pressure_mbar = NAN;
    config.surface_pressure_valid = false;
    config.zero_at_start_requested = false;

    ms5837_hal_rpi_configure(&rpi, MS5837_DEFAULT_I2C_BUS,
                             MS5837_I2C_ADDRESS);
    hal = ms5837_hal_rpi_make(&rpi);
    status = ms5837_driver_init(&driver, &hal);
    if (status < 0) {
        fprintf(stderr, "MS5837 initialization failed on %s: %d (%s)\n",
                MS5837_DEFAULT_I2C_BUS, status,
                ((status <= -1) && (status >= -4095))
                    ? strerror(-status) : "driver validation error");
        return 1;
    }

    printf("MS5837 Step 4C live diagnostic\n");
    printf("Bus=%s address=0x%02X symmetric OSR=512 duration=10 s\n",
           MS5837_DEFAULT_I2C_BUS, MS5837_I2C_ADDRESS);

    if (clock_gettime(CLOCK_MONOTONIC, &next_tick) < 0) {
        perror("clock_gettime");
        (void)ms5837_driver_shutdown(&driver);
        return 1;
    }
    start_ns = monotonic_ns();
    now_ns = start_ns;

    while ((now_ns - start_ns) < DIAGNOSTIC_DURATION_NS) {
        status = ms5837_driver_service(&driver, now_ns, &config, &stats,
                                        &sample);
        if (status == MS5837_SERVICE_SAMPLE_READY) {
            if ((sample.measurement_sequence == 1U) ||
                ((sample.measurement_sequence - last_printed_sequence) >=
                 100U)) {
                printf("seq=%" PRIu64 " t_ns=%" PRIu64
                       " D1=%" PRIu32 " D2=%" PRIu32
                       " P=%.2f mbar T=%.2f C flags=0x%08" PRIX32 "\n",
                       sample.measurement_sequence,
                       sample.measurement_complete_time_ns,
                       sample.raw_pressure_d1, sample.raw_temperature_d2,
                       sample.pressure_mbar, sample.temperature_c,
                       sample.status_flags);
                last_printed_sequence = sample.measurement_sequence;
            }
        } else if ((status < 0) &&
                   (ms5837_driver_state(&driver) <
                    MS5837_STATE_RECOVERY_OPEN)) {
            fprintf(stderr, "Unrecoverable service error: %d\n", status);
            break;
        }

        add_ns(&next_tick, SERVICE_PERIOD_NS);
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                               &next_tick, NULL) == EINTR) {
        }
        now_ns = monotonic_ns();
    }

    printf("completed=%" PRIu64 " service_ticks=%" PRIu64
           " i2c_errors=%" PRIu64 " aborted_pairs=%" PRIu64
           " recoveries=%" PRIu64 "/%" PRIu64 "\n",
           stats.measurements_completed, stats.service_ticks,
           stats.i2c_errors, stats.aborted_pairs,
           stats.recovery_successes, stats.recovery_attempts);

    status = ms5837_driver_shutdown(&driver);
    if (status < 0) {
        fprintf(stderr, "Shutdown failed: %d\n", status);
        return 1;
    }
    return (stats.measurements_completed > 0U) ? 0 : 1;
}
