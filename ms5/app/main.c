#define _POSIX_C_SOURCE 200809L

/*
 * MS5837-02BA capture application.
 *
 * CLI:
 *   sudo ./bin/ms5837_capture [--duration SECONDS] [--zero]
 *                              [--density KG_PER_M3]
 *
 * --duration sets only the post-zero capture window (default: 10 seconds).
 * --zero first collects a separate five-second surface-pressure reference;
 *         without --zero, no taring is performed and depth remains NaN.
 * --density selects fluid density (default: 1000 kg/m^3, freshwater).
 *
 * The bus (/dev/i2c-1), address (0x76), SCHED_FIFO priority (90), 100 Hz
 * frame/publication cadence, and symmetric OSR 512 are fixed settings.
 * The dedicated I2C bus permits one symmetric D1/D2 conversion pair per
 * frame: D1 is triggered at the beginning of each 10 ms frame. A concise
 * telemetry line is printed every 50 frames (500 ms). Every frame record is
 * queued to a timestamped CSV file in the current working directory.
 */

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "ms5837_contract.h"
#include "ms5837_cli.h"
#include "ms5837_driver.h"
#include "ms5837_hal_rpi.h"
#include "ms5837_logger.h"
#include "ms5837_reference.h"
#include "rt/realtime.h"

#define RT_PRIORITY 90
#define FRAME_PERIOD_SEC 0.010
#define FRAME_PERIOD_NS UINT64_C(10000000)
#define CONVERSION_POLL_NS UINT64_C(100000)
#define TELEMETRY_FRAMES UINT64_C(50)

static volatile sig_atomic_t stop_requested;

static void request_stop(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

static uint64_t monotonic_ns(void)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) < 0) {
        return 0U;
    }
    return ((uint64_t)now.tv_sec * UINT64_C(1000000000)) +
           (uint64_t)now.tv_nsec;
}

typedef struct {
    double fluid_density_kg_m3;
    double surface_pressure_mbar;
    bool surface_pressure_valid;
    bool zero_at_start_requested;
} AppDepthConfig_t;

typedef struct {
    uint64_t publications_attempted;
    uint64_t publications_not_ready;
    uint64_t publications_stale;
    uint64_t service_overruns;
    uint64_t logger_drops;
    uint64_t maximum_loop_body_ns;
} AppRuntimeStats_t;

static BaroSample_t not_ready_sample(void)
{
    BaroSample_t sample;
    ms5837_sample_set_not_ready(&sample);
    return sample;
}

static double sample_depth(const BaroSample_t *sample,
                           const AppDepthConfig_t *config)
{
    if ((sample == NULL) || (config == NULL) ||
        !config->surface_pressure_valid) {
        return NAN;
    }
    return ms5837_depth_from_reference(
        sample->pressure_mbar, config->surface_pressure_mbar,
        config->fluid_density_kg_m3);
}

static void publish_record(Ms5837Logger_t *logger,
                           const Ms5837Driver_t *driver,
                           const AppDepthConfig_t *config,
                           BaroCapturePhase_t phase,
                           uint64_t publication_sequence,
                           uint64_t publication_time_ns,
                           uint64_t *last_published_measurement,
                           AppRuntimeStats_t *app_stats)
{
    BaroLogRecord_t record;

    memset(&record, 0, sizeof(record));
    record.publication_sequence = publication_sequence;
    record.publication_time_ns = publication_time_ns;
    record.phase = phase;
    record.fluid_density_kg_m3 = config->fluid_density_kg_m3;
    record.surface_pressure_mbar = config->surface_pressure_mbar;
    record.surface_pressure_valid = config->surface_pressure_valid;
    record.sample = not_ready_sample();

    ++app_stats->publications_attempted;
    if (ms5837_driver_latest_sample(driver, &record.sample)) {
        record.sample_ready = true;
        if (record.sample.measurement_sequence == *last_published_measurement) {
            record.sample_stale = true;
            ++app_stats->publications_stale;
        } else {
            *last_published_measurement = record.sample.measurement_sequence;
        }
    } else {
        ++app_stats->publications_not_ready;
    }

    record.depth_m = sample_depth(&record.sample, config);
    if (ms5837_logger_enqueue(logger, &record) < 0) {
        ++app_stats->logger_drops;
    }
}

static void print_telemetry(uint64_t tick, BaroCapturePhase_t phase,
                            const Ms5837Driver_t *driver,
                            const AppDepthConfig_t *config,
                            const BaroDriverStats_t *driver_stats,
                            const AppRuntimeStats_t *app_stats)
{
    BaroSample_t sample;

    if (ms5837_driver_latest_sample(driver, &sample)) {
        printf("frame=%" PRIu64 " phase=%s seq=%" PRIu64
               " P=%.2f_mbar T=%.2f_C depth=%.4f_m flags=0x%08" PRIX32
               " i2c=%" PRIu64 " drops=%" PRIu64 "\n",
               tick, phase == BARO_CAPTURE_PHASE_ZERO ? "zero" : "run",
               sample.measurement_sequence, sample.pressure_mbar,
               sample.temperature_c, sample_depth(&sample, config),
               sample.status_flags, driver_stats->i2c_errors,
               app_stats->logger_drops);
    } else {
        printf("frame=%" PRIu64 " phase=%s sample=not_ready i2c=%" PRIu64
               " drops=%" PRIu64 " density=%.3f\n",
               tick, phase == BARO_CAPTURE_PHASE_ZERO ? "zero" : "run",
               driver_stats->i2c_errors, app_stats->logger_drops,
               config->fluid_density_kg_m3);
    }
}

static bool recovery_active(const Ms5837Driver_t *driver)
{
    Ms5837DriverState_t state = ms5837_driver_state(driver);
    return (state >= MS5837_STATE_RECOVERY_BACKOFF) &&
           (state <= MS5837_STATE_RECOVERY_VALIDATE);
}

static int service_until_idle(Ms5837Driver_t *driver,
                              BaroDriverStats_t *driver_stats,
                              Ms5837Reference_t *reference,
                              BaroCapturePhase_t phase,
                              uint64_t frame_deadline_ns)
{
    BaroSample_t completed;

    while (!stop_requested) {
        uint64_t now_ns = monotonic_ns();
        int service_status;

        if (now_ns >= frame_deadline_ns) {
            return -ETIMEDOUT;
        }
        service_status = ms5837_driver_service(
            driver, now_ns, driver_stats, &completed);
        if (service_status == MS5837_SERVICE_SAMPLE_READY) {
            if ((phase == BARO_CAPTURE_PHASE_ZERO) &&
                (reference != NULL)) {
                (void)ms5837_reference_add_sample(reference, &completed);
            }
            return 0;
        }
        if ((service_status < 0) && !recovery_active(driver)) {
            return service_status;
        }
        if (ms5837_driver_state(driver) == MS5837_STATE_IDLE) {
            return 0;
        }
        {
            struct timespec pause;
            pause.tv_sec = 0;
            pause.tv_nsec = (long)CONVERSION_POLL_NS;
            while ((nanosleep(&pause, &pause) < 0) && (errno == EINTR) &&
                   !stop_requested) {
            }
        }
    }
    return -EINTR;
}

static int run_phase(Ms5837Driver_t *driver, Ms5837Logger_t *logger,
                     AppDepthConfig_t *config, BaroDriverStats_t *driver_stats,
                     AppRuntimeStats_t *app_stats,
                     Ms5837Reference_t *reference,
                     BaroCapturePhase_t phase, uint64_t duration_ns,
                     uint64_t *global_frame, uint64_t *publication_sequence,
                     uint64_t *last_published_measurement)
{
    uint64_t start_ns = monotonic_ns();
    uint64_t frame_start_ns = start_ns;

    while (!stop_requested && ((frame_start_ns - start_ns) < duration_ns)) {
        uint64_t body_start_ns = monotonic_ns();
        uint64_t frame_deadline_ns = frame_start_ns + FRAME_PERIOD_NS;
        int frame_status;

        /* Trigger D1 first: this is the defined beginning of every frame. */
        frame_status = ms5837_driver_trigger(driver, driver_stats);
        if (frame_status == MS5837_DRIVER_ERR_BUSY) {
            /* Finish a late prior pair so one overrun cannot stall capture. */
            frame_status = service_until_idle(
                driver, driver_stats, reference, phase, frame_deadline_ns);
            if (frame_status == 0) {
                frame_status = -ETIMEDOUT;
            }
        } else if (frame_status < 0) {
            if (recovery_active(driver)) {
                frame_status = service_until_idle(
                    driver, driver_stats, NULL, phase, frame_deadline_ns);
            }
        } else {
            frame_status = service_until_idle(
                driver, driver_stats, reference, phase, frame_deadline_ns);
        }

        ++*global_frame;
        ++*publication_sequence;
        publish_record(logger, driver, config, phase,
                       *publication_sequence, frame_start_ns,
                       last_published_measurement, app_stats);
        if ((*global_frame % TELEMETRY_FRAMES) == 0U) {
            print_telemetry(*global_frame, phase, driver, config,
                            driver_stats, app_stats);
        }

        {
            uint64_t body_end_ns = monotonic_ns();
            uint64_t body_ns = body_end_ns - body_start_ns;
            if (body_ns > app_stats->maximum_loop_body_ns) {
                app_stats->maximum_loop_body_ns = body_ns;
            }
            if ((body_ns > FRAME_PERIOD_NS) ||
                (frame_status == -ETIMEDOUT)) {
                ++app_stats->service_overruns;
            }
        }
        if ((frame_status < 0) &&
            (frame_status != -ETIMEDOUT) &&
            (frame_status != -EINTR)) {
            return frame_status;
        }

        int rt_status = RT_SleepUntil(FRAME_PERIOD_SEC);
        if (rt_status > 0) {
            app_stats->service_overruns += (uint64_t)rt_status;
        } else if (rt_status < 0) {
            if ((rt_status == -EINTR) && stop_requested) {
                break;
            }
            return rt_status;
        }

        frame_start_ns = monotonic_ns();
    }
    return stop_requested ? -EINTR : 0;
}

int main(int argc, char *argv[])
{
    Ms5837CliOptions_t options;
    Ms5837HalRpi_t rpi;
    Ms5837Hal_t hal;
    Ms5837Driver_t driver;
    Ms5837Logger_t logger;
    Ms5837Reference_t reference;
    AppDepthConfig_t config;
    BaroDriverStats_t driver_stats;
    AppRuntimeStats_t app_stats;
    uint64_t global_frame = 0U;
    uint64_t publication_sequence = 0U;
    uint64_t last_published_measurement = 0U;
    int result = 1;
    int status;

    if (ms5837_cli_parse(argc, argv, &options) < 0) {
        fprintf(stderr, "Invalid arguments; see the CLI instructions in app/main.c.\n");
        return 2;
    }
    memset(&config, 0, sizeof(config));
    memset(&driver_stats, 0, sizeof(driver_stats));
    memset(&app_stats, 0, sizeof(app_stats));
    config.fluid_density_kg_m3 = options.density_kg_m3;
    config.surface_pressure_mbar = NAN;
    config.surface_pressure_valid = false;
    config.zero_at_start_requested = options.zero_requested;

    (void)signal(SIGINT, request_stop);
    (void)signal(SIGTERM, request_stop);
    ms5837_reference_reset(&reference);
    ms5837_hal_rpi_configure(&rpi, MS5837_DEFAULT_I2C_BUS,
                             MS5837_I2C_ADDRESS);
    hal = ms5837_hal_rpi_make(&rpi);
    status = ms5837_driver_init(&driver, &hal);
    if (status < 0) {
        fprintf(stderr, "MS5837 initialization failed: %d\n", status);
        return 1;
    }
    status = ms5837_logger_start(&logger);
    if (status < 0) {
        fprintf(stderr, "CSV logger start failed: %d\n", status);
        (void)ms5837_driver_shutdown(&driver);
        return 1;
    }
    
    RT_StartStatus_t rt_status = StartRT(RT_PRIORITY, FRAME_PERIOD_SEC);
    if (rt_status != RT_START_OK) {
        fprintf(stderr,
                "WARNING: StartRT status=0x%08" PRIX32
                "; continuing with available scheduling/locking.\n",
                rt_status);
    }

    printf("capture=%s duration=%" PRIu64 "s zero=%s density=%.3f\n",
           ms5837_logger_filename(&logger), options.duration_sec,
           options.zero_requested ? "yes" : "no", options.density_kg_m3);

    if (options.zero_requested) {
        ms5837_reference_begin(&reference);
        status = run_phase(&driver, &logger, &config, &driver_stats, &app_stats,
                           &reference,
                           BARO_CAPTURE_PHASE_ZERO, MS5837_ZERO_DURATION_NS,
                           &global_frame, &publication_sequence,
                           &last_published_measurement);
        if ((status == 0) && ms5837_reference_finalize(&reference)) {
            config.surface_pressure_mbar =
                ms5837_reference_surface_pressure(&reference);
            config.surface_pressure_valid = true;
            printf("surface_reference=%.6f_mbar valid_samples=%zu rejected=%zu\n",
                   config.surface_pressure_mbar,
                   reference.valid_sample_count,
                   reference.rejected_sample_count);
        } else if (status == 0) {
            fprintf(stderr,
                    "Surface reference failed: valid_samples=%zu rejected=%zu; "
                    "depth will remain NaN.\n",
                    reference.valid_sample_count,
                    reference.rejected_sample_count);
        }
    } else {
        status = 0;
    }

    if ((status == 0) && !stop_requested) {
        status = run_phase(&driver, &logger, &config, &driver_stats, &app_stats,
                           NULL,
                           BARO_CAPTURE_PHASE_RUN,
                           options.duration_sec * UINT64_C(1000000000),
                           &global_frame, &publication_sequence,
                           &last_published_measurement);
    }
    result = ((status == 0) && (app_stats.logger_drops == 0U)) ? 0 : 1;

    if (ms5837_logger_stop(&logger) < 0) {
        fprintf(stderr, "CSV logger shutdown failed.\n");
        result = 1;
    }
    if (ms5837_driver_shutdown(&driver) < 0) {
        fprintf(stderr, "MS5837 shutdown failed.\n");
        result = 1;
    }

    printf("completed=%" PRIu64 " publications=%" PRIu64
           " not_ready=%" PRIu64 " stale=%" PRIu64
           " i2c_errors=%" PRIu64 " aborted_pairs=%" PRIu64
           " recoveries=%" PRIu64 "/%" PRIu64
           " overruns=%" PRIu64 " logger_drops=%" PRIu64
           " max_body_ns=%" PRIu64 "\n",
           driver_stats.measurements_completed,
           app_stats.publications_attempted,
           app_stats.publications_not_ready, app_stats.publications_stale,
           driver_stats.i2c_errors, driver_stats.aborted_pairs,
           driver_stats.recovery_successes, driver_stats.recovery_attempts,
           app_stats.service_overruns, app_stats.logger_drops,
           app_stats.maximum_loop_body_ns);
    return result;
}
