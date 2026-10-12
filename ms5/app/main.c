#define _POSIX_C_SOURCE 200809L

/*
 * MS5837-02BA pressure-only reference capture.
 *
 * Usage:
 *   sudo ./bin/ms5837_capture [--duration SECONDS] [--zero]
 *
 * main owns the only real-time schedule. The nonblocking pressure driver is
 * serviced once per logical 1 ms turn. The pure publisher emits at tenth-tick
 * gates, giving a nominal 100 Hz publication rate.
 *
 * --zero adds a five-second consumer-owned reference stage before the
 * requested run. Publication and logging remain active during zeroing, with
 * phase=zero in the capture-owned CSV rows.
 */

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "ms5837_cli.h"
#include "ms5837_driver.h"
#include "ms5837_hal_rpi.h"
#include "ms5837_logger.h"
#include "ms5837_math.h"
#include "ms5837_publish.h"
#include "rt/realtime.h"
#include "rt/rt_policy.h"

#define RT_PRIORITY 90
#define SERVICE_PERIOD_SEC 0.001

#define TELEMETRY_ROWS UINT64_C(50)

#define STARTUP_MAX_ATTEMPTS 3U
#define STARTUP_RETRY_NS UINT64_C(200000000)

#define ZERO_DURATION_NS UINT64_C(5000000000)
#define ZERO_TRIM_FRACTION 0.10
#define ZERO_MIN_VALID_SAMPLES 350U
#define ZERO_MAX_SAMPLES 512U

#define CAPTURE_FLUID_DENSITY_KG_M3 1000.0

static volatile sig_atomic_t stop_requested;

typedef struct {
    double pressure_samples[ZERO_MAX_SAMPLES];
    size_t valid_sample_count;
    size_t rejected_sample_count;
    uint64_t last_measurement_sequence;
    double surface_pressure_mbar;
    bool valid;
} ZeroEstimator_t;

typedef struct {
    bool zero_requested;
    bool surface_pressure_valid;
    double surface_pressure_mbar;
    double fluid_density_kg_m3;
    ZeroEstimator_t zero;
} CapturePolicy_t;

static void request_stop(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

static bool monotonic_now(uint64_t *now_ns)
{
    struct timespec now;

    if ((now_ns == NULL) ||
        (clock_gettime(CLOCK_MONOTONIC, &now) < 0)) {
        return false;
    }

    *now_ns =
        ((uint64_t)now.tv_sec * UINT64_C(1000000000)) +
        (uint64_t)now.tv_nsec;

    return true;
}

static int sleep_relative_ns(uint64_t duration_ns)
{
    struct timespec pause;

    pause.tv_sec =
        (time_t)(duration_ns / UINT64_C(1000000000));
    pause.tv_nsec =
        (long)(duration_ns % UINT64_C(1000000000));

    while (nanosleep(&pause, &pause) < 0) {
        if (errno != EINTR) {
            return -errno;
        }

        if (stop_requested) {
            return -EINTR;
        }
    }

    return 0;
}

static int compare_double(
    const void *left,
    const void *right)
{
    const double left_value =
        *(const double *)left;
    const double right_value =
        *(const double *)right;

    if (left_value < right_value) {
        return -1;
    }

    if (left_value > right_value) {
        return 1;
    }

    return 0;
}

static void zero_estimator_begin(
    ZeroEstimator_t *estimator)
{
    memset(estimator, 0, sizeof(*estimator));
    estimator->surface_pressure_mbar = NAN;
}

static bool zero_estimator_add(
    ZeroEstimator_t *estimator,
    const BaroSample_t *sample)
{
    const uint32_t rejection_flags =
        BARO_STATUS_NOT_READY |
        BARO_STATUS_RAW_INVALID |
        BARO_STATUS_PRESSURE_INVALID;

    if ((estimator == NULL) ||
        (sample == NULL)) {
        return false;
    }

    if ((sample->measurement_sequence == 0U) ||
        (sample->measurement_sequence ==
         estimator->last_measurement_sequence) ||
        ((sample->status_flags & rejection_flags) != 0U) ||
        !isfinite(sample->pressure_mbar) ||
        (estimator->valid_sample_count >=
         ZERO_MAX_SAMPLES)) {
        ++estimator->rejected_sample_count;
        return false;
    }

    estimator->pressure_samples[
        estimator->valid_sample_count++] =
        sample->pressure_mbar;

    estimator->last_measurement_sequence =
        sample->measurement_sequence;

    return true;
}

static bool zero_estimator_finalize(
    ZeroEstimator_t *estimator)
{
    size_t trim_count;
    size_t end_index;
    size_t retained_count;
    size_t index;
    double sum = 0.0;

    if ((estimator == NULL) ||
        (estimator->valid_sample_count <
         ZERO_MIN_VALID_SAMPLES)) {
        if (estimator != NULL) {
            estimator->surface_pressure_mbar = NAN;
            estimator->valid = false;
        }

        return false;
    }

    qsort(
        estimator->pressure_samples,
        estimator->valid_sample_count,
        sizeof(estimator->pressure_samples[0]),
        compare_double);

    trim_count =
        (size_t)(
            (double)estimator->valid_sample_count *
            ZERO_TRIM_FRACTION);

    end_index =
        estimator->valid_sample_count - trim_count;

    retained_count =
        end_index - trim_count;

    if (retained_count == 0U) {
        estimator->surface_pressure_mbar = NAN;
        estimator->valid = false;
        return false;
    }

    for (index = trim_count;
         index < end_index;
         ++index) {
        sum += estimator->pressure_samples[index];
    }

    estimator->surface_pressure_mbar =
        sum / (double)retained_count;

    estimator->valid =
        isfinite(estimator->surface_pressure_mbar);

    return estimator->valid;
}

static double capture_depth(
    const Ms5837PublicationRecord_t *publication,
    const CapturePolicy_t *policy)
{
    if ((publication == NULL) ||
        (policy == NULL) ||
        (publication->sample_ready == 0U) ||
        !policy->surface_pressure_valid) {
        return NAN;
    }

    return ms5837_depth_from_reference(
        publication->sample.pressure_mbar,
        policy->surface_pressure_mbar,
        policy->fluid_density_kg_m3);
}

static bool recovery_active(
    const Ms5837Driver_t *driver)
{
    Ms5837DriverState_t state =
        ms5837_driver_state(driver);

    return
        (state >= MS5837_STATE_RECOVERY_BACKOFF) &&
        (state <= MS5837_STATE_RECOVERY_VALIDATE);
}

static int initialize_with_retry(
    Ms5837Driver_t *driver,
    const Ms5837Hal_t *hal)
{
    unsigned int attempt;
    int status =
        MS5837_DRIVER_ERR_NOT_INITIALIZED;

    for (attempt = 1U;
         attempt <= STARTUP_MAX_ATTEMPTS;
         ++attempt) {
        status = ms5837_driver_init(driver, hal);

        if (status == MS5837_DRIVER_OK) {
            return status;
        }

        fprintf(
            stderr,
            "MS5837 initialization attempt %u/%u failed: %d\n",
            attempt,
            STARTUP_MAX_ATTEMPTS,
            status);

        if (attempt < STARTUP_MAX_ATTEMPTS) {
            status =
                sleep_relative_ns(STARTUP_RETRY_NS);

            if (status < 0) {
                return status;
            }
        }
    }

    return status;
}

static int service_once(
    Ms5837Driver_t *driver,
    BaroDriverStats_t *driver_stats,
    BaroSample_t *completed,
    bool *sample_completed)
{
    uint64_t now_ns;
    int status;

    if (!monotonic_now(&now_ns)) {
        return -EIO;
    }

    status = ms5837_driver_service(
        driver,
        now_ns,
        driver_stats,
        completed);

    *sample_completed =
        status == MS5837_SERVICE_SAMPLE_READY;

    if ((status < 0) &&
        !recovery_active(driver) &&
        (ms5837_driver_state(driver) !=
         MS5837_STATE_STOPPED)) {
        return status;
    }

    return 0;
}

static int sleep_owner_turn(
    Ms5837RunStats_t *run_stats,
    uint64_t *logical_ticks)
{
    int sleep_status =
        RT_SleepUntil(SERVICE_PERIOD_SEC);

    if (sleep_status < 0) {
        return sleep_status;
    }

    if (sleep_status > 0) {
        run_stats->service_overruns +=
            (uint64_t)sleep_status;
    }

    if (logical_ticks != NULL) {
        *logical_ticks =
            UINT64_C(1) +
            (uint64_t)sleep_status;
    }

    return 0;
}

static int warm_up_sample(
    Ms5837Driver_t *driver,
    BaroDriverStats_t *driver_stats,
    Ms5837RunStats_t *run_stats,
    BaroSample_t *baseline)
{
    while (!stop_requested) {
        BaroSample_t completed;
        bool completed_now = false;
        int status;

        if (ms5837_driver_latest_sample(
                driver,
                baseline)) {
            return 0;
        }

        if (ms5837_driver_state(driver) ==
            MS5837_STATE_STOPPED) {
            return ms5837_driver_last_error(driver);
        }

        if (ms5837_driver_state(driver) ==
            MS5837_STATE_IDLE) {
            status = ms5837_driver_trigger(
                driver,
                driver_stats);

            if ((status < 0) &&
                !recovery_active(driver)) {
                return status;
            }
        }

        status = service_once(
            driver,
            driver_stats,
            &completed,
            &completed_now);

        ++run_stats->service_ticks;

        if (status < 0) {
            ++run_stats->clock_read_failures;
            return status;
        }

        if (completed_now) {
            *baseline = completed;
            return 0;
        }

        status = sleep_owner_turn(
            run_stats,
            NULL);

        if (status < 0) {
            return status;
        }
    }

    return -EINTR;
}

static void account_publication(
    Ms5837RunStats_t *stats,
    const Ms5837PublicationRecord_t *publication)
{
    ++stats->publications_attempted;

    stats->gates_skipped +=
        publication->gates_skipped_before;

    if (publication->deadline_missed != 0U) {
        ++stats->deadline_misses;
    }

    if (publication->not_ready != 0U) {
        ++stats->publications_not_ready;
    } else if (publication->sample_fresh != 0U) {
        ++stats->publications_fresh;
    } else if (publication->sample_stale != 0U) {
        ++stats->publications_stale;
    }
}

static int emit_record(
    Ms5837Logger_t *logger,
    Ms5837Driver_t *driver,
    Ms5837PublisherState_t *publisher,
    CapturePolicy_t *policy,
    Ms5837RunStats_t *run_stats,
    BaroCapturePhase_t phase,
    const Ms5837PublishGateDecision_t *decision,
    uint64_t actual_ns,
    bool actual_valid)
{
    BaroLogRecord_t record;
    BaroSample_t sample;
    bool sample_available;

    memset(&record, 0, sizeof(record));

    sample_available =
        ms5837_driver_latest_sample(
            driver,
            &sample);

    if (!ms5837_publish_evaluate(
            publisher,
            sample_available ? &sample : NULL,
            sample_available,
            (int32_t)ms5837_driver_state(driver),
            decision->scheduled_ns,
            decision->scheduled_valid,
            actual_ns,
            actual_valid,
            decision->gates_skipped_before,
            &record.publication)) {
        return -EINVAL;
    }

    account_publication(
        run_stats,
        &record.publication);

    record.schema_version =
        MS5837_CSV_SCHEMA_VERSION;
    record.phase = phase;
    record.zero_requested =
        policy->zero_requested;
    record.zero_valid =
        policy->surface_pressure_valid;
    record.zero_valid_samples =
        (uint64_t)policy->zero.valid_sample_count;
    record.zero_rejected_samples =
        (uint64_t)policy->zero.rejected_sample_count;

    record.surface_pressure_mbar =
        policy->surface_pressure_valid
        ? policy->surface_pressure_mbar
        : NAN;

    record.fluid_density_kg_m3 =
        policy->fluid_density_kg_m3;

    record.depth_m =
        capture_depth(
            &record.publication,
            policy);

    record.logger_drops_before =
        run_stats->logger_drops;

    if (ms5837_logger_enqueue(
            logger,
            &record) < 0) {
        ++run_stats->logger_drops;
        return -EAGAIN;
    }

    ++run_stats->publications_enqueued;
    return 0;
}

static void print_telemetry(
    BaroCapturePhase_t phase,
    const Ms5837Driver_t *driver,
    const CapturePolicy_t *policy,
    const BaroDriverStats_t *driver_stats,
    const Ms5837RunStats_t *run_stats)
{
    BaroSample_t sample;
    const char *phase_name =
        phase == BARO_CAPTURE_PHASE_ZERO
        ? "zero"
        : "run";

    if (ms5837_driver_latest_sample(
            driver,
            &sample)) {
        double depth =
            policy->surface_pressure_valid
            ? ms5837_depth_from_reference(
                  sample.pressure_mbar,
                  policy->surface_pressure_mbar,
                  policy->fluid_density_kg_m3)
            : NAN;

        printf(
            "row=%" PRIu64
            " phase=%s"
            " seq=%" PRIu64
            " P=%.2f_mbar"
            " T=%.2f_C"
            " depth=%.4f_m"
            " flags=0x%08" PRIX32
            " i2c=%" PRIu64
            " drops=%" PRIu64
            "\n",
            run_stats->publications_attempted,
            phase_name,
            sample.measurement_sequence,
            sample.pressure_mbar,
            sample.temperature_c,
            depth,
            sample.status_flags,
            driver_stats->i2c_errors,
            run_stats->logger_drops);
    } else {
        printf(
            "row=%" PRIu64
            " phase=%s"
            " sample=not_ready"
            " i2c=%" PRIu64
            " drops=%" PRIu64
            " density=%.3f"
            "\n",
            run_stats->publications_attempted,
            phase_name,
            driver_stats->i2c_errors,
            run_stats->logger_drops,
            policy->fluid_density_kg_m3);
    }
}

static int run_capture_phase(
    Ms5837Driver_t *driver,
    Ms5837Logger_t *logger,
    Ms5837PublisherState_t *publisher,
    CapturePolicy_t *policy,
    BaroDriverStats_t *driver_stats,
    Ms5837RunStats_t *run_stats,
    BaroCapturePhase_t phase,
    uint64_t duration_ns)
{
    Ms5837PublishGate_t gate;
    uint64_t tick1_ns;
    uint64_t logical_ticks = 1U;
    int trigger_status;

    if (!monotonic_now(&tick1_ns)) {
        ++run_stats->clock_read_failures;
        return -EIO;
    }

    if (!ms5837_publish_gate_begin(
            &gate,
            tick1_ns,
            true,
            duration_ns)) {
        return -EINVAL;
    }

    if (ms5837_driver_state(driver) ==
        MS5837_STATE_IDLE) {
        trigger_status =
            ms5837_driver_trigger(
                driver,
                driver_stats);

        if ((trigger_status < 0) &&
            !recovery_active(driver)) {
            return trigger_status;
        }
    }

    while (!stop_requested &&
           !ms5837_publish_gate_window_done(&gate)) {
        Ms5837PublishGateDecision_t decision;
        Ms5837PublishGateStatus_t gate_status;
        BaroSample_t completed;
        bool sample_completed = false;
        uint64_t body_start_ns;
        uint64_t body_end_ns;
        int status;
        int sleep_status;

        if (!monotonic_now(&body_start_ns)) {
            ++run_stats->clock_read_failures;
            return -EIO;
        }

        status = service_once(
            driver,
            driver_stats,
            &completed,
            &sample_completed);

        ++run_stats->service_ticks;

        if (status < 0) {
            return status;
        }

        if ((phase == BARO_CAPTURE_PHASE_ZERO) &&
            sample_completed) {
            (void)zero_estimator_add(
                &policy->zero,
                &completed);
        }

        gate_status =
            ms5837_publish_gate_advance(
                &gate,
                logical_ticks,
                &decision);

        if (gate_status ==
            MS5837_PUBLISH_GATE_ERROR) {
            return -ERANGE;
        }

        if (gate_status ==
            MS5837_PUBLISH_GATE_DUE) {
            uint64_t actual_ns;
            bool actual_valid =
                monotonic_now(&actual_ns);

            if (!actual_valid) {
                actual_ns = 0U;
                ++run_stats->clock_read_failures;
            }

            status = emit_record(
                logger,
                driver,
                publisher,
                policy,
                run_stats,
                phase,
                &decision,
                actual_ns,
                actual_valid);

            if (status < 0) {
                return status;
            }

            if ((run_stats->publications_attempted %
                 TELEMETRY_ROWS) == 0U) {
                print_telemetry(
                    phase,
                    driver,
                    policy,
                    driver_stats,
                    run_stats);
            }

            /*
             * Runtime exhaustion is represented by this gate before
             * controlled shutdown.
             */
            if (ms5837_driver_state(driver) ==
                MS5837_STATE_STOPPED) {
                return ms5837_driver_last_error(driver);
            }

            /*
             * Do not start an unnecessary pair after the final gate. This
             * also ensures the run phase triggers its own first pair after
             * a completed zero phase.
             */
            if (!ms5837_publish_gate_window_done(&gate) &&
                (ms5837_driver_state(driver) ==
                 MS5837_STATE_IDLE)) {
                trigger_status =
                    ms5837_driver_trigger(
                        driver,
                        driver_stats);

                if ((trigger_status < 0) &&
                    !recovery_active(driver)) {
                    return trigger_status;
                }
            }
        }

        if (!monotonic_now(&body_end_ns)) {
            ++run_stats->clock_read_failures;
            return -EIO;
        }

        if (body_end_ns >= body_start_ns) {
            uint64_t body_ns =
                body_end_ns - body_start_ns;

            if (body_ns >
                run_stats->maximum_loop_body_ns) {
                run_stats->maximum_loop_body_ns =
                    body_ns;
            }
        }

        if (ms5837_publish_gate_window_done(&gate)) {
            break;
        }

        sleep_status =
            sleep_owner_turn(
                run_stats,
                &logical_ticks);

        if (sleep_status < 0) {
            if ((sleep_status == -EINTR) &&
                stop_requested) {
                break;
            }

            return sleep_status;
        }
    }

    return stop_requested ? -EINTR : 0;
}

#ifndef MS5837_APP_OWNER_TEST

int main(int argc, char *argv[])
{
    Ms5837CliOptions_t options;
    Ms5837HalRpi_t rpi;
    Ms5837Hal_t hal;
    Ms5837Driver_t driver;
    Ms5837Logger_t logger;
    Ms5837PublisherState_t publisher;
    CapturePolicy_t policy;
    BaroDriverStats_t driver_stats;
    Ms5837RunStats_t run_stats;
    BaroSample_t baseline;
    RT_StartStatus_t rt_status;
    AppRtPolicy_t rt_policy;
    int status;
    int result = 1;

    if (ms5837_cli_parse(
            argc,
            argv,
            &options) < 0) {
        fprintf(
            stderr,
            "Usage: %s [--duration SECONDS] [--zero]\n",
            argv[0]);
        return 2;
    }

    memset(&policy, 0, sizeof(policy));
    memset(&driver_stats, 0, sizeof(driver_stats));
    memset(&run_stats, 0, sizeof(run_stats));

    policy.zero_requested =
        options.zero_requested;
    policy.surface_pressure_mbar = NAN;
    policy.fluid_density_kg_m3 =
        CAPTURE_FLUID_DENSITY_KG_M3;

    zero_estimator_begin(&policy.zero);

    run_stats.version =
        MS5837_RUN_STATS_CONTRACT_VERSION;

    (void)signal(SIGINT, request_stop);
    (void)signal(SIGTERM, request_stop);

    ms5837_hal_rpi_configure(
        &rpi,
        MS5837_DEFAULT_I2C_BUS,
        MS5837_I2C_ADDRESS);

    hal = ms5837_hal_rpi_make(&rpi);

    status = initialize_with_retry(
        &driver,
        &hal);

    if (status < 0) {
        fprintf(
            stderr,
            "MS5837 initialization exhausted: %d\n",
            status);
        return 1;
    }

    status = ms5837_logger_start(&logger);

    if (status < 0) {
        fprintf(
            stderr,
            "CSV logger start failed: %d\n",
            status);
        (void)ms5837_driver_shutdown(&driver);
        return 1;
    }

    rt_status =
        StartRT(
            RT_PRIORITY,
            SERVICE_PERIOD_SEC);

    rt_policy =
        app_rt_policy_from_start(rt_status);

    if (rt_policy == APP_RT_POLICY_FATAL) {
        fprintf(
            stderr,
            "Fatal StartRT status=0x%08" PRIX32 "\n",
            rt_status);
        (void)ms5837_logger_stop(&logger);
        (void)ms5837_driver_shutdown(&driver);
        return 1;
    }

    if (rt_policy ==
        APP_RT_POLICY_WARN_AND_CONTINUE) {
        fprintf(
            stderr,
            "WARNING: StartRT status=0x%08" PRIX32
            "; continuing without all RT privileges.\n",
            rt_status);
    }

    status = warm_up_sample(
        &driver,
        &driver_stats,
        &run_stats,
        &baseline);

    if ((status == 0) &&
        !ms5837_publish_begin_window(
            &publisher,
            &baseline,
            true)) {
        status = -EINVAL;
    }

    printf(
        "capture=%s duration=%" PRIu64
        "s zero=%s density=%.3f\n",
        ms5837_logger_filename(&logger),
        options.duration_sec,
        options.zero_requested ? "yes" : "no",
        policy.fluid_density_kg_m3);

    /*
     * Zeroing is a separate five-second phase, but it uses the same
     * publisher and logger. It therefore emits normal 100 Hz CSV rows with
     * phase=zero.
     */
    if ((status == 0) &&
        options.zero_requested &&
        !stop_requested) {
        status = run_capture_phase(
            &driver,
            &logger,
            &publisher,
            &policy,
            &driver_stats,
            &run_stats,
            BARO_CAPTURE_PHASE_ZERO,
            ZERO_DURATION_NS);

        if ((status == 0) &&
            zero_estimator_finalize(&policy.zero)) {
            policy.surface_pressure_mbar =
                policy.zero.surface_pressure_mbar;
            policy.surface_pressure_valid = true;

            printf(
                "surface_reference=%.6f_mbar"
                " valid_samples=%zu"
                " rejected=%zu\n",
                policy.surface_pressure_mbar,
                policy.zero.valid_sample_count,
                policy.zero.rejected_sample_count);
        } else if (status == 0) {
            fprintf(
                stderr,
                "Surface reference failed:"
                " valid_samples=%zu"
                " rejected=%zu;"
                " capture continues with depth=NaN.\n",
                policy.zero.valid_sample_count,
                policy.zero.rejected_sample_count);
        }
    }

    /*
     * The requested duration starts after zeroing, whether zeroing succeeds
     * or fails.
     */
    if ((status == 0) &&
        !stop_requested) {
        status = run_capture_phase(
            &driver,
            &logger,
            &publisher,
            &policy,
            &driver_stats,
            &run_stats,
            BARO_CAPTURE_PHASE_RUN,
            options.duration_sec *
            UINT64_C(1000000000));
    }

    result =
        ((status == 0) &&
         (run_stats.logger_drops == 0U))
        ? 0
        : 1;

    if (ms5837_logger_stop(&logger) < 0) {
        fprintf(
            stderr,
            "CSV logger shutdown failed.\n");
        result = 1;
    }

    if (ms5837_driver_shutdown(&driver) < 0) {
        fprintf(
            stderr,
            "MS5837 shutdown failed.\n");
        result = 1;
    }

    printf(
        "completed=%" PRIu64
        " publications=%" PRIu64
        " enqueued=%" PRIu64
        " not_ready=%" PRIu64
        " fresh=%" PRIu64
        " stale=%" PRIu64
        " i2c_errors=%" PRIu64
        " aborted_pairs=%" PRIu64
        " recoveries=%" PRIu64 "/%" PRIu64
        " skipped_ticks=%" PRIu64
        " skipped_gates=%" PRIu64
        " deadline_misses=%" PRIu64
        " logger_drops=%" PRIu64
        " max_body_ns=%" PRIu64
        "\n",
        driver_stats.measurements_completed,
        run_stats.publications_attempted,
        run_stats.publications_enqueued,
        run_stats.publications_not_ready,
        run_stats.publications_fresh,
        run_stats.publications_stale,
        driver_stats.i2c_errors,
        driver_stats.aborted_pairs,
        driver_stats.recovery_successes,
        driver_stats.recovery_attempts,
        run_stats.service_overruns,
        run_stats.gates_skipped,
        run_stats.deadline_misses,
        run_stats.logger_drops,
        run_stats.maximum_loop_body_ns);

    return result;
}

#endif /* MS5837_APP_OWNER_TEST */
