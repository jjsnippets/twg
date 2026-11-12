#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <inttypes.h>
#include <locale.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "demo_imu.h"
#include "demo_link.h"
#include "rt/realtime.h"

#define SERVICE_PERIOD_SECONDS 0.001

static volatile sig_atomic_t interrupted;

static void on_signal(int signal_number)
{
    (void)signal_number;
    interrupted = 1;
}

static void usage(const char *program)
{
    fprintf(stderr,
            "Usage: %s [--port 8765] [--rt]\n"
            "\n"
            "  --port N   TCP listening port\n"
            "  --rt       Attempt SCHED_FIFO priority 30 and memory locking\n"
            "\n"
            "Without --rt, use the same periodic timer with normal scheduling.\n"
            "Use only on your trusted, dedicated demo network.\n",
            program);
}

static bool parse_port(const char *text, uint16_t *port)
{
    char *end;
    unsigned long value;

    if (*text < '0' || *text > '9') {
        return false;
    }

    errno = 0;
    value = strtoul(text, &end, 10);

    if (errno != 0 || *end != '\0' || value == 0 || value > 65535u) {
        return false;
    }

    *port = (uint16_t)value;
    return true;
}

int main(int argc, char **argv)
{
    DemoImu imu;
    DemoLink link;
    DemoImuSnapshot snapshot;
    struct sigaction action;

    uint16_t port = DEMO_DEFAULT_PORT;
    bool use_rt = false;
    int exit_code = EXIT_SUCCESS;
    DemoPhase previous_phase;
    DemoError previous_error;

    (void)setlocale(LC_NUMERIC, "C");

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            if (!parse_port(argv[++i], &port)) {
                usage(argv[0]);
                return EXIT_FAILURE;
            }
        } else if (strcmp(argv[i], "--rt") == 0) {
            use_rt = true;
        } else if (strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return EXIT_SUCCESS;
        } else {
            usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    memset(&action, 0, sizeof(action));
    action.sa_handler = on_signal;
    sigemptyset(&action.sa_mask);

    if (sigaction(SIGINT, &action, NULL) != 0 ||
        sigaction(SIGTERM, &action, NULL) != 0) {
        perror("sigaction");
        return EXIT_FAILURE;
    }

    puts("Opening IMU; no startup tare, clear, or persistence.");

    if (!demo_imu_open(&imu)) {
        fprintf(stderr, "IMU startup failed, error=%u\n",
                (unsigned)imu.output.error);
        return EXIT_FAILURE;
    }

    demo_imu_snapshot(&imu, &snapshot);

    if (!demo_link_start(&link, port, &snapshot)) {
        perror("TCP server");
        demo_imu_close(&imu);
        return EXIT_FAILURE;
    }

    printf("Listening on TCP port %u.\n", (unsigned)port);
    puts("Laptop controls: t = temporary Z-tare, x = exit application.");

    if (use_rt) {
        RT_StartStatus_t status =
            StartRT(30, SERVICE_PERIOD_SECONDS);

        if (status != RT_START_OK) {
            fprintf(stderr,
                    "RT setup warning: status=0x%" PRIx32
                    "; continuing where possible.\n",
                    (uint32_t)status);
        }
    }

    if (RT_Reset() != 0) {
        fputs("Could not arm periodic timer.\n", stderr);
        demo_imu_close(&imu);
        demo_link_stop(&link);
        return EXIT_FAILURE;
    }

    previous_phase = snapshot.phase;
    previous_error = snapshot.error;

    while (!interrupted && !demo_link_exit_requested(&link)) {
        uint32_t request_id;
        int sleep_result;

        if (demo_link_take_tare(&link, &request_id)) {
            if (demo_imu_request_tare(&imu, request_id)) {
                printf("Tare request %" PRIu32 " accepted.\n", request_id);
            } else {
                printf("Tare request %" PRIu32
                       " ignored: IMU is not tracking.\n", request_id);
            }
        }

        demo_imu_service(&imu);
        demo_imu_snapshot(&imu, &snapshot);
        demo_link_publish(&link, &snapshot);

        if (snapshot.phase != previous_phase ||
            snapshot.error != previous_error) {
            printf("IMU phase=%u error=%u tare_result=%d reference=%"
                   PRIu32 "\n",
                   (unsigned)snapshot.phase,
                   (unsigned)snapshot.error,
                   snapshot.tare_result,
                   snapshot.reference);

            previous_phase = snapshot.phase;
            previous_error = snapshot.error;
        }

        sleep_result = RT_SleepUntil(SERVICE_PERIOD_SECONDS);

        if (sleep_result < 0 && sleep_result != -EINTR) {
            fprintf(stderr, "Periodic sleep failed: %d\n", sleep_result);
            exit_code = EXIT_FAILURE;
            break;
        }
    }

    puts("Closing IMU and stopping demo.");
    demo_imu_close(&imu);
    demo_link_stop(&link);
    puts("Demo stopped. Raspberry Pi remains running.");
    return exit_code;
}
