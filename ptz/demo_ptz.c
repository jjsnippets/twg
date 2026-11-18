#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <linux/videodev2.h>

#define ARRAY_LEN(array) (sizeof(array) / sizeof((array)[0]))
#define CORNER_ARCSEC 54000

struct waypoint {
    const char *label;
    int32_t pan;
    int32_t tilt;
};

static const struct waypoint square[] = {
    { "Upper left (start)",        -CORNER_ARCSEC,  CORNER_ARCSEC },
    { "Upper right",                CORNER_ARCSEC,  CORNER_ARCSEC },
    { "Lower right",                CORNER_ARCSEC, -CORNER_ARCSEC },
    { "Lower left",                -CORNER_ARCSEC, -CORNER_ARCSEC },
    { "Upper left (close square)", -CORNER_ARCSEC,  CORNER_ARCSEC },
    { "Center (write 1 of 2)",                   0,              0 },
    { "Center (write 2 of 2)",                   0,              0 },
};

static const int32_t axis_targets[] = { -CORNER_ARCSEC, 0, CORNER_ARCSEC };
static const int32_t zoom_targets[] = { 0, 3, 5, 12, 0 };

static volatile sig_atomic_t stop_requested;

static void request_stop(int signo)
{
    (void)signo;
    stop_requested = 1;
}

static int camera_ioctl(int fd, unsigned long request, void *arg)
{
    int result;

    do {
        if (stop_requested) {
            errno = EINTR;
            return -1;
        }
        result = ioctl(fd, request, arg);
    } while (result == -1 && errno == EINTR && !stop_requested);

    return result;
}

static int check_control(int fd, uint32_t id, const char *name,
                         const int32_t *targets, size_t count)
{
    struct v4l2_queryctrl query = { .id = id };

    if (camera_ioctl(fd, VIDIOC_QUERYCTRL, &query) == -1) {
        fprintf(stderr, "Cannot query %s: %s\n", name, strerror(errno));
        return -1;
    }

    if (query.type != V4L2_CTRL_TYPE_INTEGER ||
        (query.flags & (V4L2_CTRL_FLAG_DISABLED |
                        V4L2_CTRL_FLAG_READ_ONLY |
                        V4L2_CTRL_FLAG_INACTIVE |
                        V4L2_CTRL_FLAG_GRABBED)) ||
        query.step <= 0) {
        fprintf(stderr, "%s is not an available writable integer control.\n",
                name);
        return -1;
    }

    for (size_t i = 0; i < count; ++i) {
        const int32_t target = targets[i];

        if (target < query.minimum || target > query.maximum ||
            ((int64_t)target - query.minimum) % query.step != 0) {
            fprintf(stderr,
                    "%s does not support target %ld "
                    "(range %ld to %ld, step %ld).\n",
                    name, (long)target, (long)query.minimum,
                    (long)query.maximum, (long)query.step);
            return -1;
        }
    }

    return 0;
}

static int write_controls(int fd, const char *label,
                          struct v4l2_ext_control *values, uint32_t count)
{
    struct v4l2_ext_controls controls = {
        .which = V4L2_CTRL_WHICH_CUR_VAL,
        .count = count,
        .controls = values,
    };

    if (camera_ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) == -1) {
        const int saved_errno = errno;

        if (!stop_requested) {
            fprintf(stderr, "%s: write failed: %s\n",
                    label, strerror(saved_errno));
            if (controls.error_idx < count) {
                fprintf(stderr, "Driver error index: %u (zero-based).\n",
                        (unsigned)controls.error_idx);
            }
        }
        return -1;
    }

    return 0;
}

static int send_waypoint(int fd, const struct waypoint *point)
{
    struct v4l2_ext_control values[] = {
        { .id = V4L2_CID_PAN_ABSOLUTE,  .value = point->pan },
        { .id = V4L2_CID_TILT_ABSOLUTE, .value = point->tilt },
    };

    if (write_controls(fd, point->label, values, ARRAY_LEN(values)) == -1) {
        return -1;
    }

    printf("%s: command accepted (pan=%ld, tilt=%ld arcsec)\n",
           point->label, (long)point->pan, (long)point->tilt);
    fflush(stdout);
    return 0;
}

static int send_zoom(int fd, int32_t target)
{
    struct v4l2_ext_control value = {
        .id = V4L2_CID_ZOOM_ABSOLUTE,
        .value = target,
    };

    if (write_controls(fd, "Absolute zoom", &value, 1) == -1) {
        if (!stop_requested) {
            fprintf(stderr, "Requested zoom_absolute=%ld\n", (long)target);
        }
        return -1;
    }

    printf("Zoom: command accepted (zoom_absolute=%ld)\n", (long)target);
    fflush(stdout);
    return 0;
}

static int wait_dwell(long dwell_ms)
{
    struct timespec remaining = {
        .tv_sec = dwell_ms / 1000,
        .tv_nsec = (dwell_ms % 1000) * 1000000L,
    };

    while (!stop_requested) {
        struct timespec next;

        if (nanosleep(&remaining, &next) == 0) {
            return 0;
        }
        if (errno != EINTR) {
            perror("nanosleep");
            return -1;
        }

        remaining = next;
    }

    return -1;
}

static void print_usage(const char *program)
{
    fprintf(stderr, "Usage: %s [video-device [dwell-ms]]\n", program);
    fprintf(stderr, "Defaults: /dev/video0, 2000 ms\n");
    fprintf(stderr, "Accepted dwell: 1 to 60000 ms\n");
}

int main(int argc, char **argv)
{
    const char *device = "/dev/video0";
    long dwell_ms = 2000;
    int status = EXIT_FAILURE;
    struct sigaction action = { .sa_handler = request_stop };

    if (argc == 2 &&
        (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        print_usage(argv[0]);
        return EXIT_SUCCESS;
    }

    if (argc > 3) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }
    if (argc >= 2) {
        device = argv[1];
    }
    if (argc == 3) {
        char *end;

        errno = 0;
        dwell_ms = strtol(argv[2], &end, 10);
        if (errno != 0 || end == argv[2] || *end != '\0' ||
            dwell_ms < 1 || dwell_ms > 60000) {
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT, &action, NULL) == -1 ||
        sigaction(SIGTERM, &action, NULL) == -1) {
        perror("sigaction");
        return EXIT_FAILURE;
    }

    const int fd = open(device, O_RDWR | O_CLOEXEC);
    if (fd == -1) {
        fprintf(stderr, "Cannot open %s: %s\n", device, strerror(errno));
        return EXIT_FAILURE;
    }

    if (check_control(fd, V4L2_CID_PAN_ABSOLUTE, "pan_absolute",
                      axis_targets, ARRAY_LEN(axis_targets)) == -1 ||
        check_control(fd, V4L2_CID_TILT_ABSOLUTE, "tilt_absolute",
                      axis_targets, ARRAY_LEN(axis_targets)) == -1 ||
        check_control(fd, V4L2_CID_ZOOM_ABSOLUTE, "zoom_absolute",
                      zoom_targets, ARRAY_LEN(zoom_targets)) == -1) {
        goto cleanup;
    }

    printf("Device: %s | dwell: %ld ms\n", device, dwell_ms);
    puts("Verify physical motion and framing in the live preview.");
    fflush(stdout);

    for (size_t i = 0; i < ARRAY_LEN(square); ++i) {
        if (send_waypoint(fd, &square[i]) == -1 ||
            wait_dwell(dwell_ms) == -1) {
            goto cleanup;
        }
    }

    puts("Both center commands sent. Starting absolute zoom sequence.");
    fflush(stdout);

    for (size_t i = 0; i < ARRAY_LEN(zoom_targets); ++i) {
        if (send_zoom(fd, zoom_targets[i]) == -1 ||
            wait_dwell(dwell_ms) == -1) {
            goto cleanup;
        }
    }

    if (!stop_requested) {
        puts("Sequence sent. Verify the final centered, wide view.");
        status = EXIT_SUCCESS;
    }

cleanup:
    if (stop_requested) {
        fputs("Interrupted. No additional control commands will be sent.\n",
              stderr);
        status = EXIT_FAILURE;
    }
    close(fd);
    return status;
}
