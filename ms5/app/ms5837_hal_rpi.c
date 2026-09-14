#define _POSIX_C_SOURCE 200809L

#include "ms5837_hal_rpi.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stddef.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

static int negative_errno_or_io(void)
{
    return (errno != 0) ? -errno : -EIO;
}

static int rpi_open(void *opaque)
{
    Ms5837HalRpi_t *context = (Ms5837HalRpi_t *)opaque;
    int descriptor;
    int saved_errno;

    if ((context == NULL) || (context->bus_path == NULL) ||
        (context->address > 0x7FU)) {
        return -EINVAL;
    }
    if (context->fd >= 0) {
        return -EALREADY;
    }

    descriptor = open(context->bus_path, O_RDWR | O_CLOEXEC);
    if (descriptor < 0) {
        return negative_errno_or_io();
    }

    if (ioctl(descriptor, I2C_SLAVE, (unsigned long)context->address) < 0) {
        saved_errno = negative_errno_or_io();
        (void)close(descriptor);
        return saved_errno;
    }

    context->fd = descriptor;
    return 0;
}

static int rpi_close(void *opaque)
{
    Ms5837HalRpi_t *context = (Ms5837HalRpi_t *)opaque;
    int descriptor;

    if (context == NULL) {
        return -EINVAL;
    }
    if (context->fd < 0) {
        return 0;
    }

    descriptor = context->fd;
    context->fd = -1;
    if (close(descriptor) < 0) {
        return negative_errno_or_io();
    }
    return 0;
}

static int rpi_write(void *opaque, const uint8_t *data, size_t length)
{
    Ms5837HalRpi_t *context = (Ms5837HalRpi_t *)opaque;
    ssize_t transferred;

    if ((context == NULL) || (data == NULL) || (length == 0U)) {
        return -EINVAL;
    }
    if (context->fd < 0) {
        return -ENODEV;
    }

    do {
        transferred = write(context->fd, data, length);
    } while ((transferred < 0) && (errno == EINTR));

    if (transferred < 0) {
        return negative_errno_or_io();
    }
    if ((size_t)transferred != length) {
        return -EIO;
    }
    return 0;
}

static int rpi_read(void *opaque, uint8_t *data, size_t length)
{
    Ms5837HalRpi_t *context = (Ms5837HalRpi_t *)opaque;
    ssize_t transferred;

    if ((context == NULL) || (data == NULL) || (length == 0U)) {
        return -EINVAL;
    }
    if (context->fd < 0) {
        return -ENODEV;
    }

    do {
        transferred = read(context->fd, data, length);
    } while ((transferred < 0) && (errno == EINTR));

    if (transferred < 0) {
        return negative_errno_or_io();
    }
    if ((size_t)transferred != length) {
        return -EIO;
    }
    return 0;
}

static int rpi_sleep_ns(void *opaque, uint64_t duration_ns)
{
    struct timespec requested;
    struct timespec remaining;

    (void)opaque;
    requested.tv_sec = (time_t)(duration_ns / UINT64_C(1000000000));
    requested.tv_nsec = (long)(duration_ns % UINT64_C(1000000000));

    while (nanosleep(&requested, &remaining) < 0) {
        if (errno != EINTR) {
            return negative_errno_or_io();
        }
        requested = remaining;
    }
    return 0;
}

void ms5837_hal_rpi_configure(Ms5837HalRpi_t *context,
                              const char *bus_path,
                              uint8_t address)
{
    if (context == NULL) {
        return;
    }

    context->bus_path = (bus_path != NULL) ? bus_path : MS5837_DEFAULT_I2C_BUS;
    context->address = address;
    context->fd = -1;
}

Ms5837Hal_t ms5837_hal_rpi_make(Ms5837HalRpi_t *context)
{
    Ms5837Hal_t hal;

    hal.context = context;
    hal.open = rpi_open;
    hal.close = rpi_close;
    hal.write = rpi_write;
    hal.read = rpi_read;
    hal.sleep_ns = rpi_sleep_ns;
    return hal;
}
