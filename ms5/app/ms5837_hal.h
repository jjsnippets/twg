#ifndef MS5837_HAL_H
#define MS5837_HAL_H

#include <stddef.h>
#include <stdint.h>

/*
 * All operations return 0 on success or a negative errno-style value.
 * A successful read/write means the exact requested byte count transferred.
 */
typedef struct {
    void *context;
    int (*open)(void *context);
    int (*close)(void *context);
    int (*write)(void *context, const uint8_t *data, size_t length);
    int (*read)(void *context, uint8_t *data, size_t length);
    int (*sleep_ns)(void *context, uint64_t duration_ns);
} Ms5837Hal_t;

#endif
