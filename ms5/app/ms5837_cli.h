#ifndef MS5837_CLI_H
#define MS5837_CLI_H

#include <stdbool.h>
#include <stdint.h>

#define MS5837_DEFAULT_DURATION_SEC UINT64_C(10)

typedef struct {
    uint64_t duration_sec;
    bool zero_requested;
} Ms5837CliOptions_t;

int ms5837_cli_parse(int argc, char *argv[], Ms5837CliOptions_t *options);

#endif /* MS5837_CLI_H */
