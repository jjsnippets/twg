#ifndef MS5837_CLI_H
#define MS5837_CLI_H

#include <stdbool.h>
#include <stdint.h>

#define MS5837_DEFAULT_DURATION_SEC UINT64_C(10)
#define MS5837_DEFAULT_DENSITY_KG_M3 1000.0

typedef struct {
    uint64_t duration_sec;
    double density_kg_m3;
    bool zero_requested;
} Ms5837CliOptions_t;

int ms5837_cli_parse(int argc, char *argv[], Ms5837CliOptions_t *options);

#endif
