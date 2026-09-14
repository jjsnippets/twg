#include "ms5837_cli.h"

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static int parse_duration(const char *text, uint64_t *value)
{
    char *end = NULL;
    unsigned long long parsed;

    if ((text == NULL) || (text[0] == '\0') || (text[0] == '-')) {
        return -EINVAL;
    }
    errno = 0;
    parsed = strtoull(text, &end, 10);
    if ((errno != 0) || (end == text) || (*end != '\0') || (parsed == 0U) ||
        (parsed > (UINT64_MAX / UINT64_C(1000000000)))) {
        return -EINVAL;
    }
    *value = (uint64_t)parsed;
    return 0;
}

static int parse_density(const char *text, double *value)
{
    char *end = NULL;
    double parsed;

    if ((text == NULL) || (text[0] == '\0')) {
        return -EINVAL;
    }
    errno = 0;
    parsed = strtod(text, &end);
    if ((errno != 0) || (end == text) || (*end != '\0') ||
        !isfinite(parsed) || !(parsed > 0.0)) {
        return -EINVAL;
    }
    *value = parsed;
    return 0;
}

int ms5837_cli_parse(int argc, char *argv[], Ms5837CliOptions_t *options)
{
    int index;

    if ((argc < 1) || (argv == NULL) || (options == NULL)) {
        return -EINVAL;
    }
    options->duration_sec = MS5837_DEFAULT_DURATION_SEC;
    options->density_kg_m3 = MS5837_DEFAULT_DENSITY_KG_M3;
    options->zero_requested = false;

    for (index = 1; index < argc; ++index) {
        if (strcmp(argv[index], "--zero") == 0) {
            options->zero_requested = true;
        } else if (strcmp(argv[index], "--duration") == 0) {
            if ((++index >= argc) ||
                (parse_duration(argv[index], &options->duration_sec) < 0)) {
                return -EINVAL;
            }
        } else if (strcmp(argv[index], "--density") == 0) {
            if ((++index >= argc) ||
                (parse_density(argv[index], &options->density_kg_m3) < 0)) {
                return -EINVAL;
            }
        } else {
            return -EINVAL;
        }
    }
    return 0;
}
