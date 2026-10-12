#include "ms5837_cli.h"

#include <errno.h>
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

int ms5837_cli_parse(int argc, char *argv[], Ms5837CliOptions_t *options)
{
    int index;
    bool duration_seen = false;
    bool zero_seen = false;

    if ((argc < 1) || (argv == NULL) || (options == NULL)) {
        return -EINVAL;
    }
    options->duration_sec = MS5837_DEFAULT_DURATION_SEC;
    options->zero_requested = false;

    for (index = 1; index < argc; ++index) {
        if (strcmp(argv[index], "--zero") == 0) {
            if (zero_seen) {
                return -EINVAL;
            }

            zero_seen = true;
            options->zero_requested = true;
        } else if (strcmp(argv[index], "--duration") == 0) {
            if (duration_seen ||
                (++index >= argc) ||
                (parse_duration(argv[index], &options->duration_sec) < 0)) {
                return -EINVAL;
            }
            
            duration_seen = true;
        } else {
            return -EINVAL;
        }
    }
    return 0;
}
