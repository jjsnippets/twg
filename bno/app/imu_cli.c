#include "app/imu_cli.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

enum {
    SEEN_CAL      = 1u << 0,
    SEEN_TARE     = 1u << 1,
    SEEN_CHECK    = 1u << 2,
    SEEN_CLEAR    = 1u << 3,
    SEEN_FULL     = 1u << 4,
    SEEN_TCHECK   = 1u << 5,
    SEEN_MASK     = 1u << 6,
    SEEN_DURATION = 1u << 7,
    SEEN_HELP     = 1u << 8
};

static ImuCliStatus_t fail(ImuCliParse_t *out, ImuCliError_t error,
                           int index)
{
    if (out != NULL) {
        /* In particular, an error never returns a partially built plan. */
        memset(&out->plan, 0, sizeof(out->plan));
        out->status = IMU_CLI_STATUS_ERROR;
        out->error = error;
        out->errorIndex = index;
    }
    return IMU_CLI_STATUS_ERROR;
}

static bool digit_value(char c, unsigned base, unsigned *digit)
{
    unsigned value;

    if (c >= '0' && c <= '9') {
        value = (unsigned)(c - '0');
    } else if (base == 16u && c >= 'a' && c <= 'f') {
        value = (unsigned)(c - 'a') + 10u;
    } else if (base == 16u && c >= 'A' && c <= 'F') {
        value = (unsigned)(c - 'A') + 10u;
    } else {
        return false;
    }
    if (value >= base) {
        return false;
    }
    *digit = value;
    return true;
}

/*
 * Returns NONE on success, INVALID_VALUE for lexical errors, or
 * OUT_OF_RANGE for an otherwise numeric token exceeding maxValue.
 */
static ImuCliError_t parse_uint(const char *text, bool allowHex,
                                bool allowPlus, unsigned maxValue,
                                unsigned *out)
{
    const char *p = text;
    unsigned base = 10u;
    unsigned value = 0u;
    unsigned digit;
    bool haveDigit = false;

    if (p == NULL || out == NULL || *p == '\0') {
        return IMU_CLI_ERROR_INVALID_VALUE;
    }
    if (*p == '+') {
        if (!allowPlus) {
            return IMU_CLI_ERROR_INVALID_VALUE;
        }
        ++p;
    }
    if (allowHex && p[0] == '0' &&
        (p[1] == 'x' || p[1] == 'X')) {
        base = 16u;
        p += 2;
    }
    if (*p == '\0') {
        return IMU_CLI_ERROR_INVALID_VALUE;
    }

    for (; *p != '\0'; ++p) {
        if (!digit_value(*p, base, &digit)) {
            return IMU_CLI_ERROR_INVALID_VALUE;
        }
        haveDigit = true;
        if (value > (maxValue - digit) / base) {
            return IMU_CLI_ERROR_OUT_OF_RANGE;
        }
        value = value * base + digit;
    }

    if (!haveDigit) {
        return IMU_CLI_ERROR_INVALID_VALUE;
    }
    *out = value;
    return IMU_CLI_ERROR_NONE;
}

static bool is_long_option(const char *text)
{
    return text != NULL && text[0] == '-' && text[1] == '-';
}

ImuCliStatus_t imu_cli_parse(int argc, char *const argv[],
                             ImuCliParse_t *out)
{
    ImuCmdPlan_t plan;
    unsigned seen = 0u;
    unsigned mask = 0u;
    unsigned duration = IMU_CMD_ACQUIRE_DEFAULT_S;
    unsigned parsed;
    ImuCliError_t numberError;
    int clearIndex = -1;
    int fullIndex = -1;
    int tareCheckIndex = -1;
    int maskIndex = -1;
    int helpIndex = -1;
    int i;

    if (out == NULL) {
        return IMU_CLI_STATUS_ERROR;
    }
    memset(out, 0, sizeof(*out));
    out->status = IMU_CLI_STATUS_ERROR;
    out->error = IMU_CLI_ERROR_ARGUMENTS;
    out->errorIndex = -1;

    if (argc < 1 || argv == NULL || argv[0] == NULL) {
        return fail(out, IMU_CLI_ERROR_ARGUMENTS, -1);
    }

    for (i = 1; i < argc; ++i) {
        const char *arg = argv[i];
        unsigned bit = 0u;

        if (arg == NULL) {
            return fail(out, IMU_CLI_ERROR_ARGUMENTS, i);
        }
        if (strcmp(arg, "--cal-imu") == 0) {
            bit = SEEN_CAL;
        } else if (strcmp(arg, "--tare-imu") == 0) {
            bit = SEEN_TARE;
        } else if (strcmp(arg, "--check-imu") == 0) {
            bit = SEEN_CHECK;
        } else if (strcmp(arg, "--clear") == 0) {
            bit = SEEN_CLEAR;
            clearIndex = i;
        } else if (strcmp(arg, "--full") == 0) {
            bit = SEEN_FULL;
            fullIndex = i;
        } else if (strcmp(arg, "--check") == 0) {
            bit = SEEN_TCHECK;
            tareCheckIndex = i;
        } else if (strcmp(arg, "--mask") == 0) {
            bit = SEEN_MASK;
            maskIndex = i;
        } else if (strcmp(arg, "--duration") == 0) {
            bit = SEEN_DURATION;
        } else if (strcmp(arg, "--help") == 0) {
            bit = SEEN_HELP;
            helpIndex = i;
        } else {
            return fail(out, arg[0] == '-'
                               ? IMU_CLI_ERROR_UNKNOWN_OPTION
                               : IMU_CLI_ERROR_POSITIONAL,
                        i);
        }

        if ((seen & bit) != 0u) {
            return fail(out, IMU_CLI_ERROR_REPEATED_OPTION, i);
        }
        seen |= bit;

        if (bit == SEEN_MASK || bit == SEEN_DURATION) {
            int valueIndex;

            if (i + 1 >= argc || argv[i + 1] == NULL ||
                is_long_option(argv[i + 1])) {
                return fail(out, IMU_CLI_ERROR_MISSING_VALUE, i);
            }
            valueIndex = ++i;
            parsed = 0u;
            if (bit == SEEN_MASK) {
                numberError = parse_uint(argv[i], true, false,
                                         255u, &parsed);
                if (numberError != IMU_CLI_ERROR_NONE) {
                    return fail(out, numberError, valueIndex);
                }
                mask = parsed;
            } else {
                numberError = parse_uint(argv[i], false, true,
                                         IMU_CMD_ACQUIRE_MAX_S, &parsed);
                if (numberError != IMU_CLI_ERROR_NONE) {
                    return fail(out, numberError, valueIndex);
                }
                if (parsed < IMU_CMD_ACQUIRE_MIN_S) {
                    return fail(out, IMU_CLI_ERROR_OUT_OF_RANGE,
                                valueIndex);
                }
                duration = parsed;
            }
        }
    }

    if ((seen & SEEN_HELP) != 0u) {
        if (seen != SEEN_HELP) {
            return fail(out, IMU_CLI_ERROR_ILLEGAL_COMBINATION,
                        helpIndex);
        }
        out->status = IMU_CLI_STATUS_HELP;
        out->error = IMU_CLI_ERROR_NONE;
        out->errorIndex = -1;
        return out->status;
    }

    if ((seen & SEEN_CLEAR) != 0u) {
        bool cal = (seen & SEEN_CAL) != 0u;
        bool tare = (seen & SEEN_TARE) != 0u;

        /*
         * Amendment: one --clear applies to each requested family.
         * No family is illegal; both select two ordered clear stages.
         * Repeated --clear remains rejected by the duplicate-flag check.
         */
        if (!cal && !tare) {
            return fail(out, IMU_CLI_ERROR_ILLEGAL_COMBINATION,
                        clearIndex);
        }
    }
    if ((seen & (SEEN_FULL | SEEN_TCHECK)) != 0u &&
        (seen & SEEN_TARE) == 0u) {
        return fail(out, IMU_CLI_ERROR_ILLEGAL_COMBINATION,
                    (seen & SEEN_FULL) != 0u
                        ? fullIndex : tareCheckIndex);
    }
    if ((seen & SEEN_MASK) != 0u &&
        (seen & SEEN_CHECK) == 0u) {
        return fail(out, IMU_CLI_ERROR_ILLEGAL_COMBINATION,
                    maskIndex);
    }
    if ((seen & SEEN_FULL) != 0u &&
        (seen & (SEEN_TCHECK | SEEN_CLEAR)) != 0u) {
        return fail(out, IMU_CLI_ERROR_ILLEGAL_COMBINATION,
                    fullIndex);
    }
    if ((seen & SEEN_TCHECK) != 0u &&
        (seen & SEEN_CLEAR) != 0u) {
        return fail(out, IMU_CLI_ERROR_ILLEGAL_COMBINATION,
                    tareCheckIndex);
    }

    memset(&plan, 0, sizeof(plan));
    plan.version = IMU_CMD_PLAN_VERSION;
    plan.flightCalMask = 0x00u;
    plan.probeDeadlineS = IMU_CMD_PROBE_DEADLINE_S;
    plan.acquisitionDurationS = duration;
    plan.settleDurationMs = IMU_CMD_SETTLE_DEFAULT_MS;
    plan.servicePeriodNs = IMU_CMD_SERVICE_PERIOD_NS;
    plan.publicationPeriodNs = IMU_CMD_PUB_PERIOD_NS;

    if ((seen & SEEN_CAL) != 0u) {
        if ((seen & SEEN_CLEAR) != 0u) {
            plan.slot1 = IMU_CMD_ID_DCD_CLEAR;
            plan.confirmDcdClear = true;
        } else {
            plan.slot1 = IMU_CMD_ID_CALIBRATION;
        }
    }
    if ((seen & SEEN_TARE) != 0u) {
        if ((seen & SEEN_CLEAR) != 0u) {
            plan.slot2 = IMU_CMD_ID_TARE_CLEAR;
            plan.confirmTareClear = true;
        } else if ((seen & SEEN_TCHECK) != 0u) {
            plan.slot2 = IMU_CMD_ID_TARE_CHECK;
        } else {
            plan.slot2 = IMU_CMD_ID_TARE;
            plan.tareAxes = (seen & SEEN_FULL) != 0u
                          ? IMU_CMD_TARE_AXES_FULL
                          : IMU_CMD_TARE_AXES_Z;
            plan.persistTare = true;
            plan.confirmTare = true;
        }
    }
    if ((seen & SEEN_CHECK) != 0u) {
        if ((seen & SEEN_MASK) != 0u) {
            plan.slot3 = IMU_CMD_ID_PROBE;
            plan.probeMaskPresent = true;
            plan.probeMask = (uint8_t)mask;
        } else {
            plan.slot3 = IMU_CMD_ID_CHECK;
        }
    }

    out->plan = plan;
    out->status = IMU_CLI_STATUS_OK;
    out->error = IMU_CLI_ERROR_NONE;
    out->errorIndex = -1;
    return out->status;
}
