#ifndef IMU_CLI_H
#define IMU_CLI_H

#include "app/imu_cmd.h"

typedef enum {
    IMU_CLI_STATUS_ERROR = 0,
    IMU_CLI_STATUS_OK,
    IMU_CLI_STATUS_HELP
} ImuCliStatus_t;

typedef enum {
    IMU_CLI_ERROR_NONE = 0,
    IMU_CLI_ERROR_ARGUMENTS,
    IMU_CLI_ERROR_UNKNOWN_OPTION,
    IMU_CLI_ERROR_POSITIONAL,
    IMU_CLI_ERROR_MISSING_VALUE,
    IMU_CLI_ERROR_REPEATED_OPTION,
    IMU_CLI_ERROR_INVALID_VALUE,
    IMU_CLI_ERROR_OUT_OF_RANGE,
    IMU_CLI_ERROR_ILLEGAL_COMBINATION
} ImuCliError_t;

typedef struct {
    ImuCliStatus_t status;
    ImuCliError_t error;
    /* argv index; -1 when there is no particular offending token. */
    int errorIndex;
    /* Valid only when status == IMU_CLI_STATUS_OK. */
    ImuCmdPlan_t plan;
} ImuCliParse_t;

/*
 * Parse only. Does not modify argv, print, allocate, read input, obtain
 * time, initialize a coordinator/session, or touch hardware.
 */
ImuCliStatus_t imu_cli_parse(int argc, char *const argv[],
                             ImuCliParse_t *out);

#endif /* IMU_CLI_H */