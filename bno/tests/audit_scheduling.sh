#!/usr/bin/env bash
# Phase 3 scheduling-ownership audit.
# Reusable production modules must not own StartRT, RT_SleepUntil, or exit.
# imu_session, imu_cal, and imu_cal_adapter must not sleep, print, or terminate.

set -u

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
BNO_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
REPO_DIR=$(cd "${BNO_DIR}/.." && pwd)

IMU_C="${BNO_DIR}/app/imu_session.c"
IMU_H="${BNO_DIR}/app/imu_session.h"
CAL_C="${BNO_DIR}/app/imu_cal.c"
CAL_H="${BNO_DIR}/app/imu_cal.h"
CAL_ADAPTER_C="${BNO_DIR}/app/imu_cal_adapter.c"
CAL_ADAPTER_H="${BNO_DIR}/app/imu_cal_adapter.h"
CLI_C="${BNO_DIR}/app/imu_cli.c"
CLI_H="${BNO_DIR}/app/imu_cli.h"
CONSOLE_C="${BNO_DIR}/app/imu_console.c"
CONSOLE_H="${BNO_DIR}/app/imu_console.h"
MAIN_C="${BNO_DIR}/app/main.c"
CAL_ADAPTER_C="${BNO_DIR}/app/imu_cal_adapter.c"
TARE_ADAPTER_C="${BNO_DIR}/app/imu_tare_adapter.c"
CHECK_ADAPTER_C="${BNO_DIR}/app/imu_check_adapter.c"
HAL_C="${BNO_DIR}/app/sh2_hal_rpi.c"
RT_C="${REPO_DIR}/rt/realtime.c"

fail=0

relpath() {
    local path="$1"
    echo "${path#${REPO_DIR}/}"
}

scan_calls() {
    local file="$1"
    shift
    local token hits

    if [ ! -f "$file" ]; then
        echo "FAIL missing file: $(relpath "$file")"
        fail=$((fail + 1))
        return
    fi

    for token in "$@"; do
        hits=$(grep -nE "(^|[^[:alnum:]_])${token}[[:space:]]*\(" "$file" || true)
        if [ -n "$hits" ]; then
            echo "FAIL $(relpath "$file"): forbidden ${token}("
            echo "$hits"
            fail=$((fail + 1))
        fi
    done
}

scan_calls "$IMU_C" StartRT RT_SleepUntil usleep nanosleep sleep exit printf
scan_calls "$IMU_H" StartRT RT_SleepUntil usleep nanosleep sleep exit printf
scan_calls "$CAL_C" StartRT RT_SleepUntil usleep nanosleep sleep exit printf
scan_calls "$CAL_H" StartRT RT_SleepUntil usleep nanosleep sleep exit printf
scan_calls "$CAL_ADAPTER_C" StartRT RT_SleepUntil usleep nanosleep sleep exit printf
scan_calls "$CAL_ADAPTER_H" StartRT RT_SleepUntil usleep nanosleep sleep exit printf
scan_calls "$CLI_C" StartRT RT_SleepUntil usleep nanosleep sleep exit \
    printf fprintf fgets getchar read
scan_calls "$CLI_H" StartRT RT_SleepUntil usleep nanosleep sleep exit \
    printf fprintf fgets getchar read
# poll/read and bounded worker waiting are allowed here; scheduling,
# process exit, and coordinator/session ownership are not.
scan_calls "$CONSOLE_C" StartRT RT_SleepUntil usleep nanosleep sleep exit \
    imu_cmd_service imu_session_service
scan_calls "$CONSOLE_H" StartRT RT_SleepUntil usleep nanosleep sleep exit \
    imu_cmd_service imu_session_service
# The existing main has no owner-side blocking stdin read; Step 8.5 must
# retain this property when the console adapter is wired.
scan_calls "$MAIN_C" fgets getchar getline scanf read
scan_calls "$CAL_ADAPTER_C" StartRT RT_SleepUntil
scan_calls "$TARE_ADAPTER_C" StartRT RT_SleepUntil
scan_calls "$CHECK_ADAPTER_C" StartRT RT_SleepUntil

# Production code has one visible scheduler owner and one normal-turn
# absolute-sleep call site, both in main.c.
if [ -f "$MAIN_C" ]; then
    sleep_sites=$(grep -Ec \
        '(^|[^[:alnum:]_])RT_SleepUntil[[:space:]]*\(' "$MAIN_C" || true)
    if [ "$sleep_sites" -ne 1 ]; then
        echo "FAIL $(relpath "$MAIN_C"): expected one RT_SleepUntil call site"
        fail=$((fail + 1))
    fi
fi
scan_calls "$RT_C" exit
scan_calls "$HAL_C" StartRT RT_SleepUntil exit

if [ -f "$HAL_C" ]; then
    if grep -qE "(^|[^[:alnum:]_])nanosleep[[:space:]]*\(" "$HAL_C"; then
        echo "whitelist: $(relpath "$HAL_C") nanosleep in sleep_us() (RESET_LOW_US 10 ms, RESET_WAIT_US 120 ms, INT_POLL_STEP_US 500 us; wake poll up to 200 ms)"
    else
        echo "FAIL $(relpath "$HAL_C"): expected whitelisted nanosleep was not found"
        fail=$((fail + 1))
    fi
fi

if [ -f "$RT_C" ]; then
    if grep -qE "(^|[^[:alnum:]_])clock_nanosleep[[:space:]]*\(" "$RT_C"; then
        echo "allow: $(relpath "$RT_C") clock_nanosleep (RT_SleepUntil implementation)"
    fi
fi

if [ "$fail" -ne 0 ]; then
    echo "audit_scheduling: ${fail} failure(s)"
    exit 1
fi

echo "audit_scheduling: pass"
exit 0