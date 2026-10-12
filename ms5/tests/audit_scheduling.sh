#!/usr/bin/env bash
# MS5837 scheduling- and boundary-ownership audit.
#
# Only the top-level capture executable (app/main.c) may call StartRT or
# RT_SleepUntil. Reusable pressure modules must not schedule, sleep, print, or
# terminate the process, and the pure publisher and driver must not depend on
# consumer policy (density, baseline, depth, logging, CLI, or Linux I/O).
#
# Function attribution is textual and relies on the project style: a function
# definition starts at column 0, and calls are indented.

set -u

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
MS5_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
REPO_DIR=$(cd "${MS5_DIR}/.." && pwd)
APP_DIR="${MS5_DIR}/app"

MAIN_C="${APP_DIR}/main.c"
DRIVER_C="${APP_DIR}/ms5837_driver.c"
DRIVER_H="${APP_DIR}/ms5837_driver.h"
HAL_H="${APP_DIR}/ms5837_hal.h"
HAL_RPI_C="${APP_DIR}/ms5837_hal_rpi.c"
HAL_RPI_H="${APP_DIR}/ms5837_hal_rpi.h"
MATH_C="${APP_DIR}/ms5837_math.c"
MATH_H="${APP_DIR}/ms5837_math.h"
CONTRACT_H="${APP_DIR}/ms5837_contract.h"
PUBLISH_C="${APP_DIR}/ms5837_publish.c"
PUBLISH_H="${APP_DIR}/ms5837_publish.h"
CLI_C="${APP_DIR}/ms5837_cli.c"
CLI_H="${APP_DIR}/ms5837_cli.h"
LOGGER_C="${APP_DIR}/ms5837_logger.c"
LOGGER_H="${APP_DIR}/ms5837_logger.h"
MAKEFILE="${MS5_DIR}/Makefile"
RT_C="${REPO_DIR}/rt/realtime.c"

fail=0

relpath() {
    local path="$1"
    echo "${path#${REPO_DIR}/}"
}

need_file() {
    if [ ! -f "$1" ]; then
        echo "FAIL missing file: $(relpath "$1")"
        fail=$((fail + 1))
        return 1
    fi
    return 0
}

# scan_calls FILE TOKEN... : fail if any TOKEN( appears in FILE.
scan_calls() {
    local file="$1"
    shift
    local token hits

    need_file "$file" || return

    for token in "$@"; do
        hits=$(grep -nE "(^|[^[:alnum:]_])${token}[[:space:]]*\(" "$file" || true)
        if [ -n "$hits" ]; then
            echo "FAIL $(relpath "$file"): forbidden ${token}("
            echo "$hits"
            fail=$((fail + 1))
        fi
    done
}

# scan_pattern FILE LABEL ERE : fail if the extended regex matches FILE.
scan_pattern() {
    local file="$1"
    local label="$2"
    local pattern="$3"
    local hits

    need_file "$file" || return

    hits=$(grep -nE "$pattern" "$file" || true)
    if [ -n "$hits" ]; then
        echo "FAIL $(relpath "$file"): forbidden ${label}"
        echo "$hits"
        fail=$((fail + 1))
    fi
}

# call_sites FILE ERE : print "function:line" for each indented match.
call_sites() {
    awk -v pat="$2" '
        /^[A-Za-z_]/ && /\(/ && !/;[[:space:]]*$/ && !/^typedef/ {
            if (match($0, /[A-Za-z_][A-Za-z0-9_]*\(/)) {
                fn = substr($0, RSTART, RLENGTH - 1)
            }
            next
        }
        /^[[:space:]]/ && $0 ~ pat { print fn ":" NR }
    ' "$1"
}

# require_sites FILE LABEL ERE EXPECTED_FUNCTION EXPECTED_COUNT
require_sites() {
    local file="$1"
    local label="$2"
    local pattern="$3"
    local expected_fn="$4"
    local expected_count="$5"
    local sites count wrong

    need_file "$file" || return

    sites=$(call_sites "$file" "$pattern")
    count=0
    if [ -n "$sites" ]; then
        count=$(printf '%s\n' "$sites" | wc -l)
    fi
    wrong=$(printf '%s\n' "$sites" | grep -v "^${expected_fn}:" | grep -v '^$' || true)

    if [ "$count" -ne "$expected_count" ]; then
        echo "FAIL $(relpath "$file"): expected ${expected_count} ${label} call site(s), found ${count}"
        [ -n "$sites" ] && echo "$sites"
        fail=$((fail + 1))
    elif [ -n "$wrong" ]; then
        echo "FAIL $(relpath "$file"): ${label} outside ${expected_fn}"
        echo "$wrong"
        fail=$((fail + 1))
    fi
}

# only_inside FILE LABEL ERE ALLOWED_FUNCTION : calls must be in that function.
only_inside() {
    local file="$1"
    local label="$2"
    local pattern="$3"
    local allowed="$4"
    local wrong

    need_file "$file" || return

    wrong=$(call_sites "$file" "$pattern" | grep -v "^${allowed}:" || true)
    if [ -n "$wrong" ]; then
        echo "FAIL $(relpath "$file"): ${label} outside ${allowed}"
        echo "$wrong"
        fail=$((fail + 1))
    fi
}

SCHEDULING="StartRT RT_SleepUntil usleep nanosleep clock_nanosleep sleep"
TERMINATION="exit abort _exit"
PRINTING="printf fprintf puts putchar perror"

# ---- Pure and reusable modules: no scheduling, sleeping, I/O text, exit ----
for file in "$DRIVER_C" "$DRIVER_H" "$HAL_H" "$MATH_C" "$MATH_H" \
            "$CONTRACT_H" "$PUBLISH_C" "$PUBLISH_H" "$CLI_C" "$CLI_H" \
            "$LOGGER_H"; do
    # shellcheck disable=SC2086
    scan_calls "$file" $SCHEDULING $TERMINATION $PRINTING
done

# The logger legitimately writes CSV with fprintf/fputs, but it must not
# schedule, sleep, print to stdout, or terminate the process.
# shellcheck disable=SC2086
scan_calls "$LOGGER_C" $SCHEDULING $TERMINATION printf puts putchar perror

# The Linux HAL may sleep only inside its HAL sleep callback.
# shellcheck disable=SC2086
scan_calls "$HAL_RPI_H" $SCHEDULING $TERMINATION $PRINTING
scan_calls "$HAL_RPI_C" StartRT RT_SleepUntil usleep sleep $TERMINATION $PRINTING
if need_file "$HAL_RPI_C"; then
    if grep -qE "(^|[^[:alnum:]_])nanosleep[[:space:]]*\(" "$HAL_RPI_C"; then
        only_inside "$HAL_RPI_C" "nanosleep" \
            "(^|[^[:alnum:]_])nanosleep[[:space:]]*\\(" rpi_sleep_ns
        echo "whitelist: $(relpath "$HAL_RPI_C") nanosleep in rpi_sleep_ns() (startup-only HAL callback)"
    else
        echo "FAIL $(relpath "$HAL_RPI_C"): expected whitelisted nanosleep was not found"
        fail=$((fail + 1))
    fi
fi

# The driver may use the HAL sleep callback only during synchronous init.
only_inside "$DRIVER_C" "HAL sleep_ns call" \
    "sleep_ns[[:space:]]*\\(" ms5837_driver_init
if need_file "$DRIVER_C"; then
    if [ -z "$(call_sites "$DRIVER_C" "sleep_ns[[:space:]]*\\(")" ]; then
        echo "FAIL $(relpath "$DRIVER_C"): expected HAL sleep_ns call in ms5837_driver_init"
        fail=$((fail + 1))
    else
        echo "whitelist: $(relpath "$DRIVER_C") HAL sleep_ns in ms5837_driver_init() (startup-only)"
    fi
fi

# ---- No other app file may own the real-time schedule ----
for file in "$APP_DIR"/*.c "$APP_DIR"/*.h; do
    [ "$file" = "$MAIN_C" ] && continue
    scan_calls "$file" StartRT RT_SleepUntil
    scan_pattern "$file" "real-time header include" \
        '#[[:space:]]*include[[:space:]]+"rt/(realtime|rt_policy)\.h"'
done

# ---- Top-level owner: one scheduler start, one normal-turn sleep ----
require_sites "$MAIN_C" "StartRT" \
    "(^|[^[:alnum:]_])StartRT[[:space:]]*\\(" main 1
require_sites "$MAIN_C" "RT_SleepUntil" \
    "(^|[^[:alnum:]_])RT_SleepUntil[[:space:]]*\\(" sleep_owner_turn 1
require_sites "$MAIN_C" "pressure service" \
    "(^|[^[:alnum:]_])ms5837_driver_service[[:space:]]*\\(" service_once 1

# Process-level waiting is startup policy only, before StartRT.
require_sites "$MAIN_C" "nanosleep" \
    "(^|[^[:alnum:]_])nanosleep[[:space:]]*\\(" sleep_relative_ns 1
require_sites "$MAIN_C" "sleep_relative_ns" \
    "(^|[^[:alnum:]_])sleep_relative_ns[[:space:]]*\\(" initialize_with_retry 1
echo "whitelist: $(relpath "$MAIN_C") nanosleep in sleep_relative_ns() (startup retry gap before StartRT)"

scan_calls "$MAIN_C" usleep clock_nanosleep sleep exit abort _exit \
    fgets getchar getline scanf read

# ---- Pure publisher and driver must not know consumer policy or I/O ----
for file in "$DRIVER_C" "$DRIVER_H" "$PUBLISH_C" "$PUBLISH_H"; do
    scan_pattern "$file" "consumer policy" \
        'density|surface_pressure|BaroRunConfig|depth_m|zero_estimator|ms5837_depth_from_reference'
    scan_pattern "$file" "consumer or Linux dependency" \
        '#[[:space:]]*include[[:space:]]+"(ms5837_logger|ms5837_cli|ms5837_hal_rpi)\.h"'
done

scan_pattern "$PUBLISH_C" "driver, HAL, thread, file, or clock dependency" \
    '#[[:space:]]*include[[:space:]]+("ms5837_driver\.h"|"ms5837_hal\.h"|<pthread\.h>|<stdio\.h>|<time\.h>)'
scan_pattern "$PUBLISH_H" "driver, HAL, thread, file, or clock dependency" \
    '#[[:space:]]*include[[:space:]]+("ms5837_driver\.h"|"ms5837_hal\.h"|<pthread\.h>|<stdio\.h>|<time\.h>)'
scan_calls "$PUBLISH_C" clock_gettime malloc calloc realloc free fopen fclose

# ---- Superseded helpers must stay unlinked ----
for file in "$APP_DIR"/*.c "$APP_DIR"/*.h; do
    case "$file" in
        */ms5837_reference.c|*/ms5837_reference.h) continue ;;
    esac
    scan_pattern "$file" "superseded helper" \
        'service_until_idle|CONVERSION_POLL_NS|ms5837_reference'
done

if need_file "$MAKEFILE"; then
    link_line=$(grep -E '^\$\(CAPTURE_APP\):' "$MAKEFILE" || true)
    if [ -z "$link_line" ]; then
        echo "FAIL $(relpath "$MAKEFILE"): CAPTURE_APP link rule not found"
        fail=$((fail + 1))
    elif printf '%s' "$link_line" | grep -q 'ms5837_reference'; then
        echo "FAIL $(relpath "$MAKEFILE"): capture still links ms5837_reference"
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
