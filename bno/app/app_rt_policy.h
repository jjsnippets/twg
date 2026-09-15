#ifndef APP_RT_POLICY_H
#define APP_RT_POLICY_H

#include "rt/realtime.h"

/*
 * Main-owned StartRT fallback policy. Header-only so host tests can include
 * it without linking HAL, SH-2, or realtime.c.
 *
 * This helper must not print, sleep, call StartRT, or terminate the process.
 * main prints warnings and owns process exit.
 *
 * CLOCK or INVALID_PERIOD is fatal even when MLOCKALL/SCHEDULER bits are
 * also set, because the 1 kHz grid cannot be armed.
 */

typedef enum {
    APP_RT_POLICY_OK = 0,
    APP_RT_POLICY_WARN_AND_CONTINUE,
    APP_RT_POLICY_FATAL
} AppRtPolicy_t;

static inline AppRtPolicy_t app_rt_policy_from_start(RT_StartStatus_t status)
{
    const RT_StartStatus_t fatal_bits =
        (RT_StartStatus_t)(RT_START_ERR_INVALID_PERIOD | RT_START_ERR_CLOCK);
    const RT_StartStatus_t warn_bits =
        (RT_StartStatus_t)(RT_START_ERR_MLOCKALL | RT_START_ERR_SCHEDULER);

    if (status == RT_START_OK) {
        return APP_RT_POLICY_OK;
    }
    if ((status & fatal_bits) != 0u) {
        return APP_RT_POLICY_FATAL;
    }
    if ((status & ~warn_bits) != 0u) {
        return APP_RT_POLICY_FATAL;
    }
    return APP_RT_POLICY_WARN_AND_CONTINUE;
}

#endif /* APP_RT_POLICY_H */