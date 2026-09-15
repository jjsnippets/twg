#define _POSIX_C_SOURCE 200809L

/*
 * Host-only test: StartRT status maps to warn-and-continue vs fatal.
 *
 * Expected: print "test_rt_fallback_policy: pass" and exit 0.
 *
 * Does not link realtime.c, HAL, or SH-2. No BNO hardware or sudo.
 * bno/Makefile `make test` will invoke this later (Phase 3 Step 3.5).
 */

#include <stdio.h>

#include "app/app_rt_policy.h"

static int g_fail;

static void check_policy(RT_StartStatus_t status, AppRtPolicy_t want,
                         const char *id)
{
    AppRtPolicy_t got = app_rt_policy_from_start(status);

    if (got != want) {
        fprintf(stderr, "FAIL %s: status=0x%xu got=%d want=%d\n",
                id, (unsigned)status, (int)got, (int)want);
        g_fail++;
    }
}

int main(void)
{
    const RT_StartStatus_t unknown = (RT_StartStatus_t)(1u << 4);

    check_policy(RT_START_OK, APP_RT_POLICY_OK, "OK");

    check_policy(RT_START_ERR_MLOCKALL,
                 APP_RT_POLICY_WARN_AND_CONTINUE, "MLOCKALL");
    check_policy(RT_START_ERR_SCHEDULER,
                 APP_RT_POLICY_WARN_AND_CONTINUE, "SCHEDULER");
    check_policy((RT_StartStatus_t)(RT_START_ERR_MLOCKALL |
                                    RT_START_ERR_SCHEDULER),
                 APP_RT_POLICY_WARN_AND_CONTINUE, "MLOCKALL|SCHEDULER");

    check_policy(RT_START_ERR_INVALID_PERIOD, APP_RT_POLICY_FATAL,
                 "INVALID_PERIOD");
    check_policy(RT_START_ERR_CLOCK, APP_RT_POLICY_FATAL, "CLOCK");
    check_policy((RT_StartStatus_t)(RT_START_ERR_CLOCK |
                                    RT_START_ERR_SCHEDULER),
                 APP_RT_POLICY_FATAL, "CLOCK|SCHEDULER");
    check_policy((RT_StartStatus_t)(RT_START_ERR_INVALID_PERIOD |
                                    RT_START_ERR_MLOCKALL),
                 APP_RT_POLICY_FATAL, "INVALID_PERIOD|MLOCKALL");

    check_policy(unknown, APP_RT_POLICY_FATAL, "UNKNOWN_BIT");
    check_policy((RT_StartStatus_t)(unknown | RT_START_ERR_MLOCKALL),
                 APP_RT_POLICY_FATAL, "UNKNOWN_BIT|MLOCKALL");

    if (g_fail != 0) {
        fprintf(stderr, "test_rt_fallback_policy: %d failure(s)\n", g_fail);
        return 1;
    }

    printf("test_rt_fallback_policy: pass\n");
    return 0;
}