#define _POSIX_C_SOURCE 200809L

/*
 * Host-only test: StartRT reports failure bits and never calls exit.
 *
 * Expected: print "test_realtime_start: pass" and exit 0.
 *
 * bno/Makefile `make test` will invoke this later (Phase 3 Step 3.5).
 * Do not require BNO hardware, sudo, or FIFO privileges.
 */

#include <errno.h>
#include <math.h>
#include <sched.h>
#include <stdio.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "rt/realtime.h"

int __real_mlockall(int flags);
int __real_sched_setscheduler(pid_t pid, int policy,
                              const struct sched_param *param);
int __real_clock_gettime(clockid_t clk, struct timespec *ts);

static int g_fail;
static int g_fail_mlockall;
static int g_fail_sched;
static int g_fail_clock;
static int g_mlockall_calls;
static int g_sched_calls;
static int g_clock_calls;

int __wrap_mlockall(int flags)
{
    g_mlockall_calls++;
    if (g_fail_mlockall) {
        errno = EPERM;
        return -1;
    }
    return __real_mlockall(flags);
}

int __wrap_sched_setscheduler(pid_t pid, int policy,
                              const struct sched_param *param)
{
    g_sched_calls++;
    if (g_fail_sched) {
        errno = EPERM;
        return -1;
    }
    return __real_sched_setscheduler(pid, policy, param);
}

int __wrap_clock_gettime(clockid_t clk, struct timespec *ts)
{
    g_clock_calls++;
    if (g_fail_clock) {
        errno = EPERM;
        return -1;
    }
    return __real_clock_gettime(clk, ts);
}

static void check(int ok, const char *id, const char *what)
{
    if (!ok) {
        fprintf(stderr, "FAIL %s: %s\n", id, what);
        g_fail++;
    }
}

static void check_bit(RT_StartStatus_t status, RT_StartStatus_t bit,
                      const char *id, const char *what)
{
    check((status & bit) == bit, id, what);
}

static void reset_wraps(void)
{
    g_fail_mlockall = 0;
    g_fail_sched = 0;
    g_fail_clock = 0;
    g_mlockall_calls = 0;
    g_sched_calls = 0;
    g_clock_calls = 0;
}

static void test_invalid_period(void)
{
    RT_StartStatus_t status;

    reset_wraps();
    errno = 0;
    status = StartRT(90, 0.0);
    check(status == RT_START_ERR_INVALID_PERIOD, "INV0",
          "zero period returns INVALID_PERIOD only");
    check(errno == EINVAL, "INV0", "errno is EINVAL");
    check(g_mlockall_calls == 0 && g_sched_calls == 0 && g_clock_calls == 0,
          "INV0", "no syscalls before period reject");

    reset_wraps();
    status = StartRT(90, -1.0);
    check(status == RT_START_ERR_INVALID_PERIOD, "INVNEG",
          "negative period returns INVALID_PERIOD only");
    check(g_mlockall_calls == 0, "INVNEG", "mlockall not called");

    reset_wraps();
    status = StartRT(90, NAN);
    check(status == RT_START_ERR_INVALID_PERIOD, "INVNAN",
          "NaN period returns INVALID_PERIOD only");
    check(g_mlockall_calls == 0, "INVNAN", "mlockall not called");
}

static void test_forced_mlockall(void)
{
    RT_StartStatus_t status;

    reset_wraps();
    g_fail_mlockall = 1;
    status = StartRT(90, 0.001);
    check_bit(status, RT_START_ERR_MLOCKALL, "MLOCK",
              "forced mlockall sets MLOCKALL");
    check((status & RT_START_ERR_INVALID_PERIOD) == 0, "MLOCK",
          "valid period is not INVALID_PERIOD");
    check(g_mlockall_calls >= 1, "MLOCK", "mlockall was called");
    check(g_sched_calls >= 1, "MLOCK", "scheduler still attempted");
}

static void test_forced_sched(void)
{
    RT_StartStatus_t status;

    reset_wraps();
    g_fail_sched = 1;
    status = StartRT(90, 0.001);
    check_bit(status, RT_START_ERR_SCHEDULER, "SCHED",
              "forced sched_setscheduler sets SCHEDULER");
    check((status & RT_START_ERR_INVALID_PERIOD) == 0, "SCHED",
          "valid period is not INVALID_PERIOD");
    check(g_sched_calls >= 1, "SCHED", "sched_setscheduler was called");
}

static void test_forced_lock_and_sched(void)
{
    RT_StartStatus_t status;

    reset_wraps();
    g_fail_mlockall = 1;
    g_fail_sched = 1;
    status = StartRT(90, 0.001);
    check_bit(status, RT_START_ERR_MLOCKALL, "BOTH", "MLOCKALL set");
    check_bit(status, RT_START_ERR_SCHEDULER, "BOTH", "SCHEDULER set");
    check((status & RT_START_ERR_INVALID_PERIOD) == 0, "BOTH",
          "not INVALID_PERIOD");
}

static void test_forced_clock(void)
{
    RT_StartStatus_t status;

    reset_wraps();
    g_fail_clock = 1;
    status = StartRT(90, 0.001);
    check_bit(status, RT_START_ERR_CLOCK, "CLOCK",
              "forced clock_gettime sets CLOCK");
    check((status & RT_START_ERR_INVALID_PERIOD) == 0, "CLOCK",
              "not INVALID_PERIOD");
    check(g_clock_calls >= 1, "CLOCK", "clock_gettime was called");
}

int main(void)
{
    test_invalid_period();
    test_forced_mlockall();
    test_forced_sched();
    test_forced_lock_and_sched();
    test_forced_clock();

    if (g_fail != 0) {
        fprintf(stderr, "test_realtime_start: %d failure(s)\n", g_fail);
        return 1;
    }

    printf("test_realtime_start: pass\n");
    return 0;
}