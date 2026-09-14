#define _POSIX_C_SOURCE 200809L

#include "rt/realtime.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/mman.h>
#include <time.h>

#define RT_CLOCK CLOCK_MONOTONIC
#define NS_PER_SEC 1000000000ull

static struct timespec s_deadline;
static bool s_timer_armed;

static int period_to_ns(double period_sec, uint64_t *period_ns)
{
    if ((period_ns == NULL) || !isfinite(period_sec) ||
        (period_sec <= 0.0)) {
        return EINVAL;
    }

    long double scaled = (long double)period_sec * (long double)NS_PER_SEC;
    if ((scaled < 1.0L) || (scaled > (long double)INT64_MAX)) {
        return ERANGE;
    }

    *period_ns = (uint64_t)(scaled + 0.5L);
    return 0;
}

static void timespec_add_ns(struct timespec *value, uint64_t delta_ns)
{
    value->tv_sec += (time_t)(delta_ns / NS_PER_SEC);
    value->tv_nsec += (long)(delta_ns % NS_PER_SEC);

    if (value->tv_nsec >= (long)NS_PER_SEC) {
        value->tv_nsec -= (long)NS_PER_SEC;
        value->tv_sec++;
    }
}

static int timespec_compare(const struct timespec *lhs,
                            const struct timespec *rhs)
{
    if (lhs->tv_sec != rhs->tv_sec) {
        return (lhs->tv_sec < rhs->tv_sec) ? -1 : 1;
    }
    if (lhs->tv_nsec != rhs->tv_nsec) {
        return (lhs->tv_nsec < rhs->tv_nsec) ? -1 : 1;
    }
    return 0;
}

static uint64_t timespec_diff_ns(const struct timespec *later,
                                 const struct timespec *earlier)
{
    time_t sec = later->tv_sec - earlier->tv_sec;
    long nsec = later->tv_nsec - earlier->tv_nsec;

    if (nsec < 0) {
        nsec += (long)NS_PER_SEC;
        sec--;
    }

    return (uint64_t)sec * NS_PER_SEC + (uint64_t)nsec;
}

RT_StartStatus_t StartRT(int priority, double period_sec)
{
    RT_StartStatus_t status = RT_START_OK;
    int first_error = 0;
    uint64_t period_ns;

    int rc = period_to_ns(period_sec, &period_ns);
    (void)period_ns;
    if (rc != 0) {
        s_timer_armed = false;
        errno = rc;
        return RT_START_ERR_INVALID_PERIOD;
    }

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        status |= RT_START_ERR_MLOCKALL;
        first_error = errno;
    }

    struct sched_param param = { .sched_priority = priority };
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
        status |= RT_START_ERR_SCHEDULER;
        if (first_error == 0) {
            first_error = errno;
        }
    }

    if (clock_gettime(RT_CLOCK, &s_deadline) != 0) {
        status |= RT_START_ERR_CLOCK;
        s_timer_armed = false;
        if (first_error == 0) {
            first_error = errno;
        }
    }
    else {
        s_timer_armed = true;
    }

    if (first_error != 0) {
        errno = first_error;
    }
    return status;
}

int RT_Reset(void)
{
    if (clock_gettime(RT_CLOCK, &s_deadline) != 0) {
        int error = errno;
        s_timer_armed = false;
        errno = error;
        return -error;
    }

    s_timer_armed = true;
    return 0;
}

int RT_SleepUntil(double period_sec)
{
    uint64_t period_ns;
    int rc = period_to_ns(period_sec, &period_ns);
    if (rc != 0) {
        errno = rc;
        return -rc;
    }
    if (!s_timer_armed) {
        errno = EPERM;
        return -EPERM;
    }

    struct timespec target = s_deadline;
    timespec_add_ns(&target, period_ns);

    struct timespec now;
    if (clock_gettime(RT_CLOCK, &now) != 0) {
        int error = errno;
        errno = error;
        return -error;
    }

    uint64_t skipped = 0;
    if (timespec_compare(&target, &now) <= 0) {
        uint64_t late_ns = timespec_diff_ns(&now, &target);
        skipped = (late_ns / period_ns) + 1u;

        target = now;
        timespec_add_ns(&target, period_ns);
    }

    rc = clock_nanosleep(RT_CLOCK, TIMER_ABSTIME, &target, NULL);
    if (rc != 0) {
        errno = rc;
        return -rc;
    }

    s_deadline = target;
    return (skipped > (uint64_t)INT_MAX) ? INT_MAX : (int)skipped;
}
