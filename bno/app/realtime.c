/*
 * realtime.c -- SCHED_FIFO setup + absolute-deadline periodic sleep.
 *
 * Debug-pass changes (2026-08-31), combined-mode rate investigation:
 *
 *  1. CLOCK FIX: StartRT()/RT_SleepUntil() now keep the absolute sleep
 *     deadline on CLOCK_MONOTONIC instead of clock id 0 (CLOCK_REALTIME).
 *     main.c measures its run window with CLOCK_MONOTONIC; sleeping
 *     against REALTIME lets any runtime time correction on the Pi (no
 *     battery RTC: NTP steps/slews can happen during a run) shift the
 *     loop's deadline clock relative to the window clock, which loses
 *     loop iterations. With both on the same clock the cadence is
 *     immune to time adjustments.
 *
 *  2. ROUNDING FIX: the per-call deadline advance is now
 *     (long)(dt * 1e9 + 0.5) instead of (long)(dt / 1e-9).
 *     For LOOP_DT_SEC = 0.001 both forms are exactly 1000000 ns, but
 *     for non-decimal periods (e.g. 1/300 s) the old form truncated
 *     and silently shortened every deadline.
 *
 *  Public API unchanged (realtime.h untouched): every test that links
 *  realtime.o builds and runs exactly as before, modulo the two fixes
 *  above.
 */

#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/mman.h>

#include "realtime.h"

/* Single clock for the deadline and the sleep; matches main.c's window. */
#define RT_CLOCK CLOCK_MONOTONIC

static struct sched_param param;
static struct timespec ts;

inline void tsnorm(struct timespec *ts)
{
  while (ts->tv_nsec >= 1e9) {
    ts->tv_nsec -= 1e9;
    ts->tv_sec++;
  }
}

int StartRT(int priority, double dt) {
  (void)dt; /* first deadline advance happens in RT_SleepUntil(); parameter kept for API compatibility */

  if (mlockall( MCL_CURRENT | MCL_FUTURE )) {
    perror("mlockall failed");
    return -1;
  }

  param.sched_priority = priority;

 if(sched_setscheduler(0, SCHED_FIFO, &param)==-1){
    perror("sched_setscheduler failed");
    exit(-1);
  }

  clock_gettime(RT_CLOCK, &ts);

  return 0;
}

void RT_SleepUntil(double dt)
{
    /* Absolute-advancing deadline: advance the persistent deadline by
     * exactly round(dt in ns), then sleep until that instant. Body time
     * longer than dt is absorbed (the sleep returns immediately), so the
     * iteration count over a window is wall-time / dt, independent of
     * per-iteration overhead. */
    ts.tv_nsec+=(long) (dt * 1e9 + 0.5);
    tsnorm(&ts);

    clock_nanosleep(RT_CLOCK, TIMER_ABSTIME, &ts, NULL);

    return;
}
