#ifndef TWG_RT_REALTIME_H
#define TWG_RT_REALTIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* StartRT() result bits. More than one bit may be set. */
typedef uint32_t RT_StartStatus_t;

enum {
    RT_START_OK                 = 0u,
    RT_START_ERR_INVALID_PERIOD = 1u << 0,
    RT_START_ERR_MLOCKALL       = 1u << 1,
    RT_START_ERR_SCHEDULER      = 1u << 2,
    RT_START_ERR_CLOCK          = 1u << 3
};

/*
 * Attempt to lock process memory, select SCHED_FIFO for the calling thread,
 * and arm the monotonic periodic deadline.
 *
 * The function never prints and never terminates the process. It attempts the
 * independent memory-lock and scheduler operations even if one fails, then
 * returns their combined status bits. On failure, errno is restored to the
 * first failing operation's error. A clock failure leaves the timer unarmed.
 */
RT_StartStatus_t StartRT(int priority, double period_sec);

/*
 * Re-arm the periodic deadline at the current CLOCK_MONOTONIC time without
 * changing memory-lock or scheduling policy. Returns 0, or a negative errno
 * value if the clock cannot be read.
 */
int RT_Reset(void);

/*
 * Sleep to the next absolute CLOCK_MONOTONIC deadline.
 *
 * Return value:
 *   0  - the next nominal deadline was still in the future;
 *   >0 - that many expired deadlines were skipped before sleeping;
 *   <0 - negative errno value (for example, -EINTR).
 *
 * Recovery uses a skip-missed-frames policy: if the next deadline is already
 * expired, the function advances to "now + period". This intentionally resets
 * the phase and never runs catch-up sleeps against a series of past targets.
 * The positive result is capped at INT_MAX.
 */
int RT_SleepUntil(double period_sec);

#ifdef __cplusplus
}
#endif

#endif /* TWG_RT_REALTIME_H */
