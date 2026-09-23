#ifndef IMU_CHECK_ADAPTER_H
#define IMU_CHECK_ADAPTER_H

#include <stdbool.h>

/*
 * Thin bridge from host-only imu_check requests to the sole SH-2 owner,
 * imu_session. Call once per owner-loop turn for the active, initialized
 * CHECK or PROBE machine, after imu_session_service().
 *
 * A pump forwards one available facts snapshot, then executes at most one
 * pending request. Epochs are captured immediately before and after the
 * session action. The matching SESSION_RESULT is posted once.
 *
 * NONE is a successful no-op after facts forwarding. A session action that
 * returns success == false is still posted and is not an adapter failure:
 * imu_check owns restoration and terminal policy. False means an adapter
 * invariant failed (bad contract, unavailable epoch, unsupported request,
 * or rejected machine event).
 *
 * This module owns no clock, scheduler, input, output, session service,
 * session open/close, retry, or process lifetime.
 */
bool imu_check_adapter_pump(void);

#endif /* IMU_CHECK_ADAPTER_H */
