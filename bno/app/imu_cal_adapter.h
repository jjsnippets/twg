#ifndef IMU_CAL_ADAPTER_H
#define IMU_CAL_ADAPTER_H

#include <stdbool.h>

/*
 * Thin runtime bridge between the host-only imu_cal state machine and the
 * sole SH-2 session owner, imu_session.
 *
 * This module does not own a scheduler, clock, operator input, printing,
 * process lifetime, session open/close, session service, settling, or
 * operational transition. The caller services imu_session and imu_cal
 * separately, then calls imu_cal_adapter_pump() once per owner-loop turn.
 *
 * One successful pump:
 * - forwards the latest available ImuCalFacts_t mailbox to imu_cal;
 * - consumes at most one imu_cal pending request;
 * - executes its matching imu_session action;
 * - snapshots configuration epochs immediately before and after that action;
 * - posts exactly one matching IMU_CAL_EVENT_SESSION_RESULT.
 *
 * A NONE request is a successful no-op. Returns false only when required
 * session evidence cannot be obtained, a request is unsupported, or imu_cal
 * refuses the result event. A failed session action is represented by a
 * successfully posted result with success == false, allowing imu_cal to own
 * retry, restore, and terminal policy.
 *
 * This Phase 5.1 adapter accepts only:
 * CONFIGURE_CALIBRATION, SAVE_DCD, VERIFY_REOPEN, and RESTORE_PRODUCTION.
 */
bool imu_cal_adapter_pump(void);

#endif /* IMU_CAL_ADAPTER_H */
