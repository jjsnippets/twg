#ifndef IMU_TARE_ADAPTER_H
#define IMU_TARE_ADAPTER_H

#include <stdbool.h>

/*
 * Thin runtime bridge between the host-only imu_tare state machine and the
 * sole SH-2 session owner, imu_session.
 *
 * Does not own a scheduler, clock, operator input, printing, process
 * lifetime, session open/close, session service, settling, or the
 * operational transition. The caller services imu_session and imu_tare
 * separately, then calls imu_tare_adapter_pump() once per owner-loop turn.
 *
 * One successful pump:
 * - forwards the latest available ImuTareFacts_t mailbox to imu_tare;
 * - consumes at most one imu_tare pending request;
 * - executes its matching imu_session action;
 * - snapshots configuration epochs immediately before and after that action;
 * - posts exactly one matching IMU_TARE_EVENT_SESSION_RESULT.
 *
 * A NONE request is a successful no-op. Returns false only when required
 * session evidence cannot be obtained, a request is unsupported, or imu_tare
 * refuses the result event. A failed session action is represented by a
 * successfully posted result with success == false, allowing imu_tare to own
 * restore and terminal policy.
 */
bool imu_tare_adapter_pump(void);

#endif /* IMU_TARE_ADAPTER_H */
