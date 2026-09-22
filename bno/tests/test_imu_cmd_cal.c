#define _POSIX_C_SOURCE 200809L

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/imu_cal.h"
#include "app/imu_cal_adapter.h"
#include "app/imu_cmd.h"
#include "app/imu_session.h"

/*
 * Phase 5.2 host-only composition oracles, M family.
 *
 * The explicit owner turn is:
 *   1. imu_session_service()
 *   2. imu_cal_adapter_pump() only after calibration was initialized
 *   3. post IMU_CMD_EVENT_TICK
 *   4. optionally post one operator event
 *   5. imu_cmd_service()
 *
 * The first turn deliberately skips adapter pumping because imu_cal_init()
 * occurs inside imu_cmd_service() after that turn's tick. A request created
 * by command service is pumped on the next owner turn.
 *
 * No hardware open, SPI, sleep, stdin, scheduler, or direct session action.
 * Session actions occur only through imu_cal_adapter_pump().
 */

static int g_fail;

static const uint64_t T0 = 1000000000ull;

static void check(bool ok, const char *id, const char *what)
{
    if (!ok) {
        fprintf(stderr, "FAIL %s: %s\n", id, what);
        g_fail++;
    }
}

static ImuCmdEvent_t make_event(ImuCmdEventType_t type)
{
    ImuCmdEvent_t event;

    memset(&event, 0, sizeof(event));
    event.type = type;
    return event;
}

static ImuCmdEvent_t make_tick(uint64_t now_ns)
{
    ImuCmdEvent_t event = make_event(IMU_CMD_EVENT_TICK);

    event.monotonicNs = now_ns;
    return event;
}

static ImuCmdEvent_t make_calibration_terminal(void)
{
    ImuCmdEvent_t event = make_event(IMU_CMD_EVENT_STAGE_TERMINAL);

    event.terminal.identity = IMU_CMD_ID_CALIBRATION;
    event.terminal.state = IMU_CMD_STATE_SUCCEEDED;
    event.terminal.reason = IMU_CMD_REASON_OK;
    return event;
}

static ImuCmdProgress_t progress_of(const char *id)
{
    ImuCmdProgress_t progress;

    memset(&progress, 0, sizeof(progress));
    check(imu_cmd_get_progress(&progress), id, "get progress");
    return progress;
}

static ImuCmdResult_t result_of(ImuCmdIdentity_t identity, const char *id)
{
    ImuCmdResult_t result;

    memset(&result, 0, sizeof(result));
    check(imu_cmd_get_result(identity, &result), id, "get result");
    return result;
}

static ImuSampleSnapshot_t snapshot_of(const char *id)
{
    ImuSampleSnapshot_t snapshot;

    memset(&snapshot, 0, sizeof(snapshot));
    check(imu_session_get_snapshot(&snapshot), id, "get session snapshot");
    return snapshot;
}

static void reset_and_start_calibration(const ImuCmdPlan_t *plan,
                                        const char *id)
{
    imu_session_test_reset();
    check(imu_session_test_open(true), id, "test session open");
    check(imu_cmd_init(plan), id, "command init");

    imu_cmd_service();

    check(progress_of(id).active == IMU_CMD_ID_CALIBRATION, id,
          "calibration selected");
    check(imu_cmd_request() == IMU_CMD_REQ_RUN_COMMAND, id,
          "run-command request");
    check(imu_cmd_request_identity() == IMU_CMD_ID_CALIBRATION, id,
          "calibration request identity");
}

/*
 * One host owner turn. "cal_initialized_before_turn" belongs to this test
 * driver, not the production coordinator. It prevents the first turn from
 * pumping an adapter before imu_cmd's first tick initializes imu_cal.
 */
static bool owner_turn(uint64_t now_ns, bool cal_initialized_before_turn,
                       ImuCmdEventType_t optional_event,
                       const char *id)
{
    ImuCmdEvent_t event;
    bool cal_initialized_after_turn = cal_initialized_before_turn;

    imu_session_service();

    if (cal_initialized_before_turn) {
        if (!imu_cal_adapter_pump()) {
            event = make_event(IMU_CMD_EVENT_SESSION_UNRESTORABLE);
            check(imu_cmd_post(&event), id,
                  "adapter invariant failure posts unrestorable");
            imu_cmd_service();
            return false;
        }
    }

    event = make_tick(now_ns);
    check(imu_cmd_post(&event), id, "post command tick");

    if (optional_event != IMU_CMD_EVENT_TICK) {
        event = make_event(optional_event);
        check(imu_cmd_post(&event), id, "post optional command event");
    }

    imu_cmd_service();

    if (!cal_initialized_before_turn &&
        progress_of(id).active == IMU_CMD_ID_CALIBRATION) {
        cal_initialized_after_turn = true;
    }

    return cal_initialized_after_turn;
}

static ImuCalFacts_t good_facts(void)
{
    ImuCalFacts_t facts;

    memset(&facts, 0, sizeof(facts));
    facts.version = IMU_CAL_FACTS_VERSION;
    facts.valid = true;
    facts.accelAccuracy = 3u;
    facts.gyroAccuracy = 3u;
    facts.magAccuracy = 3u;
    facts.rvAccuracy = 3u;
    facts.rvErrRad = 0.10f;
    facts.haveMag = true;
    facts.magXuT = 20.0f;
    facts.magYuT = -5.0f;
    facts.magZuT = 40.0f;
    return facts;
}

static void inject_good_session_facts(const char *id)
{
    ImuCalFacts_t facts = good_facts();

    imu_session_test_inject_cal_facts(&facts);
    check(true, id, "inject good session facts");
}

static bool owner_turn_with_good_facts(uint64_t now_ns,
                                       bool cal_initialized_before_turn,
                                       ImuCmdEventType_t optional_event,
                                       const char *id)
{
    inject_good_session_facts(id);
    return owner_turn(now_ns, cal_initialized_before_turn, optional_event, id);
}

static bool sustain_good_facts(uint64_t *now_ns,
                               bool cal_initialized,
                               const char *id)
{
    *now_ns += 1ull;
    cal_initialized = owner_turn_with_good_facts(
        *now_ns, cal_initialized, IMU_CMD_EVENT_TICK, id);

    *now_ns += IMU_CAL_SUSTAINED_GOOD_NS + 1ull;
    cal_initialized = owner_turn_with_good_facts(
        *now_ns, cal_initialized, IMU_CMD_EVENT_TICK, id);

    return cal_initialized;
}

/*
 * Entry condition:
 * - calibration is initialized;
 * - CONFIGURE_CALIBRATION has completed;
 * - coordinator-visible progress is the first accel-face prompt.
 *
 * Exit condition:
 * - SAVE_DCD is pending;
 * - the request was produced by imu_cmd -> imu_cal service and has not
 *   yet been consumed by the adapter.
 */
static bool drive_to_save_request(uint64_t *now_ns,
                                  bool cal_initialized,
                                  const char *id)
{
    unsigned pose;
    ImuCmdProgress_t progress;
    ImuCalPendingRequest_t request;

    for (pose = 0u; pose < IMU_CAL_ACCEL_POSE_COUNT; pose++) {
        cal_initialized = owner_turn(
            *now_ns, cal_initialized, IMU_CMD_EVENT_OPERATOR_CONFIRM, id);

        *now_ns += IMU_CAL_ACCEL_FACE_WINDOW_NS;
        cal_initialized = owner_turn(
            *now_ns, cal_initialized, IMU_CMD_EVENT_TICK, id);

        cal_initialized = sustain_good_facts(now_ns, cal_initialized, id);
    }

    progress = progress_of(id);
    check(progress.cal.phase == IMU_CMD_CAL_PHASE_GYRO, id,
          "accel flow reaches gyro prompt");
    check(progress.requiredAction == IMU_CMD_ACTION_PRESS_Q_TO_END, id,
          "gyro prompt action mirrored");

    cal_initialized = owner_turn(
        *now_ns, cal_initialized, IMU_CMD_EVENT_OPERATOR_CONFIRM, id);
    cal_initialized = sustain_good_facts(now_ns, cal_initialized, id);

    progress = progress_of(id);
    check(progress.cal.phase == IMU_CMD_CAL_PHASE_MAG, id,
          "gyro gate reaches mag prompt");
    check(progress.requiredAction == IMU_CMD_ACTION_PRESS_Q_TO_END, id,
          "mag prompt action mirrored");

    cal_initialized = owner_turn(
        *now_ns, cal_initialized, IMU_CMD_EVENT_OPERATOR_CONFIRM, id);

    *now_ns += IMU_CAL_MAG_MOTION_NS;
    cal_initialized = owner_turn(
        *now_ns, cal_initialized, IMU_CMD_EVENT_TICK, id);
    cal_initialized = sustain_good_facts(now_ns, cal_initialized, id);

    progress = progress_of(id);
    check(progress.cal.phase == IMU_CMD_CAL_PHASE_HOLD, id,
          "mag gate reaches hold window");

    *now_ns += IMU_CAL_HOLD_WINDOW_NS;
    cal_initialized = owner_turn(
        *now_ns, cal_initialized, IMU_CMD_EVENT_TICK, id);
    cal_initialized = sustain_good_facts(now_ns, cal_initialized, id);

    request = imu_cal_pending_request();
    check(request.type == IMU_CAL_REQ_SAVE_DCD, id,
          "save DCD request pending");
    check(request.calMask == 0u, id, "save request mask is zero");

    return cal_initialized;
}

static bool drive_save_reopen_and_verify_config(uint64_t *now_ns,
                                                bool cal_initialized,
                                                const char *id)
{
    ImuCalPendingRequest_t request;
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;

    imu_session_test_set_save_dcd_result(true);
    before = snapshot_of(id);

    *now_ns += 1ull;
    cal_initialized = owner_turn(
        *now_ns, cal_initialized, IMU_CMD_EVENT_TICK, id);

    request = imu_cal_pending_request();
    after = snapshot_of(id);
    check(after.configurationEpoch == before.configurationEpoch, id,
          "DCD save does not increment epoch");
    check(request.type == IMU_CAL_REQ_VERIFY_REOPEN, id,
          "successful save requests verification reopen");

    imu_session_test_set_reopen_result(true);
    before = after;

    *now_ns += 1ull;
    cal_initialized = owner_turn(
        *now_ns, cal_initialized, IMU_CMD_EVENT_TICK, id);

    request = imu_cal_pending_request();
    after = snapshot_of(id);
    check(after.readerState == IMU_READER_STATE_CONFIGURING, id,
          "planned reopen returns session to configuring");
    check(after.configurationEpoch == before.configurationEpoch + 1u, id,
          "planned reopen increments epoch once");
    check(!imu_session_test_recovery_observed(), id,
          "planned reopen is not recovery");
    check(imu_session_test_recovery_attempt_count() == 0u, id,
          "planned reopen does not count recovery");
    check(request.type == IMU_CAL_REQ_CONFIGURE_CALIBRATION, id,
          "reopen requests verification configuration");
    check(request.calMask == 0u, id,
          "verification configuration uses zero calibration mask");

    *now_ns += 1ull;
    cal_initialized = owner_turn(
        *now_ns, cal_initialized, IMU_CMD_EVENT_TICK, id);

    request = imu_cal_pending_request();
    check(request.type == IMU_CAL_REQ_NONE, id,
          "verification configuration result consumed");

    return cal_initialized;
}

static bool start_and_configure(const ImuCmdPlan_t *plan,
                                uint64_t *now_ns,
                                const char *id)
{
    bool cal_initialized;

    reset_and_start_calibration(plan, id);

    cal_initialized = owner_turn(*now_ns, false, IMU_CMD_EVENT_TICK, id);
    check(cal_initialized, id, "first tick initializes calibration");

    *now_ns += 1ull;
    cal_initialized = owner_turn(
        *now_ns, cal_initialized, IMU_CMD_EVENT_TICK, id);

    check(progress_of(id).cal.phase == IMU_CMD_CAL_PHASE_ACCEL, id,
          "configure reaches accel prompt");
    return cal_initialized;
}

static ImuCmdPlan_t cal_then_tare_plan(uint8_t flight_mask)
{
    ImuCmdPlan_t plan;

    imu_cmd_plan_clear(&plan);
    plan.slot1 = IMU_CMD_ID_CALIBRATION;
    plan.slot2 = IMU_CMD_ID_TARE;
    plan.tareAxes = IMU_CMD_TARE_AXES_Z;
    plan.persistTare = true;
    plan.flightCalMask = flight_mask;
    return plan;
}

/*
 * M01:
 * - selecting calibration does not initialize imu_cal or start its clock;
 * - the first coordinator tick initializes imu_cal and leaves CONFIGURE
 *   pending for the next owner turn;
 * - the next owner turn pumps CONFIGURE through the adapter;
 * - an externally supplied calibration STAGE_TERMINAL is rejected.
 */
static void test_m01_first_tick_initializes_and_next_turn_pumps_configure(void)
{
    ImuCmdPlan_t plan;
    ImuCmdProgress_t progress;
    ImuCmdResult_t result;
    ImuSampleSnapshot_t before;
    ImuSampleSnapshot_t after;
    ImuCalPendingRequest_t request;
    ImuCmdEvent_t terminal;
    bool cal_initialized;

    imu_cmd_plan_clear(&plan);
    plan.slot1 = IMU_CMD_ID_CALIBRATION;
    plan.flightCalMask = 0x05u;

    reset_and_start_calibration(&plan, "M01");

    progress = progress_of("M01");
    result = result_of(IMU_CMD_ID_CALIBRATION, "M01");

    check(progress.active == IMU_CMD_ID_CALIBRATION, "M01",
          "calibration active before first tick");
    check(progress.cal.phase == IMU_CMD_CAL_PHASE_NONE, "M01",
          "no fabricated calibration phase before first tick");
    check(result.state == IMU_CMD_STATE_RUNNING, "M01",
          "coordinator calibration result running");
    check(result.startedNs == 0ull, "M01",
          "coordinator start time unarmed before first tick");

    terminal = make_calibration_terminal();
    check(!imu_cmd_post(&terminal), "M01",
          "external calibration terminal rejected before first tick");

    before = snapshot_of("M01");
    check(before.readerState == IMU_READER_STATE_CONFIGURING, "M01",
          "session still configuring before first tick");

    cal_initialized = owner_turn(T0, false, IMU_CMD_EVENT_TICK, "M01");
    check(cal_initialized, "M01", "first tick initialized calibration");

    progress = progress_of("M01");
    result = result_of(IMU_CMD_ID_CALIBRATION, "M01");
    request = imu_cal_pending_request();
    after = snapshot_of("M01");

    check(progress.active == IMU_CMD_ID_CALIBRATION, "M01",
          "calibration remains active after first tick");
    check(progress.cal.phase == IMU_CMD_CAL_PHASE_STARTUP, "M01",
          "first tick enters calibration startup");
    check(result.startedNs == T0, "M01",
          "first tick is coordinator start time");
    check(request.type == IMU_CAL_REQ_CONFIGURE_CALIBRATION, "M01",
          "configure remains pending until next owner turn");
    check(request.calMask == IMU_CAL_ENABLE_MASK, "M01",
          "configure uses calibration enable mask");
    check(after.readerState == IMU_READER_STATE_CONFIGURING, "M01",
          "first turn did not pump adapter");

    terminal = make_calibration_terminal();
    check(!imu_cmd_post(&terminal), "M01",
          "external calibration terminal rejected after initialization");

    cal_initialized = owner_turn(T0 + 1ull, cal_initialized,
                                 IMU_CMD_EVENT_TICK, "M01");
    check(cal_initialized, "M01", "calibration remains initialized");

    progress = progress_of("M01");
    request = imu_cal_pending_request();
    after = snapshot_of("M01");

    check(after.readerState == IMU_READER_STATE_CALIBRATION, "M01",
          "next owner turn configured session for calibration");
    check(after.configurationEpoch == before.configurationEpoch + 1u, "M01",
          "configure increments session epoch once");
    check(request.type == IMU_CAL_REQ_NONE, "M01",
          "configure result consumed");
    check(progress.active == IMU_CMD_ID_CALIBRATION, "M01",
          "calibration remains active after configure");
    check(progress.cal.phase == IMU_CMD_CAL_PHASE_ACCEL, "M01",
          "configure result advances to accel prompt");
    check(progress.cal.currentPoseIndex == 1u, "M01",
          "first accel pose selected");
    check(progress.requiredAction == IMU_CMD_ACTION_PRESS_Q_TO_END, "M01",
          "calibration prompt action mirrored");
}

static void test_m02_full_success_adopts_result_and_continues_to_tare(void)
{
    static const uint8_t flight_mask = 0x05u;

    ImuCmdPlan_t plan;
    ImuCmdProgress_t progress;
    ImuCmdResult_t result;
    ImuCalPendingRequest_t request;
    ImuSampleSnapshot_t before_restore;
    ImuSampleSnapshot_t after_restore;
    ImuCmdEvent_t terminal;
    uint8_t policy_mask = 0xffu;
    uint64_t now_ns = T0;
    bool cal_initialized;

    imu_cmd_plan_clear(&plan);
    plan.slot1 = IMU_CMD_ID_CALIBRATION;
    plan.slot2 = IMU_CMD_ID_TARE;
    plan.tareAxes = IMU_CMD_TARE_AXES_Z;
    plan.persistTare = true;
    plan.flightCalMask = flight_mask;

    reset_and_start_calibration(&plan, "M02");

    cal_initialized = owner_turn(now_ns, false, IMU_CMD_EVENT_TICK, "M02");
    check(cal_initialized, "M02", "first tick initializes calibration");

    now_ns += 1ull;
    cal_initialized = owner_turn(
        now_ns, cal_initialized, IMU_CMD_EVENT_TICK, "M02");

    progress = progress_of("M02");
    check(progress.cal.phase == IMU_CMD_CAL_PHASE_ACCEL, "M02",
          "configure reaches accel prompt");
    check(progress.cal.currentPoseIndex == 1u, "M02",
          "first accel pose active");

    cal_initialized = drive_to_save_request(
        &now_ns, cal_initialized, "M02");
    cal_initialized = drive_save_reopen_and_verify_config(
        &now_ns, cal_initialized, "M02");

    progress = progress_of("M02");
    check(progress.cal.phase == IMU_CMD_CAL_PHASE_VERIFY, "M02",
          "verification configuration reaches verify prompt");
    check(progress.requiredAction == IMU_CMD_ACTION_PRESS_Q_TO_END, "M02",
          "verify prompt action mirrored");

    cal_initialized = owner_turn(
        now_ns, cal_initialized, IMU_CMD_EVENT_OPERATOR_CONFIRM, "M02");

    now_ns += IMU_CAL_VERIFY_MOTION_NS;
    cal_initialized = owner_turn(
        now_ns, cal_initialized, IMU_CMD_EVENT_TICK, "M02");
    cal_initialized = sustain_good_facts(
        &now_ns, cal_initialized, "M02");

    request = imu_cal_pending_request();
    check(request.type == IMU_CAL_REQ_RESTORE_PRODUCTION, "M02",
          "successful verification requests production restore");
    check(request.calMask == flight_mask, "M02",
          "restore request carries immutable flight mask");

    before_restore = snapshot_of("M02");

    now_ns += 1ull;
    cal_initialized = owner_turn(
        now_ns, cal_initialized, IMU_CMD_EVENT_TICK, "M02");
    check(cal_initialized, "M02", "driver remembers initialization");

    after_restore = snapshot_of("M02");
    result = result_of(IMU_CMD_ID_CALIBRATION, "M02");

    check(result.version == IMU_CMD_RESULT_VERSION, "M02",
          "adopted result version");
    check(result.identity == IMU_CMD_ID_CALIBRATION, "M02",
          "adopted result identity");
    check(result.state == IMU_CMD_STATE_SUCCEEDED, "M02",
          "calibration succeeded");
    check(result.reason == IMU_CMD_REASON_OK, "M02",
          "calibration result reason OK");
    check(!result.warningRequired, "M02", "success has no warning");
    check(result.dcdSaved, "M02", "DCD save retained");
    check(result.verified, "M02", "verification retained");
    check(result.restoredProduction, "M02",
          "production restoration retained");
    check(result.startedNs == T0, "M02",
          "adopted first-tick start timestamp");
    check(result.endedNs >= result.startedNs, "M02",
          "adopted ordered terminal timestamps");
    check(result.epochBefore != 0u, "M02",
          "adopted initial calibration epoch");
    check(result.epochAfter == after_restore.configurationEpoch, "M02",
          "adopted final restoration epoch");
    check(result.epochAfter > result.epochBefore, "M02",
          "configuration epochs advanced through calibration");
    check(result.terminalProgress.version == IMU_CMD_PROGRESS_VERSION, "M02",
          "terminal progress version retained");
    check(result.terminalProgress.active == IMU_CMD_ID_CALIBRATION, "M02",
          "terminal progress snapshots last calibration progress");
    check(result.terminalProgress.cal.phase == IMU_CMD_CAL_PHASE_RESTORE, "M02",
          "terminal progress captures restore phase");

    check(after_restore.readerState == IMU_READER_STATE_CONFIGURING, "M02",
          "restore returns session to configuring");
    check(after_restore.configurationEpoch ==
              before_restore.configurationEpoch + 1u,
          "M02",
          "restore increments epoch once");
    check(imu_session_get_cal_policy(&policy_mask), "M02",
          "read restored production policy");
    check(policy_mask == flight_mask, "M02",
          "nonzero flight mask restored");
    check(!imu_session_test_recovery_observed(), "M02",
          "full success never enters recovery");
    check(imu_session_test_recovery_attempt_count() == 0u, "M02",
          "full success records no recovery attempt");
    check(!imu_cmd_warning_required(), "M02",
          "coordinator warning latch remains clear");
    check(!imu_cmd_do_not_acquire(), "M02",
          "successful calibration permits later acquisition");
    check(!imu_cmd_plan_complete(), "M02",
          "tare/settle/acquisition remain pending");

    imu_cmd_service();
    progress = progress_of("M02");
    check(progress.active == IMU_CMD_ID_TARE, "M02",
          "following tare stub starts normally");
    check(imu_cmd_request() == IMU_CMD_REQ_RUN_COMMAND, "M02",
          "tare is coordinator run-command request");
    check(imu_cmd_request_identity() == IMU_CMD_ID_TARE, "M02",
          "tare request identity");
    check(progress.requiredAction == IMU_CMD_ACTION_ALIGN_AND_CONFIRM, "M02",
          "tare retains existing stub action");

    terminal = make_event(IMU_CMD_EVENT_STAGE_TERMINAL);
    terminal.terminal.identity = IMU_CMD_ID_TARE;
    terminal.terminal.state = IMU_CMD_STATE_SUCCEEDED;
    terminal.terminal.reason = IMU_CMD_REASON_OK;
    terminal.terminal.sub.tareNow = IMU_CMD_SUB_SUCCEEDED;
    terminal.terminal.sub.persist = IMU_CMD_SUB_SUCCEEDED;

    check(imu_cmd_post(&terminal), "M02",
          "unmigrated tare terminal remains accepted");
}

static void test_m03_operator_q_waits_for_restore_then_continues(void)
{
    static const uint8_t flight_mask = 0x05u;

    ImuCmdPlan_t plan = cal_then_tare_plan(flight_mask);
    ImuCmdResult_t result;
    ImuCalPendingRequest_t request;
    ImuSampleSnapshot_t after;
    uint8_t policy_mask = 0xffu;
    uint64_t now_ns = T0;
    bool cal_initialized;

    cal_initialized = start_and_configure(&plan, &now_ns, "M03");

    now_ns += 1ull;
    cal_initialized = owner_turn(
        now_ns, cal_initialized, IMU_CMD_EVENT_OPERATOR_Q, "M03");

    request = imu_cal_pending_request();
    result = result_of(IMU_CMD_ID_CALIBRATION, "M03");
    check(request.type == IMU_CAL_REQ_RESTORE_PRODUCTION, "M03",
          "q requests production restore");
    check(request.calMask == flight_mask, "M03",
          "restore uses immutable flight mask");
    check(result.state == IMU_CMD_STATE_RUNNING, "M03",
          "calibration stays running while restore is pending");
    check(!imu_cmd_plan_complete(), "M03",
          "plan does not complete before restore");

    now_ns += 1ull;
    cal_initialized = owner_turn(
        now_ns, cal_initialized, IMU_CMD_EVENT_TICK, "M03");
    check(cal_initialized, "M03", "driver remembers initialization");

    result = result_of(IMU_CMD_ID_CALIBRATION, "M03");
    after = snapshot_of("M03");
    check(result.state == IMU_CMD_STATE_CANCELLED, "M03",
          "successful restore records cancellation");
    check(result.reason == IMU_CMD_REASON_OPERATOR_Q, "M03",
          "q reason retained");
    check(result.warningRequired, "M03", "q sets warning");
    check(result.restoredProduction, "M03", "restore recorded");
    check(imu_cmd_warning_required(), "M03", "coordinator latches warning");
    check(!imu_cmd_do_not_acquire(), "M03",
          "cancelled calibration may continue");
    check(after.readerState == IMU_READER_STATE_CONFIGURING, "M03",
          "restore returns configuring");
    check(imu_session_get_cal_policy(&policy_mask), "M03",
          "read restored policy");
    check(policy_mask == flight_mask, "M03",
          "nonzero flight mask restored after q");

    imu_cmd_service();
    check(progress_of("M03").active == IMU_CMD_ID_TARE, "M03",
          "next eligible stub stage starts");
}

static void test_m04_restore_failure_stops_plan(void)
{
    ImuCmdPlan_t plan = cal_then_tare_plan(0x05u);
    ImuCmdResult_t result;
    ImuCalPendingRequest_t request;
    uint64_t now_ns = T0;
    bool cal_initialized;

    cal_initialized = start_and_configure(&plan, &now_ns, "M04");

    now_ns += 1ull;
    cal_initialized = owner_turn(
        now_ns, cal_initialized, IMU_CMD_EVENT_OPERATOR_Q, "M04");
    request = imu_cal_pending_request();
    check(request.type == IMU_CAL_REQ_RESTORE_PRODUCTION, "M04",
          "restore pending before failed pump");

    imu_session_test_set_production_result(false);

    now_ns += 1ull;
    cal_initialized = owner_turn(
        now_ns, cal_initialized, IMU_CMD_EVENT_TICK, "M04");
    check(cal_initialized, "M04", "driver remembers initialization");

    result = result_of(IMU_CMD_ID_CALIBRATION, "M04");
    check(result.state == IMU_CMD_STATE_RECOVERY_FAILED, "M04",
          "failed restore is recovery failed");
    check(result.reason == IMU_CMD_REASON_CAL_RESTORE_FAILED, "M04",
          "restore-failure reason retained");
    check(!result.restoredProduction, "M04",
          "failed restore is not claimed");
    check(imu_cmd_do_not_acquire(), "M04", "do not acquire");
    check(imu_cmd_plan_complete(), "M04", "plan stopped");
    check(imu_cmd_request() == IMU_CMD_REQ_STOP_PLAN, "M04",
          "stop-plan request");

    imu_cmd_service();
    check(progress_of("M04").active == IMU_CMD_ID_NONE, "M04",
          "no later stage starts");
    check(result_of(IMU_CMD_ID_TARE, "M04").state ==
              IMU_CMD_STATE_NOT_REQUESTED,
          "M04",
          "tare not started");
    check(result_of(IMU_CMD_ID_SETTLE, "M04").state ==
              IMU_CMD_STATE_NOT_REQUESTED,
          "M04",
          "settle not started");
    check(result_of(IMU_CMD_ID_ACQUISITION, "M04").state ==
              IMU_CMD_STATE_NOT_REQUESTED,
          "M04",
          "acquisition not started");
}

static void test_m05_reopen_failure_keeps_dcd_and_stops(void)
{
    ImuCmdPlan_t plan = cal_then_tare_plan(0x05u);
    ImuCmdResult_t result;
    ImuCalPendingRequest_t request;
    uint64_t now_ns = T0;
    bool cal_initialized;

    cal_initialized = start_and_configure(&plan, &now_ns, "M05");
    cal_initialized = drive_to_save_request(&now_ns, cal_initialized, "M05");

    imu_session_test_set_save_dcd_result(true);
    now_ns += 1ull;
    cal_initialized = owner_turn(
        now_ns, cal_initialized, IMU_CMD_EVENT_TICK, "M05");
    request = imu_cal_pending_request();
    check(request.type == IMU_CAL_REQ_VERIFY_REOPEN, "M05",
          "save success requests reopen");

    imu_session_test_set_reopen_result(false);
    now_ns += 1ull;
    cal_initialized = owner_turn(
        now_ns, cal_initialized, IMU_CMD_EVENT_TICK, "M05");
    check(cal_initialized, "M05", "driver remembers initialization");

    request = imu_cal_pending_request();
    result = result_of(IMU_CMD_ID_CALIBRATION, "M05");
    check(request.type == IMU_CAL_REQ_NONE, "M05",
          "no restore after unusable session");
    check(result.state == IMU_CMD_STATE_RECOVERY_FAILED, "M05",
          "reopen failure is recovery failed");
    check(result.reason == IMU_CMD_REASON_CAL_REOPEN_FAILED, "M05",
          "reopen-failure reason retained");
    check(result.dcdSaved, "M05", "prior DCD save retained");
    check(!result.verified, "M05", "not verified");
    check(!result.restoredProduction, "M05", "not restored");
    check(imu_session_test_recovery_attempt_count() == 1u, "M05",
          "failed planned reopen counts recovery");
    check(imu_cmd_do_not_acquire(), "M05", "do not acquire");
    check(imu_cmd_plan_complete(), "M05", "plan stopped");

    imu_cmd_service();
    check(result_of(IMU_CMD_ID_TARE, "M05").state ==
              IMU_CMD_STATE_NOT_REQUESTED,
          "M05",
          "tare not started");
}

static void test_m06_progress_mirrors_and_confirm_routes_through_cmd(void)
{
    ImuCmdPlan_t plan = cal_then_tare_plan(0x05u);
    ImuCmdProgress_t progress;
    uint64_t now_ns = T0;
    bool cal_initialized;

    cal_initialized = start_and_configure(&plan, &now_ns, "M06");

    progress = progress_of("M06");
    check(progress.version == IMU_CMD_PROGRESS_VERSION, "M06",
          "progress version mirrored");
    check(progress.active == IMU_CMD_ID_CALIBRATION, "M06",
          "active identity is calibration");
    check(progress.cal.phase == IMU_CMD_CAL_PHASE_ACCEL, "M06",
          "accel phase mirrored");
    check(progress.cal.currentPoseIndex == 1u, "M06",
          "pose index mirrored");
    check(progress.cal.totalPoseCount == IMU_CAL_ACCEL_POSE_COUNT, "M06",
          "pose count mirrored");
    check(progress.cal.accelRound == 1u, "M06", "accel round mirrored");
    check(progress.requiredAction == IMU_CMD_ACTION_PRESS_Q_TO_END, "M06",
          "prompt action mirrored");

    now_ns += 1ull;
    cal_initialized = owner_turn_with_good_facts(
        now_ns, cal_initialized, IMU_CMD_EVENT_TICK, "M06");

    progress = progress_of("M06");
    check(progress.cal.accelAccuracy == 3u, "M06", "accel facts mirrored");
    check(progress.cal.gyroAccuracy == 3u, "M06", "gyro facts mirrored");
    check(progress.cal.magAccuracy == 3u, "M06", "mag facts mirrored");
    check(progress.cal.rvAccuracy == 3u, "M06", "rv facts mirrored");
    check(progress.cal.magXuT == 20.0f, "M06", "mag X fact mirrored");
    check(progress.cal.magYuT == -5.0f, "M06", "mag Y fact mirrored");
    check(progress.cal.magZuT == 40.0f, "M06", "mag Z fact mirrored");

    cal_initialized = owner_turn(
        now_ns, cal_initialized, IMU_CMD_EVENT_OPERATOR_CONFIRM, "M06");

    progress = progress_of("M06");
    check(progress.cal.phase == IMU_CMD_CAL_PHASE_ACCEL, "M06",
          "confirm through imu_cmd starts accel face window");
    check(progress.cal.deadlineNs != 0ull, "M06",
          "face-window deadline published");
    check(progress.cal.remainingNs != 0ull, "M06",
          "face-window remaining time published");
    check(progress.requiredAction == IMU_CMD_ACTION_PRESS_Q_TO_END, "M06",
          "window keeps q-to-end action");
    check(cal_initialized, "M06", "driver remembers initialization");
}

static void test_m07_process_stop_abandons_without_later_pump(void)
{
    ImuCmdPlan_t plan = cal_then_tare_plan(0x05u);
    ImuCmdEvent_t event;
    ImuCmdResult_t result;
    ImuCalPendingRequest_t request;
    ImuSampleSnapshot_t after;
    uint64_t now_ns = T0;
    bool cal_initialized;

    reset_and_start_calibration(&plan, "M07");
    cal_initialized = owner_turn(now_ns, false, IMU_CMD_EVENT_TICK, "M07");
    check(cal_initialized, "M07", "first tick initializes calibration");

    request = imu_cal_pending_request();
    check(request.type == IMU_CAL_REQ_CONFIGURE_CALIBRATION, "M07",
          "configure still pending at process stop");

    event = make_event(IMU_CMD_EVENT_PROCESS_STOP);
    check(imu_cmd_post(&event), "M07", "post process stop");
    imu_cmd_service();

    result = result_of(IMU_CMD_ID_CALIBRATION, "M07");
    after = snapshot_of("M07");
    request = imu_cal_pending_request();

    check(result.state == IMU_CMD_STATE_ABANDONED, "M07",
          "process stop abandons calibration");
    check(result.reason == IMU_CMD_REASON_PROCESS_STOP, "M07",
          "process-stop reason retained");
    check(result.state != IMU_CMD_STATE_CANCELLED, "M07",
          "process stop is not operator q");
    check(imu_cmd_process_stop_seen(), "M07", "stop seen");
    check(imu_cmd_do_not_acquire(), "M07", "do not acquire");
    check(imu_cmd_plan_complete(), "M07", "plan complete");
    check(imu_cmd_request() == IMU_CMD_REQ_STOP_PLAN, "M07",
          "stop-plan request");
    check(request.type == IMU_CAL_REQ_CONFIGURE_CALIBRATION, "M07",
          "owner did not pump after process stop");
    check(after.readerState == IMU_READER_STATE_CONFIGURING, "M07",
          "session left unconfigured by abandoned calibration");

    imu_cmd_service();
    check(progress_of("M07").active == IMU_CMD_ID_NONE, "M07",
          "no later stage after process stop");
}

static void test_m08_adapter_hard_failure_is_unrestorable(void)
{
    ImuCmdPlan_t plan = cal_then_tare_plan(0x05u);
    ImuCmdResult_t result;
    uint64_t now_ns = T0;
    bool cal_initialized;

    reset_and_start_calibration(&plan, "M08");
    cal_initialized = owner_turn(now_ns, false, IMU_CMD_EVENT_TICK, "M08");
    check(cal_initialized, "M08", "first tick initializes calibration");
    check(imu_cal_pending_request().type ==
              IMU_CAL_REQ_CONFIGURE_CALIBRATION,
          "M08",
          "configure pending before hard failure");

    imu_session_test_reset();

    now_ns += 1ull;
    (void)owner_turn(now_ns, cal_initialized, IMU_CMD_EVENT_TICK, "M08");

    result = result_of(IMU_CMD_ID_CALIBRATION, "M08");
    check(result.state == IMU_CMD_STATE_RECOVERY_FAILED, "M08",
          "adapter invariant failure is recovery failed");
    check(result.reason == IMU_CMD_REASON_SESSION_UNUSABLE, "M08",
          "unrestorable reason is session unusable");
    check(imu_cmd_do_not_acquire(), "M08", "do not acquire");
    check(imu_cmd_plan_complete(), "M08", "plan stopped");
    check(imu_cmd_request() == IMU_CMD_REQ_STOP_PLAN, "M08",
          "stop-plan request");

    imu_cmd_service();
    imu_cmd_service();
    check(result_of(IMU_CMD_ID_TARE, "M08").state ==
              IMU_CMD_STATE_NOT_REQUESTED,
          "M08",
          "tare not started");
}

int main(void)
{
    test_m01_first_tick_initializes_and_next_turn_pumps_configure();
    test_m02_full_success_adopts_result_and_continues_to_tare();
    test_m03_operator_q_waits_for_restore_then_continues();
    test_m04_restore_failure_stops_plan();
    test_m05_reopen_failure_keeps_dcd_and_stops();
    test_m06_progress_mirrors_and_confirm_routes_through_cmd();
    test_m07_process_stop_abandons_without_later_pump();
    test_m08_adapter_hard_failure_is_unrestorable();

    if (g_fail != 0) {
        fprintf(stderr, "test_imu_cmd_cal: %d failure(s)\n", g_fail);
        return 1;
    }

    printf("test_imu_cmd_cal: pass\n");
    return 0;
}
