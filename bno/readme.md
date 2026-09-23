# BNO085 IMU Acquisition — Raspberry Pi 4B (Mantatow-Thesis)

Production IMU acquisition code for the thesis data-collection system. It reads
three fused/calibrated outputs from an Adafruit BNO085 9-DOF IMU over SPI (CEVA
SHTP/SH-2 protocol). Each output is configured as an independent periodic
100 Hz stream and the session is serviced by a single-threaded 1 kHz loop.

The reusable sensor-facing component is `app/imu_session.c` together with
`app/sh2_hal_rpi.c` and the vendored `sh2/` library. `app/main.c` is the
standalone `bno_app` example/diagnostic consumer; it is not the process-level
integration owner. The planned `twg` integration executable will own the one
master loop and cooperatively service the BNO085 and the other subsystems.

`app/app_sensor.c` and `app/app_contract.h` remain in the tree as an unused
version-3 leftover. `bno_app` does not link them.

> **Integration readiness:** `bno_app` now uses contract generation 1
> (`IMU_SAMPLE_CONTRACT_VERSION`). The snapshot carries per-group identities
> and a configuration epoch. It still does not carry publisher-relative
> freshness. Do not treat a combined `ImuSampleSnapshot_t` as proof that all
> three report groups advanced in the current 10 ms print boundary. See
> [Current snapshot semantics](#current-snapshot-semantics),
> [Latest validation evidence](#latest-validation-evidence), and
> [Integration work remaining](#integration-work-remaining).

## Quick start

On the Raspberry Pi:

```bash
cd bno
make                  # builds bin/bno_app
sudo ./bin/bno_app    # runs the 100 Hz snapshot loop; Ctrl-C to stop
```

- Requires `sudo` (or suitable capabilities) for real-time priority
  (`SCHED_FIFO` 90) and memory locking (`mlockall`). `StartRT()` never
  prints or exits. `bno_app` maps the returned bits: `MLOCKALL` and/or
  `SCHEDULER` warn and continue under default scheduling; `INVALID_PERIOD`
  or `CLOCK` is fatal before the 1 kHz loop. A later `RT_SleepUntil()`
  error is counted, printed in the scheduling report, then fatal.
- Runs a 300 ms service-only settle phase, marks the session OPERATIONAL,
  then a fixed 10 s acquisition window, printing the latest snapshot at
  100 Hz. Group event sequences are not reset after settle.
- The 100 Hz print boundary is consumer decimation, not a trigger-and-complete
  acquisition transaction. SH-2 remains continuously serviced at 1 kHz.

Previously established transport and scheduling performance:

- Approximately 100.3 Hz per report and 301 events/s combined during the
  earlier final transport validation.
- Zero dropped reports, decode errors, and unexpected packets in the earlier
  32,768-event packet-integrity run.
- Data-phase SPI reads approximately 190–250 us, empty polls approximately
  3–5 us, and a measured 1000.0 Hz loop under `SCHED_FIFO`.

These are throughput and transport-integrity results. They do not establish
that rotation vector, linear acceleration, and calibrated gyroscope are all
fresh in every arbitrary 10 ms host publication frame.

## Hardware wiring

Adafruit BNO085 breakout to Raspberry Pi 4B 40-pin header:

| Breakout pin | BNO085 signal | Pi GPIO (BCM) | Physical pin | Notes |
|---|---|---|---|---|
| VIN | power | 3V3 rail | 1 | 3.3 V logic; see Adafruit guide for VIN options |
| GND | ground | any GND | 6 | common ground |
| SCL | HSCL / SCK | GPIO 11 (SPI0 SCLK) | 23 | SPI clock |
| SDA | HSDA / MISO | GPIO 9 (SPI0 MISO) | 21 | data out of the BNO085 |
| DI | HMOSI | GPIO 10 (SPI0 MOSI) | 19 | data into the BNO085 |
| CS | HCSN | **GPIO 25** | **22** | manual chip select, not CE0/GPIO 8 |
| INT | H_INTN | GPIO 6 | 31 | active-low data-ready; required for stable SPI |
| RST | NRST | GPIO 13 | 33 | active-low reset |
| P0 (PS0/WAKE) | WAKE | GPIO 5 | 29 | protocol select and SPI wake handshake |
| P1 (PS1) | — | 3V3 (tied high) | 1 | PS1=1 and PS0=1 selects SPI host mode |

Wiring notes:

- PS1 must be high before and during reset to select SPI. The HAL drives
  PS0/WAKE high before pulsing reset, so both pins are high when the BNO085
  samples its protocol pins.
- INT is polled by the HAL during each service iteration; it is not configured
  as a kernel interrupt. Do not repurpose GPIO 6.
- If CS moves, update `GPIO_LINE_CS` in `app/sh2_hal_rpi.c`. A wire/code CS
  mismatch is a common failure after rewiring.

Host interface requirements:

- SPI enabled on the Pi (`raspi-config` -> Interface Options -> SPI), using
  `/dev/spidev0.0`.
- SPI mode 3 (CPOL=1, CPHA=1), 8 bits/word, 3 MHz. Do not raise the clock above
  the BNO085 limit used by this project.
- `SPI_NO_CS` is set and CS is driven manually through libgpiod. A free GPIO is
  used because spidev retains ownership of CE0/GPIO 8 even with `SPI_NO_CS`.
- libgpiod v1 API (`libgpiod-dev` 1.6.x on the supported Raspberry Pi OS
  images). A libgpiod v2 environment requires a HAL port.

## Directory layout

```text
bno/
├── Makefile                 unified build for app, tools, and tests
├── app/                     reusable production reader + standalone consumer
│   ├── imu_contract.h       generation-1 R1/R2/R3 contract used by bno_app
│   ├── imu_session.c/.h     sole HAL/SH-2 owner for bno_app
│   ├── imu_cmd.c/.h         host-only stage coordinator (not linked into bno_app)
│   ├── imu_cal.c/.h         host-only guided-calibration state machine (not linked into bno_app)
│   ├── imu_cal_adapter.c/.h    session-backed request translator (not linked into bno_app)
│   ├── imu_tare.c/.h        host-only tare-family state machine (not linked into bno_app)
│   ├── imu_tare_adapter.c/.h   session-backed tare request translator (not linked into bno_app)
│   ├── imu_check.c/.h       host-only continuous-check/probe machine (not linked into bno_app)
│   ├── imu_check_adapter.c/.h  session-backed check/probe translator (not linked into bno_app)
│   ├── app_rt_policy.h      StartRT warn-vs-fatal policy (main-owned)
│   ├── main.c               standalone bno_app loop/example
│   ├── app_contract.h       leftover ImuSample_t version 3 (unused by bno_app)
│   ├── app_sensor.c/.h      leftover v3 reader (unused by bno_app)
│   └── sh2_hal_rpi.c        Raspberry Pi transport (SPI + libgpiod)
├── calibration/             exclusive-session dynamic-calibration tool
├── orientation/             exclusive-session tare/orientation tool
├── validation/              AMT102 encoder + BNO085 validation harness
├── tests/                   bring-up diagnostics and unit tests
├── sh2/                     vendored CEVA sh2 library submodule
├── bin/                     generated binaries
└── build/                   generated objects

../rt/                       repository-level lab real-time helper
├── realtime.c               scheduling, locking, monotonic sleep
└── realtime.h               status and timing API
```

## Software architecture

The current production path is layered and single-threaded. One caller owns
all state and no mutex is required:

```text
app/main.c (standalone owner: 1 kHz service, 100 Hz snapshot/print)
  └─ imu_session (sole SH-2 owner; production config, epoch, R1/R2 mailbox)
       └─ sh2/ (SHTP framing, fragmentation, channels, report decode)
            └─ sh2_hal_rpi.c (spidev + libgpiod transport)
                 └─ /dev/spidev0.0 + gpiochip0 -> BNO085
```

`imu_session` is intentionally non-scheduling: `imu_session_service()` only
calls `sh2_service()` on the caller's thread. `StartRT()` and
`RT_SleepUntil()` belong to the executable that owns the loop. The standalone
application and selected standalone diagnostics are separate process-level
owners when run alone; their loops must not be nested or transplanted into the
future integrated executable.

### Command coordinator

`app/imu_cmd.c` is a host-only command-stage coordinator. It accepts an
immutable `ImuCmdPlan_t`, injected operator/process/timing events, and records
per-stage `ImuCmdResult_t` values. It does not open the BNO085, call
`imu_session_*`, parse CLI, sleep, print, read stdin, or call `exit`.
`bno_app` does not link or call it.

Stage order remains slot 1 (calibration or DCD-clear), slot 2 (tare family),
slot 3 (check or probe), settle, then acquisition. `flightCalMask` is immutable
run-plan data used by the composed calibration stage when restoring the
production policy; therefore `IMU_CMD_PLAN_VERSION` is version 2. Ordinary
stage failure and stage-local `q` warn and continue when the session remains
usable. `PROCESS_STOP` is instead a coordinator-level terminal event: the
active stage becomes `ABANDONED` with reason `PROCESS_STOP`, acquisition is
inhibited, and the plan stops. `RECOVERY_FAILED` similarly sets
`do_not_acquire` and stops the plan.

Calibration is no longer a stub stage. While `CALIBRATION` is active,
`imu_cmd` owns the `imu_cal` lifecycle: it initializes the machine on the
stage's first `TICK`, forwards `TICK`, `OPERATOR_Q`, and
`OPERATOR_CONFIRM`, services the machine, mirrors its progress, and adopts its
complete result. A `q` therefore reaches `imu_cal` first; it requests
production restoration before `imu_cmd` records the terminal
`CANCELLED` / `OPERATOR_Q` result. The coordinator preserves the complete
calibration result, including epochs, DCD-save status, verification status,
production-restoration status, and terminal progress. It does not reduce that
result to the old stub terminal representation.

`IMU_CMD_EVENT_STAGE_TERMINAL` remains the host seam for DCD-clear only.
It is rejected for active composed calibration, tare-family, check, and
probe stages: an injected terminal cannot bypass their machine-owned
restoration paths. `q` is ignored during settle and acquisition. Tare-now
versus persist, active versus saved clear, and probe requested versus
observed-actual masks remain distinct evidence. Generic coordinator
coverage is in `tests/test_imu_cmd.c`; composition coverage is in
`tests/test_imu_cmd_cal.c`, `tests/test_imu_cmd_tare.c`, and
`tests/test_imu_cmd_check.c`.

`app/imu_cal.c` is the host-only guided-calibration state machine used by the
composed calibration stage. It does not own the SH-2 session, sleep, print,
read stdin, or call `exit`. It receives caller-supplied monotonic time,
`ImuCalFacts_t`, operator `q` / confirm, and completed session-action results.
It emits at most one pending request: `CONFIGURE_CALIBRATION` (mask `0x07` at
start, mask `0` after verify reopen), `SAVE_DCD`, `VERIFY_REOPEN`, or
`RESTORE_PRODUCTION` with the immutable flight mask supplied to
`imu_cal_init`. It never calls `imu_session_*` or `sh2_*`.

The calibration submachine remains independent of the session owner. Only
`imu_cal_adapter` translates its four pending-request kinds into
`imu_session_*` calls. The external host owner, not `imu_cmd`, pumps that
adapter.

Oracle: `bno/calibration/cal_main.c` and `cal_sensor.c` at
`c90b75a8f553a2774b2d46d2cd279baef208dc7f`. Keep that tool until
parity. Do not write a calibration trajectory CSV; live facts stay on
R8.

Flow:

1. Configure calibration reports / policy.
2. Six accelerometer faces, 2 s each, then 3 s sustained accel >= 2.
   Up to 3 rounds. Exhaustion is informational.
3. Gyro rest, 15 s timeout. Exhaustion is informational.
4. Up to 3 outer mag/hold/save attempts. Each mag attempt has up to 5
   inner rounds: 8 s motion, then 3 s sustained mag >= 2. Mag
   exhaustion fails the command.
5. 10 s hold, then 3 s sustained accel+mag >= 2. Hold degrade starts
   the next outer attempt, or fails after attempt 3.
6. Save DCD through the session owner. One 5 s retry on the same
   attempt. A second failure starts the next outer attempt, or fails
   after attempt 3.
7. Planned verification reopen, then configure mask 0. Reopen failure
   is `RECOVERY_FAILED` and does not restore.
8. 10 s verify motion, then 3 s sustained accel+mag. Failure restores
   production and returns `CAL_VERIFY_GATE_FAILED`.
9. Every ordinary terminal path emits `RESTORE_PRODUCTION` first.
   Restore failure is `RECOVERY_FAILED`.

`q` in any active state except restore-wait cancels, restores, and
returns `CANCELLED` / `OPERATOR_Q` with a warning. All deadlines are
nanoseconds on the caller clock: `imu_cal_post(TICK)` updates `nowNs`;
`imu_cal_service()` evaluates gates. Sustained-good needs two ticks:
the first arms `goodSinceNs`, a later tick at least 3 s after that
passes the gate.

R8 (`ImuCmdProgress_t` v2) carries phase, pose, rounds, save attempt,
deadlines, live accuracies, mag vector, and `gatePassingNow`. R7
(`ImuCmdResult_t` v2) carries epoch before/after, `dcdSaved`,
`verified`, `restoredProduction`, and `terminalProgress`.

Host coverage is `tests/test_imu_cal.c`, `tests/test_session_cal.c`
for the session seams, and `tests/test_imu_cal_adapter.c` for request
translation. `make -C bno test` injects time, facts, confirms, `q`,
and session results. It does not open SPI.

### Phase 5.1 calibration adapter contract

`app/imu_cal_adapter.c` is a thin runtime bridge between host-only
`imu_cal` and the sole SH-2 owner, `imu_session`. It does not own a scheduler,
clock, operator input, printing, process lifetime, session open/close, session
service, settling, or the operational transition. `bno_app` does not link or
call it.

For Phase 5.2 host composition tests, the owner-loop turn is deliberately
explicit and has one fixed order:

1. Call `imu_session_service()`.
2. If composed calibration is active, call `imu_cal_adapter_pump()`.
3. Post the current `TICK` to `imu_cmd`.
4. Post any injected operator event to `imu_cmd`.
5. Call `imu_cmd_service()` once.

The adapter is pumped before the coordinator's tick/service work. A request
created by `imu_cal` during step 5 is therefore executed on the next owner-loop
turn. This preserves the one-request-per-pump boundary and avoids a hidden
second `imu_cmd_service()` pass.

One successful adapter pump:

1. Forwards the latest available `ImuCalFacts_t` mailbox into `imu_cal`.
2. Consumes at most one pending `imu_cal` request.
3. Executes the matching `imu_session` action.
4. Snapshots `configurationEpoch` immediately before and after that action.
5. Posts exactly one matching `IMU_CAL_EVENT_SESSION_RESULT`.

A `NONE` request is a successful no-op. A failed session action is still
posted as `success == false`; `imu_cal` owns retry, restore, and terminal
policy. `imu_cal_adapter_pump()` returns false only when required session
evidence cannot be obtained, the request is unsupported, or `imu_cal`
refuses the result event.

An adapter `false` is not an ordinary failed session action. In the Phase 5.2
composition host loop it is routed to `imu_cmd` as
`IMU_CMD_EVENT_SESSION_UNRESTORABLE`; `imu_cmd` records
`RECOVERY_FAILED`, inhibits acquisition, and stops the plan. An ordinary
session operation failure is instead posted by the adapter as a matching
session result with `success == false`, allowing `imu_cal` to own retry,
restore, and terminal policy.

| `imu_cal` request | Session action |
|---|---|
| `CONFIGURE_CALIBRATION` | `imu_session_configure_calibration(request.calMask)` |
| `SAVE_DCD` | `imu_session_save_dcd()` |
| `VERIFY_REOPEN` | `imu_session_begin_verification_reopen()` |
| `RESTORE_PRODUCTION` | `imu_session_restore_production(request.calMask)` |

`imu_session_restore_production(flightCalMask)` is legal from
`CALIBRATION` and `CONFIGURING` only. After a configured calibration
or verification path it increments epoch once, clears validity and
calibration-only facts, applies the 100 Hz production report set plus
the supplied flight mask, and returns `CONFIGURING`. It never enters
`CLOSED` and is not counted as recovery. An initial `CONFIGURING` call
before any report set was configured behaves as the first production
apply and does not extra-increment epoch. Restore failure faults
without claiming an epoch change.

Successful planned verification reopen also increments epoch once,
returns `CONFIGURING`, and must not set recovery-observed or
recovery-attempt counters. Failed planned reopen is an
unusable/recovery failure.

Host coverage is `tests/test_imu_cal_adapter.c` (A01 configure, A02
save, A03 planned reopen, A04 restore after `q`, A05 posted restore
failure) plus `tests/test_session_cal.c` restore seams. `tests/test_imu_cmd_cal.c`
adds the M-family composed host proof that `imu_cmd` drives the real calibration state
machine through this adapter boundary. Do not treat any of these host tests as
hardware parity. `bno_cal` and `bno_orient` remain exclusive-session regression
oracles. The adapter is not a scheduler or command coordinator.

The calibration adapter is not a scheduler or command coordinator. CLI,
`main`, production publication, and `twg/integration` remain outside this
host-tested command composition.

### Phase 6 tare contract

`app/imu_tare.c` is the host-only machine for `TARE`, `TARE_CLEAR`, and
`TARE_CHECK`. It does not call `imu_session_*` or `sh2_*`, and it does not
sleep, print, read stdin, or call `exit`. `imu_cmd` owns first-tick init,
tick/`q`/confirm forwarding, `imu_tare_service()`, progress mirroring, and
full-result adoption. Only `imu_tare_adapter` translates the five tare
requests into `imu_session_*` calls. `bno_app` does not link or call any of
these modules.

Mandatory persistence: a successful normal tare is Z persist or full-axis
persist. There is no RAM-only path. Tare-now and persist stay independent
sub-results. Tare-now success plus persist failure is overall
`FAILED` / `TARE_PERSIST_FAILED`, warning required, then continue if
production restore succeeds.

Axes: Z maps to `SH2_TARE_Z`; full maps to X|Y|Z. Both use the rotation-vector
basis. Timing is tick-driven: 2 s settle after configure, then tare-now,
persist, then 500 ms verification. Verification succeeds only when a valid
rotation-vector fact arrives in the post-tare-now epoch. It does not apply a
near-zero attitude threshold.

Clear: `--tare-imu --clear` succeeds only if both active and saved clear
sub-results succeed. The current session backend observes one
`sh2_clearTare()` result and maps success to both sub-results succeeded, or
failure to both failed. It cannot observe a partial clear. Mixed
active/saved outcomes remain representable on the machine (`T05` in
`tests/test_imu_tare.c`) so the contract is not collapsed.

`q` before the first session mutation cancels with `NOT_ATTEMPTED`
sub-results and no epoch change (`E08` / `N03` / `K02`). After configure has
executed, `q` restores production before the cancelled result becomes
terminal. `PROCESS_STOP` is coordinator-owned: the stage is
`ABANDONED` / `PROCESS_STOP`, the plan stops, and the owner must not pump
the adapter. Tare-check has no confirm and no deadline; `q` ends it as
`CANCELLED` / `OPERATOR_Q` without a warning, after restore.

Successful restore reapplies the 100 Hz production report set and the
immutable flight mask, increments epoch once, returns `CONFIGURING`, and is
not recovery. Restore failure is `RECOVERY_FAILED` and inhibits acquisition.

Host-tested owner-loop turn (do not reorder or add a second
`imu_cmd_service()`):

1. `imu_session_service()`
2. Pump the adapter for the one active initialized composed command
   (`CALIBRATION` → `imu_cal_adapter_pump()`, tare family →
   `imu_tare_adapter_pump()`, `CHECK`/`PROBE` →
   `imu_check_adapter_pump()`, otherwise none). Never pump more than one
   adapter in a turn.
3. Post the current `TICK` to `imu_cmd`
4. Post at most one operator event
5. `imu_cmd_service()` exactly once

A newly selected composed stage is initialized on its first valid
coordinator `TICK`, not at plan time zero. A request created in step 5 is
pumped on step 2 of the next turn. An adapter-invariant `false` becomes
`IMU_CMD_EVENT_SESSION_UNRESTORABLE`; a failed session action delivered
as a result remains the active machine's policy decision. The production
`bno_app` loop is not yet this composed owner loop.

Host IDs: T01–T05 in `tests/test_imu_tare.c`; E07/E08 and ST* in
`tests/test_session_tare.c`; U01–U12 in `tests/test_imu_tare_adapter.c`;
N01–N18 in `tests/test_imu_cmd_tare.c` (K02/K04/E07/E08 are traced there).
These are host tests. They do not prove SPI hardware parity. `bno_orient`
remains the exclusive-session regression oracle until a later parity phase.

### Phase 7 check and probe contract

The host-tested check/probe path is `imu_cmd` → `imu_check` →
`imu_check_adapter` → `imu_session`. `imu_cmd` selects slot 3, initializes
the pure machine on the first valid stage tick, forwards ticks and `q`,
mirrors its R8 progress, and adopts its complete R7 result. Only the
adapter translates the machine's `CONFIGURE_CHECK` and
`RESTORE_PRODUCTION` requests into calls on the sole SH-2 session owner.
Neither the machine nor the coordinator opens another session, calls
`sh2_*`, schedules a loop, sleeps, reads stdin, prints, or exits.

`CHECK` is continuous: it applies dynamic-calibration mask `0x00`,
has no deadline, and ends on operator `q` or coordinator-owned
`PROCESS_STOP`. `PROBE` applies the requested, unchanged 8-bit mask;
`0x00` is a valid probe mask and does not turn probe into continuous
check. Probe runs to its 10 s deadline even when the gate is reached
early, unless `q` ends the stage early or the process stops.
`--tare-imu --check` remains a separate tare-family attitude stage,
not this `--check-imu` diagnostic mode. CLI parsing of these spellings
has not been wired into `bno_app`.

Both diagnostic reader modes request the following report set:

| SH-2 report | Interval | Requested rate | Batch interval |
|---|---:|---:|---:|
| `SH2_MAGNETIC_FIELD_CALIBRATED` | 20,000 µs | 50 Hz | 0 |
| `SH2_ACCELEROMETER` | 100,000 µs | 10 Hz | 0 |
| `SH2_GYROSCOPE_CALIBRATED` | 100,000 µs | 10 Hz | 0 |
| `SH2_ROTATION_VECTOR` | 100,000 µs | 10 Hz | 0 |

`SH2_LINEAR_ACCELERATION` is disabled on diagnostic entry. Check and
probe use their own `CHECK` and `PROBE` reader states and an epoch-scoped
`ImuCheckFacts_t` mailbox; decoded diagnostic events do not populate
production R2 or calibration/tare facts. That mailbox keeps independent
per-report observation flags, host decode times, raw statuses, RV error
estimate, and calibrated magnetic XYZ. A new configuration epoch clears
those observations and production validity. Host seam tests prove the
requested configuration and mailbox transitions, not physical report
cadence or concurrent delivery.

`imu_check` alone decides the live go/no-go verdict. It counts facts
only from the configured command epoch. `gatePassingNow` requires both
accelerometer and magnetometer observed in that epoch and both raw
statuses at least 2. Gyro and RV remain diagnostics and cannot fail
this verdict; a low gyro status under disabled dynamic calibration is
not a reader error. The latest current-epoch fact remains usable
between reports; Phase 7 specifies no age-expiry threshold. Three
continuously passing seconds set sticky `gateReached`. A bad verdict
resets the current sustained-good timer but not that sticky history.

`IMU_CMD_PLAN_VERSION` remains 2. R7 `IMU_CMD_RESULT_VERSION` and R8
`IMU_CMD_PROGRESS_VERSION` are both 4. R8 is a polled latest-state
snapshot, not a queue of every good/bad transition. R7's
`terminalProgress.check` durably records the final `gatePassingNow`
and sticky `gateReached`, so a consumer can explain a cancelled or
timed-out stage without retaining an earlier R8 read. Probe R7 also
retains its requested mask, actual-mask validity/value if observed,
`probeReachedGate`, `probeTimedOut`, and
`probeOperatorEndedEarly`. The probe terminal *state/reason* and
terminal good/bad verdict answer different questions; reaching the
gate does not imply acquisition data will later be valid.

Check/probe configuration distinguishes requested from actual mask.
A successful `sh2_getCalConfig` readback that differs from the
effective/requested mask fails configuration while preserving the
observed actual value and an actionable diagnostic session for
restoration. Unavailable readback does not fabricate an actual value
and does not alone fail configuration. A hard report/mask-configuration
failure does not falsely claim successful diagnostic entry or a new
epoch. A test seam that captures attempted report intervals does not
establish hardware application of those settings.

After diagnostic configuration, ordinary completion is withheld until
the adapter delivers production-restore results. Restore disables
check-only accelerometer and calibrated magnetic-field reports,
re-enables production rotation vector, linear acceleration, and
calibrated gyro at 10,000 µs with batching zero, applies the immutable
`flightCalMask`, advances the configuration epoch once, clears check
facts and production validity, and returns the reader to
`CONFIGURING`. The later top-level owner, not `imu_check`, must perform
the production settle and operational transition.

| End condition | R7 state / reason | Plan behavior after restore |
|---|---|---|
| Continuous check `q` | `CANCELLED` / `OPERATOR_Q` | Continue; no warning after successful restore |
| Probe deadline | `TIMED_OUT` / `PROBE_DEADLINE` | Continue; `probeTimedOut` true |
| Probe `q` before deadline | `CANCELLED` / `OPERATOR_Q` | Continue; `probeOperatorEndedEarly` true |
| Actionable configuration failure | `FAILED` / `CONFIG_FAILED` | Warn and continue if restore succeeds |
| Restore failure or unusable session | `RECOVERY_FAILED` / `SESSION_UNUSABLE` | Set `doNotAcquire`; stop the plan |
| `PROCESS_STOP` | `ABANDONED` / `PROCESS_STOP` | Coordinator stops immediately; owner does not pump again |

The first four ordinary outcomes retain the final check verdict and
gate history in R7. `PROCESS_STOP` is not an `imu_check` event and never
waits for that machine's restore request. `SETTLING` and
`OPERATIONAL` remain production reader states, not check/probe states.

### Continuous acquisition

`imu_session_open()` establishes the SH-2 session and configuration epoch 1.
`imu_session_configure_production(0)` then enables rotation vector, linear
acceleration, and calibrated gyroscope independently with
`reportInterval_us = 10000` and `batchInterval_us = 0`, and applies flight
dynamic-calibration mask 0. The first production configure after open does not
extra-increment the epoch.

The owner calls `imu_session_service()` every 1 ms. SH-2 callbacks execute
inside that call and update only the report group represented by each event.
`bno_app` settles for 300 ms in `SETTLING`, then `imu_session_mark_operational()`
before printing. Every tenth application iteration it copies
`ImuSampleSnapshot_t` and prints epoch, `validMask`, and per-group identities
for groups that are valid in the current epoch. Equal requested periods do not
guarantee equal report phase or exactly one event from each group before every
host-defined 10 ms boundary.

### HAL transport

`app/sh2_hal_rpi.c` implements the four-callback `sh2_Hal_t` interface
(open, close, read, write, and time through the HAL object):

- A read returns immediately with zero when `H_INTN` is not asserted.
- A data read holds CS low, transfers the four-byte SHTP header, decodes the
  little-endian packet length with bit 15 masked, and transfers exactly the
  remaining payload.
- Writes wrap the complete SHTP packet in manual CS. If the device may be
  asleep, the HAL performs the WAKE handshake first. A transient zero-return
  write is retryable by the SH-2 stack.
- Reset holds RST low for 10 ms, releases it, and waits 120 ms. `sh2_open()`
  drains the advertisement and startup traffic.
- Those waits, and the 500 µs wake/INT poll (up to 200 ms), use `nanosleep`
  inside `sh2_hal_rpi.c`. Phase 3 records them as a whitelist; it does not
  move them into `imu_session`.
- Host time is based on `CLOCK_MONOTONIC`.

Historical note: the original HAL transferred the full 1024-byte receive
buffer on each data poll. At 3 MHz that consumed roughly 6.3 ms and constrained
combined throughput. Header-length-driven reads reduce normal 19–23-byte wire
packets to approximately 190–250 us. Wire lengths come from the SHTP header,
not the decoded CEVA report-length table.

### Real-time support

`rt/` contains the repository-level lab real-time helper.
`StartRT(priority, dt)` independently attempts `mlockall` and `SCHED_FIFO`
(priority 90 in `bno_app`), arms a monotonic deadline, and returns a bitmask of
all failures. It never prints or exits; the process-level caller owns policy.
`RT_SleepUntil(dt)` returns zero on schedule, a positive count of skipped
expired deadlines after an overrun, or a negative errno value on a clock/sleep
error. Overruns resume at one period after the detection time, so missed
frames are reported and skipped rather than replayed in a catch-up burst.
`RT_Reset()` re-arms the deadline at the current monotonic time. Monotonic time
avoids wall-clock steps or NTP adjustments during a run.

The BNO085 needs prompt servicing after data-ready. Any integrated owner must
preserve the approximately 1 ms SH-2 service cadence and avoid blocking work in
that loop. Long I/O, printing, file writes, calibration prompts, and sleeps
must not delay cooperative sensor service.

`bno_app` is the sole production caller of `StartRT()` and `RT_SleepUntil()`.
`app_rt_policy.h` selects the process policy; `imu_session` must not schedule,
print, or call `exit`. Standalone diagnostics that have their own `main()`
may call the RT helper; reusable modules may not.

| `StartRT()` bits | `bno_app` policy |
|---|---|
| `RT_START_OK` | Use requested FIFO / lock setup |
| `MLOCKALL` and/or `SCHEDULER` only | Warn on stderr and continue |
| Any `INVALID_PERIOD` or `CLOCK` bit | Fatal before the loop |
| Unknown bits | Fatal before the loop |

`CLOCK` or `INVALID_PERIOD` wins if mixed with lock/scheduler bits, because
the 1 kHz grid cannot be armed.

### Scheduling diagnostics

`bno_app` prints one private scheduling report to stderr at normal shutdown,
and the same report on a fatal clock/sleep error. These fields are **not**
R9/R10, not publisher freshness, and not a 10 ms publication deadline.

Settle (300 ms) and the 10 s operational window are counted separately.
Loop-body time is the full pre-sleep work, including `imu_session_service()`,
snapshot copy, and the 100 Hz `printf()`.

| Field | Meaning |
|---|---|
| `loops` | Finished 1 kHz iterations in that window |
| `max_body_ns` | Longest pre-sleep body in that window |
| `overrun_iters` | Iterations where `RT_SleepUntil()` returned `> 0` |
| `skipped_1ms` | Sum of those skipped 1 ms deadlines |
| `sleep_err` | `RT_SleepUntil()` `< 0` count (clean run is 0) |
| `start_rt status` / `policy` | Raw bits and `ok` / `warn_and_continue` / `fatal` |

## Current data contract

`app/imu_contract.h` is the source of truth for `bno_app`. The production
snapshot is `IMU_SAMPLE_CONTRACT_VERSION 1` (`ImuSampleSnapshot_t`). It is not
compatible with leftover `IMU_SAMPLE_STRUCT_VERSION 3` / `ImuSample_t`.

Each IMU group carries independent R1 metadata:

- `configurationEpoch` — session/report/policy generation; 0 means no session
- `groupEventSeq` — process-lifetime per-group identity; first event is 1;
  never reset on settle, epoch, or print row 1
- `hostDecodeNs` — `CLOCK_MONOTONIC` when the host accepted the event
- `sensorTimeUs` — SH-2 `timestamp_uS` (may be briefly non-monotonic)
- `deviceReportSeq` — raw wrapping SH-2 sequence; diagnostic only
- `rawStatus` — unmodified SH-2 accuracy/status, not a go/no-go verdict

Comparable identity is `(configurationEpoch, groupEventSeq)` only.
`validMask` is sticky seen-in-this-epoch state: bit 0 rotation, bit 1 accel,
bit 2 gyro. It is cleared on epoch increment. It does not mean a group is
fresh since the previous 10 ms print.

Orientation uses Tait-Bryan ZYX angles (yaw about Z, pitch about Y, and roll
about X). `orientationErrRad` is the rotation-vector heading-error estimate in
radians; it is not the SH-2 0–3 report status scale. The snapshot also stores
the rotation-vector quaternion.

### Current snapshot semantics

The callback writes only the fields belonging to the event that arrived, while
leaving the other groups at their previous values. Those previous values are
ineligible after an epoch increment until a new decode sets the group's valid
bit again.

The snapshot can prove last-decode identity, host/device time, raw status, and
whether a group has been seen in the current epoch. It cannot prove publisher
freshness. `freshMask` / `staleMask` / `missingMask` are not snapshot fields.
`bno_app` still prints a host-side 100 Hz view; it does not yet write an R9
publication CSV.

A reset observed on the SH-2 session increments epoch, clears `validMask`, and
moves the reader to `RECOVERING`. Recovery must not pass through `CLOSED`.

## Calibration policy

The standalone `bno_app` does not call `sh2_setCalConfig()` itself.
`imu_session_configure_production(0)` disables dynamic calibration for that
session so flight acquisition uses the previously saved dynamic calibration
data (DCD). Under this policy, BNO085 firmware can report calibrated-gyro
status 0 because the runtime zero-rate observer is halted, while the saved DCD
still provides bias correction. Readiness must not reject data solely because
gyro status is 0; use rotation-vector quality and `orientationErrRad` instead.

The current `bno_cal` and `bno_orient` programs each open and own an independent
SH-2 session. They must not run concurrently with `bno_app` or another SH-2
owner. Their start/service/stop flows, prompts, waits, resets, and report
configurations are standalone tool behavior—not reusable in-process commands
for the future integration loop.

## Latest validation evidence

The most recent supplied capture is
`bno_validate_20260908_175604.csv`, recorded on 2026-09-08 at 17:56:04 +0800.
It used a BNO085 over SPI at 3 MHz and logged rotation vector, linear
acceleration, and calibrated gyroscope alongside an AMT102-V encoder. The
capture contains 1,641 publication rows over 16.400199 s; all rows report
`valid_mask = 0x07` and the logger `drops` field remains zero.

Retrospective frame freshness was evaluated by comparing each report group's
8-bit `report_seq` between adjacent 100 Hz CSV rows. The first row establishes
the baseline, leaving 1,640 evaluated frame transitions:

| Metric | Rotation vector | Linear acceleration | Calibrated gyro |
|---|---:|---:|---:|
| Report events advanced | 1,643 | 1,643 | 1,644 |
| Achieved event rate | 100.182 Hz | 100.182 Hz | 100.243 Hz |
| Frames with no group advance | 21 | 17 | 19 |
| Frames with exactly one advance | 1,595 | 1,603 | 1,598 |
| Frames with two advances | 24 | 20 | 23 |
| Fresh in evaluated frames | 98.720% | 98.963% | 98.841% |

All three groups advanced in 1,584 of 1,640 evaluated transitions (96.585%).
The remaining transitions contained two fresh groups in 55 cases and only
acceleration fresh in one case. No transition had zero fresh IMU groups. Mean
value age at publication was 4.465 ms for rotation vector, 4.645 ms for linear
acceleration, and 4.349 ms for gyro; observed maxima were 10.970 ms, 9.920 ms,
and 10.812 ms, respectively.

The host publication loop itself averaged 99.9988 Hz (10.000122 ms mean
period), with a 10.195362 ms 99th-percentile period and 10.220795 ms maximum.
These results confirm approximately 100 Hz average delivery for every report,
but they also demonstrate that average rate is not equivalent to
all-three-fresh-per-frame behavior. Generation-1 per-group identities make
that mismatch measurable later; `bno_app` still does not compute a publisher
`freshMask`. The capture above was logged with the older validation contract.


## Diagnostic scope

| Program | Establishes | Does not establish |
|---|---|---|
| `test_min_period` | Queries report metadata and confirms requested-rate support | Simultaneous three-stream behavior or frame freshness |
| `test_report_len` | Exercises report/wire lengths, generally one selected report at a time | All-three-fresh delivery in a merged 10 ms frame |
| `bno_app` | Continuously services all three reports and snapshots epoch plus per-group identities | Publisher-relative freshness or CSV publication |
| `bno_validate` | Captures independent host/device times, report sequence, aggregate sequence, and status for each group | Runtime `freshMask`, pass/fail enforcement, or production-contract health reporting |
| `test_quad_decode` | Verifies the pure AMT102 quadrature decoder | IMU timing or freshness |
| `test_imu_cmd` | Host-only stage order, `q` vs process-stop, tare sub-results | Hardware, CLI, cal/tare/check machines, CSV |
| `test_imu_cal` | Host-only guided-cal gates, save/retry, `q`, restore requests | Hardware, CLI, `imu_session` execution |
| `test_session_cal` | Calibration configure, DCD save, planned reopen, production restore, epoch/recovery accounting | Publisher freshness or CLI |
| `test_imu_cal_adapter` | One-request-per-pump translation of the four `imu_cal` requests through `imu_session` | CLI, `main`, `imu_cmd` dispatch, hardware |
| `test_imu_cmd_cal` | Host-only `imu_cmd` → `imu_cal` → `imu_cal_adapter` → `imu_session` composition, including successful calibration, `q` restoration, and unrecoverable adapter routing | SPI hardware, CLI, `main`, `bno_app` wiring, tare/check/probe, CSV |
| `test_imu_tare` | Host-only tare/clear/check machine, T01–T05, settle/verify ticks | Hardware, CLI, `imu_session` execution |
| `test_session_tare` | Configure/tare-now/persist/clear/restore epoch and state, E07/E08 | Publisher freshness or CLI |
| `test_imu_tare_adapter` | One-request-per-pump translation of the five `imu_tare` requests | CLI, `main`, hardware |
| `test_imu_cmd_tare` | Composed tare family, stub rejection, owner-loop order, N-family | SPI hardware, CLI, `main`, `bno_app` wiring, CSV |
| `test_imu_check` | Pure check/probe machine, epoch verdict, 3 s sticky gate, 10 s deadline, `q`, restore policy; CK family | SH-2 execution or physical rates |
| `test_session_check` | Diagnostic report configuration, modes, mask evidence, epochs, mailbox, production restore; SC family | Physical mask/readback or report cadence |
| `test_imu_check_adapter` | One-request-per-pump session translation and result forwarding; CA family | CLI, scheduling, physical session behavior |
| `test_imu_cmd_check` | Composed owner-turn order, full R7/R8 adoption, failure/stop routing; NC family | CLI, `main`, `bno_app` wiring or SPI parity |

Build commands from `bno/`:

```bash
make            # bin/bno_app
make tools      # app, calibration, orientation, validation, encoder bring-up
make tests      # diagnostic binaries
make test       # host: quad_decode, session_r1_epoch, realtime_start,
                # rt_fallback_policy, imu_cmd, session_cal, imu_cal,
                # imu_cal_adapter, imu_cmd_cal, session_tare, imu_tare,
                # imu_tare_adapter, imu_cmd_tare, imu_check,
                # session_check, imu_check_adapter, imu_cmd_check,
                # scheduling audit
make clean      # remove build/ and bin/
```

Focused Phase 7 host binaries can also be run individually from the
repository root after building:

```sh
make -C bno bin/test_imu_check bin/test_session_check

bin/test_imu_check_adapter bin/test_imu_cmd_check
./bno/bin/test_imu_check
./bno/bin/test_session_check
./bno/bin/test_imu_check_adapter
./bno/bin/test_imu_cmd_check
```

## Integration work remaining

Phase 3 scheduling ownership is in `bno_app`: one `StartRT` /
`RT_SleepUntil` owner, no lower-layer `exit`, warn-and-continue on lock or
scheduler failure, fatal on clock or invalid period, and private 1 kHz
loop-body diagnostics. Publisher freshness, CSV/companion metadata, CLI,
command state machines, and `twg/integration` are still later work.

Phase 4 added `app/imu_cmd.c`. `bno_app` still does open → production
configure → 300 ms settle → 10 s print and does not call the coordinator.

Phase 5.2 completed host-only composition of `imu_cmd`, `imu_cal`, and
`imu_cal_adapter`. This is not production wiring: `bno_app` still does open →
production configure → 300 ms settle → 10 s print and does not link or call
`imu_cmd`, `imu_cal`, or `imu_cal_adapter`.

Phase 6 composed the tare family; Phase 7 adds the host-tested check/probe
path through imu_cmd → imu_check → imu_check_adapter →
imu_session. bno_app retains its existing no-flag open → production
configure → 300 ms settle → 10 s print behavior and does not link the
command machines or adapters. CLI parsing, stdin interaction, main
owner-loop wiring, R9/CSV/logger/publisher work, and twg/integration
remain outside Phase 7. bno_cal and bno_orient remain untouched,
exclusive-session regression oracles. Host tests cannot establish
hardware parity; CLI/main wiring and integrated BNO085 validation are
Phase 8 work, subject to separate authorization.

Before the BNO reader is integrated with pressure acquisition, networking,
video, or a GUI, the process-level publisher must still derive consumer-relative
freshness and age at each merged frame boundary from R1 identities. That work
must not ask `imu_session` to compute integration-relative freshness.

Calibration and tare remain exclusive-session binaries. They should ultimately
become nonblocking commands owned by the same active SH-2 session as normal
acquisition. The top-level integration executable must remain the sole
scheduler and continue servicing SH-2 at 1 kHz.

Implementation continues on the `bno-integrate` branch rather than `main`.
