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
  (`SCHED_FIFO` 90) and memory locking (`mlockall`). `StartRT()` failure is
  non-fatal: the application logs a warning and continues under default
  scheduling.
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

Build commands from `bno/`:

```bash
make            # bin/bno_app
make tools      # app, calibration, orientation, validation, encoder bring-up
make tests      # diagnostic binaries
make test       # host-only test_quad_decode and test_session_r1_epoch
make clean      # remove build/ and bin/
```

## Integration work remaining

Phase 2 session-owner consolidation is in `bno_app`: one HAL/SH-2 owner,
generation-1 R1/R2 snapshot, configuration epoch, and production flight-cal
mask 0 applied by the session. Publisher freshness, CSV/companion metadata,
CLI, command state machines, and `twg/integration` are not in this phase.

Before the BNO reader is integrated with pressure acquisition, networking,
video, or a GUI, the process-level publisher must still derive consumer-relative
freshness and age at each merged frame boundary from R1 identities. That work
must not ask `imu_session` to compute integration-relative freshness.

Calibration and tare remain exclusive-session binaries. They should ultimately
become nonblocking commands owned by the same active SH-2 session as normal
acquisition. The top-level integration executable must remain the sole
scheduler and continue servicing SH-2 at 1 kHz.

Implementation continues on the `bno-integrate` branch rather than `main`.
