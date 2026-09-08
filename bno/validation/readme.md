# validation/ — AMT102 encoder vs BNO085 IMU validation

Tools that check the BNO085's orientation data against an AMT102 rotary
encoder mounted on the same pivot, with the encoder as ground truth. The
main artifact is `sensor_validate`: a synchronized capture binary that
logs encoder angle and IMU data into one timestamped CSV.

The comparison is angle-vs-angle (graphical comparison of the two
waveforms), so rig geometry does not enter the data path — arm length,
mass, and pivot details are recorded only as run metadata. Analysis and
formal acceptance criteria are applied post-hoc by a separate program
(to be determined); this directory covers **acquisition only**.

Related documentation:
- `../readme.md` — app architecture, data contract, sudo/RT policy
- `../calibration/readme.md` — `bno_cal` guided calibration, the
  flight-time calibration policy (both acquisition programs fly on the
  saved DCD with all dynamic calibration disabled), and `bno_orient`
  swing-axis tare

## Hardware setup

### Encoder (AMT102-V)

- Resolution: 2048 PPR (DIP switches 1–4 all **Off**, factory preset),
  x4 quadrature decode → **8192 counts/rev**, 0.044° per count
- Series 1 kΩ resistors on each signal line into the Pi header:

| Encoder signal | Through | Pi header pin | BCM line |
|---|---|---|---|
| A (quadrature) | 1 kΩ | pin 11 | GPIO 17 |
| B (quadrature) | 1 kΩ | pin 13 | GPIO 27 |
| X (index, 1/rev) | 1 kΩ | pin 15 | GPIO 22 |
| 5 V | — | 5 V rail | — |
| G | — | GND | — |

- The encoder connector pin order is **B, 5V, A, X, G** (T unused) — 5 V
  sits between B and A, which makes a power/ground swap easy to make and
  destructive. Double-check before powering.

### IMU (BNO085)

- Same SPI wiring and HAL as the application (`../app/sh2_hal_rpi.c`);
  see `../readme.md`.

### Rig facts (placeholders — fill in per build)

These are recorded as run metadata only; they do not affect the capture:

- Arm length: **[TBD] m** (`ARM_LENGTH_M` in `main.c`)
- IMU mount orientation / which IMU axis lies in the swing plane: **[TBD]**
  (the persisted `bno_orient` tare encodes this alignment — re-tare
  after any remount)
- Pivot shaft diameter / encoder sleeve: **[TBD]**
- Encoder mounting: base at pivot, shaft end accessible: **[TBD]**

When a rig fact is settled, update `ARM_LENGTH_M` and `RUN_NOTES` in
`main.c` so the CSV header block carries it per capture.

## Prerequisites

1. **Calibration**: `bno_cal --check` must exit 0 (see
   `../calibration/readme.md`). Recalibrate with `bno_cal` in the
   deployment area whenever the magnetic environment changes. In the
   CSV, `rv_accuracy` = 3.1416 (π) means "unreliable" — if it reads π
   throughout a capture, the calibration is absent or degraded.
2. **Orientation (tare)**: `bno_orient --persist` must have been run
   with the fixture at its swing-axis zero (see
   `../calibration/readme.md`), so logged yaw zero equals swing zero
   and the angle-vs-angle comparison needs no post-hoc offset. Confirm
   with `bno_orient --check` (yaw ≈ 0 at the zero); re-run after
   remounting the IMU or changing the mechanical zero. The tare lives
   in a separate flash record from the DCD — recalibrating does not
   disturb it.
3. **sudo**: required for spidev + gpiochip0 access and SCHED_FIFO
   rtprio limits (same policy as `bno_app`).
4. **No competing sh2 consumer**: `bno_app`, `bno_cal`, or any other
   program holding the BNO085 SPI/HAL must not be running — one HAL
   instance per process. `sensor_validate` replaces `bno_app` for the
   duration of a capture.
5. **GPIO lines free**: `gpioinfo gpiochip0` should show lines 17, 27,
   and 22 unused.

## Build

```sh
cd validation
make            # bin/amt102_bringup, bin/test_quad_decode, bin/sensor_validate
make validate   # bin/sensor_validate only
make test       # build and run the decoder unit test (no hardware)
make clean
```

Flags match `../Makefile`: `gcc -Wall -Wextra -O2 -std=c11 -D_GNU_SOURCE`.
The `sensor_validate` target compiles the sh2 library, the app HAL and
RT helpers, and the encoder modules in this directory.

## Binaries

| Binary | Purpose |
|---|---|
| `bin/sensor_validate` | Synchronized encoder + IMU capture to CSV (the validation tool) |
| `bin/amt102_bringup` | Standalone encoder bring-up / hardware integration test |
| `bin/test_quad_decode` | Pure quadrature-decoder unit test, no hardware, no libgpiod |

## sensor_validate — usage and runtime behavior

```sh
sudo ./bin/sensor_validate [-o out.csv] [-d seconds]
```

- `-o out.csv` — output file (default `sensor_validate_YYYYMMDD_HHMMSS.csv`)
- `-d seconds` — log duration, counted from the end of the settle phase;
  `0` (default) runs until Ctrl+C

Runtime structure:

- **Three threads.** The main thread becomes the RT loop (SCHED_FIFO
  priority 90, 1 kHz): it services the SH-2 session every ~1 ms and,
  every 10th iteration, assembles one 100 Hz record. The encoder thread
  (default scheduling) drains kernel-timestamped line events and
  publishes a snapshot under a mutex. The logger thread (default
  scheduling) drains a bounded ring to the CSV file — the RT loop never
  touches the file, so SD-card write stalls cannot perturb the service
  cadence.
- **Why only the IMU loop is RT:** the BNO085's SHTP protocol has a
  service deadline (~1 ms; the device retries after ~10 ms and starves
  if delayed). The encoder has no such deadline — edge capture and
  timestamping happen in kernel interrupt context, and the user-space
  thread only drains a kernel-buffered queue.
- **Calibration policy:** `sh2_setCalConfig(0)` at session start;
  `sensor_validate` flies on the saved DCD only with all dynamic
  calibration disabled, mirroring `bno_app` (see `../calibration/readme.md`).
- **Settle then seq reset:** after startup there is a 0.3 s service-only
  drain, then `sensor_validate_resetSeq()` zeroes the decode counter —
  same contract as `sensor_reader_resetSeq()` in `bno_app`. Logged `seq`
  counts capture-window events only.
- **1 Hz status line** while running:
  `ticks`, `drops`, encoder `count`/angle/`inv`/`ev/s`, `imu/s`,
  `valid_mask`. On exit: a summary block and an informational verdict.
- Exit codes: 0 normal end, 1 usage error, 2 setup failure.

## CSV file contract

Layout: a `#`-prefixed metadata block, one plain header row (the first
non-comment line — `read_csv(comment='#')` works directly), one row per
100 Hz tick, then a `#`-prefixed footer (`records`, `dropped`,
`duration_sec`).

Clocks: `host_ts_ns`, `*_host_ts_ns`, and `enc_event_ts_ns` are
CLOCK_MONOTONIC nanoseconds — the same clock domain, including the
kernel's encoder event stamps. `*_sensor_ts_us` are BNO085 device
timestamps (also MONOTONIC-domain via the HAL). All alignment for
analysis is by timestamp, never by row position.

| Column | Unit | Meaning |
|---|---|---|
| `host_ts_ns` | ns | record assembly time (tick) |
| `enc_count` | counts | x4 encoder counts since open |
| `enc_angle_deg` | deg | `enc_count` × 360/8192, continuous (unwrapped) |
| `enc_event_ts_ns` | ns | kernel stamp of the newest encoder event |
| `enc_x_pulses` | — | index pulses since open |
| `enc_invalid` | — | illegal quadrature transitions since open |
| `enc_edges_ab` | — | A+B edge events since open |
| `rv_sensor_ts_us` / `rv_host_ts_ns` / `rv_seq` | µs / ns / – | latest rotation-vector report: device stamp, host decode time, device per-sensor sequence (mod 256) |
| `rv_qw`, `rv_qx`, `rv_qy`, `rv_qz` | — | raw rotation vector unit quaternion ($w$ real, $x=i$, $y=j$, $z=k$) |
| `yaw`, `pitch`, `roll` | rad | orientation from the RV quaternion (`euler.c` convention) |
| `rv_accuracy` | rad | RV accuracy estimate (`rvErrRad`); π = unreliable |
| `acc_sensor_ts_us` / `acc_host_ts_ns` / `acc_seq` | µs / ns / – | latest linear-acceleration report metadata |
| `ax`, `ay`, `az` | m/s² | linear acceleration |
| `gyr_sensor_ts_us` / `gyr_host_ts_ns` / `gyr_seq` | µs / ns / – | latest calibrated-gyro report metadata |
| `gx`, `gy`, `gz` | rad/s | calibrated gyroscope |
| `valid_mask` | – | bit 0: RV, bit 1: accel, bit 2: gyro (set once a group has ever reported) |
| `drops` | – | cumulative logger-ring drops at push time |

Sequencing notes:

- The global `seq` (inside the IMU sample) is the host-side decode
  counter over all three sensors, counted from the post-settle reset.
  At ~300 events/s it is the live IMU rate indicator.
- `*_seq` are the device's per-sensor rolling sequences (mod 256).
  Because a 100 Hz logger samples 100 Hz report streams, a small
  fraction of reports (~1–2%) lands in no record — consecutive records
  occasionally show `*_seq` jumping by 2. This is sampling aliasing,
  **not** data loss: a genuinely missing report would show up as a
  ~20 ms arrival gap, which the capture data rules out (decode age
  stays ≤ ~11 ms). Captured values are exact and individually
  timestamped.

## Capture procedure

1. Pre-flight: `bno_cal --check` (expect exit 0); `bno_orient --check`
   (yaw ≈ 0 at the swing zero — if not, run `bno_orient --persist`);
   confirm no other sh2 consumer is running; `gpioinfo gpiochip0`
   shows 17/27/22 free; glance at wiring and DIP switches.
2. If rig facts are known, set `ARM_LENGTH_M` and `RUN_NOTES` in
   `main.c` and rebuild; otherwise leave the placeholders.
3. `sudo ./bin/sensor_validate -o <run-name>.csv -d <capture seconds>`
4. **Warm-up:** after start, let both sensors warm up for ~10 s before
   releasing the swing. Both sensors are logged into the same CSV from
   the first tick, so no external start alignment is needed — the
   synchronization is inherent (one file, one clock domain), and the
   warm-up rows form the pre-release baseline.
5. Perform the swing/release. Keep captures single-release where
   possible; reversal windows are inherently noisier for the encoder
   (see known behaviors).
6. Stop via `-d` expiry or Ctrl+C. Check the summary: `drops=0`,
   `invalid=0`, `mask=0x07`.
7. Post-run: `sudo chown` the CSV if run under sudo; spot-check the
   footer (`records`, `dropped`, `duration_sec`).
8. Log the run in the evidence summary (file name, date, calibration
   state, rig notes, anything unusual).

## Runtime sanity checks

These are operator checks that a capture is not broken — **not** the
formal acceptance criteria, which are applied post-hoc by the analysis
program:

- `drops=0` throughout; `inv=0` (zero illegal quadrature transitions)
- `mask=0x07` from the first status line; `imu/s ≈ 300` steady-state
- `ev/s` tracks shaft motion and falls to ~0 (plus dither) at rest
- `rv_accuracy` small and stable — never π (π means recalibrate)
- Inter-record gaps cluster at 10.00 ms:

```sh
awk -F, '!/^#/ && $1+0>0 { if (p) printf "%.2f\n", ($1-p)/1e6; p=$1 }' <file>.csv | sort -n | uniq -c
```

## Known behaviors

- **Boundary dither at rest:** with the shaft released, the encoder
  sits near a state transition and the count toggles ±1 (~120 ev/s of
  net-zero edges; 0.044° amplitude). Harmless for angle comparison;
  avoid raw per-tick velocity differentiation (±1 count over 10 ms is
  ±4.4°/s of noise).
- **First-record seq artifact:** the seq reset applies to events
  decoded after it; the first record can carry a settle-phase seq value.
  Cosmetic, one record at most.
- **Decode latency:** ~2–8 ms per group between device stamp and host
  decode (bounded). It is measured in the data itself
  (`*_host_ts_ns − *_sensor_ts_us`); subtract it in analysis.
- **Stationary gyro recalibration:** SH-2 calibrates the gyro whenever
  the device is stationary, regardless of the all-off dynamic-calibration
  setting — rests before and after a swing may show tiny gyro bias
  adjustments. The RV output is the comparison signal.
- **Index pulse:** `enc_x_pulses` increments once per revolution; use
  index-to-index windows (bring-up tool) for exact per-rev encoder
  checks.

## Evidence convention

Run CSVs and console logs are not committed. Evidence is summarized in
the run/fork reports; the CSV metadata block (date, encoder config,
sensors, arm length, notes) carries the per-capture context.
