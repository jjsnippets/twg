# BNO085 Calibration — Raspberry Pi 4B (twg)

Standalone tools for the BNO085's **internal dynamic calibration** (the
DCD: accelerometer zero-g offset, gyroscope zero-rate offset and the
magnetometer's hard/soft-iron fit). This directory is deliberately
separate from `validation/`: the encoder is a *reference instrument*
for judging IMU output afterwards; it is never an input to the BNO085's
own calibration. The only thing that crosses from here to `bno_app` is
the DCD saved in the BNO085's flash.

This directory also holds `bno_orient`, the **frame-alignment (tare)**
tool. Dynamic calibration fixes the sensor's internal physics; tare
rotates the fused output frame so its zero matches the fixture's swing
axis. The two are independent — the tare is stored in the BNO085's
separate sensor-orientation flash record, so taring never touches a
calibration and calibrating never disturbs a tare.

The calibration procedure implemented here is CEVA's "BNO08X Sensor
Calibration Procedure" (doc 1000-4044), driven through the vendored
SH-2 library's dynamic-calibration API (`sh2_setCalConfig`,
`sh2_saveDcdNow`, `sh2_clearDcdAndReset`, `sh2_setFrs`).

## Binaries

| Binary | Purpose |
|---|---|
| `bin/bno_cal` | Guided calibration, field go/no-go check, cal-config probes |
| `bin/bno_cal_clear` | Full DCD erase (flash + RAM) with confirmation prompt |
| `bin/bno_orient` | Swing-axis tare; `--persist` stores the frame in flash |

Build:

```sh
cd bno/calibration
make          # builds bin/bno_cal, bin/bno_cal_clear and bin/bno_orient
make clear    # builds only bin/bno_cal_clear
make orient   # builds only bin/bno_orient
```

`bno_cal` and `bno_cal_clear` link the shared session owner
`sensor_calibrate.c`; `bno_orient` uses its own (`sensor_orient.c`,
which decodes the full rotation-vector quaternion — `CalSample_t`
carries accuracy bits only). All link the production decode chain
(`../sh2` and the Pi HAL in `../app`). None may run at the same time
as `bno_app` or the validation binaries — one SPI HAL instance per
process.

## bno_cal — guided calibration

```sh
sudo ./bin/bno_cal              # full guided run; logs to
                                # bno_cal_<date>_<time>.csv in the cwd
sudo ./bin/bno_cal --check      # read-only go/no-go (field use)
sudo ./bin/bno_cal --check --mask 0xNN   # probe a different cal mask
```

The guided flow:

1. Enables ME calibration for accel + gyro + mag (bitwise OR of the
   `SH2_CAL_*` bits; the gyro flag is required for hand-held
   calibration per 1000-4044).
2. **Accelerometer**: six resting orientations, ~2 s each.
3. **Gyroscope**: device stationary on a surface (informational only).
4. **Magnetometer**: full 180°-and-back swing patterns about each axis
   (roll, pitch, yaw), minimum 25 s per round, until the Magnetic Field
   status bit reads 2 or 3 — the procedure's official progress metric —
   sustained for 3 s.
5. **Save**: ~10 s stationary hold (the hub snapshots the DCD to RAM
   every 5 s; Save DCD persists the last snapshot), a pre-save gate
   that refuses to save a degraded state, then `sh2_saveDcdNow()` to
   flash (FRS record 0x1F1F).
6. **Verify**: the session is reopened (the HAL open toggles RST, so
   the chip reboots and reloads the DCD from flash), all dynamic
   calibration is disabled exactly like `bno_app`, a ~10 s motion
   window runs (the rotation vector needs ~10 s of motion to converge
   after a reset), and then the accel + mag accuracies must hold at
   ≥ 2 for 3 s.

The old DCD is never cleared during calibration: Save DCD overwrites
the flash record wholesale, so recalibrating is always safe.

### `--check` (field go/no-go)

Opens a session, applies `bno_app`'s flight policy (all dynamic
calibration off), prints the ME cal config plus the accuracy bits after
~10 s, and exits 0 (READY) or 3 (NOT CALIBRATED). Run this before every
deployment.

Pass criteria: accelerometer >= 2, magnetometer >= 2, and rotation
vector status >= 2 with a heading error estimate <= 0.35 rad (~20 deg).
The gyro bit is not part of the verdict — it reads 0 whenever the gyro
cal flag is off (see Known behaviors). Expected output on a calibrated
unit:

```text
[acc 2  gyr 0  mag 2  rv 2]  rv_err= 0.14 rad
RESULT: READY - saved calibration looks good (exit 0)
```

### `--check --mask 0xNN` (probe mode)

Same as `--check` but with an arbitrary ME cal mask — e.g. `0x05`
(accel+mag, the chip's default) or `0x02` (gyro only). Informational
only, always exit 0. Used to isolate which cal flags affect which
status bits.

### Exit codes

| Code | Meaning |
|---|---|
| 0 | success (calibrated, saved, verified) / `--check`: READY or probe complete |
| 1 | runtime error (SPI/SH-2 failure, DCD save failed) |
| 2 | aborted by the user (`q` at a prompt or Ctrl-C) |
| 3 | not calibrated (`--check` verdict) or verification failed |

### CSV log

One row per decoded sensor event:

```text
host_us,device_us,phase,seq,accel_acc,gyro_acc,mag_acc,rv_acc,
rv_err_rad,mag_ut_x,mag_ut_y,mag_ut_z
```

The `*_acc` columns are the per-report status bits (0–3). `rv_err_rad`
is the rotation vector's heading-error estimate in radians — a
different quantity, not a 0–3 scale.

## bno_cal_clear — full DCD erase

Erases **all** dynamic calibration data from **both flash and RAM**, so
the sensor returns to a genuinely uncalibrated state (used to document
the uncalibrated baseline and what calibration actually changes).
Requires typing `CLEAR` to proceed; anything else aborts with nothing
erased.

It implements the SH-2 Reference Manual §6.4.9 recommended sequence:
(1) hub reset (the session open), (2) delete the flash DCD record via
`sh2_setFrs(0x1F1F, …, 0)` ("0 to delete record"), (3) Clear DCD and
Reset, which atomically clears the RAM DCD and resets so the old state
cannot re-persist to flash. Steps 2 and 3 are both required — the
clear command alone only clears RAM, and the flash copy would reload
at the next boot. If the flash delete fails, the tool aborts *before*
the RAM clear, leaving the existing calibration intact.

Prints the accuracy state before and after (~5 s each) for
documentation. Exit codes: 0 cleared, 1 error, 2 declined/aborted.

## bno_orient — swing-axis tare (frame alignment)

```sh
sudo ./bin/bno_orient              # Z-axis (heading) tare at the fixture's
                                   # swing-axis zero; volatile
sudo ./bin/bno_orient --persist    # tare, write to flash, verify across a
                                   # session reopen
sudo ./bin/bno_orient --all        # full 3-axis tare (level + mag North!)
sudo ./bin/bno_orient --clear      # drop the volatile tare
sudo ./bin/bno_orient --check      # read-only: heading + RV accuracy
```

Rotates the fused output frame so that the attitude held at tare time
reads as zero — aligning IMU yaw zero with the rig's swing-axis zero,
which makes the encoder-vs-IMU angle comparison direct (no post-hoc
θ₀ offset in analysis). Both the live monitoring display and the static
attitude checkpoints print the raw rotation-vector unit quaternion
`[quatW, quatX, quatY, quatZ]` alongside the derived Euler angles.

Semantics (CEVA "BNO085 Tare Function" note; SH-2 Reference Manual,
Tare command):

- **Tare Now** (`sh2_setTareNow`) is volatile: the next session open
  performs the HAL reset, so the chip reboots and the tare is lost.
  `--persist` (`sh2_persistTare`) writes it to the separate
  sensor-orientation flash record, so every later session — `bno_app`
  and `sensor_validate` included — boots in the tared frame.
- **The default is the Z-axis (heading) tare**: the fixture just needs
  to sit level at its swing-axis zero; no magnetic-North alignment is
  required. `--all` additionally zeroes pitch and roll and requires
  the device dead level **and** pointed at magnetic North when the
  tare is applied (confirmed when `quatW ≈ 1.0` and `quatX ≈ quatY ≈ quatZ ≈ 0.0`).
- **`--clear` drops only the volatile tare**: a tare previously
  persisted to flash reloads at the next reset and must be overwritten
  by a fresh tare + `--persist`.

Flow: opens the session under `bno_app`'s flight policy (all dynamic
calibration off), settles the rotation vector (accuracy ≥ 2, up to
15 s — aborts with a "run bno_cal first" hint otherwise), prints the
current quaternion and yaw/pitch/roll, prompts at the swing-axis zero, tares, and
with `--persist` reopens the session to verify the tare survived the
reset (the same reopen-verify pattern as `bno_cal`'s step 6).

Interactions with the other tools:

- `bno_cal` and `bno_cal_clear` never touch the tare, and `bno_orient`
  never touches the DCD — recalibrating, clearing calibration and
  taring are all independent operations.
- Re-run `bno_orient --persist` whenever the IMU is remounted or the
  fixture's mechanical zero changes.

Exit codes: 0 success (verified across reset with `--persist`),
1 runtime error (SPI/SH-2 failure, heading never settled), 2 aborted
by the user.

## Flight-time calibration policy (bno_app and sensor_validate)

Both acquisition programs disable all dynamic calibration
(`sh2_setCalConfig(0)`) at session start and fly on the saved DCD only.
The enable bits are RAM-only state that revert to chip defaults at
every reset, so every SH-2 consumer must set its own policy — `bno_cal`
cannot set it on anyone's behalf. (The gyro is still bias-corrected
when the device is very stable, per the BNO08X datasheet §3.1.3.)

## Known behaviors (measured on this unit; gyro bit + DCD confirmed 2026-09-07)

- **Gyro status bit reads 0 whenever the gyro cal flag is off**, and 3
  whenever it is on, regardless of the DCD. Confirmed 2026-09-07
  across a guided calibration, `--check` probes of masks
  0x00/0x01/0x02/0x07, and a full Pi power-down/unplug/reboot cycle:
  the bit tracks the runtime flag exactly. It is therefore never gated
  on, only displayed.
- **DCD persistence is verified**: after the 2026-09-07 calibration
  the device survived shutdown, unplugging and reboot, and the
  rotation vector came back converged (status 2–3, error ~0.1–0.2 rad)
  under the all-off flight policy.
- **RV needs a motion window after a reset**: the earlier observation
  of RV status 0 pinned at ~1.45 rad while stationary under the
  all-off policy (2026-09-04) was a warm-up artifact — after ~10 s of
  gentle motion the RV reports status 2–3 with a ~0.1–0.2 rad error
  estimate even with all dynamic calibration off. The verify step's
  motion window exists for this reason.
- **The mag status bit fluctuates at rest** (0–3 within seconds) and
  the DCD snapshot is taken every 5 s, which is why the save is gated
  on a sustained-good state rather than a single sample.

## Report set

`sensor_calibrate.c` subscribes: Magnetic Field Calibrated at 50 Hz
(the rate 1000-4044 requires for magnetometer work) plus Accelerometer,
Gyroscope Calibrated and Rotation Vector at 10 Hz for their status
bits — ~80 events/s, serviced from a plain ~1 kHz `usleep` loop (no
SCHED_FIFO needed for an operator-paced tool).

`sensor_orient.c` (`bno_orient`) subscribes only to the Rotation
Vector at 20 Hz, decoded in full (quaternion + accuracy).
