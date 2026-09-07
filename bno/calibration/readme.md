# BNO085 Calibration — Raspberry Pi 4B (twg)

Standalone tools for the BNO085's **internal dynamic calibration** (the
DCD: accelerometer zero-g offset, gyroscope zero-rate offset and the
magnetometer's hard/soft-iron fit). This directory is deliberately
separate from `validation/`: the encoder is a *reference instrument*
for judging IMU output afterwards; it is never an input to the BNO085's
own calibration. The only thing that crosses from here to `bno_app` is
the DCD saved in the BNO085's flash.

The procedure implemented here is CEVA's "BNO08X Sensor Calibration
Procedure" (doc 1000-4044), driven through the vendored SH-2 library's
dynamic-calibration API (`sh2_setCalConfig`, `sh2_saveDcdNow`,
`sh2_clearDcdAndReset`, `sh2_setFrs`).

## Binaries

| Binary | Purpose |
|---|---|
| `bin/bno_cal` | Guided calibration, field go/no-go check, cal-config probes |
| `bin/bno_cal_clear` | Full DCD erase (flash + RAM) with confirmation prompt |

Build:

```sh
cd bno/calibration
make          # builds bin/bno_cal and bin/bno_cal_clear
make clear    # builds only bin/bno_cal_clear
```

Both link the production decode chain (`../sh2`, the Pi HAL in
`../app`, and the shared session owner `sensor_calibrate.c`). Neither
may run at the same time as `bno_app` or the validation binaries —
one SPI HAL instance per process.

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
