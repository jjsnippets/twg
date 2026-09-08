# bno/calibration/ — BNO085 Dynamic Calibration Tools

Standalone tools for the BNO085's internal dynamic calibration — the DCD
(accelerometer zero-g offset, gyroscope zero-rate offset, and the
magnetometer's hard/soft-iron fit).

This directory is deliberately separate from `validation/`: the encoder is a
reference instrument for judging IMU output afterwards; it is never an input
to the BNO085's own calibration. The only thing that crosses from here to
`bno_app` is the DCD saved in the BNO085's flash.

Frame alignment and swing-axis taring tools live in `bno/orientation/`
(see `orientation/readme.md`). Dynamic calibration fixes the sensor's
internal physics; tare rotates the fused output frame so its zero matches
the fixture's swing axis. The two are independent: the tare is stored in the
BNO085's separate sensor-orientation flash record, so taring never touches a
calibration, and calibrating never disturbs a tare.

The calibration procedure implemented here is CEVA's **"BNO08X Sensor
Calibration Procedure" (doc 1000-4044)**, driven through the vendored SH-2
library's dynamic-calibration API (`sh2_setCalConfig`, `sh2_saveDcdNow`,
`sh2_clearDcdAndReset`, `sh2_setFrs`).

---

## Binaries

All binaries are built from the root `bno/` Makefile into `bno/bin/`:

| Binary | Purpose |
|---|---|
| `bin/bno_cal` | Guided calibration, field go/no-go check, cal-config probes |
| `bin/bno_cal --clear` | Full DCD erase (flash + RAM) with confirmation prompt |

### Build
From the `bno/` directory:

```bash
cd bno
make cal            # builds bin/bno_cal
# or
make tools          # builds all production & field tools
```

`bno_cal` links the session owner `cal_sensor.c` along with the vendored
`sh2/` library and the Pi HAL in `app/sh2_hal_rpi.c`.

**Safety Rule:** Only **one SPI HAL instance** may run per process. `bno_cal`
must not run at the same time as `bno_app`, `bno_orient`, `bno_validate`, or
the test binaries.

---

## Command Usage

```bash
# 1. Full guided run (logs accuracy trace to bno_cal_<date>_<time>.csv in cwd)
sudo ./bin/bno_cal

# 2. Field go/no-go check (read-only inspection under bno_app's all-off policy)
sudo ./bin/bno_cal --check

# 3. Cal-config probe (inspect behavior under custom ME calibration mask)
sudo ./bin/bno_cal --check --mask 0xNN

# 4. Erase all dynamic calibration from flash and RAM
sudo ./bin/bno_cal --clear
```

---

## Workflows

### Guided Calibration (`bno_cal`)

1. **Enable ME calibration**: accel + gyro + mag (bitwise OR of the
   `SH2_CAL_*` bits). The gyro flag is required for hand-held calibration
   per 1000-4044.
2. **Accelerometer**: six resting orientations, ~2 s each.
3. **Gyroscope**: device stationary on a surface (~2-3 s; informational).
4. **Magnetometer**: full 180° swing patterns about each axis (roll,
   pitch, yaw), minimum 25 s per round, until the Magnetic Field status bit
   reads 2 or 3 (sustained for 3 s).
5. **Save**: 10 s stationary hold (hub snapshots DCD to RAM every 5 s),
   pre-save gate refusing degraded states, then `sh2_saveDcdNow()` to flash
   FRS record `0x1F1F`.
6. **Verify**: session reopened (chip reset, DCD reload from flash), all
   dynamic calibration disabled (mirroring `bno_app`), 10 s motion window,
   then accel + mag accuracies must sustain >= 2 for 3 s.

Recalibration is always safe without clearing first: `Save DCD` overwrites
the flash record wholesale.

### Full DCD Erase (`bno_cal --clear`)

Erases all dynamic calibration data from both flash and RAM, returning the
sensor to a genuinely uncalibrated state.

Implements the SH-2 Reference Manual 6.4.9 sequence:
1. Session open (HAL reset).
2. Delete flash DCD record via `sh2_setFrs(0x1F1F, ..., 0)`.
3. `sh2_clearDcdAndReset()` to atomically clear RAM DCD and reboot the chip.

Interactive prompt:
```text
permanently erase calibration and reset [y/N]: 
```
Pressing `y` or `Y` proceeds with the erase. Anything else aborts without
erasing.

---

## Exit Codes

| Code | Meaning |
|:---:|:---|
| `0` | Success (calibrated, saved, verified; DCD erased; or `--check` READY / probe) |
| `1` | Runtime error (SPI / SH-2 failure, DCD save failed) |
| `2` | Aborted by user (`n`, `q`, or `Ctrl-C`) |
| `3` | Not calibrated (`--check` verdict or post-calibration verification failed) |

---

## Source Structure

```text
calibration/
├── cal_contract.h   # CalSample_t — per-sensor accuracy bits and calibrated B-field
├── cal_sensor.c     # SH-2 session owner; 50 Hz mag, 10 Hz accel/gyro/RV
├── cal_sensor.h     # API for session start/service/stop
├── cal_main.c       # CLI driver for guided calibration, --check, and --clear
└── readme.md        # This documentation
```
