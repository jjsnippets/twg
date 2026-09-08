# bno/orientation/ — BNO085 Swing-Axis Orientation (Tare) Tools

Aligns the BNO085 fused output frame to the mechanical fixture's swing axis using the SH-2 tare commands (`sh2_setTareNow`, `sh2_persistTare`, `sh2_clearTare`).

---

## Background & Architecture

Dynamic calibration (`bno_cal`) corrects the sensor's internal MEMS physics (accelerometer zero-g offset, gyro zero-rate offset, magnetometer hard/soft-iron matrix) and stores it in the DCD FRS record (`0x1F1F`).

**Tare is completely independent of dynamic calibration:**
- Tare rotates the coordinate frame of the fused rotation-vector output so that the orientation held at tare time reads as $(0, 0, 0)$ deg.
- The tare transformation is saved in the BNO085's separate **sensor-orientation flash record**.
- Recalibrating with `bno_cal` never touches the tare, and taring with `bno_orient` never alters or erases the DCD calibration.
- Re-run `bno_orient --persist` whenever the IMU is physically remounted or the fixture's mechanical zero changes.

---

## Tool Binaries

All tools are compiled from the root `bno/` Makefile into `bno/bin/`:

```bash
cd bno
make orientation        # builds bin/bno_orient
```

### Safety Rule
Only **one SPI HAL instance** can run per process. `bno_orient` must not run concurrently with `bno_app`, `bno_cal`, `bno_validate`, or any hardware test binaries.

---

## Command Usage

```bash
# 1. Z-axis heading tare at fixture swing-axis zero (volatile in RAM only)
sudo ./bin/bno_orient

# 2. Tare and persist to flash FRS record (reloads across reboots)
sudo ./bin/bno_orient --persist

# 3. Full 3-axis tare (requires device dead level and facing magnetic North!)
sudo ./bin/bno_orient --all
sudo ./bin/bno_orient --all --persist

# 4. Drop volatile tare and revert to raw frame
sudo ./bin/bno_orient --clear

# 5. Read-only inspection (prints current quaternion, Euler angles, and RV accuracy)
sudo ./bin/bno_orient --check
```

---

## Confirmation & Prompts

For safety and repeatability during bench setup, interactive operations prompt for single-key confirmation:
```text
move the fixture to its swing-axis ZERO, keep it stationary, then tare [y/N]: 
```
Pressing `y` or `Y` proceeds with the operation. Pressing `n`, `N`, `q`, or `<Enter>` aborts cleanly with exit code `2` without altering sensor state.

---

## Exit Codes

| Code | Meaning |
|:---:|:---|
| `0` | Success (tare applied and verified across reset if `--persist`) |
| `1` | Runtime error (SPI / SH-2 failure, heading never settled) |
| `2` | Aborted by user (`n`, `q`, or `Ctrl-C`) |

---

## Source Structure

```text
orientation/
├── orient_main.c     # CLI driver, tare workflow, and post-tare verification
├── orient_sensor.c   # SH-2 session owner; subscribes to Rotation Vector (20 Hz)
├── orient_sensor.h   # OrientSample_t data struct and session lifecycle API
└── readme.md         # Documentation and operational manual
```
