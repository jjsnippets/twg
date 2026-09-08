# bno/validation/ — AMT102 Encoder vs BNO085 IMU Validation

Tools that check the BNO085's orientation data against an AMT102 rotary
encoder mounted on the same pivot, with the encoder as ground truth. The
main artifact is `bno_validate`: a synchronized capture binary that
logs encoder angle and IMU data into one timestamped CSV.

The comparison is angle-vs-angle (graphical comparison of the two
waveforms), so rig geometry does not enter the data path — arm length,
mass, and pivot details are recorded only as run metadata. Analysis and
formal acceptance criteria are applied post-hoc by a separate program;
this directory covers **acquisition only**.

Related documentation:
- `../readme.md` — app architecture, data contract, sudo/RT policy
- `../calibration/readme.md` — `bno_cal` guided calibration, the
  flight-time calibration policy (both acquisition programs fly on the
  saved DCD with all dynamic calibration disabled)
- `../orientation/readme.md` — `bno_orient` swing-axis tare

---

## Hardware Setup

### Encoder (AMT102-V)

- Resolution: 2048 PPR (DIP switches 1–4 all **Off**, factory preset),
  x4 quadrature decode → **8192 counts/rev**, 0.044° per count.
- Series 1 kΩ resistors on each signal line into the Pi header:

| Encoder signal | Through | Pi header pin | BCM line |
|---|---|---|---|
| A (quadrature) | 1 kΩ | pin 11 | GPIO 17 |
| B (quadrature) | 1 kΩ | pin 13 | GPIO 27 |
| X (index, 1/rev) | 1 kΩ | pin 15 | GPIO 22 |
| 5 V | — | pin 2 / 4 (5 V rail) | — |
| G | — | pin 6 (GND) | — |

- The encoder connector pin order is **B, 5V, A, X, G** (T unused) — 5 V
  sits between B and A. Double-check wiring before powering.

### IMU (BNO085)

- Same SPI wiring and HAL as the application (`../app/sh2_hal_rpi.c`);
  see `../readme.md`.

---

## Tool Binaries

All tools are built from the root `bno/` Makefile into `bno/bin/`:

```bash
cd bno
make validate       # builds bin/bno_validate
make bringup        # builds bin/amt102_bringup
make test           # builds and executes decoder unit tests (bin/test_quad_decode)
```

| Binary | Source | Purpose |
|---|---|---|
| `bin/amt102_bringup` | `amt102_bringup.c`, `amt102.c`, `quad_decode.c` | Live terminal diagnostic streaming encoder telemetry every 500 ms |
| `bin/bno_validate` | `validate_main.c`, `validate_sensor.c`, `validate_logger.c`, `amt102.c`, `quad_decode.c`, `../app/realtime.c`, `../app/sh2_hal_rpi.c` | Synchronized 100 Hz capture to timestamped CSV |
| `bin/test_quad_decode` | `../tests/test_quad_decode.c`, `quad_decode.c` | Decoder state machine unit test (no hardware required) |

---

## Tool Details & Telemetry

### 1. Encoder Bring-Up Diagnostic (`amt102_bringup`)

Live diagnostic to verify encoder wiring, line states, count direction, and index pulses. It does **not** require rotating a full 360° to observe counts:

```bash
sudo ./bin/amt102_bringup
```

Streams a persistent live telemetry line every 500 ms:
```text
[t=  0.5s] count=    +120 (  +5.27 deg) | edges: A=120     B=120     | x_pulses=0   | inv=0
[t=  1.0s] count=    +422 ( +18.54 deg) | edges: A=5796    B=5791    | x_pulses=0   | inv=0
[t=  1.5s] count=    +850 ( +37.35 deg) | edges: A=6210    B=6215    | x_pulses=0   | inv=0
```
Upon pressing `Ctrl-C`, it prints the total session summary (net counts, total degrees, revolutions, and validity diagnosis).

### 2. Synchronized Capture (`bno_validate`)

Executes 100 Hz synchronized IMU and encoder data capture:

```bash
# Default capture (logs to bno_validate_YYYYMMDD_HHMMSS.csv)
sudo ./bin/bno_validate

# Custom capture with duration, arm length, and run notes
sudo ./bin/bno_validate -o run1.csv -d 30 -a 0.25 -n "swing release test 1"
```

Features:
- **0.3s Settle Phase:** Rapidly clears startup traffic and synchronizes clock domains before logging begins.
- **Timestamped CSV Output:** Default filename automatically appends the current timestamp (`bno_validate_YYYYMMDD_HHMMSS.csv`).
- **Live 1 Hz Console Telemetry:** Displays elapsed time, encoder position, the full Rotation Vector unit quaternion, and Tait-Bryan Euler angles ($y, p, r$):
```text
[t=  1.0s] enc= +45.20 deg (d= +1028) | q=(+0.924,+0.002,-0.001,+0.382) | ypr=( +45.01,  -0.14,  +0.22) deg | acc=2 err=0.15rad | drops=0
```

---

## Capture Procedure

1. **Pre-flight:**
   - Run `bno_cal --check`: verify exit `0` (calibrated).
   - Run `bno_orient --check`: verify yaw reads ~0 deg at the swing zero; if not, run `bno_orient --persist`.
   - Confirm no other SPI/SH-2 consumer is running.
2. **Execute capture:**
   ```bash
   sudo ./bin/bno_validate -d 30 -a 0.25 -n "pendulum release trial"
   ```
3. **Release & log:** Move or release the swing during the acquisition window. Stop early with `Ctrl-C` if desired.

---

## Source Structure

```text
validation/
├── amt102.c/.h          # AMT102-V incremental encoder driver (libgpiod v1)
├── amt102_bringup.c     # Standalone live encoder diagnostic
├── quad_decode.c/.h     # Pure x4 quadrature state machine
├── validate_contract.h  # ImuValidateSample_t data contract
├── validate_logger.c/.h # Bounded ring buffer CSV logger thread
├── validate_sensor.c/.h # 100 Hz 3-sensor SH-2 session owner
├── validate_main.c      # bno_validate main capture harness
└── readme.md            # Validation documentation
```
