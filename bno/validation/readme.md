# validation/ — AMT102 encoder vs BNO085 IMU validation

Tools that check the BNO085's orientation data against an AMT102 rotary
encoder mounted on the same pivot, with the encoder as ground truth. The
main artifact is `bno_validate`: a synchronized capture binary that
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
  saved DCD with all dynamic calibration disabled)
- `../orientation/readme.md` — `bno_orient` swing-axis tare

## Hardware setup

### Encoder (AMT102-V)

- Resolution: 2048 PPR (DIP switches 1–4 all **Off**, factory preset),
  x4 quadrature decode → **8192 counts/rev**, 0.044° per count
- Series 1 kΩ resistors on each signal line into the Pi header:

| Encoder signal | Through | Pi header pin | BCM line |
|---|---|---|---|
| A (quadrature) | 1 kΩ | pin 29 | GPIO 5 |
| B (quadrature) | 1 kΩ | pin 31 | GPIO 6 |
| X (index, 1/rev) | 1 kΩ | pin 33 | GPIO 13 |
| 5 V | — | 5 V rail | — |
| G | — | GND | — |

- The encoder connector pin order is **B, 5V, A, X, G** (T unused) — 5 V
  sits between B and A, which makes a power/ground swap easy to make and
  destructive. Double-check before powering.

### IMU (BNO085)

- Same SPI wiring and HAL as the application (`../app/sh2_hal_rpi.c`);
  see `../readme.md`.

## Tool binaries

Built from the root `bno/` Makefile into `bno/bin/`:

```bash
cd bno
make validate       # builds bin/bno_validate
make bringup        # builds bin/amt102_bringup
make test           # builds and runs unit tests (bin/test_quad_decode)
```

| Binary | Source | Purpose |
|---|---|---|
| `bin/amt102_bringup` | `amt102_bringup.c`, `amt102.c`, `quad_decode.c` | Live terminal diagnostic for encoder lines, direction, and index pulse |
| `bin/bno_validate` | `validate_main.c`, `validate_sensor.c`, `validate_logger.c`, `amt102.c`, `quad_decode.c`, `../app/realtime.c`, `../app/sh2_hal_rpi.c` | Synchronized 100 Hz capture to timestamped CSV |
| `bin/test_quad_decode` | `../tests/test_quad_decode.c`, `quad_decode.c` | Decoder state machine unit test (no hardware required) |

## Capture procedure

1. **Pre-flight:**
   - `bno_cal --check`: expect exit 0.
   - `bno_orient --check`: yaw ~0 at the swing zero; if not, run `bno_orient --persist`.
   - Confirm no other sh2 consumer is running.
2. **Execute validation capture:**
   ```bash
   sudo ./bin/bno_validate -o run1.csv -d 30 -a 0.25 -n "swing release test 1"
   ```
3. **Warm-up:** Let the sensors settle for ~500 ms to establish baseline before releasing the swing.
4. **Release & log:** Perform swing cycles; finish capture with Ctrl-C or duration expiry.

## Source structure

```text
validation/
├── amt102.c/.h          # AMT102-V incremental encoder driver (libgpiod)
├── amt102_bringup.c     # Standalone encoder bring-up diagnostic
├── quad_decode.c/.h     # Pure x4 quadrature state machine
├── validate_contract.h  # ImuValidateSample_t data contract
├── validate_logger.c/.h # Lock-free ring CSV logger thread
├── validate_sensor.c/.h # 100 Hz SH-2 session owner
├── validate_main.c      # bno_validate main capture harness
└── readme.md            # Validation documentation
```
