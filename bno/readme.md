# BNO085 IMU Acquisition — Raspberry Pi 4B (Mantatow-Thesis)

Production IMU acquisition code for the thesis data-collection system. It reads
three fused/calibrated outputs from an Adafruit BNO085 9-DOF IMU over SPI
(CEVA SHTP/SH-2 protocol) at a sustained **100 Hz per output (~300 events/s
combined)**, driven by a single-threaded real-time 1 kHz service loop.

The sensor-facing layers (HAL, `sh2/`, `sensor_reader`) are validated and
intended to remain stable. `app/main.c` is the designated integration point —
its current console printing is a placeholder until real data consumers
(pressure sensing, video tracking, networking) are connected.

## What this code does

- Enables three BNO085 reports at 100 Hz each, with batching disabled:
  - **Rotation vector** (9-axis fused orientation) → yaw/pitch/roll + accuracy
  - **Linear acceleration** (gravity removed) → ax/ay/az
  - **Calibrated gyroscope** → gx/gy/gz
- Services the BNO085 from a 1 kHz SCHED_FIFO loop so its active-low interrupt
  (`H_INTN`) is always answered within ~1 ms.
- Runs a 300 ms service-only spin-up, then a fixed 10 s acquisition window,
  printing the latest combined sample at 100 Hz to stdout.

Measured performance (final validation):

- ~100.3 Hz per sensor, ~301 events/s combined while producing
- Zero dropped reports, zero decode errors, zero unexpected packets
  (packet integrity verified over 32,768 events)
- Data-phase SPI reads ~190–250 µs; empty polls ~3–5 µs; loop measured at
  exactly 1000.0 Hz under SCHED_FIFO

## Hardware wiring

Adafruit BNO085 breakout → Raspberry Pi 4B 40-pin header:

| Breakout pin | BNO085 signal | Pi GPIO (BCM) | Physical pin | Notes |
|---|---|---|---|---|
| VIN | power | 3V3 rail | 1 | 3.3 V logic; see Adafruit guide for VIN options |
| GND | ground | any GND | 6 | common ground |
| SCL | HSCL / SCK | GPIO 11 (SPI0 SCLK) | 23 | SPI clock |
| SDA | HSDA / MISO | GPIO 9 (SPI0 MISO) | 21 | data out of the BNO085 |
| DI | HMOSI | GPIO 10 (SPI0 MOSI) | 19 | data into the BNO085 |
| CS | HCSN | **GPIO 25** | **22** | manual chip select — NOT CE0/GPIO 8 |
| INT | H_INTN | GPIO 6 | 31 | active-low data-ready; required for stable SPI |
| RST | NRST | GPIO 13 | 33 | active-low reset |
| P0 (PS0/WAKE) | WAKE | GPIO 5 | 29 | protocol select + SPI wake handshake |
| P1 (PS1) | — | 3V3 (tied high) | 1 | PS1=1 and PS0=1 selects SPI host mode |

Wiring notes:

- PS1 must be high before and during reset to select the SPI interface. The
  HAL drives PS0/WAKE high itself before pulsing reset, so both are high when
  the BNO085 samples its protocol pins.
- INT is **not** a kernel interrupt here — the HAL polls the line each service
  iteration. Do not repurpose GPIO 6.
- If you ever move CS to a different pin, update `GPIO_LINE_CS` in
  `app/sh2_hal_rpi.c` to match the wire. A mismatched CS pin is the most
  common "suddenly broken" failure after rewiring.

Host interface requirements:

- SPI enabled on the Pi (`raspi-config` → Interface Options → SPI), device
  `/dev/spidev0.0`.
- SPI mode 3 (CPOL=1, CPHA=1), 8 bits/word, **3 MHz** — the BNO085 datasheet
  maximum. Do not raise it.
- `SPI_NO_CS` is set and CS is driven manually via libgpiod. Reason: spidev
  does not release CE0 (GPIO 8) from the SPI controller even with `SPI_NO_CS`,
  so libgpiod cannot claim GPIO 8; a free GPIO avoids the conflict.
- **libgpiod v1** API (`libgpiod-dev` 1.6.x on Raspberry Pi OS Bullseye /
  Bookworm). The libgpiod v2 API is incompatible with this HAL and would
  require a port.

## Directory layout

```text
bno/
├── Makefile                 builds bin/bno_app and the test binaries
├── app/                     production application + Raspberry Pi HAL
│   ├── imu_sample.h         ImuSample_t — the shared data contract
│   ├── main.c               application loop (integration point)
│   ├── realtime.c/.h        SCHED_FIFO + mlockall, absolute CLOCK_MONOTONIC sleep
│   ├── sensor_reader.c/.h   SH-2 session owner, decodes into latest ImuSample_t
│   └── sh2_hal_rpi.c        Raspberry Pi sh2_Hal_t transport (SPI + libgpiod)
├── calibration/             BNO085 calibration tools (see calibration/readme.md)
├── validation/              encoder-vs-IMU validation harnesses (see validation/readme.md)
├── sh2/                     vendored CEVA sh2 library (submodule, unmodified)
├── tests/                   bring-up and timing test programs
├── bin/                     (generated) binaries
└── build/                   (generated) objects
```

## Software architecture

Layered, strictly single-threaded (no mutexes anywhere — one loop owns all
state):

```text
main.c (1 kHz loop, 300 ms settle, 100 Hz consumer)
  └─ sensor_reader (opens session, enables 3 reports, decodes events)
       └─ sh2/ — CEVA SH-2 library: SHTP framing, fragmentation, channel
         sequence numbers, report decode, FRS, sensor configuration
            └─ sh2_hal_rpi.c — sh2_Hal_t transport: spidev + libgpiod,
               manual CS, two-phase reads, reset/wake sequencing
                 └─ /dev/spidev0.0 + gpiochip0 → BNO085
```

### The HAL (`app/sh2_hal_rpi.c`)

Implements the four-callback `sh2_Hal_t` interface (open / close / read /
write / getTimeUs):

- **Non-blocking read:** if `H_INTN` is not asserted, returns 0 immediately.
- **Two-phase, length-driven data read** (CS held low across both phases):
  transfer the 4-byte SHTP header, decode the little-endian packet length
  (bit 15 masked), then transfer exactly the remaining payload bytes. Returns
  the header-derived length — never a fixed buffer size.
- **Writes** wrap the whole outgoing SHTP packet in manual CS. If the device
  may be asleep, the HAL first performs the datasheet WAKE handshake (drive
  PS0 low → wait for `H_INTN` → release). A failed wake returns 0 so the sh2
  library retries — transient 0-return writes are normal, not errors.
- **Reset sequence:** RST low 10 ms, release, wait 120 ms (datasheet: ~90 ms
  internal init + ~4 ms config). The SHTP advertisement and startup messages
  are deliberately drained by `sh2_open`, not by the HAL.
- All timestamps are CLOCK_MONOTONIC microseconds.

Historical note (why two-phase reads exist): `shtp_service()` always offers a
1024-byte buffer, and the original HAL transferred all 1024 bytes on every
poll. At 3 MHz (~6.2 µs/byte) that is ~6.3 ms per data read, which capped the
combined stream around 90–125 Hz. Real packets are only 19–23 bytes; reading
just the header-declared length cut data reads to ~190–250 µs. Wire lengths
always come from the SHTP header, never from CEVA's `sh2ReportLens[]` tables
(decoded event lengths are 14/10/10 while wire packets are 23/19/19 bytes).

### Real-time support (`app/realtime.c`)

`StartRT(priority, dt)` does `mlockall` + SCHED_FIFO (priority 90 by default);
failure is non-fatal (warning + default scheduling). `RT_SleepUntil(dt)`
maintains a persistent absolute-advancing deadline on CLOCK_MONOTONIC with a
rounded nanosecond advance. CLOCK_MONOTONIC is deliberate: the Pi has no
battery RTC, and CLOCK_REALTIME can be stepped or slewed by NTP mid-run.

### Timing requirements

The BNO085 datasheet asks the host to answer an asserted `H_INTN` within about
one-tenth of the fastest sensor period (~1 ms here); the device times out and
retries after ~10 ms, and frequent delays cause internal processing starvation
and erroneous outputs. The 1 kHz loop gives ~3.3× headroom over 301
packets/s. **Any future consumer that adds work to the loop must preserve the
~1 ms service cadence** (call `sensor_reader_service()` every iteration).

## Data contract for consumers

`ImuSample_t` (`app/imu_sample.h`) is the structure everything reads:

| Field | Units | Meaning |
|---|---|---|
| `version` | — | struct version (`IMU_SAMPLE_STRUCT_VERSION` = 3) |
| `seq` | — | +1 per decoded event (any of the three sensors), since start |
| `timestamp_uS` | µs | device-side timestamp of the most recent event |
| `yaw`, `pitch`, `roll` | rad | orientation from the rotation-vector quaternion |
| `orientationErrRad` | rad | rotation-vector heading-error estimate (lower is better; NOT the 0–3 status scale) |
| `ax`, `ay`, `az` | m/s² | linear acceleration (gravity removed) |
| `gx`, `gy`, `gz` | rad/s | calibrated angular velocity |
| `validMask` | bit flags | bit 0 orientation, bit 1 accel, bit 2 gyro |

Read these semantics before consuming:

1. **It is a latest-value mailbox, not a queue.** `sensor_reader_getLatestSample()`
   returns the newest combined state. Polling faster than events arrive
   returns the same `seq` — a repeated `seq` means "no new event since the
   last read," not a new sample.
2. **Field groups update independently.** yaw/pitch/roll, ax/ay/az, and
   gx/gy/gz are each refreshed by their own sensor's events, so one sample can
   mix values from up to three events ~10 ms apart. `seq` and `timestamp_uS`
   always reflect the single most recent event, whichever sensor produced it.
   If you need per-sensor alignment, extend `sensor_reader` (per-field
   timestamps or a raw callback) rather than assuming alignment.
3. **`validMask` accumulates** (`|=`): it reports which outputs have *ever*
   been received, not which fields changed in the last event.
4. **`seq` counts from `sensor_reader_start()`**, including the 300 ms settle
   phase. Expect ~3000 increments per 10 s window plus ~50–60 settle-phase
   events (~301 events/s once reporting starts; first reports arrive
   ~130–137 ms after the enable commands).
5. **`timestamp_uS` is measured by the BNO085's own hub timer.** It is
   generally monotonic but can occasionally step backwards a few microseconds
   (known SH-2/SHTP artifact). Guard any dt math against `dt <= 0`. For
   wall-clock timing on the host, use CLOCK_MONOTONIC only.
6. **Rates:** configured at 100 Hz per sensor; the device actually delivers
   ~100.3 Hz (within CEVA's configured-rate tolerance). BNO085 metadata
   claims a 1 kHz minimum period per sensor; measured single-sensor ceilings
   were ~331–390 Hz. Operation is validated at 100 Hz × 3.
7. **Report identity (for debugging):** rotation vector = report ID 0x05,
   linear acceleration = 0x04, calibrated gyroscope = 0x02, all on SHTP
   channel 3. Wire packets are 23 B (rotation vector) and 19 B
   (accelerometer/gyro) including the 4-byte SHTP header, one event per
   packet.
8. **No status bits in the sample — and the gyro bit would read 0
   anyway.** `ImuSample_t` deliberately carries no per-report status
   bytes. Under the flight policy (all dynamic calibration off) the
   BNO085 reports the gyro status bit as 0 (unreliable) by design — the
   real-time ZRO estimator is halted while the saved DCD keeps
   bias-correcting the data — so readiness must be judged from
   `orientationErrRad` (<= ~0.35 rad once converged), never from a
   gyro status bit.

## Building and running

On the Pi (Raspberry Pi OS; requires `gcc`, `make`, `libgpiod-dev` v1):

```bash
sudo apt install libgpiod-dev     # if not already present
cd "rpi4b prod code/bno"
cd bno
make                       # builds bin/bno_app
make tests                 # builds the five test binaries
sudo ./bin/bno_app
cd calibration && make     # builds bin/bno_cal and bin/bno_cal_clear
```

`sudo` is required (spidev + gpiochip0 access and SCHED_FIFO rtprio limits).
Expected output format, from an actual 10 s run:

```text
main: running IMU reader for 10 seconds...
seq=3041 ts=305857144 yaw= -2.998 pitch=  0.078 roll= -0.099 ax= -0.004 ay=  0.000 az= -0.039 gx= -0.018 gy= -0.010 gz= -0.012
...
main: finished. Printed 1000 lines.
```

Quick post-build validation:

- `sudo ./bin/test_report_len` — wire-length integrity: rotation-vector
  packets 23 B, accelerometer/gyro 19 B, 1:1 events-to-packets, zero
  unexpected reports.
- Run `bno_app` and hand-rotate the IMU: gx/gy/gz must change with motion and
  settle near zero when stationary (small residual offsets are normal — the
  gyroscope's zero-rate offset is dynamically calibrated).

## Tests

| Target | Purpose |
|---|---|
| `test_hal_raw` | Raw HAL bring-up without sh2 (SPI, GPIO, reset, wake) |
| `test_sh2_open` | HAL + sh2 session open smoke test (advertisement drain) |
| `test_single_sensor` | Enable one sensor and verify report flow |
| `test_min_period` | Query sensor metadata (FRS) for minimum report periods |
| `test_report_len` | Wire packet-length verification — the key integrity test |

There were also other debug tooling now retired and removed from the 100 Hz rate
investigation (`test_hal_debug`, `test_timing_rv/accel/gyro`). They depended
on a HAL timing-accessor API that was removed in the production cleanup and
will not build as-is; the complete instrumented state they were written
against is preserved at commit `56ba70b`.

## Vendored CEVA SH-2 library

`sh2/` is cloned directly from CEVA's SH-2 reference source (the library tree
distributed with CEVA's `sh2-demo-nucleo` / `bno080-nucleo-demo` examples) and
is intentionally **unmodified**. It owns all protocol logic: SHTP framing,
fragmentation, channel sequence numbers, report decoding, FRS access, and
sensor configuration. This project supplies only the transport
(`sh2_hal_rpi.c` implementing `sh2_Hal_t`) and the consumer logic.

Do not hand-patch `sh2/`. If behavior looks wrong, suspect the HAL or app
layers first, and re-run `test_report_len` before and after any change. If
CEVA publishes updated sources, replace the directory wholesale.

## Calibration

The BNO085's internal calibration is managed by the tools in
`calibration/` (see `calibration/readme.md` for full details):

- `bno_cal` — guided dynamic calibration per CEVA BNO08X Sensor Calibration
  Procedure, plus `--check` (field go/no-go) and `--check --mask` (cal-config
  probes).
- `bno_cal_clear` — full DCD erase (flash + RAM) for documenting the
  uncalibrated baseline.

Field procedure: run `bno_cal --check` (expect exit 0) before every
deployment; recalibrate with `bno_cal` in the deployment area when the
magnetic environment changes. Both `bno_app` and `sensor_validate`
disable all dynamic calibration at startup and fly on the saved DCD —
the enable bits are RAM-only and revert at every reset, so the policy
is set per program, not per calibration.

Note: under the all-off flight policy the BNO085 reports the gyro
status bit as 0 (unreliable) by design (the ZRO estimator is halted;
the saved DCD still bias-corrects the data), and the rotation vector
needs ~10 s of motion after a reset to converge. Neither is a fault:
`bno_cal --check` accounts for both, and consumers gate readiness on
`orientationErrRad`, not status bits.

## Provenance

- HAL rewrite (manual CS + two-phase length-driven reads): PR #3
  (commit `1e60882`, merge `f86e26c`).
- CS pin reassignment (GPIO 8 → 25): commit `7fcb9e1`.
- Final instrumented debug state (rate investigation, timing CSVs): commit
  `56ba70b` — reference point for the archived tests and the validation
  numbers above.
- Production cleanup: debug instrumentation stripped from the HAL, sensor
  reader, and main; debug tests archived; 300 ms settle phase added.

## Constraints and gotchas

- The BNO085 CS wire must physically match `GPIO_LINE_CS` (GPIO 25, physical
  pin 22).
- SPI stays at 3 MHz — datasheet maximum.
- Preserve the ~1 ms service cadence in any consumer loop.
- One HAL instance per process (static instance); one BNO085 per SPI bus.
- Batching is disabled (`batchInterval_us = 0`): every event is its own SHTP
  packet.
- The BNO085 boots with all sensors disabled — nothing streams until
  `sensor_reader_start()` enables the three reports.
- Failed host-initiated writes (asleep device, wake timeout) return 0 and are
  retried by the sh2 library; they are normal.
- With all dynamic calibration off (the flight policy), the gyro status
  bit reads 0 by design; gate readiness on `orientationErrRad`, never on
  gyro status.

## References

- CEVA BNO08X Datasheet, doc 1000-3927 (v1.17)
- CEVA SH-2 Reference Manual, doc 1000-3625 (v1.9)
- CEVA SH-2 SHTP Reference Manual, doc 1000-3600 (v1.6)
- CEVA `sh2-demo-nucleo` / `bno080-nucleo-demo` reference sources (origin of
  `sh2/`)
- Adafruit BNO085 breakout guide (learn.adafruit.com)
- Raspberry Pi GPIO / spidev documentation
