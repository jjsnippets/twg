# BNO085 IMU Acquisition — Raspberry Pi 4B (Mantatow-Thesis)

Production IMU acquisition code for the thesis data-collection system. It reads
three fused/calibrated outputs from an Adafruit BNO085 9-DOF IMU over SPI (CEVA
SHTP/SH-2 protocol) at a sustained **100 Hz per output** (300 events/s
combined), driven by a single-threaded real-time 1 kHz service loop.

The sensor-facing layers (`app/sh2_hal_rpi.c`, `sh2/`, `app/app_sensor.c`) are
validated and intended to remain stable. `app/main.c` is the designated
integration point — its current console printing is a placeholder until real
data consumers (pressure sensing, video tracking, networking) are connected.

## Quick start

On the Raspberry Pi:

```bash
cd bno
make                  # builds bin/bno_app
sudo ./bin/bno_app    # runs the 100 Hz acquisition loop; Ctrl-C to stop
```

- Requires `sudo` (or `CAP_SYS_NICE`) for real-time priority (`SCHED_FIFO` 90)
  and memory locking (`mlockall`). Without privileges, it logs a warning and
  falls back to standard scheduling (`SCHED_OTHER`).
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
├── Makefile                 unified build system for app, tools, and tests
├── app/                     production application + Raspberry Pi HAL
│   ├── app_contract.h       ImuSample_t — shared production data contract
│   ├── app_sensor.c/.h      SH-2 session owner, decodes into latest ImuSample_t
│   ├── main.c               application loop (integration point)
│   ├── realtime.c/.h        SCHED_FIFO + mlockall, absolute CLOCK_MONOTONIC sleep
│   └── sh2_hal_rpi.c        Raspberry Pi sh2_Hal_t transport (SPI + libgpiod)
├── calibration/             BNO085 dynamic-calibration tools (see calibration/readme.md)
│   ├── cal_contract.h       CalSample_t — calibration progress data contract
│   ├── cal_sensor.c/.h      SH-2 session owner for calibration
│   ├── cal_main.c           bno_cal (guided cal + --clear + --check)
│   └── readme.md
├── orientation/             BNO085 swing-axis tare tools (see orientation/readme.md)
│   ├── orient_sensor.c/.h   SH-2 session owner for rotation vector
│   ├── orient_main.c        bno_orient (swing-axis tare + --persist)
│   └── readme.md
├── validation/              encoder-vs-IMU validation harness (see validation/readme.md)
│   ├── amt102.c/.h          AMT102-V incremental encoder driver (libgpiod)
│   ├── amt102_bringup.c     standalone encoder hardware diagnostic
│   ├── quad_decode.c/.h     pure x4 quadrature state machine
│   ├── validate_contract.h  ImuValidateSample_t — timestamped validation contract
│   ├── validate_logger.c/.h bounded-ring real-time CSV logger
│   ├── validate_sensor.c/.h SH-2 session owner for validation (100 Hz 3-sensor)
│   ├── validate_main.c      bno_validate (synchronized 100 Hz capture)
│   └── readme.md
├── tests/                   bring-up diagnostics and unit test suite
│   ├── test_hal_raw.c       SPI/HAL hardware bring-up test
│   ├── test_min_period.c    minimum sensor reporting period diagnostic
│   ├── test_quad_decode.c   pure quadrature decoder unit test suite
│   ├── test_report_len.c    SHTP packet length verification
│   ├── test_sh2_open.c      session open/handshake diagnostic
│   └── test_single_sensor.c single-sensor decode diagnostic
├── sh2/                     vendored CEVA sh2 library (unmodified submodule)
├── bin/                     (generated) binaries
└── build/                   (generated) intermediate objects
```

## Software architecture

Layered, strictly single-threaded (no mutexes anywhere — one loop owns all
state):

```text
main.c (1 kHz loop, 300 ms settle, 100 Hz consumer)
  └─ app_sensor (opens session, enables 3 reports, decodes events)
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
~1 ms service cadence** (call `app_sensor_service()` every iteration).

## Data contract (`app/app_contract.h`)

`ImuSample_t` is the boundary between the driver and the rest of the
system. All values are scaled to engineering units:

```c
typedef struct {
    uint8_t  version;        /* IMU_SAMPLE_STRUCT_VERSION (3) */
    uint32_t seq;            /* rolling counter; increments per sensor event */
    uint64_t tHost_uS;       /* CLOCK_MONOTONIC microsecond timestamp at decode */
    uint64_t tDevice_uS;     /* BNO085 internal timestamp (32-bit counter, us) */

    float    yaw;            /* rad, rotation about Z (-pi to +pi) */
    float    pitch;          /* rad, rotation about Y (-pi/2 to +pi/2) */
    float    roll;           /* rad, rotation about X (-pi to +pi) */

    float    ax;             /* m/s^2, body-frame linear acceleration (X) */
    float    ay;             /* m/s^2, body-frame linear acceleration (Y) */
    float    az;             /* m/s^2, body-frame linear acceleration (Z) */

    float    gx;             /* rad/s, body-frame calibrated angular velocity (X) */
    float    gy;             /* rad/s, body-frame calibrated angular velocity (Y) */
    float    gz;             /* rad/s, body-frame calibrated angular velocity (Z) */

    uint8_t  status;         /* 2-bit accuracy from the report: 0=unreliable, */
                             /* 1=low, 2=medium, 3=high */
    uint8_t  report_seq;     /* raw sequence from the BNO085 report byte 1 */
    uint8_t  validMask;      /* bit 0: RV valid, bit 1: Accel, bit 2: Gyro */
} ImuSample_t;
```

Orientation angles use the **Tait-Bryan ZYX convention** (yaw about Z, then
pitch about Y, then roll about X). The conversion is handled by
`sh2/euler.c`.

## Build & test reference

Build commands (from `bno/`):

```bash
make            # builds bin/bno_app
make tools      # builds bin/bno_app, bin/bno_cal, bin/bno_orient, bin/bno_validate, bin/amt102_bringup
make tests      # builds all diagnostic test binaries
make test       # builds and executes the host-only unit test (test_quad_decode)
make clean      # removes build/ and bin/
```

### Diagnostic tests (`tests/`)

- `bin/test_hal_raw`: tests `sh2_hal_rpi.c` directly without the SH-2 library.
- `bin/test_sh2_open`: tests the `sh2_open()` handshake.
- `bin/test_single_sensor`: enables a single sensor to verify report decoding.
- `bin/test_min_period`: queries FRS metadata for minimum supported periods.
- `bin/test_report_len`: comprehensive validation of SHTP packet lengths.
- `bin/test_quad_decode`: verifies the quadrature decoder state machine against synthetic test sequences.
