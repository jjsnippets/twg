# MS5837-02BA Pressure and Temperature Subsystem (`ms5`)

Linux userspace driver, real-time capture application, diagnostics, and archived benchmarks for the TE Connectivity MS5837-02BA on a Raspberry Pi 4B running Raspberry Pi OS with PREEMPT_RT.

The production path is implemented through Step 4D. It uses a dedicated I2C1 bus, fixed symmetric OSR 512 pressure/temperature pairs, explicit acquisition triggering at the start of each 10 ms frame, 100 Hz publication, optional five-second surface taring, and asynchronous CSV logging.

## Final configuration

| Item | Production setting |
|---|---|
| Sensor | MS5837-02BA |
| Linux device | `/dev/i2c-1` |
| 7-bit address | `0x76` |
| Bus ownership | Dedicated to the pressure sensor |
| Bus speed used for final benchmark | 400 kHz Fast Mode |
| Conversion strategy | Symmetric D1/D2 pair |
| Oversampling | `MS5837_OSR_512` for both D1 and D2 |
| Frame rate | 100 Hz, one 10 ms frame |
| Trigger point | Beginning of every frame through `ms5837_driver_trigger()` |
| Zeroing rate | 100 Hz |
| Zeroing duration | 5 seconds |
| Minimum valid zeroing samples | 350 |
| Surface estimator | 10% trimmed mean |
| Default fluid density | 1000 kg/m3 |
| Console telemetry | Every 50 frames, approximately 500 ms |
| CSV publication | Every frame, 100 Hz |
| CSV location | Current working directory |
| CSV name | `ms5_capture_YYYYMMDD_HHMMSS.csv` |
| Real-time policy requested | `SCHED_FIFO`, priority 90 |

There is no production interleaved mode and no runtime OSR option. The compile-time definition `MS5837_ACQUISITION_OSR` is fixed to `MS5837_OSR_512`; changing it currently causes a compile-time error.

## Hardware

### Wiring

| Breakout pin | Raspberry Pi 4B | Physical pin | BCM | Notes |
|---|---|---:|---:|---|
| VCC/VIN | 3.3 V | 1 or 17 | - | Confirm the breakout board accepts 3.3 V |
| GND | Ground | 6, 9, 14, 20, 25, 30, 34, or 39 | - | Common ground is required |
| SDA | I2C1 SDA | 3 | GPIO2 | 3.3 V logic only |
| SCL | I2C1 SCL | 5 | GPIO3 | 3.3 V logic only |

The sensor does not require a chip-select, interrupt, or reset GPIO. The production driver therefore adds only physical pins 3 and 5 plus one 3.3 V pin and one ground pin.

The previously listed occupied physical pins are 11, 13, 15, 19, 21, 22, 23, 29, 31, and 33. None conflicts with physical pins 3 and 5. The BNO085 uses SPI0 and GPIO lines, while the pressure sensor uses the dedicated I2C1 bus.

### Electrical notes

Raspberry Pi GPIO is 3.3 V logic and is not 5 V tolerant. Do not allow SDA or SCL to be pulled up to 5 V. Verify the actual breakout-board regulator and pull-up arrangement rather than assuming that every MS5837 breakout is wired identically.

The bare sensor requires local supply decoupling; if the breakout already includes the specified capacitor and pull-ups, do not add duplicates blindly. With power removed, check continuity and resistance from SDA/SCL to 3.3 V before connecting the Pi. Keep the I2C wiring short during bring-up and establish a common ground before applying power.

### Fast Mode

The final benchmark used I2C Fast Mode. On current Raspberry Pi OS releases, the bus rate can be configured in `/boot/firmware/config.txt`:

```ini
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=400000
```

Reboot after changing the configuration. Confirm that the breakout pull-ups, cable length, and bus waveform are suitable for 400 kHz; do not treat a successful `i2cdetect` scan as proof of signal integrity.

## Software prerequisites

Install the build and I2C packages:

```bash
sudo apt update
sudo apt install build-essential i2c-tools libi2c-dev
```

Verify that I2C is enabled and that the sensor responds:

```bash
ls -l /dev/i2c-1
sudo i2cdetect -y 1
```

The expected address is `0x76`. Stop other programs that may access the same sensor during diagnostics or capture. The production design assumes the bus is dedicated to this pressure sensor.

## Build

From `twg/ms5`:

```bash
make clean
make all
```

Important targets:

| Target | Result |
|---|---|
| `make app` | Builds `bin/ms5837_capture` |
| `make host-tests` | Builds production math, reference, initialization mock, acquisition mock, and application/logger tests |
| `make live-tests` | Builds the initialization and acquisition hardware diagnostics |
| `make tests` | Builds archived diagnostics, benchmarks, production host tests, and live diagnostics |
| `make all` | Builds all tests and the capture application |
| `make clean` | Removes `build/` and `bin/` |

The normal build uses C99 with `-O2 -Wall -Wextra -Werror -pedantic`. The archived Step 2 and Step 3 files remain unchanged; only their ignored-I/O-result warnings are suppressed in their dedicated Makefile rules.

## Capture application

### Command line

The production executable accepts only:

```text
sudo ./bin/ms5837_capture [--duration SECONDS] [--zero]
                           [--density KG_PER_M3]
```

| Option | Meaning |
|---|---|
| `--duration SECONDS` | Positive integer duration of the post-zero capture phase; default is 10 seconds |
| `--zero` | Perform a separate five-second surface-pressure tare before capture |
| `--density KG_PER_M3` | Positive finite fluid density; default is 1000 kg/m3 |

Without `--zero`, no surface reference is created and depth stays `NaN`. The options `--bus`, `--priority`, and `--help` are intentionally not implemented; concise CLI instructions are also kept in the comment at the beginning of `app/main.c`.

Examples:

```bash
# Ten seconds, freshwater default, no tare
sudo ./bin/ms5837_capture

# Thirty seconds, no tare
sudo ./bin/ms5837_capture --duration 30

# Five-second tare followed by thirty seconds of freshwater capture
sudo ./bin/ms5837_capture --duration 30 --zero

# Five-second tare followed by thirty seconds at a specified density
sudo ./bin/ms5837_capture --duration 30 --zero --density 1025
```

`--duration` covers only the run phase. A command that includes `--zero` therefore spends an additional five seconds collecting the surface reference.

### Frame behavior

Each phase runs as a sequence of 10 ms frames. At the defined beginning of each frame, the application calls `ms5837_driver_trigger()` to start D1. The driver services the D1 deadline, reads pressure ADC data, starts D2 at the same OSR, reads temperature ADC data, compensates the completed pair, and returns to `IDLE`.

The application publishes exactly one record per scheduled frame. If a new pair is unavailable, the record is marked not ready; if the latest completed measurement has already been published, it is marked stale. A late prior pair is serviced so that one overrun does not permanently stall acquisition.

Console telemetry is printed every 50 frames, approximately every 500 ms. CSV writing runs in a separate thread so routine file I/O is not performed directly in the time-sensitive acquisition path.

### Real-time behavior

The application requests `SCHED_FIFO` priority 90 through the timing implementation shared with `bno/app/realtime.c`. If real-time setup fails, it emits a warning and continues without `SCHED_FIFO`; run with the required privileges and limits when deterministic scheduling is needed.

`SIGINT` and `SIGTERM` request a controlled stop. Shutdown stops and joins the logger, drains queued records, flushes and closes the CSV stream, and then closes the driver HAL.

## Surface zeroing

When `--zero` is supplied, the application first collects completed pressure samples for five seconds at the same 100 Hz frame rate used during capture. Duplicate measurement sequences and invalid measurements are rejected.

Finalization requires at least 350 valid measurements. Up to 512 pressure values can be retained. The values are sorted, 10% is trimmed from each end, and the mean of the remaining values becomes `surface_pressure_mbar`.

If zeroing fails, `surface_pressure_valid` remains false and depth remains `NaN`; pressure and temperature capture can still be inspected. Zeroing should be performed with the sensing face stationary at the intended zero-depth pressure and with no transient squeezing, airflow, or water motion.

## Measurement contract

`BaroSample_t` contains:

| Field | Type | Meaning |
|---|---|---|
| `contract_version` | `uint32_t` | Sample contract version |
| `status_flags` | `uint32_t` | Per-measurement validity and range flags |
| `measurement_sequence` | `uint64_t` | Completed-pair sequence number |
| `measurement_complete_time_ns` | `uint64_t` | Monotonic completion timestamp |
| `raw_pressure_d1` | `uint32_t` | Raw pressure ADC result |
| `raw_temperature_d2` | `uint32_t` | Raw temperature ADC result |
| `pressure_mbar` | `double` | Compensated pressure |
| `temperature_c` | `double` | Compensated temperature |
| `depth_m` | `double` | Signed depth relative to the surface reference |

Depth is computed from the pressure difference:

```text
depth_m = (pressure_mbar - surface_pressure_mbar) * 100
          / (fluid_density_kg_m3 * 9.80665)
```

Depth is intentionally signed. Pressure above the reference produces positive depth, while pressure below the reference produces negative depth. Missing or invalid inputs produce `NaN` rather than a misleading zero.

### Status flags

| Flag | Bit | Meaning |
|---|---:|---|
| `BARO_STATUS_NOT_READY` | 0 | No completed measurement is available |
| `BARO_STATUS_RAW_INVALID` | 1 | A raw ADC value failed validation |
| `BARO_STATUS_PRESSURE_EXTENDED` | 2 | Pressure is in the extended linear range but outside the nominal range |
| `BARO_STATUS_PRESSURE_INVALID` | 3 | Pressure is outside the accepted linear range |
| `BARO_STATUS_TEMPERATURE_EXTENDED` | 4 | Temperature is outside the nominal range |
| `BARO_STATUS_DEPTH_REFERENCE_PENDING` | 5 | No surface tare was requested or completed yet |
| `BARO_STATUS_DEPTH_REFERENCE_INVALID` | 6 | The surface reference or density is invalid |

The production classification boundaries are 300 to 1200 mbar nominal pressure, 10 to below 300 mbar and above 1200 to 2000 mbar extended pressure, and below 10 or above 2000 mbar invalid pressure. The nominal temperature range is -20 to 85 degrees C.

## CSV output

The logger creates this file directly in the current working directory:

```text
ms5_capture_YYYYMMDD_HHMMSS.csv
```

It uses exclusive creation, so an existing file is not overwritten. The exact header is:

```text
publication_sequence,publication_time_ns,phase,sample_ready,sample_stale,fluid_density_kg_m3,surface_pressure_mbar,surface_pressure_valid,contract_version,status_flags,measurement_sequence,measurement_complete_time_ns,raw_pressure_d1,raw_temperature_d2,pressure_mbar,temperature_c,depth_m
```

| Column | Meaning |
|---|---|
| `publication_sequence` | Sequence of the scheduled 100 Hz publication |
| `publication_time_ns` | Monotonic timestamp assigned to the frame |
| `phase` | `zero` or `run` |
| `sample_ready` | 1 when a completed sample was available |
| `sample_stale` | 1 when the measurement sequence matches the previous publication |
| `fluid_density_kg_m3` | Density used for depth calculation |
| `surface_pressure_mbar` | Current tare reference or `NaN` |
| `surface_pressure_valid` | 1 when the surface reference is usable |
| `contract_version` | `BaroSample_t` contract version |
| `status_flags` | Decimal bit mask from the status table |
| `measurement_sequence` | Sequence of the completed D1/D2 pair |
| `measurement_complete_time_ns` | Monotonic completion time of that pair |
| `raw_pressure_d1` | Raw D1 ADC value |
| `raw_temperature_d2` | Raw D2 ADC value |
| `pressure_mbar` | Compensated pressure |
| `temperature_c` | Compensated temperature |
| `depth_m` | Signed depth or `NaN` |

The logger queue capacity is 2048 records. Enqueue uses a non-blocking mutex attempt; contention, a full queue, shutdown, or an earlier write failure can reject a record. Rejections increment `logger_drops`, and the application returns failure if drops occurred.

## Driver architecture

### Modules

| File | Responsibility |
|---|---|
| `app/app_contract.h` | Versioned sample, run configuration, status flags, and runtime statistics |
| `app/ms5837_math.c/.h` | CRC-4, factory compensation, range classification, and signed depth |
| `app/ms5837_reference.c/.h` | Five-second 100 Hz tare collection and trimmed-mean finalization |
| `app/ms5837_hal.h` | Injectable platform-independent I/O contract |
| `app/ms5837_hal_rpi.c/.h` | Linux `/dev/i2c-*` implementation and `I2C_SLAVE` setup |
| `app/ms5837_driver.c/.h` | Reset, PROM validation, triggered D1/D2 state machine, and recovery |
| `app/ms5837_cli.c/.h` | Restricted argument parsing and defaults |
| `app/ms5837_logger.c/.h` | Asynchronous timestamped CSV writer |
| `app/main.c` | 100 Hz orchestration, zeroing, telemetry, signals, and shutdown |

### Initialization

`ms5837_driver_init()` validates the HAL, opens the bus, issues reset `0x1E`, waits 3 ms for PROM reload, reads seven PROM words, checks coefficient sanity, validates CRC-4, and leaves the driver initialized in `IDLE`. Any partial initialization failure closes resources before returning an error.

### Acquisition

`ms5837_driver_trigger()` is accepted only when the driver is initialized and idle. It starts a D1 OSR-512 conversion and establishes the conversion deadline. Calling it while an acquisition is active returns `MS5837_DRIVER_ERR_BUSY`.

`ms5837_driver_service()` advances the nonblocking state machine according to the supplied monotonic time. It reads D1 after its deadline, starts D2 OSR 512, reads D2 after its deadline, performs compensation, assigns the completion timestamp and sequence number, updates the latest sample, and returns `MS5837_SERVICE_SAMPLE_READY`.

### Recovery

An I2C failure aborts the partial pair and moves the state machine into recovery. Service calls then close/reopen the HAL, reset the sensor, wait for PROM reload, reread all PROM words, revalidate coefficients and CRC, and return the driver to `IDLE`. Runtime statistics retain I2C errors, aborted pairs, recovery attempts, and successful recoveries.

## Tests

### Production host tests

Build and run the host-only suite:

```bash
make clean
make host-tests

./bin/test_ms5837_math
./bin/test_ms5837_reference
./bin/test_ms5837_init_mock
./bin/test_ms5837_acquisition_mock
./bin/test_ms5837_app
```

Coverage includes CRC corruption, datasheet compensation, second-order low-temperature compensation, raw rejection, pressure and temperature boundaries, signed depth, `NaN` behavior, trimmed-mean taring, the exact 350-sample threshold, initialization command order, every injected initialization failure point, cleanup, explicit triggering, busy retrigger rejection, D1/D2 acquisition, recovery, CLI restrictions, and asynchronous logger behavior.

Expected success lines are:

```text
PASS: Step 4A production math contract
PASS: Step 4A surface-reference contract
PASS: Step 4B mock initialization and cleanup
PASS: Step 4C symmetric OSR-512 acquisition and recovery
PASS: Step 4D CLI and asynchronous CSV logger
```

### Live diagnostics

Build the hardware diagnostics:

```bash
make live-tests
```

Run initialization diagnostics:

```bash
sudo ./bin/test_ms5837_init_live
```

This resets the physical sensor, reads the seven PROM words, checks coefficient sanity, and compares stored and computed CRC values using the fixed `/dev/i2c-1` and `0x76` configuration.

Run the acquisition diagnostic:

```bash
sudo ./bin/test_ms5837_acquisition_live
```

This exercises the explicit-trigger, symmetric OSR-512 acquisition path against the physical device. Stop any other process using the sensor before running either live test.

### Archived bring-up tools

The following Step 1 to Step 3 programs are retained for investigation and historical comparison:

```bash
./bin/test_i2c_raw
./bin/test_crc_math
sudo ./bin/test_rate_bench
```

`tests/test_crc_math.c` remains archived and independent; it is not converted to call `ms5837_math.c`.

## Verified sensor data

The original live PROM capture was:

| Word | Hex | Decimal | Meaning |
|---|---:|---:|---|
| C0 | `0x4BA1` | - | CRC/version word; stored CRC nibble `0x4` |
| C1 | `0xBA4A` | 47690 | Pressure sensitivity |
| C2 | `0xB779` | 46969 | Pressure offset |
| C3 | `0x7395` | 29589 | Temperature coefficient of sensitivity |
| C4 | `0x78A1` | 30881 | Temperature coefficient of offset |
| C5 | `0x793D` | 31037 | Reference temperature |
| C6 | `0x6ACF` | 27343 | Temperature coefficient |

The archived CRC calculation returned `0x4`, matching the stored nibble. The datasheet vector with D1 `6465444` and D2 `8077636` produced 1100.02 mbar and 20.00 degrees C in the archived math proof.

## OSR decision

The archived Step 3 benchmark compared OSR and pressure/temperature cadence choices using a 1 kHz service loop and 100 Hz publication windows. The final production choice is symmetric OSR 512 rather than an interleaved or asymmetric mode.

| Paired mode | 100 kHz measured rate | 100 kHz stale | 400 kHz measured rate | 400 kHz stale |
|---|---:|---:|---:|---:|
| OSR 2048 / 2048 | 79.00 Hz | 21.10% | 83.30 Hz | 16.80% |
| OSR 1024 / 1024 | 124.99 Hz | 0.10% | 125.00 Hz | 0.10% |
| OSR 512 / 512 | 166.70 Hz | 0.00% | 166.70 Hz | 0.00% |
| OSR 256 / 256 | 175.20 Hz | 0.00% | 250.00 Hz | 0.00% |

At 400 kHz, the archived OSR-512 test measured approximately 180.73 microseconds average I2C transaction time and no stale frames in 1000 publication windows. The selected setting preserves 100 Hz headroom while providing a better noise/resolution trade-off than OSR 256. Production still enforces one explicitly triggered D1/D2 pair per 10 ms frame instead of running continuously at the maximum benchmark throughput.

## Runtime checks

During a normal capture, inspect the final summary:

```text
completed=... publications=... not_ready=... stale=... i2c_errors=...
aborted_pairs=... recoveries=.../... overruns=... logger_drops=...
max_body_ns=...
```

For a healthy stationary run, investigate any nonzero `i2c_errors`, repeated `aborted_pairs`, failed recoveries, sustained overruns, or logger drops. A few not-ready or stale rows should be interpreted together with timing and recovery counters rather than discarded silently.

Useful checks:

```bash
# Device and address
ls -l /dev/i2c-1
sudo i2cdetect -y 1

# Kernel messages related to I2C
dmesg | grep -i i2c

# Confirm real-time scheduling while capture is running
ps -eLo pid,tid,cls,rtprio,pri,comm | grep ms5837

# Inspect the newest output
ls -lt ms5_capture_*.csv | head
head -n 5 ms5_capture_YYYYMMDD_HHMMSS.csv
```

## Troubleshooting

### No `/dev/i2c-1`

Enable I2C in Raspberry Pi configuration, verify `dtparam=i2c_arm=on`, and reboot. Check that the relevant kernel modules are loaded.

### Address `0x76` absent

Power down before rewiring. Recheck 3.3 V, common ground, SDA/SCL orientation, connector continuity, and pull-up voltage. Then retry with only the pressure sensor connected.

### Initialization CRC failure

Repeat the live initialization diagnostic. Persistent CRC mismatch suggests communication integrity, wiring, power, timing, or device problems; do not bypass CRC validation in production.

### Depth remains `NaN`

This is expected without `--zero`. With `--zero`, inspect the printed valid/rejected counts and confirm at least 350 valid unique samples were collected. Also verify that density is positive and finite.

### Capture warns about real-time setup

Run with suitable privileges and confirm the PREEMPT_RT scheduling configuration. The application continues without `SCHED_FIFO`, but timing should then be evaluated from the CSV and runtime counters.

### Logger drops

Check disk space, filesystem health, storage latency, and competing I/O. The logger intentionally rejects rather than blocking the acquisition thread when it cannot immediately lock or accept another queued record.

## Directory layout

```text
twg/
├── bno/
│   └── app/
│       ├── realtime.c
│       └── realtime.h
└── ms5/
    ├── Makefile
    ├── readme.md
    ├── app/
    │   ├── app_contract.h
    │   ├── main.c
    │   ├── ms5837_cli.c/.h
    │   ├── ms5837_driver.c/.h
    │   ├── ms5837_hal.c/.h        # Generic contract is ms5837_hal.h
    │   ├── ms5837_hal_rpi.c/.h
    │   ├── ms5837_logger.c/.h
    │   ├── ms5837_math.c/.h
    │   └── ms5837_reference.c/.h
    ├── tests/
    │   ├── test_i2c_raw.c
    │   ├── test_crc_math.c
    │   ├── test_rate_bench.c
    │   ├── test_ms5837_math.c
    │   ├── test_ms5837_reference.c
    │   ├── test_ms5837_init_mock.c
    │   ├── test_ms5837_init_live.c
    │   ├── test_ms5837_acquisition_mock.c
    │   ├── test_ms5837_acquisition_live.c
    │   └── test_ms5837_app.c
    ├── build/
    └── bin/
```

`build/` and `bin/` are generated and removed by `make clean`. CSV captures are deliberately written to the working directory rather than a `logs/` subdirectory.
