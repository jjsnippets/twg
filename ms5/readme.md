# MS5837-02BA Pressure and Temperature Subsystem (`ms5`)

Linux userspace driver, pure publication layer, and reference capture
executable for the TE Connectivity MS5837-02BA on a Raspberry Pi 4B running
Raspberry Pi OS with PREEMPT_RT.

The subsystem acquires fixed symmetric OSR-512 pressure/temperature pairs on a
dedicated I2C1 bus. One top-level executable owns the 1 kHz service loop and
the 100 Hz publication gate. The reusable modules report acquisition facts
only. Surface reference, fluid density, depth, logging, and process policy
belong to the consumer, and `ms5837_capture` is the reference consumer.

## Quick start

Build, test, and audit without hardware:

```bash
cd ~/twg/ms5
make clean
make check
make app
```

Run on the Raspberry Pi (root is needed for I2C access and real-time
scheduling):

```bash
sudo ./bin/ms5837_capture --duration 10
sudo ./bin/ms5837_capture --zero --duration 10
```

## Architecture

### Ownership rules

| Rule | Owner |
|---|---|
| `StartRT()` and `RT_SleepUntil()` | `app/main.c` only |
| Active HAL, I2C descriptor, and driver state machine | One driver instance, owned by `main.c` |
| Surface reference, zeroing, density, depth | `main.c` (consumer) |
| Pressure acquisition facts, recovery | `ms5837_driver` |
| Publication freshness and gate arithmetic | `ms5837_publish` (pure) |
| CSV serialization and file I/O | `ms5837_logger` |

The driver performs no autonomous periodic scheduling and owns no thread. The
publisher performs no clock reads, sleeps, I2C, logging, allocation, baseline
selection, density selection, or depth calculation. `make audit` enforces the
scheduling and boundary rules as a static source scan.

### Dependency direction

```text
ms5837_capture (main.c)  policy and scheduling owner
 |-- rt/realtime, rt/rt_policy   StartRT, RT_SleepUntil, status policy
 |-- ms5837_cli                  --duration and --zero parsing
 |-- ms5837_publish              pure 100 Hz gate and freshness logic
 |-- ms5837_logger               asynchronous CSV sink
 `-- ms5837_driver               owner-triggered nonblocking D1/D2 state machine
      |-- ms5837_hal             injectable I/O contract
      |-- ms5837_math            CRC-4, compensation, range classes, depth helper
      `-- ms5837_hal_rpi         Linux I2C and monotonic-clock adapter
```

### Modules

| File | Responsibility |
|---|---|
| `app/ms5837_contract.h` | Version-2 sample contract, status flags, driver-owned statistics |
| `app/ms5837_math.c/.h` | CRC-4, factory compensation, range classification, signed depth helper |
| `app/ms5837_hal.h` | Platform-independent I/O contract |
| `app/ms5837_hal_rpi.c/.h` | Linux `/dev/i2c-*` implementation, `I2C_SLAVE` setup, monotonic clock |
| `app/ms5837_driver.c/.h` | Reset, PROM validation, triggered D1/D2 state machine, bounded recovery |
| `app/ms5837_publish.c/.h` | Publication record, freshness/age classification, 10-tick gate |
| `app/ms5837_cli.c/.h` | Restricted argument parsing |
| `app/ms5837_logger.c/.h` | Asynchronous CSV writer, schema version 2 |
| `app/main.c` | Startup retry, real-time setup, 1 kHz loop, zeroing, depth, telemetry, signals, shutdown |

## Final configuration

| Item | Setting |
|---|---|
| Sensor | MS5837-02BA |
| Linux device | `/dev/i2c-1` |
| 7-bit address | `0x76` |
| Bus ownership | Dedicated to the pressure sensor |
| Bus speed for final benchmark | 400 kHz Fast Mode |
| Conversion strategy | Symmetric D1/D2 pair, OSR 512 for both |
| Owner-loop period | 1 ms (1 kHz) |
| Publication rate | 100 Hz (every tenth logical service tick) |
| Zero stage | 5 seconds, 10% trimmed mean, at least 350 valid samples |
| Fluid density | 1000.0 kg/m3, compile-time constant in `main.c` |
| Console telemetry | Every 50 publication rows, about 500 ms |
| CSV name | `ms5_capture_YYYYMMDD_HHMMSS.csv` in the working directory |
| Real-time policy requested | `SCHED_FIFO`, priority 90 |

There is no interleaved mode and no runtime OSR option. `MS5837_ACQUISITION_OSR`
is fixed to `MS5837_OSR_512`; changing it causes a compile-time error.

## Hardware

### Wiring

| Breakout pin | Raspberry Pi 4B | Physical pin | BCM | Notes |
|---|---|---:|---:|---|
| VCC/VIN | 3.3 V | 1 or 17 | - | Confirm the breakout accepts 3.3 V |
| GND | Ground | 6, 9, 14, 20, 25, 30, 34, or 39 | - | Common ground required |
| SDA | I2C1 SDA | 3 | GPIO2 | 3.3 V logic only |
| SCL | I2C1 SCL | 5 | GPIO3 | 3.3 V logic only |

The sensor needs no chip-select, interrupt, or reset GPIO. The previously
listed occupied physical pins are 11, 13, 15, 19, 21, 22, 23, 29, 31, and 33;
none conflicts with pins 3 and 5. The BNO085 uses SPI0 and GPIO lines.

### Electrical notes

Raspberry Pi GPIO is 3.3 V logic and is not 5 V tolerant. Do not allow SDA or
SCL to be pulled up to 5 V. Verify the actual breakout regulator and pull-up
arrangement rather than assuming every MS5837 breakout is wired identically.
If the breakout already includes the specified decoupling capacitor and
pull-ups, do not add duplicates blindly. With power removed, check continuity
and resistance from SDA/SCL to 3.3 V before connecting the Pi, and establish a
common ground before applying power.

### Fast Mode

The final benchmark used I2C Fast Mode. On current Raspberry Pi OS releases,
set the bus rate in `/boot/firmware/config.txt`:

```ini
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=400000
```

Reboot after changing the configuration. Confirm that pull-ups, cable length,
and waveform suit 400 kHz; a successful `i2cdetect` scan is not proof of
signal integrity.

## Software prerequisites

```bash
sudo apt update
sudo apt install build-essential i2c-tools libi2c-dev
ls -l /dev/i2c-1
sudo i2cdetect -y 1
```

The expected address is `0x76`. Stop other programs that access the sensor
during diagnostics or capture.

## Build and test

The build uses C99 with `-O2 -Wall -Wextra -Werror -pedantic`.

| Target | Result |
|---|---|
| `make app` | Builds `bin/ms5837_capture` |
| `make host-tests` | Builds the production host proofs (no hardware) |
| `make check` | Builds and runs every host proof, then the scheduling audit |
| `make audit` | Runs `tests/audit_scheduling.sh` |
| `make live-tests` | Builds the opt-in initialization and acquisition hardware diagnostics |
| `make tests` | Builds archived tools, host proofs, and live diagnostics |
| `make all` | Builds all tests and the capture application |
| `make clean` | Removes `build/` and `bin/` only |

### Host proofs

| Binary | Proves |
|---|---|
| `test_ms5837_math` | CRC, compensation, range classification, signed depth, nan behavior |
| `test_ms5837_init_mock` | Initialization order, every injected failure point, cleanup |
| `test_ms5837_acquisition_mock` | Version-2 contract, trigger/busy rules, D1/D2 pairing, bounded recovery |
| `test_ms5837_publish` | Pure publisher and 100 Hz gate |
| `test_ms5837_app` | Restricted CLI (including rejected `--density`) and CSV logger |
| `test_ms5837_ownerloop` | Capture owner: zero stage, run, recovery |

`make audit` rejects scheduling calls, direct sleeps, printing, and process
termination in reusable modules. Only `main.c` may call `StartRT` or
`RT_SleepUntil`. Startup-only HAL sleeps are whitelisted.

### Live diagnostics

```bash
make live-tests
sudo ./bin/test_ms5837_init_live
sudo ./bin/test_ms5837_acquisition_live
```

The initialization diagnostic resets the sensor, reads the seven PROM words,
checks coefficient sanity, and compares stored and computed CRC values. The
acquisition diagnostic exercises the explicit-trigger OSR-512 path against the
physical device. Live tests are never part of `host-tests` or `check`.

### Archived bring-up tools

```bash
./bin/test_i2c_raw
./bin/test_crc_math
sudo ./bin/test_rate_bench
```

These Step 1 to Step 3 programs are retained for investigation and historical
comparison. They stay independent of the production modules. Only their
ignored-I/O-result warnings are suppressed in their dedicated Makefile rules.

## Capture application

### Command line

```text
sudo ./bin/ms5837_capture [--duration SECONDS] [--zero]
```

| Option | Meaning |
|---|---|
| `--duration SECONDS` | Positive integer length of the capture phase; default 10 |
| `--zero` | Run a five-second surface-reference stage before the capture phase |

Unknown, duplicate, malformed, zero, or overflowing arguments are rejected,
including the removed `--density`. There are no `--bus`, `--priority`, or
`--help` options.

```bash
sudo ./bin/ms5837_capture                      # 10 s, no reference
sudo ./bin/ms5837_capture --duration 30        # 30 s, no reference
sudo ./bin/ms5837_capture --zero --duration 30 # 5 s zero stage, then 30 s
```

### Startup

Initialization allows three total attempts with 200 ms between failed
attempts. This delay is process-level waiting before `StartRT()` and before the
timed window. If all three attempts fail, the executable exits nonzero before
opening the acquisition window.

After the logger starts, `StartRT()` is called exactly once. An invalid period
or monotonic-clock failure is fatal. A memory-lock or scheduler privilege
failure prints a warning and continues in best-effort mode.

### Owner loop and publication

The executable runs one 1 ms loop with one `RT_SleepUntil()` call site. Each
turn performs at most one bounded driver service action. A positive
`RT_SleepUntil()` return means expired periods were skipped; the logical tick
count advances by the same amount and the skipped service periods are counted.
A negative return is an owner failure and ends the run through controlled
drain and shutdown.

The driver is triggered at the start of each phase and again after each
evaluated gate. Every tenth logical tick is a publication gate. At each gate
the owner reads the latest completed sample, evaluates it through the
publisher, derives consumer fields, and enqueues exactly one CSV row. It then
triggers the next pair if the driver is idle. Gates missed during an overrun
are counted in `skipped_gates` and are never backfilled in a burst.

During recovery the loop and publication grid continue, and each gate emits a
truthful `not_ready` row until the driver produces a new valid sample. The
driver never sleeps in trigger, service, or recovery. Console telemetry prints
every 50 publication rows.

`SIGINT` and `SIGTERM` request a controlled stop. The logger drains and closes
its CSV stream, then the driver closes its HAL. Runtime recovery exhaustion
ends the capture with a nonzero exit status after the current gate has been
recorded.

### Publication semantics

| Class | Meaning |
|---|---|
| Ready | A valid current sample exists in the driver mailbox |
| Not ready | No valid mailbox sample (startup, recovery, or contract violation) |
| Fresh | Ready, and `measurement_sequence` is newer than the previous comparable one |
| Stale | Ready, and `measurement_sequence` repeats the previous one |

Age is valid only when both times are valid, the sample completion time is
nonzero, and `actual_ns >= measurement_complete_time_ns`. Otherwise age is
invalid and the CSV records `nan`. A publication is flagged
`deadline_missed` when it has skipped gates before it, invalid timing, or an
actual time more than 1 ms after its scheduled time.

### Surface zeroing

With `--zero`, the capture runs a separate five-second stage after real-time
startup and the initial warm-up sample. Publication and CSV logging stay active
during this stage, and its rows carry `phase=zero`, so there is no output gap.

The consumer-side estimator offers each fresh completed sample at most once by
measurement sequence. It rejects not-ready, raw-invalid, pressure-invalid,
non-finite, duplicate, and over-capacity samples, and counts them. At the end
of the stage it requires at least 350 valid samples, retains up to 512, sorts
them, trims 10% from each end, and averages the remainder. That mean becomes
`surface_pressure_mbar`.

Zeroing is optional and nonfatal. If it fails, the capture still runs with the
reference invalid and depth `nan`. The requested `--duration` begins after the
zero stage whether it succeeds or fails. Without `--zero`, the reference stays
invalid for the whole run. Perform zeroing with the sensing face stationary at
the intended zero-depth pressure.

The estimator is private static logic in `main.c`. It is not a driver or
publisher service.

### Depth

Depth is computed only by the consumer, only when pressure and the surface
reference are both valid:

```text
depth_m = (pressure_mbar - surface_pressure_mbar) * 100
          / (fluid_density_kg_m3 * 9.80665)
```

Depth is signed: pressure above the reference gives positive depth and below
gives negative depth. Missing or invalid inputs give `nan`. Density is the
named compile-time constant `CAPTURE_FLUID_DENSITY_KG_M3` (1000.0 kg/m3) and
is repeated in every CSV row.

## Sample contract (version 2)

`BaroSample_t` in `app/ms5837_contract.h`:

| Field | Type | Meaning |
|---|---|---|
| `contract_version` | `uint32_t` | Sample contract version (2) |
| `status_flags` | `uint32_t` | Per-measurement validity and range flags |
| `measurement_sequence` | `uint64_t` | Completed-pair sequence number |
| `measurement_complete_time_ns` | `uint64_t` | Monotonic completion timestamp |
| `raw_pressure_d1` | `uint32_t` | Raw pressure ADC result |
| `raw_temperature_d2` | `uint32_t` | Raw temperature ADC result |
| `pressure_mbar` | `double` | Compensated pressure |
| `temperature_c` | `double` | Compensated temperature |

The sample carries no depth, density, or surface reference.

### Status flags

| Flag | Bit | Meaning |
|---|---:|---|
| `BARO_STATUS_NOT_READY` | 0 | No completed measurement is available |
| `BARO_STATUS_RAW_INVALID` | 1 | A raw ADC value failed validation |
| `BARO_STATUS_PRESSURE_EXTENDED` | 2 | Pressure is in the extended range but outside nominal |
| `BARO_STATUS_PRESSURE_INVALID` | 3 | Pressure is outside the accepted range |
| `BARO_STATUS_TEMPERATURE_EXTENDED` | 4 | Temperature is outside the nominal range |
| Reserved | 5-6 | Always zero; consumers must not reinterpret them |

Pressure is nominal from 300 to 1200 mbar, extended from 10 to below 300 mbar
and above 1200 to 2000 mbar, and invalid below 10 or above 2000 mbar. The
nominal temperature range is -20 to 85 degrees C.

## Driver

### Initialization

`ms5837_driver_init()` validates the HAL, opens the bus, issues reset `0x1E`,
waits 3 ms for PROM reload, reads seven PROM words, checks coefficient sanity,
validates CRC-4, and leaves the driver in `IDLE`. Any partial failure closes
resources before returning an error. This synchronous path may use the HAL
sleep callback because it runs before real-time ownership begins.

### Acquisition

`ms5837_driver_trigger()` is accepted only when the driver is initialized and
idle; otherwise it returns `MS5837_DRIVER_ERR_BUSY`. It starts a D1 OSR-512
conversion and sets a monotonic deadline. `ms5837_driver_service()` takes the
caller's monotonic time and never sleeps. After the D1 deadline it reads D1
and starts D2, after the D2 deadline it reads D2, then it compensates the pair,
assigns the completion time and sequence number, updates the latest-sample
mailbox, and returns `MS5837_SERVICE_SAMPLE_READY`. Service before a deadline
performs no ADC read.

### Recovery

An observed I2C or validation failure aborts the partial pair, closes the HAL,
clears the latest-sample mailbox so stale data is never presented as current,
and starts a recovery incident. Each attempt runs open, reset, PROM reload by
monotonic deadline, PROM read, coefficient validation, and CRC validation. A
failed attempt arms a 200 ms monotonic backoff deadline while attempts remain;
the driver does no I/O during that gap. An incident allows three total
attempts. The third failure records `recovery_exhaustions` and moves the driver
to `MS5837_STATE_STOPPED`. A successful attempt returns the driver to `IDLE`,
increments `recovery_successes`, and resets the limit for the next incident.
Measurement sequence numbers stay monotonic across recovery. There is no
manual reinitialize API.

Driver statistics cover service ticks, completed measurements, validity
classification counts, I2C errors, aborted pairs, recovery incidents,
attempts, successes, and exhaustions, and the last error. Only driver
functions modify them. Publication, skip, deadline, loop-body, and logger
counters belong to the capture, not the driver.

## Publisher (`ms5837_publish`)

The publisher is a pure library. Its window-begin function records the
comparable pre-window measurement sequence. Its evaluate function classifies
one snapshot as ready or not ready, fresh or stale, with age, and copies the
sensor facts, scheduled and actual times, deadline status, and driver state
into a versioned `Ms5837PublicationRecord_t`. Its gate counter advances by
logical ticks, reports one due gate at each tenth tick, and counts skipped
gates without backfilling. The lateness threshold `MS5837_PUBLISH_T_LATE_NS`
is a provisional 1 ms constant.

## CSV output

The logger creates `ms5_capture_YYYYMMDD_HHMMSS.csv` in the working directory
using exclusive creation, so an existing file is never overwritten. Schema
version 2 is intentionally incompatible with the earlier unversioned format.
The exact header is:

```text
schema_version,publication_sequence,scheduled_ns,scheduled_valid,actual_ns,actual_valid,deadline_missed,gates_skipped_before,phase,sample_ready,sample_fresh,sample_stale,age_valid,age_ns,driver_state,contract_version,status_flags,measurement_sequence,measurement_complete_time_ns,raw_pressure_d1,raw_temperature_d2,pressure_mbar,temperature_c,zero_requested,zero_valid,zero_valid_samples,zero_rejected_samples,surface_pressure_mbar,fluid_density_kg_m3,depth_m,logger_drops_before,not_ready
```

| Column | Meaning |
|---|---|
| `schema_version` | CSV schema version (2) |
| `publication_sequence` | Contiguous sequence of emitted publications |
| `scheduled_ns`, `scheduled_valid` | Intended monotonic gate time and its validity |
| `actual_ns`, `actual_valid` | Monotonic time when the gate was evaluated and its validity |
| `deadline_missed` | 1 for late, invalid-timing, or skipped-gate rows |
| `gates_skipped_before` | Gates skipped immediately before this row |
| `phase` | `zero` or `run` |
| `sample_ready`, `sample_fresh`, `sample_stale` | Publication classification |
| `age_valid`, `age_ns` | Sample age at evaluation, `nan` when invalid |
| `driver_state` | Numeric driver state at evaluation (see `ms5837_driver.h`) |
| `contract_version`, `status_flags` | Sample contract version and decimal flag mask |
| `measurement_sequence`, `measurement_complete_time_ns` | Identity and completion time of the pair |
| `raw_pressure_d1`, `raw_temperature_d2` | Raw ADC values |
| `pressure_mbar`, `temperature_c` | Compensated values, nan when not ready |
| `zero_requested`, `zero_valid` | Consumer zero policy state |
| `zero_valid_samples`, `zero_rejected_samples` | Estimator progress counts |
| `surface_pressure_mbar` | Surface reference or nan |
| `fluid_density_kg_m3` | Compile-time density used for depth |
| `depth_m` | Signed depth or nan |
| `logger_drops_before` | Logger drops observed before this enqueue attempt |
| `not_ready` | 1 when no valid sample was available |

Not-ready rows write raw values as 0 and status flag bit 0. Floating-point
columns print NaN through `printf`, so the token appears as `nan`.

The logger queue holds 2048 records. Enqueue uses a nonblocking mutex attempt,
so contention, a full queue, shutdown, or an earlier write failure rejects the
record, counts a logger drop, and makes the application return failure.

## Runtime summary

A capture ends with a summary line:

```text
completed=... publications=... enqueued=... not_ready=... fresh=... stale=...
i2c_errors=... aborted_pairs=... recoveries=.../... skipped_ticks=...
skipped_gates=... deadline_misses=... logger_drops=... max_body_ns=...
```

`completed` is one higher than `publications` in a healthy run because the
warm-up sample completes before the window opens. For a stationary healthy run,
investigate any nonzero `i2c_errors`, repeated `aborted_pairs`, failed
recoveries, skipped ticks or gates, deadline misses, or logger drops. Read
not-ready or stale rows together with the timing and recovery counters.

Useful checks:

```bash
ls -l /dev/i2c-1
sudo i2cdetect -y 1
dmesg | grep -i i2c
ps -eLo pid,tid,cls,rtprio,pri,comm | grep ms5837
ls -lt ms5_capture_*.csv | head
```

## Hardware acceptance evidence

Build under test: `4aed8995bdc87175184dcbf8d1cecb067b6cc478`, Raspberry Pi
(`twg1`), kernel `6.12.68-v8-rt+` (PREEMPT_RT, aarch64), CPU governor
`ondemand`, run as root via `sudo`. SHA-256 of `bin/ms5837_capture`:
`58b6ab99e92f5bd8ba47c1793965aaa60d9e9e037cb1740ce105fe4426318b61`.

Captures are retained outside Git. Each run reported zero I2C errors, aborted
pairs, recoveries, skipped gates, deadline misses, stale rows, not-ready rows,
and logger drops unless stated otherwise.

| Run | Command | Result |
|---|---|---|
| Basic, no zero | `--duration 10` | 1000 rows, depth nan, reference invalid (`ms5_capture_20261012_190438.csv`) |
| Zero success | `--zero --duration 10` | 500 zero rows plus 1000 run rows, 500 valid zero samples (`ms5_capture_20261012_184509.csv`) |
| Stability | `--zero --duration 60` | 500 zero rows plus 6000 run rows, max loop body 608992 ns (`ms5_capture_20261012_184529.csv`) |
| SIGINT, run phase | `--zero --duration 60`, Ctrl-C | 775 rows drained (`ms5_capture_20261012_184638.csv`) |
| SIGINT, zero phase | `--zero --duration 60`, Ctrl-C | 204 rows drained (`ms5_capture_20261012_184652.csv`) |
| SIGINT, no zero | `--duration 60`, Ctrl-C | 590 rows drained, startup attempt 1/3 failed then succeeded (`ms5_capture_20261012_185249.csv`) |
| SIGTERM | `--zero --duration 60`, `pkill -TERM` | 557 rows drained (`ms5_capture_20261012_184718.csv`) |

## Known limitations

- The optional runtime-recovery hardware run was not fully recorded. A capture
  with a 41-row not-ready period followed by a stopped driver state exists,
  but its console output and exit status were not retained. Recovery behavior
  is otherwise proven only by host tests.
- Startup retry was observed once on hardware (attempt 1 failed, attempt 2
  succeeded). The 200 ms spacing was not measured there.
- `scheduled_ns` is anchored to a monotonic read at the start of each phase.
  `actual_ns - scheduled_ns` is therefore a constant phase offset (about
  -0.17 to -0.6 ms in the retained runs), not a jitter measurement.
  `deadline_missed` flags only late rows.
- With `--zero`, the first run row follows the last zero row after about 9 ms
  rather than 10 ms. This is not a skipped gate.
- Loop-body maximum was 0.49 to 0.61 ms against a 1 ms period in the retained
  runs. No worst-case latency bound is claimed, and runs used the `ondemand`
  governor.
- Pressure is quantized by the sensor's 0.01 mbar resolution at OSR 512.

## OSR decision

The archived Step 3 benchmark compared OSR and cadence choices with a 1 kHz
service loop and 100 Hz publication windows. The production choice is
symmetric OSR 512 instead of an interleaved or asymmetric mode.

| Paired mode | 100 kHz rate | 100 kHz stale | 400 kHz rate | 400 kHz stale |
|---|---:|---:|---:|---:|
| OSR 2048 / 2048 | 79.00 Hz | 21.10% | 83.30 Hz | 16.80% |
| OSR 1024 / 1024 | 124.99 Hz | 0.10% | 125.00 Hz | 0.10% |
| OSR 512 / 512 | 166.70 Hz | 0.00% | 166.70 Hz | 0.00% |
| OSR 256 / 256 | 175.20 Hz | 0.00% | 250.00 Hz | 0.00% |

At 400 kHz, the OSR-512 test measured about 180.73 microseconds average I2C
transaction time with no stale frames in 1000 publication windows. OSR 512
preserves 100 Hz headroom with a better noise/resolution trade-off than OSR 256.
Production still triggers exactly one pair per 10 ms frame instead of running
at maximum benchmark throughput.

## Verified sensor data

The original live PROM capture:

| Word | Hex | Decimal | Meaning |
|---|---:|---:|---|
| C0 | `0x4BA1` | - | CRC/version word; stored CRC nibble `0x4` |
| C1 | `0xBA4A` | 47690 | Pressure sensitivity |
| C2 | `0xB779` | 46969 | Pressure offset |
| C3 | `0x7395` | 29589 | Temperature coefficient of sensitivity |
| C4 | `0x78A1` | 30881 | Temperature coefficient of offset |
| C5 | `0x793D` | 31037 | Reference temperature |
| C6 | `0x6ACF` | 27343 | Temperature coefficient |

The archived CRC calculation returned `0x4`, matching the stored nibble. The
datasheet vector with D1 `6465444` and D2 `8077636` produced 1100.02 mbar and
20.00 degrees C in the archived math proof.

## Troubleshooting

### No `/dev/i2c-1`

Enable I2C, verify `dtparam=i2c_arm=on`, reboot, and check that the I2C kernel
modules are loaded.

### Address `0x76` absent

Power down before rewiring. Recheck 3.3 V, common ground, SDA/SCL orientation,
connector continuity, and pull-up voltage, then retry with only the pressure
sensor connected.

### Initialization failure or CRC failure

Startup retries three times. Repeat the live initialization diagnostic.
Persistent failure suggests wiring, power, timing, or device problems. Do not
bypass CRC validation in production.

### Depth is `nan`

This is expected without `--zero`. With `--zero`, check the printed
`surface_reference` line or the zero-failure warning, and confirm at least 350
valid unique samples were collected.

### Capture warns about real-time setup

Run with suitable privileges and a PREEMPT_RT kernel. The capture continues in
best-effort mode, but timing must then be judged from the CSV and counters.

### Not-ready or stopped rows

A burst of not-ready rows with a nonzero `driver_state` indicates driver
recovery. If recovery exhausts, the driver stops, the capture exits nonzero,
and the CSV remains drained up to that gate.

### Logger drops

Check disk space, filesystem health, storage latency, and competing I/O. The
logger rejects instead of blocking the owner loop.

## Directory layout

```text
twg/
├── rt/
│   ├── realtime.c/.h
│   ├── rt_policy.h
│   └── test_realtime_start.c           # the bno makefile runs this
└── ms5/
    ├── Makefile
    ├── readme.md
    ├── app/
    │   ├── main.c
    │   ├── ms5837_contract.h
    │   ├── ms5837_cli.c/.h
    │   ├── ms5837_driver.c/.h
    │   ├── ms5837_hal.h
    │   ├── ms5837_hal_rpi.c/.h
    │   ├── ms5837_logger.c/.h
    │   ├── ms5837_math.c/.h
    │   └── ms5837_publish.c/.h
    ├── tests/
    │   ├── audit_scheduling.sh
    │   ├── test_i2c_raw.c              # archived
    │   ├── test_crc_math.c             # archived
    │   ├── test_rate_bench.c           # archived
    │   ├── test_ms5837_math.c
    │   ├── test_ms5837_init_mock.c
    │   ├── test_ms5837_init_live.c
    │   ├── test_ms5837_acquisition_mock.c
    │   ├── test_ms5837_acquisition_live.c
    │   ├── test_ms5837_publish.c
    │   ├── test_ms5837_app.c
    │   └── test_ms5837_ownerloop.c
    ├── build/
    └── bin/
```

`build/` and `bin/` are generated and removed by `make clean`. CSV captures are
written to the working directory.

## Integration handoff

Later combined-system behavior belongs under `twg/integration`, not in `ms5`.
The pattern to reuse is: one top-level owner runs the 1 kHz loop and the only
`StartRT`/`RT_SleepUntil` calls; it services this nonblocking driver once per
tick, publishes at every tenth tick through `ms5837_publish`, and applies its
own zero, density, depth, and logging policy. A combined application should use
its own logger rather than the pressure-only `ms5837_logger`, and must not open
a second handle on the sensor.
s