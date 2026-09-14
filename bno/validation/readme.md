# Validation and Diagnostics

This directory contains the BNO085 + AMT102 validation harness used to measure
sensor timing, freshness, and dynamic agreement against an independent rotary
encoder. It also contains the reusable AMT102 quadrature decoder and its
host-only unit test support.

## Hardware

### BNO085

The validation harness uses the same SPI wiring and HAL as the production IMU
application in `../app/sh2_hal_rpi.c`.

### AMT102-V

| AMT102 signal | Raspberry Pi BCM | Physical pin |
|---|---:|---:|
| Channel A | 17 | 11 |
| Channel B | 27 | 13 |
| Index X | 22 | 15 |
| +5 V | 5 V rail | 2 or 4 |
| GND | GND | any ground pin |

The AMT102 is configured for 2048 PPR. With quadrature x4 decoding this yields
8192 counts per revolution.

## Programs

### `amt102_bringup`

A standalone terminal diagnostic for the encoder. It reports count, angle,
A/B/X line state, invalid transitions, edge count, and index pulses every
500 ms.

```bash
cd bno
make bringup
sudo ./bin/amt102_bringup
```

### `bno_validate`

The synchronized capture tool. It records one 100 Hz row containing:

- host monotonic publication timestamp;
- AMT102 count, angle, event timestamp, index pulses, and decoder diagnostics;
- the latest rotation-vector, linear-acceleration, and calibrated-gyro values;
- per-report host/device timestamps, sequence bytes, SH-2 status, and lengths;
- aggregate IMU sequence/valid mask and logger drop count.

```bash
cd bno
make validate
sudo ./bin/bno_validate                    # until Ctrl+C
sudo ./bin/bno_validate -d 20              # 20 seconds
sudo ./bin/bno_validate -o run.csv -d 20   # explicit output path
```

The logger runs on a non-real-time worker thread and receives fixed-size
records through a bounded ring. The main thread services SH-2 at 1 kHz and
publishes every tenth iteration. The encoder has its own polling thread.

## Build targets

```bash
cd bno
make validate       # bin/bno_validate
make bringup        # bin/amt102_bringup
make test           # builds and executes decoder unit tests (bin/test_quad_decode)
```

| Binary | Source | Purpose |
|---|---|---|
| `bin/amt102_bringup` | `amt102_bringup.c`, `amt102.c`, `quad_decode.c` | Live terminal diagnostic streaming encoder telemetry every 500 ms |
| `bin/bno_validate` | `validate_main.c`, `validate_sensor.c`, `validate_logger.c`, `amt102.c`, `quad_decode.c`, `../../rt/realtime.c`, `../app/sh2_hal_rpi.c` | Synchronized 100 Hz capture to timestamped CSV |
| `bin/test_quad_decode` | `../tests/test_quad_decode.c`, `quad_decode.c` | Decoder state machine unit test (no hardware required) |

---

## Validation data contract

`validate_contract.h` defines `ImuValidateSample_t`. It is intentionally richer
than the production `ImuSample_t` so timing and freshness can be investigated
without first changing the production ABI.

Each report group contains:

```c
typedef struct {
    uint8_t  status;
    uint8_t  report_seq;
    uint16_t event_len;
    uint64_t host_ts_ns;
    uint64_t device_ts_us;
} ImuValidateReportMeta_t;
```

The top-level validation sample also contains the decoded values, quaternion,
rotation-vector error, aggregate sequence, aggregate timestamp, and sticky
`valid_mask`.

## CSV columns

The CSV logger writes the following groups of fields:

| Group | Columns |
|---|---|
| Host | `host_ts_ns` |
| Encoder | `enc_count`, `enc_angle_deg`, `enc_event_ts_ns`, `enc_x_pulses`, `enc_invalid`, `enc_edges_ab` |
| IMU aggregate | `imu_seq`, `imu_timestamp_us`, `valid_mask` |
| Rotation vector | `rv_status`, `rv_report_seq`, `rv_event_len`, `rv_host_ts_ns`, `rv_device_ts_us`, quaternion, yaw/pitch/roll, `rv_err_rad` |
| Linear acceleration | `la_status`, `la_report_seq`, `la_event_len`, `la_host_ts_ns`, `la_device_ts_us`, `ax`, `ay`, `az` |
| Calibrated gyro | `gyro_status`, `gyro_report_seq`, `gyro_event_len`, `gyro_host_ts_ns`, `gyro_device_ts_us`, `gx`, `gy`, `gz` |
| Logger | `drops` |

## Interpreting timestamps

`host_ts_ns` uses `CLOCK_MONOTONIC` and is suitable for elapsed-time and age
calculations inside one boot. The BNO085 device timestamps are microseconds from
the SH-2 transport and can occasionally regress by a few microseconds. Guard
all device-time differences against `dt <= 0`.

## Freshness analysis

A report group is fresh at a 100 Hz publication row if its `report_seq` differs
from the previous row. Sequence arithmetic is modulo 256. A sequence jump
greater than one means multiple events arrived between publication rows; an
unchanged sequence means the copied value is stale relative to that boundary.

`valid_mask` is not a freshness mask. It is sticky session-seen state and only
states whether at least one report from each group has been decoded during the
current run.

## Practical cautions

- Run only one SH-2/BNO085 owner at a time. Do not run `bno_app`, `bno_cal`,
  `bno_orient`, or another validation process concurrently.
- Keep validation output on local storage. Network filesystems and terminal
  flooding can introduce avoidable scheduling noise.
- Check the logger's `drops` field before trusting a capture.
- Populate `ARM_LENGTH_M` and `RUN_NOTES` in `validate_main.c` if those fields
  are needed for the run metadata.
