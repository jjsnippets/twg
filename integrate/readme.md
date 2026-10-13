# TWG Integrated Acquisition

`twg_app` is the single process-level owner for the BNO085 IMU and MS5837
pressure sensor.

It maintains one 1 kHz owner loop. The loop services the BNO085 session,
the current IMU command adapter, the MS5837 state machine, the shared
publication grid, and nonblocking worker queues for terminal input and CSV
output.

The terminal worker and logger worker remain `SCHED_OTHER`. Neither owns a
sensor, calls `RT_SleepUntil`, configures reports, or issues command-adapter
events.

## Build

From the repository root:

```sh
git submodule update --init --recursive

make -C integrate clean
make -C integrate app
```

The executable is:

```text
integrate/bin/twg_app
```

## Run

The default acquisition duration is ten seconds:

```sh
sudo ./integrate/bin/twg_app
```

Use another acquisition duration:

```sh
sudo ./integrate/bin/twg_app --duration 60
```

Startup calibration, tare, check, clear, and zero command-line operations
are intentionally not supported. Runtime controls replace them.

## Runtime controls

Terminal input is raw/noncanonical while the program runs. Pressing one of
the following keys takes effect immediately; Enter is not required.

| Key | Action |
|---|---|
| `c` | Start IMU calibration |
| `z` | Start Z-axis IMU tare |
| `a` | Start full X/Y/Z IMU tare |
| `w` | Start five-second pressure-reference zero |
| `x` | Request complete sensor clear |
| `y` | Confirm pending tare or full clear |
| `q` | Cancel the current action and return to acquisition |

`x` requests one destructive operation sequence:

1. BNO085 DCD calibration clear;
2. BNO085 active and saved tare clear; and
3. pressure-reference clear.

After pressing `x`, press `y` once to authorize the complete sequence.
`q` cancels while the program is still waiting for that confirmation.

Pressure clear does not erase MS5837 hardware calibration. It discards the
host-side pressure reference. Therefore `depth_m` becomes `NaN` until a new
pressure reference is established with `w`.

## Timing behavior

The requested duration measures only normal acquisition time.

For example, a default ten-second run can acquire for four seconds, perform
an IMU tare for any amount of wall-clock time, then resume acquisition for
the remaining six seconds.

CSV logging does not stop during calibration, tare, pressure zero, clear, or
production-report restoration. The `phase` column identifies the current
runtime mode:

```text
run
calibration
tare_z
tare_full
zero
clear
restore
```

Rows during non-acquisition phases do not consume acquisition duration.
They may contain incomplete IMU production groups while the BNO085 is in a
calibration, tare, or clear report configuration. Those rows remain useful
diagnostic evidence and should not be treated as normal synchronized
production samples.

## Console telemetry

Every 50th combined row prints fields including:

```text
qw qx qy qz
yaw pitch roll
ax ay az
gx gy gz
pressure_mbar
depth_m
```

Quaternion fields use the BNO085 rotation-vector order:

```text
qw qx qy qz
```

Euler angles are in radians. Linear acceleration and calibrated gyroscope
units follow the existing BNO085 session contract.

## Pressure reference

Pressing `w` starts a five-second pressure-reference capture. The estimator:

1. accepts up to 512 pressure samples;
2. requires at least 350 valid samples;
3. rejects not-ready, raw-invalid, pressure-invalid, and
   extended-temperature samples;
4. sorts accepted pressures;
5. trims ten percent from each tail; and
6. averages the retained pressures.

Depth is calculated only after a valid reference exists. Until then,
`depth_m` is `NaN`.

## CSV output

A combined CSV is written in the current directory:

```text
twg_acq_YYYYMMDD_HHMMSS.csv
```

The CSV preserves the IMU publisher semantics:

- `imu_valid_mask`: group observed in the current configuration epoch;
- `imu_fresh_mask`: group identity advanced since the preceding row;
- `imu_stale_mask`: a valid group did not advance;
- `imu_missing_mask`: a required group has not yet been observed; and
- `imu_multi_mask`: a group advanced more than once between rows.

IMU and pressure values share a host publication boundary. They are not
guaranteed to have identical physical sensor-conversion timestamps.

## Initial test sequence

```sh
make -C bno clean
make -C bno test
make -C bno app

make -C ms5 clean
make -C ms5 check
make -C ms5 app

make -C integrate clean
make -C integrate app

git diff --check
```

Then perform at least these hardware checks:

```text
1. normal default-duration acquisition;
2. c then q;
3. z then y;
4. a then y;
5. w and successful reference completion;
6. w then q;
7. x then q;
8. x then y;
9. SIGINT while acquiring;
10. SIGINT while an action is active.
```

Verify that the final shutdown line shows zero logger drops and that the
terminal settings are restored after normal exit, cancellation, and Ctrl-C.
