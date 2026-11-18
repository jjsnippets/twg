# IMU and encoder validation

`bno_validate` compares the production BNO085 observation stream with an
AMT102-V incremental encoder. It reuses the production IMU session,
calibration machine, tare machine, adapters, publisher, and IMU CSV
formatter.

This executable is a validation tool, not the glider application.

## Build and run

From `bno/`:

```bash
make validate
sudo ./bin/bno_validate
```

No command-line arguments are accepted. In particular, there are no
`--duration` or `--output` options.

Run from the directory in which CSV files should be created. Filenames use
the Raspberry Pi's configured local time:

```text
imu_validate_YYYYMMDD_hhmmss.csv
```

Existing files are never overwritten. A timestamp collision is an error.

Do not run another executable against the same IMU or encoder GPIO lines.

## Hardware

AMT102-V wiring uses BCM numbering:

| Signal | BCM GPIO | Header pin |
|---|---:|---:|
| A | 16 | 36 |
| B | 20 | 38 |
| X | 21 | 40 |

The encoder resolution is assumed to be 2048 PPR, or 8192 x4 counts per
revolution. Set the encoder DIP switches accordingly.

Use the established level-shifted wiring and common ground. Do not connect
an unsuitable encoder output voltage directly to Raspberry Pi GPIO.

The encoder reader uses the libgpiod v1.6 API. It requires readable initial
A/B levels and nonblocking event-request file descriptors.

Open the program with the pendulum stationary.

## Startup

Every program launch performs:

1. Guided calibration.
2. Automatic Z-axis tare.
3. Tare persistence and verification.
4. Production restoration and 300 ms software settle.
5. READY.

Guided calibration retains the existing q and Enter interactions. Follow
the printed production calibration instructions.

The validation owner automatically confirms Z tare. There is no separate
y prompt or alignment pause. Finish calibration with the rig aligned in
the intended resting reference pose.

Calibration and tare run once per program launch, not once per recording.

A cancelled, failed, timed-out, unverified, or unrestored preparation stage
prevents recording. Calibration q cancellation completes the machine's
restoration path before the validation owner exits.

Ctrl+C is a process stop and may terminate preparation without completing
the operator-q restoration path.

## Trial procedure

After READY:

1. Return the pendulum to its resting reference position.
2. Press lowercase s without Enter.
3. The program creates and prints the CSV filename.
4. The encoder software reference is zeroed.
5. A 20-second monotonic recording window starts.
6. Allow the arm to rest for approximately 2–3 seconds.
7. Lift it approximately 45 degrees and release it.
8. Wait for FINALIZING to complete and READY to return.

The initial rest, lifting, and free swinging are all inside the same
20-second recording. There is no automatic pre-recording countdown.

Repeat as many times as needed. Three attempts are an experimental
procedure, not a hard-coded limit.

Press Ctrl+C to terminate the program.

Only s is accepted in READY. Keys received during RECORDING or FINALIZING
are discarded rather than queued for another attempt.

## State and ownership model

The main thread owns both sensor readers:

```text
calibration -> Z tare -> settle -> READY
                                 |
                                 s
                                 |
                                 v
                             RECORDING
                                 |
                              20 seconds
                                 |
                                 v
                             FINALIZING
                                 |
                                 v
                               READY
```

The IMU remains operational between attempts. Starting another trial does
not reopen, reconfigure, recalibrate, or retare it.

Encoder events are drained during preparation, READY, RECORDING, and
FINALIZING. Encoder zeroing changes only the software position reference;
it preserves decoder phase and lifetime diagnostic counters.

An unexpected runtime IMU state, configuration-epoch change, or report
watchdog failure terminates the process. The tool does not silently recover
and continue an experiment under a changed session.

The watchdog permits up to one second without a report-group update. This
is a runtime liveness check, not a validation-accuracy acceptance criterion.

## Scheduling and recording window

The main loop targets a 1 ms service interval using CLOCK_MONOTONIC absolute
sleep. It does not request a real-time scheduling policy.

Publications use a 10 ms elapsed-time grid. The first gate is 10 ms after
the encoder-zero boundary.

The recording interval is half-open:

```text
[start_ns, start_ns + 20 seconds)
```

No row is admitted at or after the end deadline. Late/missed gates are
counted, never backfilled with fabricated catch-up rows.

An uninterrupted, fully serviced window normally contains 1999 rows:
10 ms through 19.990 seconds. Row count does not determine trial duration.

The final footer distinguishes the exact recording boundary from the time
at which the owner observed that recording should stop.

A host-merged row is not proof of physically simultaneous IMU and encoder
measurements. Use the recorded timestamps and IMU group identities during
analysis.

## CSV and buffering

During recording, records accumulate in a bounded 2048-record memory
buffer. There are no per-row disk writes in the acquisition path.

After the recording window closes, a worker formats and writes the frozen
buffer. Sensor servicing and the main status heartbeat continue while the
program is FINALIZING.

The CSV consists of:

- Comment lines beginning with #.
- A single table header.
- Validation and encoder columns.
- The existing production IMU CSV columns.
- A final outcome comment.

Analysis scripts must skip comment lines beginning with #.

The validation schema version is recorded in a comment. The table's
schema_ver column belongs to the embedded production IMU CSV schema.

Validation prefix columns:

| Column | Meaning |
|---|---|
| trial | Attempt number within this process |
| elapsed_ns | Publication actual time minus trial start |
| encoder_snapshot_ns | Encoder drain cutoff, CLOCK_MONOTONIC |
| encoder_zero_ns | Software-zero boundary, CLOCK_MONOTONIC |
| encoder_count | Signed x4 count relative to this trial's zero |
| encoder_angle_rad | Relative count converted to radians |
| encoder_raw_count | Lifetime signed x4 count |
| encoder_a_edges | Lifetime A edge count |
| encoder_b_edges | Lifetime B edge count |
| encoder_x_pulses | Lifetime index pulse count |
| encoder_invalid | Lifetime illegal-transition count |
| encoder_synced | Decoder phase is known |
| encoder_bias_fallback | Bias-disable request required fallback |
| encoder_last_event_ns | Most recent processed encoder event timestamp |
| encoder_last_index_ns | Most recent processed index timestamp |

Production IMU columns retain quaternion, yaw/pitch/roll, linear
acceleration, calibrated gyro, per-group metadata, and publication-relative
freshness classifications.

Encoder diagnostics are lifetime counters. Obtain trial-specific changes
by subtracting the relevant trial baseline.

The encoder GPIO v1 event interface does not provide reliable queue-overflow
evidence. Zero reported invalid transitions does not prove that no kernel
events were lost.

## Completion and interruption

The final footer records:

- outcome
- duration_ns
- observed_stop_ns
- signal
- rows
- gates_skipped

Outcomes are:

| Outcome | Meaning |
|---|---|
| complete | Recording reached its 20-second deadline |
| interrupted | An orderly process stop interrupted recording |
| failed | A runtime error interrupted recording |
| aborted_before_start | A CSV was created but the trial never started |

An orderly Ctrl+C preserves the buffered partial trial and waits for
finalization before exiting.

A CSV without a final outcome footer is incomplete. A power failure,
SIGKILL, or crash during recording can lose buffered rows, because records
are written only during finalization.

Write, flush, fsync, close, or worker failures produce a nonzero exit status.

## Terminal output

Status output targets a 500 ms interval during preparation, READY,
RECORDING, and FINALIZING. State changes and the CSV filename may print
immediately.

This is not a hard real-time output guarantee. Synchronous hardware
operations, terminal backpressure, scheduling delays, and the exceptional
worker-creation failure fallback can delay output.

## Existing diagnostics

The following remain separate and unchanged:

```bash
make bringup
make test
```

`amt102_bringup` continues to use the lifetime encoder count. The
quadrature-decoder tests remain unchanged.

The obsolete validate_sensor.c/.h implementation is not part of the new
validation binary and should be removed.

## Hardware smoke test

Check at least:

1. Calibration begins automatically.
2. Calibration q restores and exits without proceeding to tare.
3. Successful calibration proceeds directly to persisted Z tare.
4. Failed preparation never reaches READY.
5. READY creates no CSV and continues printing every 500 ms.
6. A single s starts recording without Enter and prints its filename.
7. Encoder position starts at zero without resetting lifetime diagnostics.
8. Recording closes at its monotonic 20-second boundary.
9. Additional s keys during recording/finalization do not queue attempts.
10. Three successive attempts create separate CSV files without repeating
    calibration or tare.
11. Ctrl+C during recording preserves a partial CSV with an interrupted
    outcome.
12. Ctrl+C restores terminal behavior.
13. CSV columns, comment handling, IMU freshness fields, and encoder
    direction are checked using the new analysis scripts.

No hardware performance or validation-accuracy claim is established by
building this executable alone.
