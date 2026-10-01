# Phase 8 Integration Results

**Status:** Step 8.6 host/no-hardware integration evidence in progress.  
**Implementation baseline:** `0a27e5971b0ac94153e506dd2847055de8adf5b2`  
**Later documentation baseline:** Update this file with the accepted Step 8.6
checkpoint after the exact commands below are executed. Do not embed a future
self-referential final Phase 8 squash SHA in this file.

## Scope and limits

This record distinguishes host proof from physical BNO085 evidence.

### Final captured device state

The destructive dual-clear capture (`36-dual-clear.txt`) intentionally erased
DCD and tare state at that point in the physical sequence. It is not the final
captured device state. The later authorized combined calibration/full-axis
tare capture (`37-cal-full-tare.txt`) completed same-session calibration,
recorded `dcdSaved=1`, and persisted a full-axis tare.

Therefore, the final captured device state for Phase 8 evidence is
**same-session calibrated and full-axis tared**, not a cleared-device state.
This is not a claim that DCD calibration or tare persistence was verified
across process restart, BNO085 reset, or power cycle.

Phase 8 currently provides:

- one integrated `bno_app` process with one active BNO085 session owner;
- one visible/tested 1 kHz owner-loop boundary;
- plain CLI parsing before console, signals, scheduling, or hardware startup;
- session-scoped guided calibration with best-effort DCD save;
- same-session `0x07 → 0x00` calibration verification;
- persistent tare/tare clear, check/probe, asynchronous console input, and
  structured R8/R7 terminal output;
- production restore, 300 ms settle, and host-side 10 ms acquisition
  observation.

Phase 8 does not prove:

- DCD persistence across reboot, restart, or power loss;
- physical report cadence, SPI transport parity, or simultaneous report
  freshness;
- physical DCD clear, guided calibration, persistent tare, or tare clear
  unless separately authorized and captured;
- CSV/R9 freshness, companion metadata, logger behavior, or combined-system
  behavior.

Daily operation requires calibration after BNO085 reset, process restart, or
power cycle. `dcdSaved=1` is a truthful best-effort save result, not
persistence acceptance.

## Host environment

| Field | Recorded value |
|---|---|
| Operator | Local execution account `twg` on host `twg1` |
| Date/time and time zone | Environment capture began `2026-09-30T17:36:56+08:00`; system clock synchronized, NTP active |
| Host / OS / compiler | `twg1`; Debian GNU/Linux 12 (bookworm), arm64/aarch64; Linux `6.12.68-v8-rt+`, PREEMPT_RT; GCC `(Debian 12.2.0-14+deb12u1) 12.2.0` |
| Branch | `bno-integrate` |
| Step 8.6 starting SHA | `0a27e5971b0ac94153e506dd2847055de8adf5b2` |
| Working-tree status before proof | Modified: `bno/readme.md`, `bno/tests/audit_scheduling.sh`; untracked: `bno/phase8-integration-results.md` |
| SH-2 submodule SHA | `b514b1e2586ddc195e553dac89fc94c637b25298`, path `bno/sh2`, describe `v1.3.0-10-gb514b1e`; no submodule mismatch prefix shown |
| Build commands | Executed sequentially: `make -C bno clean`, `make -C bno test`, `make -C bno app` |

## Host build proof

| Case | Command | Expected | Observed | Disposition |
|---|---|---|---|---|
| Clean host suite | `make -C bno clean`, then `make -C bno test` | All host tests pass | Clean removed `build` and `bin`; suite rebuilt and all 20 test executables passed; quad decoder reported 19/19 checks; `bash tests/audit_scheduling.sh` reported pass. Make exit codes not separately printed. | Pass |
| Application build | `make -C bno app` | `bin/bno_app` builds | Compilation and final link completed without reported errors; no standalone calibration/orientation objects in application link. Make exit code not separately printed. | Pass |
| CLI unit test | `./bno/bin/test_imu_cli` | Parser tests pass | `test_imu_cli: pass`; explicitly recorded `exit=0` | Pass |
| Console unit test | `./bno/bin/test_imu_console` | Console tests pass | `test_imu_console: pass`; explicitly recorded `exit=0` | Pass |
| Owner-loop unit test | `./bno/bin/test_app_owner_loop` | Owner-loop trace tests pass | `test_app_owner_loop: pass`; explicitly recorded `exit=0` | Pass |
| DCD-clear composition | `./bno/bin/test_imu_cmd_cal` | Host composition tests pass | `test_imu_cmd_cal: pass`; explicitly recorded `exit=0` | Pass |
| Check diagnostics | `./bno/bin/test_session_check` | Receipt diagnostics tests pass | `test_session_check: pass`; explicitly recorded `exit=0` | Pass |
| Diff whitespace | `git diff --check` | No output; zero status | No diagnostic output; explicitly recorded `diff-check exit=0` | Pass |

## No-hardware CLI matrix

Run these commands without `sudo`. A valid result must return before console
worker creation, HAL/session open, SPI/GPIO access, or real-time setup.

| Case | Command | Expected exit | Expected classification | Observed exit/output | Disposition |
|---|---|---:|---|---|---|
| Help | `bno/bin/bno_app --help` | 0 | Help; no hardware | Exit 0; usage and ordered-stage/clear-confirmation guidance printed | Pass |
| Help exclusive | `bno/bin/bno_app --help --duration 1` | 2 | Illegal combination | Exit 2; `main: CLI error=8 argvIndex=1` | Pass |
| Orphan mask | `bno/bin/bno_app --mask 0x00` | 2 | CLI error | Exit 2; `main: CLI error=8 argvIndex=1` | Pass |
| Missing mask value | `bno/bin/bno_app --check-imu --mask` | 2 | CLI error | Exit 2; `main: CLI error=4 argvIndex=2` | Pass |
| Oversize mask | `bno/bin/bno_app --check-imu --mask 0x100` | 2 | CLI error | Exit 2; `main: CLI error=7 argvIndex=3` | Pass |
| Signed mask | `bno/bin/bno_app --check-imu --mask -1` | 2 | CLI error | Exit 2; `main: CLI error=6 argvIndex=3` | Pass |
| Zero duration | `bno/bin/bno_app --duration 0` | 2 | CLI error | Exit 2; `main: CLI error=7 argvIndex=2` | Pass |
| Oversize duration | `bno/bin/bno_app --duration 3601` | 2 | CLI error | Exit 2; `main: CLI error=7 argvIndex=2` | Pass |
| Decimal duration | `bno/bin/bno_app --duration 10.5` | 2 | CLI error | Exit 2; `main: CLI error=6 argvIndex=2` | Pass |
| Illegal clear/full | `bno/bin/bno_app --cal-imu --tare-imu --clear --full` | 2 | CLI error | Exit 2; `main: CLI error=8 argvIndex=4` | Pass |
| Positional input | `bno/bin/bno_app unexpected-positional` | 2 | CLI error | Exit 2; `main: CLI error=3 argvIndex=1` | Pass |
 
All matrix commands were executed without sudo. Output contained only help
or parser-error diagnostics, with no HAL, SPI/GPIO, scheduling, or session
startup messages. This terminal capture is not a system-call trace; absence
of messages alone does not independently prove absence of device access.

`bno_app --cal-imu --tare-imu --clear` is a valid destructive two-stage plan.
Do not run it as a no-hardware matrix case. Its parser behavior is covered by
unit tests; any physical execution requires separate authorization.

## Ownership and link audit

Commands used to collect ownership and working-tree evidence:

```bash
grep -R -nE 'StartRT|RT_SleepUntil|usleep|nanosleep|clock_nanosleep' \
  bno/app rt

grep -R -nE 'exit[[:space:]]*\(' bno/app

grep -R -nE '\bsh2_' bno/app

grep -R -nE 'stdin|fgets|getchar|read[[:space:]]*\(' bno/app

git diff --check
git diff --stat 0a27e5971b0ac94153e506dd2847055de8adf5b2
git status --short
```

### Observed audit results

| Check | Observed evidence | Disposition |
|---|---|---|
| Automated scheduling/ownership audit | `make -C bno test` invoked `bash tests/audit_scheduling.sh`; output ended `audit_scheduling: pass` | Pass via suite |
| Standalone Bash invocation | `bash bno/tests/audit_scheduling.sh` reported `audit_scheduling: pass`; explicitly recorded exit 0. Earlier direct execution failed because the script was not executable. | Pass |
| Production scheduling owner | `main.c:683` calls `RT_SleepUntil`; `main.c:775` calls `StartRT`; other matches include declarations, comments, helper implementations, and RT tests | Pass |
| Allowed sleep implementations | `sh2_hal_rpi.c:100` contains the whitelisted `nanosleep`; `rt/realtime.c:162` contains `clock_nanosleep` | Pass |
| Process exit calls | No matches from `grep -R -nE 'exit[[:space:]]*\(' bno/app` | Pass for scanned pattern |
| Unified SH-2 ownership | Actual SH-2 API calls appear in `imu_session.c`; HAL supplies transport callbacks. Legacy `app_sensor.c` also contains calls but is absent from the application link. Header/comment matches are not calls. | Pass within unified-path scope |
| Console input ownership | `imu_console.c:132` performs worker `read`; no direct stdin read call shown in `main.c`. Matches for `stdint.h` are search noise. HAL `rpi_read` is transport, not console input. | Pass |
| Application link composition | APPOBJS contains main, session, coordinator, calibration/tare/check machines and adapters, CLI, and console; final application link includes SH-2/HAL/RT dependencies but no standalone oracle objects | Pass |
| Standalone object definitions | `CALOBJS` and `ORIENTOBJS` remain defined separately in the Makefile; neither appears in the final `bno_app` link | Pass |
| Whitespace | `git diff --check` produced no output; recorded exit 0 | Pass |

The audit's HAL whitelist reported RESET_LOW_US 10 ms, RESET_WAIT_US
120 ms, INT_POLL_STEP_US 500 us, and wake polling up to 200 ms.

At the end of this capture, HEAD remained
`0a27e5971b0ac94153e506dd2847055de8adf5b2`.
The tracked diff contained 209 added lines in `bno/readme.md` and 53 added
lines in `bno/tests/audit_scheduling.sh` (262 total).
`bno/phase8-integration-results.md` remained untracked and therefore was
not included in that diff stat.

Final captured working-tree status:

```text
 M bno/readme.md
 M bno/tests/audit_scheduling.sh
?? bno/phase8-integration-results.md
```

The README diff displayed a missing final newline. Whitespace checking
passed, but the final newline should be restored before checkpointing.

Physical validation remains outside this host/no-hardware record.
Leave the Step 8.7 physical rows Pending or Not authorized.

## Physical matrix

The following rows remain pending until Step 8.7 operator-coordinated
captures are collected. Do not infer a pass from host tests.

| Case | Command / setup | Required evidence | Disposition |
|---|---|---|---|
| Production smoke | `sudo bno/bin/bno_app --duration 2` | Settle; production reports; movement response; normal acquisition termination | Pass — `p8-evidence/10-production-smoke.txt`: settle R7 succeeded; acquisition epoch 1 had `validMask=0x07`; 200 observations; displayed orientation, linear acceleration, and gyro changed after motion; acquisition R7 succeeded; exit 0. `skippedDeadlines=6`. |
| Continuous CHECK | `sudo bno/bin/bno_app --check-imu --duration 2`, then `q` | All four report types, mask `0x00`, gate transitions, restore/settle/acquire | Pass for receipt/cancel/continuation — `p8-evidence/21-check-mask00-q.txt`: active R8 showed actual mask `0x00`, current-epoch `haveAccel=haveGyro=haveMag=haveRv=1`; `q` produced `CANCELLED / OPERATOR_Q`; restore, settle, and 200-observation acquisition succeeded; exit 0. Gate did not reach: accel 2, mag rose 0→1, gyro/RV 0. `skippedDeadlines=12`. |
| Probe 0x00 deadline | `sudo bno/bin/bno_app --check-imu --mask 0x00 --duration 2` | Full 10-second deadline, requested/effective/actual mask evidence, receipt facts, restore/settle/acquire | Pass for configuration/receipt/deadline/continuation — `p8-evidence/20-probe-mask00-deadline.txt`: requested/effective/actual masks all `0x00`; facts epoch 2 matched; all four `have*` flags reached 1; terminal result `TIMED_OUT / PROBE_DEADLINE`; `operatorEndedEarly=0`; restore, settle, and 200-observation acquisition succeeded; exit 0. Gate did not reach: accel 2, mag 1, gyro/RV 0. `skippedDeadlines=13`. |
| Probe 0x00 early q | `sudo bno/bin/bno_app --check-imu --mask 0x00 --duration 2`, then `q` before deadline | `CANCELLED / OPERATOR_Q`, `operatorEndedEarly=1`, no probe timeout, restore/settle/acquire | Pass — `p8-evidence/22-probe-mask00-early-q.txt`: cancelled with about 7.77 seconds remaining; `probeTimedOut=0`, `operatorEndedEarly=1`, `restoredProduction=1`; epoch 1→3; settle and 200-observation acquisition succeeded; acquisition `validMask=0x07`; exit 0; `skippedDeadlines=11`. |
| Probe 0x00 decode-rate diagnostics | Full mask-zero probe at `5410c552c62009abf89c3214a5ef21778a1f429f` | Retained eligible-epoch counts and first/latest host-decode timestamps for all four reports | Pass for diagnostic retention/output and lifecycle — mag 50.096 Hz, accel 16.119 Hz, gyro 10.021 Hz, and RV 10.021 Hz. Faster accel reporting is permitted by SH-2 rate-control semantics; exact requested-rate equality is not claimed. See `p8-evidence/25-probe-mask00-diagnostics.txt`. |
| Probe 0x05 | `sudo bno/bin/bno_app --check-imu --mask 0x05 --duration 2` | Readback/configuration and receipt evidence | Pass — `p8-evidence/28-probe-mask05-deadline.txt`: requested/effective/actual masks all `0x05`; all four receipt flags became true in epoch 2; full deadline produced `TIMED_OUT / PROBE_DEADLINE`; `dcdSaved=0`, `verified=0`, and `restoredProduction=1`; settle and 200-observation acquisition succeeded; exit 0; `skippedDeadlines=11`. Accel status 2, gyro status 0, mag status 1, RV status 0; readiness gate did not reach. |
| Probe 0x07 | `sudo bno/bin/bno_app --check-imu --mask 0x07 --duration 2` | Readback/configuration and receipt evidence | Pass — `p8-evidence/29-probe-mask07-deadline.txt`: requested/effective/actual masks all `0x07`; all four receipt flags became true in epoch 2; full deadline produced `TIMED_OUT / PROBE_DEADLINE`; `dcdSaved=0`, `verified=0`, and `restoredProduction=1`; settle and 200-observation acquisition succeeded; exit 0; `skippedDeadlines=12`. Accel status 2, gyro status 3, mag status 1, RV status 0; readiness gate did not reach. |
| Probe 0x80 | `sudo bno/bin/bno_app --check-imu --mask 0x80 --duration 2` | Record acceptance, mismatch, or rejection truthfully | Pass for failure/recovery evidence — `p8-evidence/30-probe-mask80-deadline.txt`: requested mask `0x80`; actual readback valid as `0x00`; terminal `FAILED / CONFIG_FAILED`, warning required, `doNotAcquire=0`, no report receipts in the brief diagnostic epoch before restoration, and `restoredProduction=1`. Settle and 200-observation acquisition then succeeded; exit 0; `skippedDeadlines=13`. |
| CHECK SIGINT | Ctrl-C during active `sudo bno/bin/bno_app --check-imu --duration 2` | Process-stop result distinct from `q`; no subsequent acquisition | Pass for observable stop behavior — `p8-evidence/23-check-sigint.txt`: `ABANDONED / PROCESS_STOP`, `doNotAcquire=1`; no subsequent settle/acquisition output; shutdown `observations=0`; exit 0; `skippedDeadlines=6`. Internal no-later-adapter-pump ordering remains supported by host tests, not inferred from terminal silence. |
| CHECK SIGTERM | `sudo kill -TERM 35774` during active CHECK | Process-stop result distinct from `q`; no subsequent acquisition | Pass for observable stop behavior — `p8-evidence/24-check-sigterm.txt`: PID 35774 was the actual `bno_app` process; `ABANDONED / PROCESS_STOP`, `doNotAcquire=1`; no subsequent settle/acquisition output; shutdown `observations=0`; exit 0; `skippedDeadlines=5`. Internal no-later-adapter-pump ordering remains supported by host tests. |
| Same-session calibration | Authorized `sudo bno/bin/bno_app --cal-imu --duration 10` | DCD-save outcome, same-session verification, restore, settle, acquisition; no reboot-persistence claim | Pass — `p8-evidence/31-cal-same-session.txt`: calibration `SUCCEEDED / OK`; `dcdSaved=1`, `verified=1`, `restoredProduction=1`, `doNotAcquire=0`; epoch 1→4; no rendered reset phase; settle succeeded; epoch-4 acquisition completed 1,000 observations over about 10 seconds with `validMask=0x07`; exit 0; `skippedDeadlines=31`. |
| Combined DCD clear then tare clear | Authorized `sudo bno/bin/bno_app --cal-imu --tare-imu --clear --duration 2` invocation; two fresh `CLEAR` confirmations | Ordered DCD-clear reset/recovery, independent tare-clear confirmation, truthful terminal results, restore/settle/post-clear diagnostic acquisition | Pass — `p8-evidence/36-dual-clear.txt`: DCD clear succeeded and restored production (epoch 1→2); a later separate tare-clear prompt accepted the second fresh confirmation and succeeded (`clearActive=1`, `clearSaved=1`, epoch 2→5); post-clear diagnostic acquisition completed 200 observations and exited 0. `skippedDeadlines=354` is an accepted bounded destructive-only DCD-reset/recovery exception; see timing disposition below. |
| Z-axis persistent tare | Authorized `sudo bno/bin/bno_app --tare-imu --duration 10` invocation | Separate tare-now/persist outcomes, same-session verification, restore, settle, acquisition | Pass — `p8-evidence/32-tare-z-persistent.txt`: `SUCCEEDED / OK`; `tareNow=1`, `persist=1`, `verified=1`, `verificationEvidence=1`, `restoredProduction=1`; no clear operation; epoch 1→4; heading re-referenced near zero while tilt remained substantially unchanged; settle and 1,000-observation acquisition succeeded; exit 0; `skippedDeadlines=13`. |
| Combined calibrated full-axis tare | Authorized `sudo bno/bin/bno_app --cal-imu --tare-imu --full --duration 10` invocation | Guided calibration, best-effort DCD save, same-session verification, full XYZ tare-now/persist/verify, restore/settle/acquisition | Pass for combined behavior — `p8-evidence/37-cal-full-tare.txt`: CALIBRATION and TARE both `SUCCEEDED / OK`; calibration `dcdSaved=1`; tare `tareNow=1`, `persist=1`, `requestedAxes=2`, `verificationEvidence=1`; both verified and restored production; epoch 1→4→7; settle and 1,000-observation acquisition succeeded; exit 0; `skippedDeadlines=34`. Retain the separate build/provenance record. |
| Integrated tare-check | `sudo bno/bin/bno_app --tare-imu --check --duration 2`, then `q` | Live epoch-matched attitude, motion response, no tare mutation, cancellation, restore/settle/acquire | Pass — `p8-evidence/33-tare-check-q.txt`: valid epoch-matched attitude appeared; intentional motion changed displayed yaw/pitch/roll; `q` produced `CANCELLED / OPERATOR_Q`; `tareNow=0`, `persist=0`, `clearActive=0`, and `clearSaved=0`; production restored; settle and 200-observation acquisition succeeded; exit 0; `skippedDeadlines=12`. |
| Standalone calibration-check comparison | Sequential `bno_app` evidence versus `sudo bno/bin/bno_cal --check` | Report/status observations and intentional lifecycle/policy differences; no simultaneous sensor owner | Pass for comparison evidence — `p8-evidence/34-oracle-cal-check.txt`: standalone check applied mask `0x00`, reported accel 2, gyro 0, mag 0, RV 0, RV heading error 3.14 rad, and `NOT CALIBRATED`; exited 3 after Ctrl-C. This is not a contradiction of same-session integrated calibration success and supports the daily recalibration-after-new-session policy. |
| Standalone orientation-check comparison | Sequential `bno_app` tare evidence versus `sudo bno/bin/bno_orient --check` | Attitude observations and intentional lifecycle/policy differences; no new tare/persistence/clear action | Pass for comparison evidence — `p8-evidence/35-oracle-orient-check.txt`: standalone orientation check printed stable live quaternion/Euler data with RV accuracy 3/High and exited 0 after Ctrl-C. It did not execute tare-now, persistence, or clear. Exact Euler equality with the prior integrated tare run was not expected because the checks were separate processes/times/physical poses. |
| CHECK/PROBE readiness transitions | Sustained gate pass; current verdict and sticky gate history distinguished | Host-machine proof plus physical observations | Not reproduced physically — mask `0x00`, `0x05`, and `0x07` probes truthfully did not reach the CHECK/PROBE gate because mag status remained 1. Host tests cover gate timer/sticky-history behavior. The separate guided-calibration machine physically reached its own mag/hold/verification gates, but that is not relabelled as a CHECK/PROBE sticky-gate pass. |

### Physical-capture limitations and observations

- The earlier CHECK/PROBE R8 captures establish receipt at least once in the
  active epoch, not report cadence. The later retained `checkReceipt`
  diagnostics expose counts and first/latest host-decode timestamps for
  approximate host-decode-rate assessment. Neither output proves continuous
  freshness, synchronization, sensor sampling jitter, or report-loss rate.
- The mask-zero readiness gate did not pass in either capture. This was
  expected from the displayed facts: magnetometer status reached only 1, below
  the gate threshold of 2. This is recorded as a truthful non-pass, not a
  report-receipt failure.
- The terminal CHECK summary's `actualMaskValid=0` comes from the
  probe-specific result subfield. `imu_check.c` populates
  `sub.probeMaskActualValid` only for PROBE; CHECK readback remains in
  `terminalProgress.check`. It is not evidence of failed live readback or
  restoration clearing the machine's readback.
- `skippedDeadlines` ranged from 5 to 13 across these captures. No acceptance
  threshold or mapping from this owner-loop counter to sensor-report loss has
  been defined; values are recorded without claiming data loss.
- Early probe cancellation preserved live mask-zero readback and matching
  epoch-2 receipt facts. All four report types were observed; accel status was
  2, mag status 1, and gyro/RV status 0. Neither current nor historical
  readiness gate passed.
- SIGINT and SIGTERM produced `ABANDONED / PROCESS_STOP`, not
  `CANCELLED / OPERATOR_Q`. Both inhibited acquisition and exited normally.
  `restoredProduction=0` records that these process-stop paths did not
  complete normal production restoration; it is not an ordinary
  restoration-failure result.
- Signal captures do not measure signal-to-shutdown latency because the
  signal-send time was not independently recorded.
- The SIGTERM command targeted PID 35774, the executable itself, rather than
  the `script`, shell, or `sudo` wrapper processes.

### Retained evidence index

| File | Start/end time | Command | Result |
|---|---|---|---|
| `p8-evidence/00-environment.txt` | 2026-09-30T17:56:23+08:00 | Environment/build inventory | Captured |
| `p8-evidence/10-production-smoke.txt` | 2026-09-30T17:57:34+08:00 to 17:57:37+08:00 | `sudo bno/bin/bno_app --duration 2` | Exit 0 |
| `p8-evidence/20-probe-mask00-deadline.txt` | 2026-09-30T17:57:58+08:00 to 17:58:11+08:00 | `sudo bno/bin/bno_app --check-imu --mask 0x00 --duration 2` | Exit 0; probe deadline |
| `p8-evidence/21-check-mask00-q.txt` | 2026-09-30T17:58:22+08:00 to 17:58:43+08:00 | `sudo bno/bin/bno_app --check-imu --duration 2`, then `q` | Exit 0; CHECK cancelled |
| `p8-evidence/22-probe-mask00-early-q.txt` | 2026-09-30T18:09:58+08:00 to 18:10:03+08:00 | `sudo bno/bin/bno_app --check-imu --mask 0x00 --duration 2`, then early `q` | Exit 0; probe cancelled; restore/settle/acquisition completed |
| `p8-evidence/23-check-sigint.txt` | 2026-09-30T18:11:11+08:00 to 18:11:16+08:00 | `sudo bno/bin/bno_app --check-imu --duration 2`, then Ctrl-C | Exit 0; process stop; no acquisition |
| `p8-evidence/24-check-sigterm.txt` | 2026-09-30T18:14:09+08:00 to 18:14:18+08:00 | Active CHECK; `sudo kill -TERM 35774` from second terminal | Exit 0; process stop; no acquisition |
| `p8-evidence/24-check-sigterm-control.txt` | Not independently timestamped | Copied second-terminal PID listing and `sudo kill -TERM 35774` command | Identifies signal and target executable |
| `p8-evidence/25-probe-mask00-diagnostics-build.txt` | 2026-09-30T18:36:09+08:00 | Source SHA, worktree status, executable hash | Source `5410c552c62009abf89c3214a5ef21778a1f429f`; only the newly created build-record file was untracked |
| `p8-evidence/25-probe-mask00-diagnostics.txt` | 2026-09-30T18:37:43+08:00 to 18:37:58+08:00 | `sudo bno/bin/bno_app --check-imu --mask 0x00 --duration 2` | Exit 0; deadline, retained diagnostics, restoration, settle, and acquisition completed |

### Decode-rate capture 25

Implementation SHA: `5410c552c62009abf89c3214a5ef21778a1f429f`.

Executable SHA-256:
`89d22d6f107bd766ab47b0a977956299d32c918658d67df21853a5482684e524`.

The retained snapshot was eligible PROBE epoch 2. Each report had
`hostTimesValid=1`. Rates below use `(epochDecodes - 1)` divided by the
first-to-latest host-decode span.

| Report | Requested Hz | Epoch decodes | Host span seconds | Estimated host-decode Hz | Assessment |
|---|---:|---:|---:|---:|---|
| Mag | 50 | 488 | 9.721241 | 50.096 | Consistent with requested average rate |
| Accel | 10 | 158 | 9.740180 | 16.119 | Faster than requested; permitted by SH-2 |
| Gyro | 10 | 97 | 9.580151 | 10.021 | Consistent with requested average rate |
| RV | 10 | 97 | 9.580171 | 10.021 | Consistent with requested average rate |

`requestedHz` is application-request evidence, not device feature-report
readback. Calibration-mask readback does not establish report intervals.
SH-2 permits reports at the requested interval or more frequently; this
capture alone does not establish the cause of the accelerometer excess.

The probe timed out normally with no early operator cancellation.
Production restoration, settle, and 200-observation acquisition succeeded.
Exit status was 0; `skippedDeadlines=11`. Readiness did not pass:
accel status 2, mag status 1, gyro/RV status 0.

### Decode-rate follow-up captures 26 and 27

Capture 26 repeated the full mask-zero PROBE. It terminated as
`TIMED_OUT / PROBE_DEADLINE`, with `operatorEndedEarly=0` and
`restoredProduction=1`. Settle and the 200-observation acquisition succeeded.
Exit status was 0; `skippedDeadlines=12`.

Capture 27 used continuous mask-zero CHECK, then operator `q`. The
CHECK command elapsed approximately 33.301 seconds through its restored
terminal result, despite the filename's nominal "30s" label. It terminated
as `CANCELLED / OPERATOR_Q`, with `restoredProduction=1`. Settle and the
200-observation acquisition succeeded. Exit status was 0;
`skippedDeadlines=11`.

Both retained eligible epoch-2 diagnostics with valid host timestamps.
Rates use `(epochDecodes - 1)` divided by the first-to-latest host-decode span.

| Capture | Report | Requested Hz | Epoch decodes | Host span seconds | Estimated host-decode Hz |
|---|---|---:|---:|---:|---:|
| 26 PROBE | Mag | 50 | 488 | 9.723121 | 50.087 |
| 26 PROBE | Accel | 10 | 158 | 9.742119 | 16.116 |
| 26 PROBE | Gyro | 10 | 97 | 9.583123 | 10.018 |
| 26 PROBE | RV | 10 | 97 | 9.583137 | 10.018 |
| 27 CHECK | Mag | 50 | 1656 | 33.017014 | 50.126 |
| 27 CHECK | Accel | 10 | 534 | 33.034996 | 16.134 |
| 27 CHECK | Gyro | 10 | 331 | 32.915031 | 10.026 |
| 27 CHECK | RV | 10 | 331 | 32.915025 | 10.026 |

Assessment:

- Mag, gyro, and RV average host-decode rates remain close to their
  requested rates across captures 25, 26, and 27.
- Accel consistently averages approximately 16.12–16.13 Hz despite
  the application's 10 Hz request, including the longer CHECK window.
  A fixed short startup burst alone is not an adequate explanation.
- No requested-rate, readiness-gate, or counter-filtering change is
  justified by these captures alone.
- Readiness did not pass: accel status 2, mag status 1, and gyro/RV
  status 0 remained the reported verdict.
- The next diagnostic is device feature-configuration readback for the
  enabled report types; report-sequence evidence may be needed afterward.

Additional retained evidence:

| File | Start/end time | Command | Result |
|---|---|---|---|
| `p8-evidence/26-probe-mask00-diagnostics-repeat.txt` | 2026-09-30T18:49:27+08:00 to 18:49:40+08:00 | `sudo bno/bin/bno_app --check-imu --mask 0x00 --duration 2` | Exit 0; repeated full probe; accel excess reproduced |
| `p8-evidence/27-check-mask00-diagnostics-30s.txt` | 2026-09-30T18:52:31+08:00 to 18:53:07+08:00 | `sudo bno/bin/bno_app --check-imu --duration 2`, then `q` | Exit 0; longer CHECK; accel excess persisted |
| `p8-evidence/28-probe-mask05-deadline.txt` | 2026-09-30T19:10:24+08:00 to 19:10:36+08:00 | `sudo bno/bin/bno_app --check-imu --mask 0x05 --duration 2` | Exit 0; exact `0x05` readback; full deadline; restore/settle/acquisition |
| `p8-evidence/29-probe-mask07-deadline.txt` | 2026-09-30T19:11:12+08:00 to 19:11:25+08:00 | `sudo bno/bin/bno_app --check-imu --mask 0x07 --duration 2` | Exit 0; exact `0x07` readback; full deadline; restore/settle/acquisition |
| `p8-evidence/30-probe-mask80-deadline.txt` | 2026-09-30T19:16:28+08:00 to 19:16:31+08:00 | `sudo bno/bin/bno_app --check-imu --mask 0x80 --duration 2` | Exit 0; invalid/mismatched mask configuration failed; production restored; settle/acquisition continued |
| `p8-evidence/31-cal-same-session-build.txt` | 2026-09-30T19:24:56+08:00 | Build/provenance record | Source `2d231540f2787c77651a548145a60c8dcb1b4400`; boot ID and uptime recorded; executable SHA-256 recorded; worktree had documentation/evidence changes only |
| `p8-evidence/31-cal-same-session.txt` | 2026-09-30T19:25:29+08:00 to 19:27:17+08:00 | `sudo bno/bin/bno_app --cal-imu --duration 10` | Authorized guided calibration; DCD save and same-session verification succeeded; restore/settle/acquisition completed; exit 0 |

Provenance note: capture 27's monotonic timestamps have a substantially
different origin from capture 26. Do not compare absolute monotonic
timestamps across these runs. Record whether the host rebooted or another
clock/environment change occurred, and confirm the source SHA and executable
hash used for capture 27.

### Report-rate acceptance interpretation

SH-2 Reference Manual v1.9, section 5.4.1 permits reports at the requested
interval or more frequently. Exact equality between requested and observed
report rates is therefore not an acceptance requirement for CHECK/PROBE.

Captures 25, 26, and 27 consistently showed average host-decode rates of
approximately 50 Hz for magnetic field, 16.12–16.13 Hz for accelerometer,
and 10 Hz for calibrated gyro and rotation vector.

The faster accelerometer output is compatible with the documented
rate-control semantics and is not treated as an implementation defect or
Phase 8 blocker. Its specific cause was not established. Device feature
readback and report-sequence investigation remain optional characterization,
not required corrective work.

Acceptance is limited to report receipt and approximate average host-decode
throughput. These captures do not prove exact sensor sampling cadence,
per-report freshness, absence of duplicates, timing jitter, or loss rate.
Readiness-gate acceptance remains separate and was not achieved in the
captured `0x00`, `0x05`, or `0x07` diagnostic probes.

### Nonzero-mask probe observations

The mask `0x05` probe enabled the selected accelerometer/magnetometer
dynamic-calibration policy, while `0x07` also enabled gyro calibration.
Both masks were read back exactly by the session owner.

Both probes retained the same report receipt shape seen in mask-zero probes:
488 magnetic-field decodes, 158 accelerometer decodes, and 97 each of
calibrated-gyro and rotation-vector decodes over approximately ten seconds.
This is receipt and approximate host-decode-throughput evidence only.

The visible gyro-status difference is consistent with the selected policy:

| Mask | Accel status | Gyro status | Mag status | RV status | Gate reached |
|---|---:|---:|---:|---:|---:|
| `0x05` | 2 | 0 | 1 | 0 | No |
| `0x07` | 2 | 3 | 1 | 0 | No |

Neither probe ran guided calibration, DCD save, DCD clear, tare, or tare
clear. Both terminal R7 records truthfully reported `dcdSaved=0` and
`verified=0`. Their successful restore/settle/acquisition lifecycle does not
claim calibration readiness or DCD persistence.

### Invalid-mask probe observation

The `0x80` probe intentionally tested an unsupported/high-bit policy request.
The application accepted the CLI mask syntax and attempted configuration, but
the observed actual mask was `0x00`, not `0x80`. It therefore reported:

```text
identity=PROBE
state=FAILED
reason=CONFIG_FAILED
warningRequired=1
doNotAcquire=0
restoredProduction=1
```

The requested policy did not pass configuration validation. The retained
diagnostic snapshot nevertheless identified eligible epoch 2, with zero
decodes and invalid host timestamps before production restoration.
This establishes no observed receipts in that brief epoch; it does not
establish report loss or successful activation of the requested `0x80` policy.

Because production restoration succeeded, the coordinator performed the
ordinary 300 ms settle and subsequent two-second acquisition. This establishes
the recoverable configuration-failure path; it does not establish acceptance
or meaning of bit `0x80`.

### Combined DCD-clear and tare-clear capture

One combined destructive invocation was authorized to validate the ordered
two-stage clear plan:

```text
sudo bno/bin/bno_app --cal-imu --tare-imu --clear --duration 2
```

The operator entered `CLEAR` plus Enter at the DCD-clear prompt, waited for
the DCD-clear terminal result and later TARE_CLEAR prompt, then entered a
second fresh `CLEAR` plus Enter.

The DCD-clear result was:

```text
identity=DCD_CLEAR
state=SUCCEEDED
reason=OK
doNotAcquire=0
epochBefore=1
epochAfter=2
restoredProduction=1
```

The subsequent tare-clear result was:

```text
identity=TARE_CLEAR
state=SUCCEEDED
reason=OK
doNotAcquire=0
epochBefore=2
epochAfter=5
restoredProduction=1
clearActive=1
clearSaved=1
tareNow=0
persist=0
```

The application then performed normal settle and a two-second epoch-5
acquisition with `validMask=0x07` and 200 observations. The renderer
explicitly labelled that output as post-clear diagnostic data; it is not
evidence of calibration or tare readiness. The process exited 0.

The capture reported `skippedDeadlines=354`, far above the 6–31 range in
prior normal, probe, calibration, and tare captures. The subsequent
source/contract audit classified this as the accepted destructive-only
DCD-reset/recovery exception described below. No sensor-report loss is
claimed.

### DCD-clear timing disposition

The source and scheduling audit resolved the elevated dual-clear timing count.
`skippedDeadlines` is accumulated only when the sole `main.c` call to
`RT_SleepUntil()` reports one or more missed absolute 1 ms deadlines.
It is not a report-decode or sensor-data-loss counter.

The destructive DCD-clear adapter action synchronously invokes:

```text
imu_session_clear_dcd()
→ delete FRS DCD record
→ sh2_clearDcdAndReset()
→ imu_session_begin_recovery()
```

The SH-2 DCD-clear command intentionally clears RAM DCD and resets the hub.
The HAL reset/recovery boundary is the scheduler-audit exception: its
whitelisted timing includes 10 ms reset-low, 120 ms reset wait, and wake
polling up to 200 ms. This roughly bounded reset/recovery interval, plus
command/reopen overhead, is consistent with the observed
`skippedDeadlines=354`.

This is accepted as a destructive-only reset/recovery exception. It does not
apply to ordinary acquisition, CHECK/PROBE, same-session calibration, or
same-session tare flows. The physical result recovered, restored production,
settled, and completed post-clear diagnostic acquisition. No sensor-report
loss is claimed from this counter.

Capture 36 establishes the ordered clear behavior only. Its post-clear device
state was later superseded by the authorized calibration/full-axis tare capture
in `37-cal-full-tare.txt`.


The source audit also confirmed that reusable command/session/adapter modules
do not call `RT_SleepUntil`, `StartRT`, `usleep`, or `nanosleep`; the ordinary
owner loop remains singular in `main.c`.

Additional retained evidence:

| File | Start/end time | Command | Result |
|---|---|---|---|
| `p8-evidence/36-dual-clear-build.txt` | 2026-10-01T12:34:09+08:00 | Source, boot, worktree, executable hash, authorization | Source `62d9d70c215adcdbdb1992b960dd46825e94fb1e`; explicit dual-clear authorization recorded |
| `p8-evidence/36-dual-clear.txt` | 2026-10-01T12:34:26+08:00 to 12:34:39+08:00 | Combined DCD clear then tare clear, each separately confirmed | Exit 0; both clear stages succeeded; production restored; post-clear acquisition completed |

### Same-session calibration capture

One guided calibration invocation was expressly authorized with best-effort
DCD save and without DCD clear, tare, or tare clear.

The calibration capture showed the rendered progression:

```text
startup → accel faces → gyro rest → mag motion → hold → save
→ same-session verification → production restoration
→ 300 ms settle → 10-second acquisition
```

Rendered phases included 1, 2, 3, 4, 5, 6, 8, and 9. No rendered reset
phase 7 occurred. This physically supports the Step 8.a same-session policy;
host tests remain the primary proof that the integrated path does not request
a verification reopen.

The terminal calibration result was:

```text
state=SUCCEEDED
reason=OK
warningRequired=0
doNotAcquire=0
epochBefore=1
epochAfter=4
restoredProduction=1
dcdSaved=1
verified=1
```

The DCD save outcome is a truthful best-effort device-copy result. It is not
evidence that calibration survives reset, process restart, or power loss.

During the accepted mag/hold/save/verification portions of this capture,
reported status values reached accel 3, gyro 3, mag 2, and RV 3. The
subsequent epoch-4 acquisition showed `validMask=0x07` and 1,000 host
observations over approximately 10.001 seconds. Printed acquisition
snapshots were stationary-looking, but do not establish ground-truth
orientation accuracy, per-frame freshness, or reboot persistence.

`skippedDeadlines=31` is recorded without treating it as sensor-data loss or
a failure: this was a long interactive calibration with repeated diagnostic
rendering, and no acceptance threshold has been defined.

### Z-axis persistent-tare capture

One Z-axis persistent-tare invocation was authorized without full tare,
tare clear, or DCD clear.

Source checkpoint:
`3cc9cab1c46c9dd1c56877ffeb64f8f75351ed92`.

Executable SHA-256:
`89d22d6f107bd766ab47b0a977956299d32c918658d67df21853a5482684e524`.

Boot ID:
`9dfd53f1-60b9-47cc-a8fb-148285dc1ddd`.

The pre-run status listed only the newly created build-record file as
untracked. No tracked implementation modifications were shown.

The operator's `y` confirmation advanced the machine through report
configuration, settle, tare-now, persistence, same-session verification,
and production restoration.

The terminal TARE result reported:

```text
state=SUCCEEDED
reason=OK
warningRequired=0
doNotAcquire=0
epochBefore=1
epochAfter=4
restoredProduction=1
verified=1
tareNow=1
persist=1
clearActive=0
clearSaved=0
requestedAxes=1
verificationEvidence=1
```

The last displayed pre-tare attitude and verified post-tare attitude were:

| Component | Pre-tare radians | Verified post-tare radians |
|---|---:|---:|
| Yaw | -2.1980 | 0.0001 |
| Pitch | -0.2937 | -0.2936 |
| Roll | 0.1064 | 0.1060 |

This supports the intended heading-only re-reference without intentional
pitch/roll leveling. The subsequent epoch-4 acquisition completed 1,000
observations over the ten-second window with `validMask=0x07`.
Printed yaw remained near zero at the renderer's three-decimal precision.
Exit status was 0; `skippedDeadlines=13`.

Acceptance covers tare-now, the observed persistence-action outcome,
same-session verification, and production continuation. It does not prove
reload across reboot, ground-truth orientation accuracy, calibration
readiness, or per-observation report freshness.

This run used a different host boot from calibration capture 31. It is
therefore not evidence of a continuous same-process calibration→tare plan.

Additional retained evidence:

| File | Start/end time | Command | Result |
|---|---|---|---|
| `p8-evidence/32-tare-z-build.txt` | 2026-10-01T11:54:59+08:00 | Source, boot, worktree, executable hash, authorization | Recorded |
| `p8-evidence/32-tare-z-persistent.txt` | 2026-10-01T11:55:20+08:00 to 11:55:41+08:00 | `sudo bno/bin/bno_app --tare-imu --duration 10` | Exit 0; Z tare-now/persist/verification and production continuation succeeded |
| `p8-evidence/33-tare-check-build.txt` | 2026-10-01T12:12:41+08:00 | Source, boot, worktree, executable hash | Source `108438dbad561d89c7bd5469109c543fb53fbb0e`; same boot as capture 32; only the new build-record file was untracked |
| `p8-evidence/33-tare-check-q.txt` | 2026-10-01T12:14:16+08:00 to 12:14:41+08:00 | `sudo bno/bin/bno_app --tare-imu --check --duration 2`, then `q` | Exit 0; read-only tare-check motion/cancellation/restoration evidence |

### Integrated tare-check capture

The integrated `TARE_CHECK` mode started with invalid attitude placeholders,
then obtained valid epoch-matched attitude in the active check epoch.
Intentional motion changed the displayed orientation materially:

```text
initial stable yaw approximately +0.084 rad
observed yaw excursion approximately -1.097 rad
returned-check yaw approximately -0.039 rad
```

Pitch and roll also changed with the physical motion. This establishes live
attitude response; it does not establish heading lock, zero drift, or
ground-truth orientation accuracy.

`q` caused `CANCELLED / OPERATOR_Q` and did not perform a new tare mutation:

```text
tareNow=0
persist=0
clearActive=0
clearSaved=0
```

Production restoration, normal settle, and two-second acquisition then
completed. The test ran in the same host boot as capture 32 but in a separate
process, so it is not a continuous session with the prior tare-now/persist
operation.

### Combined calibration and full-axis tare

One combined invocation was authorized:

```text
sudo bno/bin/bno_app --cal-imu --tare-imu --full --duration 10
```

The capture demonstrated the ordered single-process flow:

```text
guided calibration → DCD save → same-session verification
→ production restoration → full-tare confirmation/configuration
→ tare-now → persistence → same-session tare verification
→ production restoration → settle → acquisition
```

CALIBRATION completed as `SUCCEEDED / OK`, with `dcdSaved=1`,
`verified=1`, and `restoredProduction=1`, taking epoch 1→4.
No calibration RESET phase was rendered.

TARE completed as `SUCCEEDED / OK`, with `tareNow=1`, `persist=1`,
`requestedAxes=2`, `verificationEvidence=1`, `verified=1`, and
`restoredProduction=1`, taking epoch 4→7. The application reports
the full-axis selection as 2; this is not the raw sensor XYZ bitmap.
No clear operation occurred.

| Component | Last displayed pre-tare radians | Verified post-tare radians |
|---|---:|---:|
| Yaw | 2.8686 | -0.0002 |
| Pitch | 0.0048 | -0.0004 |
| Roll | -0.0018 | -0.0004 |

Pitch and roll were already near zero before tare. This run establishes
full-axis selection and successful execution, but does not strongly
characterize correction of a large mounting tilt.

Mag status intermittently fell to 0 during hold and verification motion,
then recovered. The final sustained calibration-verification gate completed
with accel 3, gyro 3, mag 2, and RV 3. Continuous high confidence throughout
the entire procedure is not claimed.

Production settle completed in approximately 301 ms. Epoch-7 acquisition
completed 1,000 observations over approximately 10.001 seconds.
Printed yaw ranged from 0.000 to 0.005 rad; printed pitch/roll remained
within approximately 0.001 rad of zero. These are rounded console
snapshots, not full-rate ground-truth or freshness evidence.

Exit status was 0; `skippedDeadlines=34`.
This is the final captured Phase 8 device state and supersedes capture 36's
cleared state: same-session calibration completed with `dcdSaved=1`, and
full-axis tare persistence succeeded in capture 37. Reload after process
restart, BNO085 reset, or power cycle was not tested and is not claimed.


| File | Start/end time | Command | Result |
|---|---|---|---|
| `p8-evidence/37-cal-full-tare.txt` | 2026-10-01T13:00:31+08:00 to script close at 13:02:25+08:00 | Combined calibration and full-axis persistent tare | Exit 0; both stages and acquisition succeeded |

Retain and index `p8-evidence/37-cal-full-tare-build.txt` using its actual
timestamp, source SHA, boot ID, worktree status, and executable hash.
Those values are not embedded in the terminal capture.

## Evidence integrity

Create `bno/p8-evidence/` only when retaining a genuine reviewed capture.
Every retained file must be indexed here with command, environment, start/end
time, expected behavior, observed behavior, limitation, and disposition.

This results document and `bno/p8-evidence/` are temporary Phase 8 artifacts.
Phase 12/13 is expected to distill accepted durable behavior into
`bno/readme.md` and remove this document and evidence directory unless a later
requirement explicitly promotes an artifact.
