# IMU Glider Orientation Demo

Live orientation-tracking demo for the research presentation. A BNO085 IMU on the glider is read by a Raspberry Pi, and the orientation is streamed over Wi-Fi to a laptop. The laptop draws a procedural 3D glider that rotates in place to match the physical glider.

The Pi does the sensing and the laptop does the rendering. No video is streamed.

```
BNO085 -> Raspberry Pi (C backend) -> TCP over Wi-Fi -> Laptop (Python + tkinter)
```

This directory is independent of `integrate/`. It uses only the `bno` and `rt` components of this repository. It does not read the pressure sensor.

## Controls

Both keys work in the laptop window. Click the window first so it has keyboard focus.

| Key | Action |
|---|---|
| `t` | Temporary hardware Z-tare, then reset the display reference |
| `x` | Stop the Pi demo application and close the viewer |

Closing the window behaves like `x`. Neither key shuts down the Raspberry Pi itself.

## Directory layout

| Path | Purpose |
|---|---|
| `demo/rpi/main.c` | Lifecycle, sensor-owner loop, command dispatch, cleanup |
| `demo/rpi/demo_imu.c`, `.h` | BNO session wrapper, fresh-orientation acquisition, Z-tare sequence |
| `demo/rpi/demo_link.c`, `.h` | TCP server, latest-value orientation mailbox, command handling |
| `demo/rpi/Makefile` | Builds `demo/rpi/bin/imu_demo` |
| `demo/laptop/main.py` | Network worker, tkinter window, reference handling, keys, redraw loop |
| `demo/laptop/glider.py` | Procedural glider mesh, quaternion math, axis mapping, drawing |

## Requirements

### Raspberry Pi

- The BNO085 wired and working as in the rest of this repository.
- The same build dependencies as the other Pi applications here: a C compiler, `make`, `libgpiod` with its development headers, and pthreads.
- The SH-2 submodule checked out.

### Laptop

- Python 3 with tkinter. On Windows, the python.org installer includes Tk.
- No third-party Python packages.
- A dedicated Wi-Fi network shared by the laptop and the Pi.

## Setup

### Branch

```bash
git status --short            # must be clean first
git fetch origin
git switch --no-track -c imu-demo origin/main
git submodule update --init --recursive
git rev-parse HEAD            # record this hash
```

Verified revision: `551fbdf991a344134773735a88ace01936c9fea4`

### Build on the Pi

From the repository root:

```bash
git submodule update --init --recursive
make -C demo/rpi
```

To rebuild from scratch:

```bash
make -C demo/rpi clean && make -C demo/rpi
```

## Running the demo

### 1. Find the Pi's Wi-Fi address

On the Pi:

```bash
hostname -I
```

Use the address on the demo Wi-Fi network.

### 2. Start the Pi backend

From the repository root:

```bash
sudo ./demo/rpi/bin/imu_demo
```

Options:

| Option | Meaning |
|---|---|
| `--port N` | TCP port to listen on. Default is 8765. |
| `--rt` | Try SCHED_FIFO priority 30 and memory locking. The demo continues with a warning if this setup fails. |

The backend prints `Listening on TCP port 8765.` when it is ready. Whether `sudo` is required depends on your Pi's hardware permissions. Use whatever you use for the other IMU applications.

### 3. Hold the glider in the reference pose

**Reference pose:** nose pointing to the audience's screen-left, body parallel to the screen, top facing up.

The program assumes the glider is already in this pose at startup. The first usable orientation becomes the initial display reference. No hardware tare happens at startup.

### 4. Start the laptop viewer

From `demo/laptop`:

```powershell
python main.py <PI_WIFI_IP>
```

Fullscreen:

```powershell
python main.py <PI_WIFI_IP> --fullscreen
```

Viewer options:

| Option | Meaning |
|---|---|
| `host` | Pi IPv4 address or hostname (required) |
| `--port N` | Must match the Pi. Default is 8765. |
| `--fullscreen` | Fullscreen window |
| `--axes SPEC` | Sensor-to-model axis mapping. Default is `-x,-y,-z`. If the value starts with `-`, write `--axes=-x,-y,-z`. |
| `--azimuth DEG`, `--elevation DEG` | Fixed camera angles. Defaults are 20 and 25. |
| `--inverse-quaternion` | Diagnostic option that transposes incoming quaternion rotations. |

`--inverse-quaternion` changes the quaternion convention used by the viewer. Do not change `--axes` and `--inverse-quaternion` at the same time while diagnosing. Record which one the validation below required.

### 5. Re-reference during the demo

Hold the real glider in the reference pose and press `t` in the viewer. The viewer shows a taring message while the Pi works. When the tare finishes, the virtual glider returns to the reference pose and tracking resumes.

Repeated `t` presses while a tare is running are ignored.

### 6. Stop

Press `x` or close the window. The viewer asks the Pi application to exit, waits briefly for its acknowledgment, then closes. The Pi remains powered and running.

If the Pi does not acknowledge, the viewer prints a warning and the backend may still be running. Stop it with Ctrl+C in its terminal.

## What `t` actually does

1. The viewer sends a tare request with a unique ID.
2. The Pi stops publishing valid orientation and reconfigures the sensor for tare.
3. After a settling interval, it issues a temporary Z-axis tare (`IMU_SESSION_TARE_AXES_Z`). Z-tare changes heading without changing tilt.
4. It never calls persist-tare, so nothing is written to flash. The change is lost on a sensor reset.
5. It restores normal acquisition and waits for fresh data.
6. The Pi increments a reference counter. The viewer sees the change and captures the new orientation as its display baseline.

The display reset makes your held pose match the on-screen reference pose, including any small tilt in how you hold it. It is a display baseline and does not recalibrate or level the sensor.

The tare is not instantaneous. It has these built-in intervals, plus configuration and command time:

| Stage | Interval |
|---|---|
| Settle before tare | 2 s |
| Post-tare observation | 0.5 s |
| Restore settle | 0.3 s |

The README makes no end-to-end latency claim. Measure it on your hardware before presenting.

## Network setup

- The Pi listens on TCP port 8765 on all interfaces. The laptop connects outbound to the Pi.
- The protocol has no authentication or encryption. Use it only on your own isolated demo network.
- Many access points enable client isolation, which blocks laptop-to-Pi traffic. If the viewer cannot connect, check this first.
- Test reachability from the laptop before the demo:

```powershell
Test-NetConnection <PI_WIFI_IP> -Port 8765
```

This test needs the Pi backend already running.

- Use a fixed address or DHCP reservation for the Pi so the launch command does not change.
- Do not depend on the venue's network. Bring your own router or hotspot.

## Reconnection and stale data

| Condition | Viewer behavior |
|---|---|
| Not connected | Shows "Disconnected" and keeps retrying about every 0.75 s |
| No telemetry for more than 0.75 s | Holds the last pose and shows a stale message |
| IMU data older than 250 ms | Holds the last pose and shows a stale message |
| No telemetry for 2 s | Drops the connection and reconnects |
| IMU fault reported by the Pi | Shows the fault reason and holds the last pose |

Telemetry is a latest-value stream. If the viewer falls behind, older poses are skipped. A new connection starts a new session, so the viewer resets its display reference.

## Pre-demo validation checklist

Run this on the actual hardware, laptop, and network, using the same launch command as the presentation.

1. The Pi builds from a clean checkout of the branch.
2. The viewer connects and shows "Tracking" with the glider in the reference pose.
3. Rotation directions, each tested separately:
   - nose up and down
   - each wing down in turn
   - nose left and right
4. A combined rotation stays consistent with the real glider.
5. Repeat step 3 after pressing `t`.
6. Stop and restart both programs, then repeat steps 2 and 3. Results must be the same on every run.
7. Press `t` several times in a row.
8. Press `x` at different moments, including while a tare is running. The Pi should stay powered.
9. Disconnect and reconnect Wi-Fi while running. The viewer should recover without a restart.
10. If presenting fullscreen, confirm `t` and `x` work in that mode.
11. Run the whole sequence with no internet connection.

Record the result of each item before the presentation.

## Known limitations

- **Axis mapping needs verification.** The stated mounting (+X forward, +Y left, +Z down) is not a right-handed frame. The default `--axes=-x,-y,-z` is a reflected mapping, and the viewer prints a warning when it is used. In the viewer's rotation math this default mapping cancels out. If rotations behave wrongly, the cause may be the quaternion convention (`--inverse-quaternion`) or a real mounting mismatch. Resolve it by testing, and write the working configuration into the "Validated launch command" field above.
- **Wrong convention can look correct at rest.** The glider can look right in the reference pose while its pitch and roll axes are wrong. Test each rotation separately, as in the checklist.
- **Heading depends on the Rotation Vector.** It uses the magnetometer. Magnetic interference near the demo area can affect heading and make results differ between runs.
- **Tare failure is not fully handled.** A failed tare returns to normal tracking where possible, but sensor recovery is not guaranteed. An unrecoverable sensor fault leaves the viewer open with a warning and requires restarting the Pi backend.
- **Unused sensor reports.** The reused session also enables linear acceleration and calibrated gyro reports at 100 Hz. The demo does not use them.
- **Rendering is approximate.** The glider uses painter's-algorithm depth ordering with flat shading. Some overlapping faces may sort incorrectly at certain angles.
- **Single client.** The Pi accepts one laptop connection at a time.
- **Rotation only.** The glider turns in place. Moving the real glider sideways does not move the virtual one.
- **No measured latency.** No end-to-end motion-to-screen latency has been measured.

## Troubleshooting

| Symptom | What to check |
|---|---|
| Viewer stays on "Disconnected" | Pi backend running? Correct IP and port? Same Wi-Fi network? Client isolation off? |
| Build fails on missing SH-2 files | Run `git submodule update --init --recursive` |
| Build fails on `gpiod` | Install the libgpiod development package your Pi OS uses |
| Pi prints an IMU startup failure | Check wiring and power. Confirm the other IMU applications still run. |
| "Tare unavailable" | Wait for "Tracking" and fresh IMU data before pressing `t` |
| Keys do nothing | Click the window to give it focus |
| Port already in use | A previous backend may still be running. Stop it or pass `--port`. |
| Glider looks wrong but moves | Re-run the validation checklist. See the axis mapping limitation. |
| Window blurry on a high-DPI display | Fix DPI scaling in Windows and restart the viewer |

## Protocol reference

The link is one TCP connection carrying newline-delimited ASCII text.

### Pi to laptop

| Line | Meaning |
|---|---|
| `H 1 <session>` | Handshake with protocol version 1 and a session ID |
| `D 1 ...` | Orientation frame, sent about every 20 ms |
| `E <id>` | Tare request `<id>` was rejected |
| `BYE` | The application has cleaned up and is stopping |

The `D` frame carries these fields after the version:

```
serial reference epoch sequence age_ms valid phase tare_id tare_result error accuracy qw qx qy qz
```

### Laptop to Pi

| Line | Meaning |
|---|---|
| `T <id>` | Request a Z-tare with a non-zero unique ID |
| `X` | Request application exit |

### Phase codes

| Code | Phase |
|---|---|
| 0 | Starting |
| 1 | Tracking |
| 2 | Tare settle |
| 3 | Tare post-observation |
| 4 | Restoring |
| 5 | Fault |

### Tare result codes

| Code | Meaning |
|---|---|
| 0 | No tare requested |
| 1 | In progress |
| 2 | Completed |
| -1 | Failed |

### Error codes

| Code | Meaning |
|---|---|
| 0 | None |
| 1 | Startup failed |
| 2 | Session fault |
| 3 | Tare configuration failed |
| 4 | Tare command failed |
| 5 | No usable tare-stage orientation |
| 6 | Could not restore acquisition |
| 7 | No usable startup orientation |
| 8 | Sensor reset detected |

## Operating notes

- Start the Pi backend first, then the viewer.
- Use `x` for a normal shutdown instead of killing the processes.
- After a sensor fault or reset, restart the Pi backend.
- Do not run another application that uses the IMU at the same time.
- Keep a rehearsed fallback, such as a short screen recording of a working run, in case hardware or Wi-Fi fails during the presentation.
