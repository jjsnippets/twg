# MS5837 + BNO085 Dynamic Validation Capture

Combined 100 Hz IMU and pressure logger for the 4.5-minute (270 s) horizontal dynamic pressure protocol.

## Build

From `integrate/`:

```bash
make ms5_validation
```

This builds `integrate/bin/ms5_validation` using the shared BNO085, SH-2, MS5837, real-time, and logging modules in `integrate/`.

## Run Flow

```bash
sudo ./bin/ms5_validation
```

1. **Startup preparation:** Runs guided BNO085 calibration followed by an automatic persisted Z-axis tare (dynamic calibration disabled afterward), then settles into operational mode.
2. **Ready state:** Prints live pressure and linear acceleration every 2 s.
   - Press `s` to open a new CSV and start a 270 s trial.
   - Press `Ctrl+C` to exit cleanly.
3. **Recording (270 s):** Logs 100 Hz combined IMU + MS5837 rows via `twg_log`, prints phase banners on every transition, and prints 2 Hz status lines. When the 270 s window finishes, the CSV is closed and the program returns to the ready state so the next trial can be started with `s` without recalibrating.

## 4.5-Minute Protocol (9 Cycles × 30 s)

- Cycles 1–3: **SLOW**
- Cycles 4–6: **MODERATE**
- Cycles 7–9: **FAST**

Within each 30 s cycle:

| Segment | Window | CSV `stage_active` | Activity |
|---|---|---|---|
| a | 0–5 s | `REST_PRE` | Hold still (baseline, hands off) |
| b | 5–15 s | `PUSH` | Prescribed push (metronome-paced) |
| c | 15–20 s | `REST_POST` | Hold still (post-push rest, hands off) |
| d | 20–30 s | `RETURN` | Slow return to start |
