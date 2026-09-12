# MS5837-02BA Pressure & Temperature Subsystem (`ms5`)

Standalone driver, real-time acquisition tests, and diagnostics for the TE Connectivity MS5837-02BA barometric pressure and temperature sensor on Raspberry Pi 4B (Raspberry Pi OS with PREEMPT_RT).

---

## 1. Physical Hardware Pinout

The MS5837-02BA communicates via the primary Raspberry Pi I²C bus (`/dev/i2c-1`). It operates without conflict alongside the existing SPI0-based BNO085 IMU wiring.

| Breakout Pin | Raspberry Pi 4B Connection | Header Pin | Notes |
|---|---|---:|---|
| **VCC / VIN** | 3.3V Power Rail | **Pin 1 or 17** | Direct 3.3V supply (shared with BNO085) |
| **GND** | Ground | **Pin 6, 9, or 14** | Common ground reference |
| **SDA** | GPIO2 (I2C1_SDA) | **Pin 3** | 3.3V logic level with internal/breakout pull-up |
| **SCL** | GPIO3 (I2C1_SCL) | **Pin 5** | 3.3V logic level with internal/breakout pull-up |

> **Warning:** Do not connect SDA or SCL to a 5V rail or external 5V pull-ups. The Raspberry Pi 4B GPIO lines are not 5V tolerant.

---

## 2. Linux I²C Verification

Before running compiled binaries, verify that the Linux I²C peripheral is enabled and detecting the device at slave address `0x76`:

```bash
# 1. Check if the i2c-dev driver is loaded
ls -l /dev/i2c*

# 2. Probe bus 1 for attached devices
i2cdetect -y 1
```

Expected `i2cdetect` output:
```text
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- 76 --                         
```

---

## 3. Building and Running Step 1 Bring-Up

From the `twg/ms5/` directory:

```bash
# Compile the raw I2C test
make clean && make

# Execute raw bring-up probe
./bin/test_i2c_raw
```

### Verification Checklist:
- [ ] Device responds with ACK to reset command `0x1E`.
- [ ] Calibration words `C1` through `C6` read back non-zero and non-`0xFFFF`.
- [ ] No bus timeouts or I/O errors reported in `dmesg`.