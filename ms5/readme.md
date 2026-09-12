# MS5837-02BA Pressure & Temperature Subsystem (`ms5`)

Standalone Linux userspace driver, real-time acquisition tests, and benchmarking harness for the TE Connectivity **MS5837-02BA** gel-filled pressure and temperature sensor on a Raspberry Pi 4B running **Raspberry Pi OS with PREEMPT_RT**.

This subsystem operates as an independent module alongside the existing SPI0-based `bno/` (BNO085 IMU) subsystem within `twg/`.

---

## 1. Hardware Interface & Pin Allocation

The MS5837-02BA communicates via the Raspberry Pi 4B's primary hardware I²C bus (`/dev/i2c-1`) at 7-bit slave address **`0x76`**. It requires no external chip-select, interrupt, or reset GPIO pins.

### Wiring Table
| MS5837 Breakout Pin | Raspberry Pi 4B Connection | Header Physical Pin | BCM GPIO | Notes |
|---|---|---:|---:|---|
| **VCC / VIN** | 3.3V Power Rail | **Pin 1 or 17** | — | Shared 3.3V rail (common with BNO085 VIN) |
| **GND** | Ground Rail | **Pin 6, 9, 14, 20, 25, 30, 34, 39** | — | Common system ground |
| **SDA** | I2C1_SDA | **Pin 3** | **GPIO2** | 3.3V logic level (onboard pull-ups) |
| **SCL** | I2C1_SCL | **Pin 5** | **GPIO3** | 3.3V logic level (onboard pull-ups) |

> **Electrical Warnings:**
> 1. Raspberry Pi 4B GPIO lines are **3.3V logic only** and are **not 5V tolerant**. Do not connect SDA or SCL pull-ups to 5V.
> 2. The MS5837 bare sensor requires a $100\text{ nF}$ to $470\text{ nF}$ ceramic decoupling capacitor between VDD and GND placed close to the device.
> 3. Bus separation: The BNO085 IMU uses SPI0 (Pins 19, 21, 22, 23, 29, 31, 33), while the MS5837 uses I²C1 (Pins 3 and 5). There are zero hardware bus conflicts.

---

## 2. Sensor Identification & Calibration Verification

During Step 1 and Step 2 bring-up, the sensor was probed and validated using `test_i2c_raw` and `test_crc_math`.

### PROM Mapping (112-bit Factory Calibration)
On soft-reset (`0x1E`), the internal factory calibration coefficients are latched into PROM words $C_0$ through $C_6$:

| Word | Name | Stored Hex | Raw Dec | Description |
|:---:|:---:|:---:|:---:|---|
| **0** | `CRC / Version` | `0x4BA1` | — | Bits [15:12] = CRC nibble (`0x4`); Bits [11:5] = `0b0010101` (**MS5837-02BA21 Shielded**) |
| **1** | `C1` | `0xBA4A` | 47690 | Pressure sensitivity ($SENS_{T1}$) |
| **2** | `C2` | `0xB779` | 46969 | Pressure offset ($OFF_{T1}$) |
| **3** | `C3` | `0x7395` | 29589 | Temperature coefficient of pressure sensitivity ($TCS$) |
| **4** | `C4` | `0x78A1` | 30881 | Temperature coefficient of pressure offset ($TCO$) |
| **5** | `C5` | `0x793D` | 31037 | Reference temperature ($T_{REF}$) |
| **6** | `C6` | `0x6ACF` | 27343 | Temperature coefficient of temperature ($TEMPSENS$) |

### Mathematical Validation
1. **CRC-4 Remainder:** The factory CRC-4 algorithm (Datasheet page 13) executed on the live PROM returned an exact remainder of **`0x4`**, confirming zero read corruption.
2. **Worked Example Test:** Using the datasheet test vectors ($C_1\dots C_6$, $D_1=6465444$, $D_2=8077636$), the first-order compensation evaluates to exactly **$1100.02\text{ mbar}$** and **$20.00\ ^\circ\text{C}$**.
3. **Ambient Baseline:** Laboratory ambient tests produced stable readings of **$\approx 23.2\ ^\circ\text{C}$** and **$\approx 1000.5\text{ mbar}$ ($0.9875\text{ atm}$)** with $< 0.08\text{ mbar}$ peak-to-peak jitter.

---

## 3. Glider Depth Resolution Analysis

The glider system requirement is **$\sim 10\text{ gradations per cm}$** of water column ($1\text{ mm}$ vertical depth resolution).

Hydrostatic relationship:
$$\Delta P = \rho \cdot g \cdot \Delta h \implies \Delta h = \frac{\Delta P}{\rho \cdot g}$$
*(Using $\rho_{\text{fresh}} = 1000\text{ kg/m}^3$, $\rho_{\text{sea}} = 1025\text{ kg/m}^3$, $g = 9.80665\text{ m/s}^2$)*

### OSR Comparison Table
| OSR | Max Conversion Time | Datasheet RMS Noise | Freshwater Depth Resolution | Seawater Depth Resolution | Resolvable Gradations | Glider Target (~10 grad/cm) |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **256** | **$0.56\text{ ms}$** | $0.110\text{ mbar}$ | $1.12\text{ mm}$ | $1.09\text{ mm}$ | **$8.9\text{ grad/cm}$** | Marginal ($1.1\text{ mm}$) |
| **512** | **$1.10\text{ ms}$** | $0.062\text{ mbar}$ | **$0.63\text{ mm}$** | **$0.62\text{ mm}$** | **$15.8\text{ grad/cm}$** | **Exceeds target ($1.6\times$)** |
| **1024** | **$2.17\text{ ms}$** | $0.039\text{ mbar}$ | **$0.40\text{ mm}$** | **$0.39\text{ mm}$** | **$25.1\text{ grad/cm}$** | Over-specified ($2.5\times$) |
| **2048** | **$4.32\text{ ms}$** | $0.028\text{ mbar}$ | $0.29\text{ mm}$ | $0.28\text{ mm}$ | $35.0\text{ grad/cm}$ | Over-specified ($3.5\times$) |
| **4096** | **$8.61\text{ ms}$** | $0.021\text{ mbar}$ | $0.21\text{ mm}$ | $0.21\text{ mm}$ | $46.7\text{ grad/cm}$ | Unusable at 100 Hz paired |
| **8192** | **$17.20\text{ ms}$** | $0.016\text{ mbar}$ | $0.16\text{ mm}$ | $0.16\text{ mm}$ | $61.3\text{ grad/cm}$ | Unusable at 100 Hz paired |

**Conclusion:** OSR 512 delivers **$0.63\text{ mm}$ depth resolution** ($15.8\text{ gradations/cm}$), meeting and exceeding the glider's operational requirements without wasting CPU cycles or bus time.

---

## 4. Real-Time Timing & Benchmark Results

All benchmarks were run for **10.0 seconds (10,000 service ticks)** under PREEMPT_RT (`SCHED_FIFO` priority 80) driven by `realtime.c` (`CLOCK_MONOTONIC`) at a **1 kHz tick cadence** ($1.0\text{ ms}$).

### What "Average I2C Tx Time" Measures
On Linux, calling `write()` or `read()` on `/dev/i2c-1` enters the kernel `i2c-bcm2835` driver. At standard $100\text{ kHz}$ I²C bus clocking, serializing address bytes, command bytes, ACKs, and repeated-starts requires **$650\text{--}700\ \mu\text{s}$ per system call**. 

A complete paired reading executes 4 distinct bus operations (6 system calls):
1. Write start $D_1$ command (1 byte)
2. Write ADC read command `0x00` (1 byte) + Read $D_1$ ADC (3 bytes)
3. Write start $D_2$ command (1 byte)
4. Write ADC read command `0x00` (1 byte) + Read $D_2$ ADC (3 bytes)

### Empirical Benchmark Summary

| Benchmark Configuration | Measured Sustained Rate | 100 Hz Publish Frames | Stale Frames Count | Stale Frame % | Average I2C Tx Time |
|---|:---:|:---:|:---:|:---:|:---:|
| **OSR 2048 Paired (1:1)** | $73.50\text{ Hz}$ | 1000 | 266 | **$26.60\%$** | $692\ \mu\text{s}$ |
| **OSR 2048 Decoupled (1:2)** | $98.90\text{ Hz}$ | 1000 | 179 | **$17.90\%$** | $695\ \mu\text{s}$ |
| **OSR 2048 Decoupled (1:5)** | $122.70\text{ Hz}$ | 1000 | 91 | **$9.10\%$** | $695\ \mu\text{s}$ |
| **OSR 2048 P / OSR 1024 T (1:1)**| $93.10\text{ Hz}$ | 1000 | 70 | **$7.00\%$** | $694\ \mu\text{s}$ |
| **OSR 2048 P / OSR 1024 T (1:5)**| $133.50\text{ Hz}$ | 1000 | 28 | **$2.80\%$** | $694\ \mu\text{s}$ |
| **OSR 1024 Paired (1:1)** | $124.99\text{ Hz}$ | 1000 | 1 | **$0.10\%$** | $666\ \mu\text{s}$ |
| **OSR 512 Paired (1:1)** | **$166.70\text{ Hz}$** | **1000** | **0** | **$0.00\%$** | **$652\ \mu\text{s}$** |
| **OSR 256 Paired (1:1)** | **$175.20\text{ Hz}$** | **1000** | **0** | **$0.00\%$** | **$652\ \mu\text{s}$** |

### Key Benchmark Observations
1. **Why OSR 2048 fails at 100 Hz:** Each conversion takes $4.32\text{ ms}$ plus $\approx 1.5\text{ ms}$ I²C overhead ($5.8\text{ ms}$ per channel). Converting both channels sequentially requires $\approx 11.6\text{ ms} > 10.0\text{ ms}$, resulting in missed deadlines and stale frame repeating.
2. **Why Decoupled Ratios still exhibit stale frames:** Even when the *average* pressure rate is $133.5\text{ Hz}$, the single tick where $D_2$ is scheduled introduces an $11\text{ ms}$ pipeline stall, producing a stale frame during that specific 10 ms window.
3. **Why OSR 512 is Optimal:** 
   - $1.15\text{ ms}$ conversion delay $\times 2 = 2.30\text{ ms}$ total ADC wait.
   - Entire paired sequence finishes in **$< 5.0\text{ ms}$** (spanning 6 ticks in a 1 kHz loop).
   - Generates **$166.7\text{ Hz}$ maximum throughput** with **$0.00\%$ stale frames** and leaves **$\ge 5.0\text{ ms}$ of idle CPU headroom** in every 10 ms publication window for other tasks (IMU servicing, encoder handling, Kalman filtering).

---

## 5. Architectural Decision for Production (`app/`)

Based on empirical testing and glider control requirements, the production system (`ms5/app/`) will implement:

- **Operating Mode:** **Paired 1:1 Conversion** (Symmetric $D_1$ Pressure and $D_2$ Temperature).
- **ADC Configuration:** **OSR 512** for both channels (`0x42` and `0x52`).
- **Timing Architecture:**
  - Fast periodic service tick driven by `realtime.c` (`StartRT` + `RT_SleepUntil`) at **1 kHz** ($1.0\text{ ms}$).
  - Non-blocking 4-state machine: `STATE_START_D1` $\rightarrow$ `STATE_WAIT_D1` $\rightarrow$ `STATE_START_D2` $\rightarrow$ `STATE_WAIT_D2`.
  - Independent 100 Hz publication timer consuming completed records every 10 ticks.
- **Output Contract:** `BaroSample_t` containing monotonic timestamps, compensated pressure (mbar), temperature (°C), water depth (m), and diagnostic status flags.

---

## 6. Directory Layout & Build System

```text
twg/
├── bno/                         # BNO085 IMU subsystem (SPI0 + GPIO)
│   └── app/
│       ├── realtime.c           # Shared PREEMPT_RT timing implementation
│       └── realtime.h           # (Single source of truth)
└── ms5/                         # MS5837-02BA Barometer subsystem (I2C1)
    ├── Makefile                 # Builds tests and app (links bno/app/realtime.c)
    ├── readme.md                # System documentation and test findings
    ├── app/                     # (To be generated in Step 4)
    │   ├── app_contract.h       # BaroSample_t data contract
    │   ├── ms5837_driver.c/.h   # OSR 512 protocol & compensation math
    │   ├── ms5837_hal_rpi.c/.h  # Linux I2C HAL (/dev/i2c-1)
    │   └── main.c               # 100 Hz RT acquisition & publishing loop
    ├── tests/
    │   ├── test_i2c_raw.c       # Step 1: Bus probe, soft reset, PROM dump
    │   ├── test_crc_math.c      # Step 2: PROM CRC-4 & datasheet math proof
    │   └── test_rate_bench.c    # Step 3: OSR and cadence benchmark harness
    ├── bin/                     # Generated executables
    └── build/                   # Compiled intermediate objects (.o)
```

### Build Instructions
```bash
# From twg/ms5:
make clean
make tests

# Run diagnostics and benchmarks (requires sudo for PREEMPT_RT SCHED_FIFO):
./bin/test_i2c_raw
./bin/test_crc_math
sudo ./bin/test_rate_bench
```