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

### 4.1 Bus Frequency Comparison: 100 kHz vs 400 kHz Fast Mode

The Raspberry Pi hardware I2C controller (`/dev/i2c-1`) was initially benchmarked at the default standard speed (100 kHz) and subsequently configured to **Fast Mode (400 kHz)** by adding `dtparam=i2c_arm_baudrate=400000` to `/boot/firmware/config.txt`.

Switching to Fast Mode resulted in an approximate **3.5x reduction** in I2C transaction latency across all system calls:
* **Standard 100 kHz Mode:** Average I2C transaction duration was **~650–700 µs** per syscall.
* **Fast Mode (400 kHz):** Average I2C transaction duration dropped to **~180–200 µs** per syscall.

While the sensor internal ADC conversion wait times remain constant (dictated by internal RC oscillator timings), the faster bus drastically reduces thread blocking during command dispatch and data readout, leaving substantial CPU and scheduling headroom for concurrent sensor loops.

---

### 4.2 Comprehensive Benchmark Matrix

Benchmarks were evaluated over 10.0-second runs using `tests/test_rate_bench.c` in a 1 kHz PREEMPT-RT `SCHED_FIFO` service loop (`TIMER_ABSTIME`, monotonic clock), evaluating actual pressure acquisition throughput against a 100 Hz publication schedule (1,000 frames total).

| Configuration Set | Ratio (Temp:Press) | D1 Delay (ms) | D2 Delay (ms) | 100 kHz Tx Avg (µs) | 100 kHz Rate (Hz) | 100 kHz Stale (%) | 400 kHz Tx Avg (µs) | 400 kHz Rate (Hz) | 400 kHz Stale (%) |
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **SET A: OSR 2048 / 2048** | 1 : 1 | 4.40 | 4.40 | 480.39 | 79.00 | 21.10% | 199.88 | 83.30 | 16.80% |
| | 1 : 2 | 4.40 | 4.40 | 692.85 | 101.60 | 14.10% | 199.99 | 111.10 | 11.30% |
| | 1 : 3 | 4.40 | 4.40 | 697.32 | 110.29 | 13.60% | 200.41 | 125.00 | 8.50% |
| | 1 : 4 | 4.40 | 4.40 | 691.36 | 120.00 | 11.70% | 200.84 | 133.30 | 0.30% |
| | 1 : 5 | 4.40 | 4.40 | 695.04 | 122.40 | 9.40% | 200.62 | 138.90 | 5.90% |
| **SET B: OSR 2048 / 1024** | 1 : 1 | 4.40 | 2.25 | 694.63 | 92.89 | 7.20% | 197.16 | 100.00 | 0.10% |
| | 1 : 2 | 4.40 | 2.25 | 684.87 | 115.59 | 5.20% | 196.39 | 125.00 | 0.20% |
| | 1 : 3 | 4.40 | 2.25 | 695.05 | 122.59 | 0.60% | 198.91 | 136.40 | 0.20% |
| | 1 : 4 | 4.40 | 2.25 | 688.01 | 129.40 | 2.90% | 199.72 | 142.80 | 0.30% |
| | 1 : 5 | 4.40 | 2.25 | 692.67 | 132.00 | 2.10% | 199.01 | 147.00 | 0.30% |
| **BASELINES: Paired (1:1)** | | | | | | | | | |
| *OSR 1024 / 1024* | 1 : 1 | 2.25 | 2.25 | 666.25 | 124.99 | 0.10% | 193.12 | 125.00 | 0.10% |
| *OSR 512 / 512* | 1 : 1 | 1.15 | 1.15 | 652.19 | 166.70 | **0.00%** | 180.73 | 166.70 | **0.00%** |
| *OSR 256 / 256* | 1 : 1 | 0.60 | 0.60 | 652.05 | 175.20 | **0.00%** | 179.69 | 250.00 | **0.00%** |

---

### 4.3 Analysis & Key Observations

1. **Physical Limit of Symmetrical OSR 2048 (1:1):**
   * Even with bus latency reduced to ~200 µs at 400 kHz, 1:1 paired OSR 2048 only achieved **83.30 Hz** with **16.8% stale frames**.
   * Hardware conversion delays ($4.32\text{ ms} + 4.32\text{ ms} = 8.64\text{ ms}$) quantized across a 1 kHz discrete service loop consume 10–12 ms per pair, making true 100 Hz 1:1 operation physically impossible at OSR 2048.
2. **Phase Jitter in Asymmetric OSR Ratios:**
   * In SET A, asymmetric ratios (e.g., 1:2 to 1:5) achieved average throughputs above 100 Hz (111–138 Hz), yet still suffered up to 11.3% stale frames. 
   * This is caused by conversion phase misalignment: cycles executing a temperature conversion take longer than pure pressure cycles, periodically straddling 100 Hz publication boundaries and causing stale samples.
3. **SET B Viability at 400 kHz:**
   * Shortening temperature conversion to OSR 1024 ($2.25\text{ ms}$) allowed 1:1 operation to reach **100.00 Hz** with only 1 stale frame (0.10%), proving viable if maximum pressure oversampling is required in the future.

---

### 4.4 Final Operating Decision: Symmetrical OSR 512 (1:1)

For the Step 4 production driver implementation and subsequent IMU integration, **Symmetrical OSR 512 (1:1)** on **400 kHz Fast Mode I2C** is selected as the primary operating configuration:

* **Zero Stale Publications:** Achieved **0.00% stale frames** across 1,000 consecutive 100 Hz publication windows.
* **Low Bus Occupancy:** Average transaction time of **180.73 µs** minimizes total I2C bus holding time.
* **Guaranteed Frame Headroom:**
  * D1 Conversion: ~1.15 ms
  * D2 Conversion: ~1.15 ms
  * Combined I2C Transactions: ~0.72 ms
  * **Total Frame Execution Time:** **~3.02 ms**
  * **Idle Headroom:** **~6.98 ms per 10 ms frame**
* **Noise vs Timing Trade-off:** RMS noise at OSR 512 is **0.062 mbar** (equivalent to approximately **0.63 mm of hydrostatic water depth**), which provides sufficient resolution for underwater towbody and glider depth estimation while leaving ample idle headroom to service BNO085 SPI transactions and encoder interrupts on the shared real-time thread.
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