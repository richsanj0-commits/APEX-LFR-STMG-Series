<div align="center">

# 🏎️ APEX-LFR-STMG-Series
### High-Performance Autonomous Line Follower Robot (LFR)
**Engineered for the STM32G431CB (Arm Cortex-M4 @ 170MHz)**

[![PlatformIO](https://img.shields.io/badge/PlatformIO-Core%206.0%2B-orange?logo=platformio)](https://platformio.org/)
[![Framework](https://img.shields.io/badge/Framework-STM32duino%20%2F%20Arduino-blue?logo=arduino)](https://github.com/stm32duino)
[![MCU](https://img.shields.io/badge/MCU-STM32G431CB%20(170MHz)-blueviolet?logo=stmicroelectronics)](https://www.st.com/en/microcontrollers-microprocessors/stm32g431cb.html)
[![Language](https://img.shields.io/badge/Language-C%2B%2B17%20%2F%20C-00599C?logo=c%2B%2B)](https://en.wikipedia.org/wiki/C%2B%2B)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

<p align="center">
  A precision-engineered competition line-follower robot firmware featuring a 14-sensor multiplexed IR array, 12-bit ADC centroid calculation, advanced anti-windup PID control with low-pass filtered derivative, dynamic corner auto-braking, and a full on-device OLED menu system.
</p>

---

</div>

## 📑 Table of Contents
1. [System Architecture](#-system-architecture)
2. [Complete Hardware Pinout & Wiring](#-complete-hardware-pinout--wiring)
   - [1. TB6612FNG Dual H-Bridge Motor Driver](#1-tb6612fng-dual-h-bridge-motor-driver)
   - [2. CD74HC4067 16-Channel Multiplexer (14 IR Sensors)](#2-cd74hc4067-16-channel-multiplexer-14-ir-sensors)
   - [3. SSD1306 0.96" OLED Display](#3-ssd1306-096-oled-display)
   - [4. Navigation & Control Pushbuttons](#4-navigation--control-pushbuttons)
   - [5. Complete Master Pin Mapping Table](#5-complete-master-pin-mapping-table)
3. [Subprojects in this Repository](#-subprojects-in-this-repository)
   - [Project A: `line_follower_mux14` (Main Race Firmware)](#project-a-line_follower_mux14-main-race-firmware)
   - [Project B: `sensor_8array_test` (Bench Verification)](#project-b-sensor_8array_test-bench-verification)
4. [Control Algorithm & Mathematics](#-control-algorithm--mathematics)
5. [Interactive OLED Menu Interface](#-interactive-oled-menu-interface)
6. [Building & Flashing Guide](#-building--flashing-guide)

---

## 🏛️ System Architecture

```
                       +-----------------------------------+
                       |         Power Supply              |
                       |  2S LiPo (7.4V) -> VM / Motors   |
                       |  3.3V / 5V Reg   -> STM32 Logic   |
                       +-----------------+-----------------+
                                         |
                                         v
+------------------------+     +-------------------+     +--------------------------+
|  14-Sensor IR Array    |     |   STM32G431CB     |     |   TB6612FNG Driver       |
|  (Ground Line Sensors) |     |   Arm Cortex-M4   |     |   Dual H-Bridge          |
|           |            |     |   @ 170 MHz       |     +-------------+------------+
|           v            |     +---------+---------+                   |
|  CD74HC4067 Multiplexer+---->|PA2 (ADC)|         |                   |
|  S0-S3 Address Select  |<----+PA4-PA7  |   PWM   +------------------>| Left Motor  (Ch B)
+------------------------+     |         | PA8,PA9 |                   | Right Motor (Ch A)
                               |         |         |                   +--------------------+
+------------------------+     |         | GPIO    +------------------>| Direction & STBY
|  SSD1306 128x32 OLED   |<----+PB6, PB7 | PB0,PB1 |
|  Software I2C Display  |     |         | PB10,11 |
+------------------------+     |         |         |
                               |         | Inputs  |<------------------+ 4x Pushbuttons
                               |         | PC13,PB3|                    (Enter, Exit,
                               |         | PB4,PB5 |                    Up, Down)
                               +---------+---------+
```

---

## 🔌 Complete Hardware Pinout & Wiring

All pin connections are strictly mapped and verified according to `line_follower_mux14/src/pins.h`.

### 1. TB6612FNG Dual H-Bridge Motor Driver
Controls two high-speed DC coreless/geared motors using independent direction lines and hardware PWM speed control pins.

| TB6612FNG Pin | Label / Function | Connected to STM32 Pin | Notes & Direction Mapping |
|:---:|:---:|:---:|:---|
| **PWMB** | Left Motor PWM Speed | `PA9` | Hardware PWM duty cycle (0 – 255) |
| **BIN1** | Left Motor Direction 1 | `PB10` | Set `LOW` for Forward, `HIGH` for Reverse |
| **BIN2** | Left Motor Direction 2 | `PB11` | Set `HIGH` for Forward, `LOW` for Reverse |
| **PWMA** | Right Motor PWM Speed | `PA8` | Hardware PWM duty cycle (0 – 255) |
| **AIN1** | Right Motor Direction 1 | `PB0` | Set `HIGH` for Forward, `LOW` for Reverse |
| **AIN2** | Right Motor Direction 2 | `PB1` | Set `LOW` for Forward, `HIGH` for Reverse |
| **STBY** | Driver Standby Enable | `PA10` | Must be held `HIGH` to activate the driver |
| **VM** | Motor Power Supply | `+7.4V` (2S LiPo) | Motor drive voltage (2.5V – 13.5V max) |
| **VCC** | Driver Logic Power | `+3.3V` or `+5V` | Logic supply from regulator |
| **GND** | Power Ground | `GND` | **Must share a common ground with STM32** |
| **A01 / A02** | Motor Output A | Right DC Motor | Polarity determines forward rotation |
| **B01 / B02** | Motor Output B | Left DC Motor | Polarity determines forward rotation |

> ⚠️ **Physical Wiring Notice:** As designated in `motors.cpp`, **Channel B** drives the **Left Motor**, and **Channel A** drives the **Right Motor**.

---

### 2. CD74HC4067 16-Channel Multiplexer (14 IR Sensors)
Enables reading all 14 ground phototransistors using only 1 high-speed analog pin (`PA2`) and 4 digital address selector lines (`PA4`–`PA7`).

| Multiplexer Pin | Function | Connected to STM32 Pin | Notes |
|:---:|:---:|:---:|:---|
| **SIG / COM** | Common Analog Output | `PA2` (ADC1_IN3) | Fast 12-bit ADC reading (50µs settling delay) |
| **S0** | Address Select Bit 0 (LSB) | `PA7` | Binary select bit 0 (`i & 0x01`) |
| **S1** | Address Select Bit 1 | `PA6` | Binary select bit 1 (`(i >> 1) & 0x01`) |
| **S2** | Address Select Bit 2 | `PA5` | Binary select bit 2 (`(i >> 2) & 0x01`) |
| **S3** | Address Select Bit 3 (MSB) | `PA4` | Binary select bit 3 (`(i >> 3) & 0x01`) |
| **EN** | Active-LOW Enable | `GND` | Tied to GND to keep MUX continuously active |
| **VCC** | Power Supply | `+3.3V` / `+5V` | Regulated logic power |
| **GND** | Ground | `GND` | Common ground reference |

#### Sensor Array Channels (C0 – C13)
The sensors are laid out physically along the front bar from left to right:
* `C0`: **Sensor 0** (Extreme Left, Weight: `-150.0`)
* `C1` – `C5`: **Sensors 1 to 5** (Left Side, Weights: `-100.0` to `-10.0`)
* `C6`: **Sensor 6** (Left Center, Weight: `-5.0`)
* `C7`: **Sensor 7** (Right Center, Weight: `+5.0`)
* `C8` – `C12`: **Sensors 8 to 12** (Right Side, Weights: `+10.0` to `+100.0`)
* `C13`: **Sensor 13** (Extreme Right, Weight: `+150.0`)
* `C14`, `C15`: *Unused (tied to GND or left open)*

---

### 3. SSD1306 0.96" OLED Display
Provides visual diagnostics, live calibration bars, tuning menus, and sensor readouts.

| OLED Pin | Function | Connected to STM32 Pin | Details |
|:---:|:---:|:---:|:---|
| **SCL** | Clock Line | `PB6` | Software I2C via U8g2lib |
| **SDA** | Data Line | `PB7` | Software I2C via U8g2lib |
| **VCC** | Power | `+3.3V` | Regulated 3.3V rail |
| **GND** | Ground | `GND` | Common ground |

---

### 4. Navigation & Control Pushbuttons
Tactile buttons for menu navigation and initiating auto-calibration. Configured as **Active LOW** with STM32 internal pull-up resistors (`INPUT_PULLUP`). The opposite leg of each switch connects directly to `GND`.

| Button Name | Action / Event | Connected to STM32 Pin | Active Level |
|:---:|:---:|:---:|:---:|
| **BTN_ENTER** | Select / Confirm / Calibrate | `PC13` | Press = `LOW` |
| **BTN_EXIT** | Back / Stop / Cancel | `PB3` | Press = `LOW` |
| **BTN_UP** | Cursor Up / Value Increment | `PB5` | Press = `LOW` |
| **BTN_DOWN** | Cursor Down / Value Decrement | `PB4` | Press = `LOW` |

---

### 5. Complete Master Pin Mapping Table
Quick lookup for the **STM32G431CB** 48-pin package:

| STM32 Pin | Peripheral / Function | Connected Device & Pin | Signal Type |
|:---:|:---|:---|:---:|
| **PA2** | ADC1_IN3 | CD74HC4067 `SIG` | Analog Input (0–4095) |
| **PA4** | GPIO Output | CD74HC4067 `S3` (MSB) | Digital Output |
| **PA5** | GPIO Output | CD74HC4067 `S2` | Digital Output |
| **PA6** | GPIO Output | CD74HC4067 `S1` | Digital Output |
| **PA7** | GPIO Output | CD74HC4067 `S0` (LSB) | Digital Output |
| **PA8** | TIM1_CH1 (PWM) | TB6612FNG `PWMA` (Right Motor) | PWM Output (0–255) |
| **PA9** | TIM1_CH2 (PWM) | TB6612FNG `PWMB` (Left Motor) | PWM Output (0–255) |
| **PA10**| GPIO Output | TB6612FNG `STBY` (Enable) | Digital Output (Active HIGH) |
| **PB0** | GPIO Output | TB6612FNG `AIN1` (Right Dir 1) | Digital Output |
| **PB1** | GPIO Output | TB6612FNG `AIN2` (Right Dir 2) | Digital Output |
| **PB3** | GPIO Input (Pull-up) | Tactile Switch `BTN_EXIT` | Digital Input (Active LOW) |
| **PB4** | GPIO Input (Pull-up) | Tactile Switch `BTN_DOWN` | Digital Input (Active LOW) |
| **PB5** | GPIO Input (Pull-up) | Tactile Switch `BTN_UP` | Digital Input (Active LOW) |
| **PB6** | Software I2C SCL | SSD1306 OLED `SCL` | I2C Clock |
| **PB7** | Software I2C SDA | SSD1306 OLED `SDA` | I2C Data |
| **PB10**| GPIO Output | TB6612FNG `BIN1` (Left Dir 1) | Digital Output |
| **PB11**| GPIO Output | TB6612FNG `BIN2` (Left Dir 2) | Digital Output |
| **PC13**| GPIO Input (Pull-up) | Tactile Switch `BTN_ENTER` | Digital Input (Active LOW) |

---

## 📦 Subprojects in this Repository

### Project A: `line_follower_mux14` (Main Race Firmware)
The complete competitive firmware running the robot during trials and races:
* **`src/sensors.cpp`**: Controls CD74HC4067 address cycling, dynamic threshold calculation, and weighted error calculation.
* **`src/follower.cpp`**: Core PID control loop with anti-windup, derivative filtering, dynamic base-speed corner braking, and edge-recovery spin.
* **`src/motors.cpp`**: TB6612 motor direction and PWM generation.
* **`src/display.cpp` & `src/menu.cpp`**: Multi-page interactive UI for on-field parameter adjustments without a computer.

### Project B: `sensor_8array_test` (Bench Verification & Wheel Direction Test)
A standalone validation program used to test analog sensor response, calibration, noise margins, and wheel motor directions without the multiplexer:
* **Pinout:** 8 analog sensors directly connected to `PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7`.
* **Wheel Control & Buttons:**
  * **`PB5`**: Press & hold &rarr; **Both wheels move FORWARD**.
  * **`PB4`**: Press & hold &rarr; **Both wheels move BACKWARD**.
  * **Release**: Wheels **STOP** immediately.
  * **`PC13`**: Initiates 10-second sweep calibration over black line and white floor.
* **Motors:** TB6612FNG driver mapped with Left Motor (`PB10`, `PB11`, `PA9`) and Right Motor (`PB0`, `PB1`, `PA8`) with polarity configured for forward rotation.
* **Display:** SSD1306 OLED on `PB6` (SCL) and `PB7` (SDA).
* **Operation:**
  1. Sweep sensors across white floor and black line during calibration to establish midpoint thresholds $\text{Mid} = \frac{\text{Min} + \text{Max}}{2}$.
  2. Live binary mode renders real-time `0` (white) / `1` (black) telemetry to Serial Monitor (115200 baud) and draws filled/hollow indicator blocks on the OLED screen while jogging the wheels.

---

## 🧮 Control Algorithm & Mathematics

### 1. Weighted Centroid Error Formula
The line position is determined by calculating the centroid of all sensors detecting the line:

$$\text{Error} = \frac{\sum_{i=0}^{13} (w_i \times D_i)}{\sum_{i=0}^{13} D_i}$$

Where:
* $D_i \in \{0, 1\}$ is the binary status of sensor $i$ based on dynamic calibration thresholds.
* $w_i$ is the symmetrical geometric weight:
  $$\{-150.0, -100.0, -70.0, -40.0, -20.0, -10.0, -5.0, +5.0, +10.0, +20.0, +40.0, +70.0, +100.0, +150.0\}$$

### 2. Line-Loss Memory Recovery
If the robot completely loses the line ($D_i = 0$ for all $i$):
$$\text{Error} = \begin{cases} +150.0, & \text{if } \text{last\_error} > 0 \text{ (Sharp Right Turn)} \\ -150.0, & \text{if } \text{last\_error} < 0 \text{ (Sharp Left Turn)} \\ 0.0, & \text{otherwise} \end{cases}$$

### 3. Filtered PID Calculation
$$\text{Correction} = (K_p \cdot e) + \left(K_i \int e \, dt\right) + (K_d \cdot D_{\text{filtered}})$$

* **Anti-Windup:** Integral accumulation is clamped to $[-1000, +1000]$.
* **Derivative Low-Pass Filter ($\alpha = 0.7$):**
  $$D_{\text{filtered}}(t) = 0.7 \cdot D_{\text{filtered}}(t-1) + 0.3 \cdot (e(t) - e(t-1))$$

### 4. Dynamic Corner Auto-Braking
To avoid overshooting tight corners at high speed, forward momentum scales down as the steering error grows:
$$\text{SpeedFactor} = 1.0 - \frac{|e| - 10.0}{175.0} \quad (\text{clamped to } \ge 0.20)$$
$$\text{BaseSpeed}_{\text{effective}} = \text{BaseSpeed} \times \text{SpeedFactor}$$

$$\text{LeftSpeed} = \text{BaseSpeed}_{\text{effective}} + \text{Correction}$$
$$\text{RightSpeed} = \text{BaseSpeed}_{\text{effective}} - \text{Correction}$$

---

## 📺 Interactive OLED Menu Interface

Operated via **UP (`PB5`)**, **DOWN (`PB4`)**, **ENTER (`PC13`)**, and **EXIT (`PB3`)**:

```
[ WELCOME SCREEN ]
      |
      v
[ MAIN MENU ]
 ├── 1. Start        --> Executes follower_loop() with configured PID & speed
 ├── 2. Calibrate    --> Spins robot in place for N seconds to capture thresholds
 ├── 3. Tuning
 │    ├── PID        --> Live adjust Kp, Ki, Kd
 │    ├── Motor Speed--> Adjust max speed (0 - 255)
 │    ├── Cal Time   --> Adjust calibration duration (sec)
 │    └── Line Color --> Toggle Black Line (0) / White Line (1)
 └── 4. Testing
      ├── Motor Test --> Manual spin Left, Right, Forward, Backward
      └── Sensor Test--> Live visual bar graph of 14 sensor readings
```

---

## 🚀 Building & Flashing Guide

### Prerequisites
* Install [PlatformIO Core](https://platformio.org/install/cli) or VS Code with the PlatformIO extension.
* STM32 USB DFU driver or ST-Link V2 programmer.

### Build & Upload Commands

```bash
# Clone the repository
git clone https://github.com/richsanj0-commits/APEX-LFR-STMG-Series.git
cd APEX-LFR-STMG-Series

# 1. Flash the main 14-sensor line follower firmware
cd line_follower_mux14
pio run --target upload

# 2. Flash the 8-sensor bench verification firmware
cd ../sensor_8array_test
pio run --target upload
```

---

<div align="center">
  <b>Developed & Maintained by <a href="https://github.com/richsanj0-commits">richsanj0-commits</a></b>
</div>
