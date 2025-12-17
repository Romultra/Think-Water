# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Coaster-Clip is a smart coaster system for hydration tracking. It consists of two hardware components:
- **Coaster (clip)**: Seeed XIAO nRF52840 Sense with load cell, hall sensor, and NeoPixel LEDs
- **Base Station**: ESP32 with SHT41 temperature/humidity sensor, HW-111 RTC, SD card, buzzer, OLED display (SSD1333), and rotary encoder

## Build Commands

This project uses the **PlatformIO IDE extension**. Build from terminal using:

```bash
# Build coaster firmware (main target)
"$HOME/.platformio/penv/Scripts/pio.exe" run -e coaster-main

# Build base station firmware (serial interface)
"$HOME/.platformio/penv/Scripts/pio.exe" run -e base-main

# Build base station with OLED UI (full-featured)
"$HOME/.platformio/penv/Scripts/pio.exe" run -e base-ui

# Build calibration firmwares
"$HOME/.platformio/penv/Scripts/pio.exe" run -e coaster-calibration
"$HOME/.platformio/penv/Scripts/pio.exe" run -e base-calibration

# Upload to connected device
"$HOME/.platformio/penv/Scripts/pio.exe" run -e coaster-main -t upload
"$HOME/.platformio/penv/Scripts/pio.exe" run -e base-ui -t upload

# Monitor serial output
"$HOME/.platformio/penv/Scripts/pio.exe" device monitor
```

## Architecture

### PlatformIO Environments (`platformio.ini`)
- `base-main`: ESP32 base station with serial interface (BLE client, RTC, sensors)
- `base-ui`: ESP32 base station with OLED display and rotary encoder UI
- `base-oled`: OLED display test/development environment
- `base-calibration`: ESP32 calibration controller for wireless sensor calibration
- `coaster-main`: Seeed XIAO nRF52840 coaster (BLE peripheral)
- `coaster-calibration`: nRF52840 calibration firmware for load cell and hall sensor

### Coaster Firmware (`src/clip/main-coaster.cpp`)

**State Machine**: The coaster operates as a state machine with states:
- `IDLE_WITH_GLASS` / `IDLE_NO_GLASS`: Slow polling (2s), deep sleep duty cycle
- `MOVING_WITH_GLASS` / `MOVING_NO_GLASS`: Fast polling (200ms) during activity
- `TEST`: Debug mode for hardware testing

**FreeRTOS Tasks**:
- `loadcellStabilityTask`: Background task sampling HX711 load cell for stability detection
- `ledAlertTask`: Handles NeoPixel alert animations (warm red pulses)
- `ledFeedbackTask`: Shows hydration progress on NeoPixels with animated green gradient
  - LED mapping: starts at LED 6 (7th LED) and rotates in reverse direction
  - Animation duration: 1.1s proportional to progress
  - Holds final state for 2 seconds
  - Smooth 600ms fade-out before turning off
- Event queue (`eventQueue`) for inter-task communication

**Stability Detection**: Uses load cell readings (not IMU) to determine if coaster is stationary:
- Requires weight above `LOADCELL_BOTTOM_PLATE_THRESHOLD` (30g)
- Average deviation must be below `LOADCELL_DEVIATION_THRESHOLD` (0.5g)

**BLE Protocol (Main)**:
- Service UUID: `19B10010-E8F2-537E-4F6C-D104768A1214`
- Consumption characteristic (notify): timestamp(4 bytes) + scaled_value(4 bytes, value×100)
- Command characteristic (write): 0x01=LED alert, 0x02=status query
- Goal characteristic (write): float hydration goal in ml

**TEST Mode Serial Commands**:
| Command | Description |
|---------|-------------|
| `T` | Send test consumption event via BLE |
| `A` | Start BLE advertising |
| `S` | Stop BLE advertising |
| `F` | Trigger LED feedback animation |

### Base Station (`src/base/main-base.cpp`)

ESP32 BLE client that:
- Scans for CoasterClip devices and receives consumption notifications
- Tracks daily hydration goals based on temperature/humidity
- Sends alerts when user is behind hydration trajectory
- Logs data to SD card
- Uses HW-111 (DS1307) RTC for timekeeping

**Serial Commands**:
| Command | Description | Example |
|---------|-------------|---------|
| `TIME:<unix>` | Set RTC from Unix timestamp | `TIME:1734268800` |
| `SETTIME:Y,M,D,H,M,S` | Set RTC from components | `SETTIME:2025,12,15,14,30,0` |
| `T` or `TIME` | Print current time | `T` |
| `H` or `HELP` | Show help | `H` |

### Base Station UI (`src/base/main-base-ui.cpp`)

Full-featured base station with OLED display (SSD1333, 176x176 color) and rotary encoder interface.

**UI Screens:**
- `SETUP_*`: Initial setup wizard (age, sex, weight, time)
- `MAIN_DISPLAY`: Hydration progress with temperature/humidity
- `MENU`: Settings navigation
- `DEBUG_MENU`: Debug actions (send alert, send goal, trigger day change)

**User Configuration:**
- Age, sex, weight stored on SD card (`/config/user.cfg`)
- Used for water turnover calculation via `predictWaterTurnoverCH()`

**Rotary Encoder:**
- Rotation: Navigate menus, adjust values
- Press: Confirm selection, advance wizard
- 5-minute inactivity screensaver (display off)

**Display Features:**
- Progress percentage with color coding (red < 30%, yellow < 70%, green ≥ 70%)
- Progress bar visualization
- Real-time temperature and humidity
- Clock display (if RTC available)

### OLED Display Driver (`src/base/oled-ssd1333.cpp`)

Custom driver for NHD-1.91-176176UBC3 OLED display:
- 176x176 pixels, 65K colors (RGB565)
- SPI interface at 20MHz
- Inherits from Adafruit_GFX for graphics primitives
- Optimized `fillRect`, `fillScreen`, and line drawing

### Calibration System

The calibration system allows wireless calibration of load cell and hall sensor via BLE (no cables interfering with measurements).

**Coaster Calibration Firmware** (`src/clip/main-calibration.cpp`):
- BLE peripheral advertising as "CoasterCalib"
- Service UUID: `19B10020-E8F2-537E-4F6C-D104768A1214`
- Command 0x10: Load cell measurement (50 samples averaged)
  - Response: `[0x10][avg:4 bytes]` = 5 bytes
- Command 0x11: Hall sensor measurement (100 samples)
  - Response: `[0x11][min:4][max:4][avg:4]` = 13 bytes

**Base Calibration Controller** (`src/base/main-calibration.cpp`):

| Command | Description |
|---------|-------------|
| `W <grams>` | Set weight & measure load cell (save to CSV) |
| `M` | Measure load cell only (don't save) |
| `P` | Print load cell calibration CSV |
| `C` | Clear load cell calibration data |
| `HG` | Hall measurement WITH glass (save to CSV) |
| `HN` | Hall measurement WITHOUT glass (save to CSV) |
| `HM` | Hall measure only (don't save) |
| `HP` | Print hall sensor calibration CSV |
| `HC` | Clear hall sensor calibration data |
| `S` | Scan for coaster |
| `?` | Show help |

**Calibration Files** (SD card):
- `/CALIB.CSV` - Load cell: `reference_g,raw_value`
- `/HALL.CSV` - Hall sensor: `state,min,max,avg`

**Load Cell Calibration Workflow**:
1. Flash `coaster-calibration` to nRF52840
2. Flash `base-calibration` to ESP32
3. Open serial monitor on ESP32 (115200 baud)
4. Place known weight, enter `W <weight>` to measure and save
5. Repeat for all reference weights
6. Enter `P` to print calibration CSV

**Hall Sensor Calibration Workflow**:
1. Place glass on coaster, enter `HG`
2. Remove glass, enter `HN`
3. Repeat several times for each state to get reliable data
4. Enter `HP` to print hall sensor CSV
5. Analyze min/max/avg values to determine optimal thresholds

## Hardware Pins

### Coaster (nRF52840)
| Pin | Function |
|-----|----------|
| `A0` | Hall sensor (glass detection, hysteresis 466-470) |
| `D2` | HX711 DOUT |
| `D3` | HX711 SCK |
| `D7` | NeoPixel data (10 LEDs) |
| Pin 22 | Battery charging control (LOW = 100mA) |

### Base Station (ESP32)
| Pin | Function |
|-----|----------|
| GPIO 21 | I2C SDA (SHT41 + RTC) |
| GPIO 22 | I2C SCL (SHT41 + RTC) |
| GPIO 23 | SPI MOSI (SD card, OLED shared) |
| GPIO 19 | SPI MISO (SD card) |
| GPIO 18 | SPI SCK (SD card, OLED shared) |
| GPIO 5 | SPI CS (SD card) |
| GPIO 4 | Active buzzer |
| GPIO 15 | OLED CS (SSD1333) |
| GPIO 2 | OLED DC (Data/Command) |
| GPIO 16 | OLED RST (Reset) |
| GPIO 25 | Rotary encoder CLK |
| GPIO 26 | Rotary encoder DT |
| GPIO 34 | Rotary encoder SW (push button) |

## Key Implementation Details

- Consumption values are scaled by 100 for decimal precision (1550 = 15.50g)
- SPI Flash is put in deep power-down mode at startup to save power
- Volatile variables used for cross-task communication (documented trade-offs in code)
- Glass presence uses hysteresis to prevent flickering between states
- HX711 load cell waits up to 200ms for ready state between readings to prevent timing issues
- RTC (DS1307) maintains time even when ESP32 is powered off
- Base station calculates daily hydration goal using water turnover model based on temperature, humidity, body metrics
- Base station BLE uses continuous scanning with fast parameters (62.5ms interval) for quick device discovery

## File Structure

```
src/
├── base/
│   ├── main-base.cpp        # Base station firmware (serial interface)
│   ├── main-base-ui.cpp     # Base station firmware with OLED UI
│   ├── main-ble-example.cpp # BLE example/test firmware
│   ├── main-oled.cpp        # OLED display test firmware
│   ├── main-calibration.cpp # Calibration controller for wireless sensor calibration
│   ├── oled-ssd1333.cpp     # SSD1333 OLED display driver
│   ├── oled-ssd1333.h       # OLED driver header
│   └── water-turnover.cpp   # Water turnover prediction model
└── clip/
    ├── main-coaster.cpp     # Main coaster firmware
    └── main-calibration.cpp # Calibration firmware for load cell and hall sensor
```
