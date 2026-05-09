# KiCad Circuit Diagram Guide - Think Water Project

This guide provides everything needed to quickly create circuit schematics in KiCad for both the Coaster and Base Station.

**IMPORTANT:** All custom component symbols are in `Personal_Library.kicad_sym`. Add this library to your KiCad project first:
1. Preferences > Manage Symbol Libraries
2. Project Specific Libraries tab
3. Add library, browse to `Personal_Library.kicad_sym`

---

## COASTER CIRCUIT (Seeed XIAO nRF52840 Sense)

### Component List

| Ref | Component | KiCad Symbol Library | Symbol Name | Notes |
|-----|-----------|---------------------|-------------|-------|
| U1 | Seeed XIAO nRF52840 Sense | **Personal_Library** | **XIAO_nRF52840** | Custom symbol |
| U2 | HX711 Load Cell Amplifier | **Personal_Library** | **HX711** | Custom symbol |
| U3 | Hall Effect Sensor | **Personal_Library** | **Hall_Sensor** | Custom symbol |
| LED1 | WS2812B LED Ring (10x) | LED | WS2812B | Built-in KiCad symbol |
| LC1 | Load Cell (Wheatstone Bridge) | **Personal_Library** | **Load_Cell** | Custom symbol |
| BT1 | Li-ion Battery 820mAh | Device | Battery_Cell | Built-in KiCad symbol |

### Pin Connections Table

#### XIAO nRF52840 Connections

| XIAO Pin | Connected To | Component Pin | Signal Type |
|----------|--------------|---------------|-------------|
| A0 | Hall Sensor | OUT | Analog Input |
| D2 | HX711 | DOUT | Digital Input |
| D3 | HX711 | PD_SCK | Digital Output |
| D7 | WS2812B Ring | DIN | Digital Output |
| 3V3 | Hall Sensor | VCC | Power |
| 5V | WS2812B Ring | VCC | Power |
| GND | All components | GND | Ground |
| BAT | Li-ion Battery | + | Battery |

#### HX711 Connections

| HX711 Pin | Connected To | Notes |
|-----------|--------------|-------|
| VCC | XIAO 3V3 | 3.3V supply |
| GND | Common GND | |
| DOUT | XIAO D2 | Data output |
| PD_SCK | XIAO D3 | Clock input |
| E+ | Load Cell | Excitation + (Red) |
| E- | Load Cell | Excitation - (Black) |
| A+ | Load Cell | Signal + (White) |
| A- | Load Cell | Signal - (Green) |

#### Load Cell Wiring (Typical 4-wire)

| Wire Color | Function | HX711 Pin |
|------------|----------|-----------|
| Red | Excitation + | E+ |
| Black | Excitation - | E- |
| White | Signal + | A+ |
| Green | Signal - | A- |

#### WS2812B Ring

| LED Pin | Connected To | Notes |
|---------|--------------|-------|
| VCC | XIAO 5V | 5V supply |
| GND | Common GND | |
| DIN | XIAO D7 | Data input, first LED |
| DOUT | Next LED DIN | Daisy-chain (internal) |

### Build Steps

1. **Create new schematic** in KiCad
2. **Place U1** (XIAO nRF52840) in center
3. **Place U2** (HX711) to the left of U1
4. **Place LC1** (Load Cell) to the left of HX711
5. **Place U3** (Hall Sensor) above HX711
6. **Place LED1** (WS2812B) to the right of U1
7. **Place BT1** (Battery) below U1
8. **Add power symbols**: VCC, +5V, GND
9. **Wire connections** per table above
10. **Add power flags** on battery connections
11. **Run ERC** to check for errors

---

## BASE STATION CIRCUIT (ESP32)

### Component List

| Ref | Component | KiCad Symbol Library | Symbol Name | Notes |
|-----|-----------|---------------------|-------------|-------|
| U1 | ESP32 DevKit | **Personal_Library** | **ESP32_DevKit** | Custom symbol |
| U2 | SHT41 Temp/Humidity | **Personal_Library** | **SHT41** | Custom symbol |
| U3 | HW-111 RTC (DS1307) | **Personal_Library** | **DS1307_RTC** | Custom symbol |
| U4 | SD Card Module | **Personal_Library** | **SD_Card_Module** | Custom symbol |
| U5 | SSD1333 OLED Display | **Personal_Library** | **SSD1333_OLED** | Custom symbol |
| SW1 | Rotary Encoder KY-040 | **Personal_Library** | **KY-040** | Custom symbol |
| BZ1 | Active Buzzer | Device | Buzzer | Built-in KiCad symbol |

### Pin Connections Table

#### ESP32 Connections

| ESP32 GPIO | Connected To | Component Pin | Signal Type | Bus |
|------------|--------------|---------------|-------------|-----|
| GPIO21 | SHT41, RTC | SDA | I2C Data | I2C |
| GPIO22 | SHT41, RTC | SCL | I2C Clock | I2C |
| GPIO23 | SD Card, OLED | MOSI | SPI Data Out | SPI |
| GPIO19 | SD Card | MISO | SPI Data In | SPI |
| GPIO18 | SD Card, OLED | SCK | SPI Clock | SPI |
| GPIO5 | SD Card | CS | Chip Select | SPI |
| GPIO15 | OLED | CS | Chip Select | SPI |
| GPIO2 | OLED | DC | Data/Command | - |
| GPIO16 | OLED | RST | Reset | - |
| GPIO25 | Rotary Encoder | CLK | Encoder A | - |
| GPIO26 | Rotary Encoder | DT | Encoder B | - |
| GPIO34 | Rotary Encoder | SW | Push Button | - |
| GPIO4 | Buzzer | + | PWM Output | - |
| 3V3 | All peripherals | VCC | Power | - |
| GND | All peripherals | GND | Ground | - |

#### I2C Bus (GPIO21=SDA, GPIO22=SCL)

| Device | I2C Address | Connections |
|--------|-------------|-------------|
| SHT41 | 0x44 | SDA, SCL, VCC (3V3), GND |
| DS1307 (HW-111) | 0x68 | SDA, SCL, VCC (3V3), GND |

#### SPI Bus (GPIO18=SCK, GPIO23=MOSI, GPIO19=MISO)

| Device | CS Pin | MOSI | MISO | SCK | Notes |
|--------|--------|------|------|-----|-------|
| SD Card | GPIO5 | Yes | Yes | Yes | Full SPI |
| OLED | GPIO15 | Yes | No | Yes | Write-only SPI |

#### SHT41 Connections

| SHT41 Pin | Connected To |
|-----------|--------------|
| VDD | ESP32 3V3 |
| GND | Common GND |
| SDA | ESP32 GPIO21 |
| SCL | ESP32 GPIO22 |

#### HW-111 RTC Connections

| RTC Pin | Connected To | Notes |
|---------|--------------|-------|
| VCC | ESP32 3V3 | |
| GND | Common GND | |
| SDA | ESP32 GPIO21 | Shared I2C |
| SCL | ESP32 GPIO22 | Shared I2C |
| SQW | Not connected | Optional |
| BAT | Coin cell | Backup battery |

#### SD Card Module Connections

| SD Pin | Connected To |
|--------|--------------|
| VCC | ESP32 3V3 or 5V |
| GND | Common GND |
| MOSI | ESP32 GPIO23 |
| MISO | ESP32 GPIO19 |
| SCK | ESP32 GPIO18 |
| CS | ESP32 GPIO5 |

#### OLED Display (SSD1333) Connections

| OLED Pin | Connected To | Notes |
|----------|--------------|-------|
| VCC | ESP32 3V3 | |
| GND | Common GND | |
| MOSI/SDA | ESP32 GPIO23 | Shared SPI |
| SCK/SCL | ESP32 GPIO18 | Shared SPI |
| CS | ESP32 GPIO15 | |
| DC | ESP32 GPIO2 | Data/Command |
| RST | ESP32 GPIO16 | Reset |

#### Rotary Encoder Connections

| Encoder Pin | Connected To | Notes |
|-------------|--------------|-------|
| CLK (A) | ESP32 GPIO25 | |
| DT (B) | ESP32 GPIO26 | |
| SW | ESP32 GPIO34 | Push button, input-only pin |
| + | ESP32 3V3 | |
| GND | Common GND | |

#### Buzzer Connections

| Buzzer Pin | Connected To | Notes |
|------------|--------------|-------|
| + | ESP32 GPIO4 | Active buzzer |
| - | Common GND | |

### Build Steps

1. **Create new schematic** in KiCad
2. **Place U1** (ESP32) in center
3. **Place I2C devices** (U2, U3) above ESP32
4. **Place SPI devices** (U4, U5) to the left of ESP32
5. **Place SW1** (Rotary Encoder) to the right of ESP32
6. **Place BZ1** (Buzzer) below ESP32
7. **Add power symbols**: +3V3, GND
8. **Draw I2C bus** with labels (SDA, SCL)
9. **Draw SPI bus** with labels (MOSI, MISO, SCK)
10. **Wire all connections** per tables above
11. **Add net labels** for clarity (I2C_SDA, I2C_SCL, SPI_MOSI, etc.)
12. **Run ERC** to check for errors

---

## Tips for KiCad

1. **Use net labels** instead of long wires for cleaner schematics
2. **Group by function**: I2C devices together, SPI devices together
3. **Use bus notation** for multi-signal connections
4. **Add power flags** on power input pins
5. **Use hierarchical sheets** if schematic gets complex
6. **Add notes** using text labels for clarity

## Export for Report

1. **Plot** > **PDF** for vector output
2. Or **File** > **Export** > **SVG** for editing
3. Recommended: Use monochrome color scheme for print
