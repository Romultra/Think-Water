/**
 * SSD1333 OLED Driver Library - Implementation
 *
 * Driver pour écran OLED NHD-1.91-176176UBC3
 * 176x176 pixels, 65K couleurs, interface SPI 4-wire
 *
 * Hérite d'Adafruit_GFX - seules les fonctions spécifiques au hardware sont implémentées
 */

#include "oled-ssd1333.h"

// ============================================
// Constructeur
// ============================================

SSD1333::SSD1333(int8_t cs, int8_t dc, int8_t rst, int8_t sclk, int8_t mosi)
    : Adafruit_GFX(SSD1333_WIDTH, SSD1333_HEIGHT),
      _cs(cs), _dc(dc), _rst(rst), _sclk(sclk), _mosi(mosi) {
}

// ============================================
// Initialisation
// ============================================

bool SSD1333::begin() {
    pinMode(_cs, OUTPUT);
    pinMode(_dc, OUTPUT);
    digitalWrite(_cs, HIGH);
    digitalWrite(_dc, HIGH);

    if (_sclk >= 0 && _mosi >= 0) {
        _spi = new SPIClass(HSPI);
        _spi->begin(_sclk, -1, _mosi, _cs);
    } else {
        _spi = &SPI;
        _spi->begin();
    }

    _spiSettings = SPISettings(20000000, MSBFIRST, SPI_MODE0);

    hardwareReset();
    initSequence();
    fillScreen(COLOR_BLACK);

    return true;
}

void SSD1333::hardwareReset() {
    if (_rst >= 0) {
        pinMode(_rst, OUTPUT);
        digitalWrite(_rst, HIGH);
        delay(20);
        digitalWrite(_rst, LOW);
        delay(1);
        digitalWrite(_rst, HIGH);
        delay(20);
    }
}

void SSD1333::initSequence() {
    spiBegin();

    // Unlock driver IC
    writeCommand(SSD1333_CMD_SET_LOCK);
    writeData(0x12);

    // Display OFF
    writeCommand(SSD1333_CMD_DISPLAY_OFF);

    // Set Remap/Color Depth
    // A[0]=0: Horizontal address increment
    // A[1]=1: Column address remap (flip horizontal)
    // A[2]=1: Color sequence A->B->C
    // A[5]=1: Enable COM Split Odd Even
    // A[7:6]=01: 65k color
    writeCommand(SSD1333_CMD_SET_REMAP);
    writeData(0x66);

    // Display Start Line = 0
    writeCommand(SSD1333_CMD_SET_START_LINE);
    writeData(0x00);

    // Display Offset = 0 (was incorrectly 0x30)
    writeCommand(SSD1333_CMD_SET_OFFSET);
    writeData(0x00);

    // Normal Display Mode
    writeCommand(SSD1333_CMD_NORMAL_DISPLAY);

    // Phase 1 & 2 period (0x24 per NHD datasheet)
    // Phase 1 = 4 (8 DCLKs), Phase 2 = 2 (4 DCLKs)
    writeCommand(SSD1333_CMD_SET_PRECHARGE);
    writeData(0x24);

    // Clock Divider / Oscillator Frequency (0x80 per NHD datasheet)
    // Divide by 1, oscillator at level 8
    writeCommand(SSD1333_CMD_SET_CLK_DIV);
    writeData(0x80);

    // Second Pre-charge Period (was missing!)
    writeCommand(SSD1333_CMD_SET_PRECHARGE2);
    writeData(0x0F);

    // Pre-charge Voltage Level = 0.40*VCC
    writeCommand(SSD1333_CMD_SET_PRECHARGE_V);
    writeData(0x1F);

    // VCOMH Deselect Level = 0.82*VCC
    writeCommand(SSD1333_CMD_SET_VCOMH);
    writeData(0x05);

    // Contrast for Color A, B, C (per NHD datasheet)
    writeCommand(SSD1333_CMD_CONTRAST_A);
    writeData(0x3C);
    writeCommand(SSD1333_CMD_CONTRAST_B);
    writeData(0x39);
    writeCommand(SSD1333_CMD_CONTRAST_C);
    writeData(0x43);

    // Master Current Control (no change)
    writeCommand(SSD1333_CMD_MASTER_CURRENT);
    writeData(0x0F);

    // MUX Ratio = 176 (0xAF = 175, so 176 MUX)
    writeCommand(SSD1333_CMD_SET_MUX_RATIO);
    writeData(0xAF);

    // Gamma Look-Up Table (required for proper grayscale)
    writeCommand(SSD1333_CMD_SET_GRAY_LUT);
    // GS1-GS63 values from NHD datasheet example
    static const uint8_t gamma_lut[] = {
        0x00, 0x02, 0x04, 0x06, 0x07, 0x08, 0x09, 0x0A,  // GS1-8
        0x0B, 0x0C, 0x0D, 0x0E, 0x10, 0x11, 0x12, 0x14,  // GS9-16
        0x15, 0x16, 0x18, 0x1A, 0x1B, 0x1D, 0x1F, 0x20,  // GS17-24
        0x22, 0x24, 0x26, 0x27, 0x29, 0x2B, 0x2D, 0x2F,  // GS25-32
        0x31, 0x33, 0x35, 0x37, 0x39, 0x3B, 0x3D, 0x3F,  // GS33-40
        0x42, 0x44, 0x46, 0x48, 0x4B, 0x4D, 0x50, 0x52,  // GS41-48
        0x55, 0x57, 0x5A, 0x5B, 0x5F, 0x62, 0x65, 0x67,  // GS49-56
        0x6A, 0x6D, 0x70, 0x73, 0x76, 0x79, 0x7C         // GS57-63
    };
    for (int i = 0; i < 63; i++) {
        writeData(gamma_lut[i]);
    }

    // Display ON
    writeCommand(SSD1333_CMD_DISPLAY_ON);

    spiEnd();
    delay(100);
}

// ============================================
// Contrôle affichage
// ============================================

void SSD1333::displayOn() {
    spiBegin();
    writeCommand(SSD1333_CMD_DISPLAY_ON);
    spiEnd();
}

void SSD1333::displayOff() {
    spiBegin();
    writeCommand(SSD1333_CMD_DISPLAY_OFF);
    spiEnd();
}

void SSD1333::setContrast(uint8_t level) {
    uint8_t val = map(level, 0, 15, 0, 255);
    setContrastRGB(val, val, val);
}

void SSD1333::setContrastRGB(uint8_t r, uint8_t g, uint8_t b) {
    spiBegin();
    writeCommand(SSD1333_CMD_CONTRAST_C);
    writeData(r);
    writeCommand(SSD1333_CMD_CONTRAST_B);
    writeData(g);
    writeCommand(SSD1333_CMD_CONTRAST_A);
    writeData(b);
    spiEnd();
}

void SSD1333::invertDisplay(bool invert) {
    spiBegin();
    writeCommand(invert ? SSD1333_CMD_INVERT_DISPLAY : SSD1333_CMD_NORMAL_DISPLAY);
    spiEnd();
}

// ============================================
// Fonctions de dessin (override Adafruit_GFX)
// ============================================

void SSD1333::setAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    writeCommand(SSD1333_CMD_SET_COLUMN);
    writeData(x0);
    writeData(x1);

    writeCommand(SSD1333_CMD_SET_ROW);
    writeData(y0);
    writeData(y1);

    writeCommand(SSD1333_CMD_WRITE_RAM);
}

void SSD1333::drawPixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || x >= width() || y < 0 || y >= height()) return;

    spiBegin();
    setAddressWindow(x, y, x, y);
    writeData16(color);
    spiEnd();
}

void SSD1333::fillScreen(uint16_t color) {
    fillRect(0, 0, width(), height(), color);
}

void SSD1333::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
    if (y < 0 || y >= height()) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > width()) { w = width() - x; }
    if (w <= 0) return;

    spiBegin();
    setAddressWindow(x, y, x + w - 1, y);
    writeColor(color, w);
    spiEnd();
}

void SSD1333::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
    if (x < 0 || x >= width()) return;
    if (y < 0) { h += y; y = 0; }
    if (y + h > height()) { h = height() - y; }
    if (h <= 0) return;

    spiBegin();
    setAddressWindow(x, y, x, y + h - 1);
    writeColor(color, h);
    spiEnd();
}

void SSD1333::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > width()) { w = width() - x; }
    if (y + h > height()) { h = height() - y; }
    if (w <= 0 || h <= 0) return;

    spiBegin();
    setAddressWindow(x, y, x + w - 1, y + h - 1);
    writeColor(color, (uint32_t)w * h);
    spiEnd();
}

void SSD1333::drawRGBBitmap(int16_t x, int16_t y, const uint16_t* bitmap, int16_t w, int16_t h) {
    if (x >= width() || y >= height()) return;

    spiBegin();
    setAddressWindow(x, y, x + w - 1, y + h - 1);

    for (int16_t j = 0; j < h; j++) {
        for (int16_t i = 0; i < w; i++) {
            writeData16(bitmap[j * w + i]);
        }
    }
    spiEnd();
}

// ============================================
// Communication SPI
// ============================================

void SSD1333::spiBegin() {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_cs, LOW);
}

void SSD1333::spiEnd() {
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();
}

void SSD1333::writeCommand(uint8_t cmd) {
    digitalWrite(_dc, LOW);
    _spi->transfer(cmd);
    digitalWrite(_dc, HIGH);
}

void SSD1333::writeData(uint8_t data) {
    _spi->transfer(data);
}

void SSD1333::writeData16(uint16_t data) {
    _spi->transfer16(data);
}

void SSD1333::writeColor(uint16_t color, uint32_t count) {
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    while (count--) {
        _spi->transfer(hi);
        _spi->transfer(lo);
    }
}
