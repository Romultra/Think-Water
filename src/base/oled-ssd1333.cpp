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

    writeCommand(SSD1333_CMD_SET_LOCK);
    writeData(0x12);

    writeCommand(SSD1333_CMD_DISPLAY_OFF);

    writeCommand(SSD1333_CMD_SET_CLK_DIV);
    writeData(0xD1);

    writeCommand(SSD1333_CMD_SET_MUX_RATIO);
    writeData(0xAF);

    writeCommand(SSD1333_CMD_SET_OFFSET);
    writeData(0x30);

    writeCommand(SSD1333_CMD_SET_START_LINE);
    writeData(0x00);

    writeCommand(SSD1333_CMD_SET_REMAP);
    writeData(0x74);

    writeCommand(SSD1333_CMD_SET_MASTER_CFG);
    writeData(0x8E);

    writeCommand(SSD1333_CMD_CONTRAST_A);
    writeData(0x64);

    writeCommand(SSD1333_CMD_CONTRAST_B);
    writeData(0x78);

    writeCommand(SSD1333_CMD_CONTRAST_C);
    writeData(0x64);

    writeCommand(SSD1333_CMD_MASTER_CURRENT);
    writeData(0x0F);

    writeCommand(SSD1333_CMD_SET_PRECHARGE);
    writeData(0x22);

    writeCommand(SSD1333_CMD_SET_PRECHARGE_V);
    writeData(0x1F);

    writeCommand(SSD1333_CMD_SET_VCOMH);
    writeData(0x05);

    writeCommand(SSD1333_CMD_NORMAL_DISPLAY);
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
