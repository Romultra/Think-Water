/**
 * SSD1333 OLED Driver Library
 *
 * Driver pour écran OLED NHD-1.91-176176UBC3
 * 176x176 pixels, 65K couleurs, interface SPI
 *
 * Hérite d'Adafruit_GFX pour toutes les primitives graphiques et texte
 */

#ifndef OLED_SSD1333_H
#define OLED_SSD1333_H

#include <Adafruit_GFX.h>
#include <SPI.h>

// Dimensions écran NHD-1.91-176176UBC3
#define SSD1333_WIDTH  176
#define SSD1333_HEIGHT 176

// ============================================
// Commandes SSD1333 (datasheet)
// ============================================

#define SSD1333_CMD_NOP             0xE3
#define SSD1333_CMD_SET_COLUMN      0x15
#define SSD1333_CMD_SET_ROW         0x75
#define SSD1333_CMD_WRITE_RAM       0x5C
#define SSD1333_CMD_READ_RAM        0x5D

#define SSD1333_CMD_DISPLAY_OFF     0xAE
#define SSD1333_CMD_DISPLAY_ON      0xAF
#define SSD1333_CMD_DISPLAY_ALL_OFF 0xA4
#define SSD1333_CMD_DISPLAY_ALL_ON  0xA5
#define SSD1333_CMD_NORMAL_DISPLAY  0xA6
#define SSD1333_CMD_INVERT_DISPLAY  0xA7

#define SSD1333_CMD_SET_REMAP       0xA0
#define SSD1333_CMD_SET_START_LINE  0xA1
#define SSD1333_CMD_SET_OFFSET      0xA2
#define SSD1333_CMD_SET_MUX_RATIO   0xA8

#define SSD1333_CMD_CONTRAST_A      0x81
#define SSD1333_CMD_CONTRAST_B      0x82
#define SSD1333_CMD_CONTRAST_C      0x83
#define SSD1333_CMD_MASTER_CURRENT  0x87

#define SSD1333_CMD_SET_PRECHARGE   0xB1
#define SSD1333_CMD_SET_CLK_DIV     0xB3
#define SSD1333_CMD_SET_PRECHARGE2  0xB6
#define SSD1333_CMD_SET_GRAY_LUT    0xB8
#define SSD1333_CMD_SET_PRECHARGE_V 0xBB
#define SSD1333_CMD_SET_VCOMH       0xBE

#define SSD1333_CMD_SET_LOCK        0xFD

// ============================================
// Couleurs RGB565 prédéfinies
// ============================================
#define COLOR_BLACK       0x0000
#define COLOR_WHITE       0xFFFF
#define COLOR_RED         0xF800
#define COLOR_GREEN       0x07E0
#define COLOR_BLUE        0x001F
#define COLOR_YELLOW      0xFFE0
#define COLOR_CYAN        0x07FF
#define COLOR_MAGENTA     0xF81F
#define COLOR_ORANGE      0xFD20
#define COLOR_GRAY        0x8410

// Macro pour créer une couleur RGB565
#define RGB565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))

class SSD1333 : public Adafruit_GFX {
public:
    /**
     * Constructeur pour interface SPI (4-wire)
     * @param cs   Pin Chip Select
     * @param dc   Pin Data/Command
     * @param rst  Pin Reset (-1 si non utilisé)
     * @param sclk Pin SPI Clock (optionnel, défaut VSPI)
     * @param mosi Pin SPI MOSI (optionnel, défaut VSPI)
     */
    SSD1333(int8_t cs, int8_t dc, int8_t rst = -1, int8_t sclk = -1, int8_t mosi = -1);

    bool begin();

    // Contrôle affichage
    void displayOn();
    void displayOff();
    void setContrast(uint8_t level);
    void setContrastRGB(uint8_t r, uint8_t g, uint8_t b);
    void invertDisplay(bool invert);

    // Override Adafruit_GFX - OBLIGATOIRE
    void drawPixel(int16_t x, int16_t y, uint16_t color) override;

    // Override Adafruit_GFX - OPTIMISATIONS (évite drawPixel en boucle)
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override;
    void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) override;
    void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) override;
    void fillScreen(uint16_t color) override;

    // Bitmap optimisé
    void drawRGBBitmap(int16_t x, int16_t y, const uint16_t* bitmap, int16_t w, int16_t h);

private:
    int8_t _cs, _dc, _rst, _sclk, _mosi;

    SPIClass* _spi;
    SPISettings _spiSettings;

    void hardwareReset();
    void initSequence();
    void setAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

    void spiBegin();
    void spiEnd();
    void writeCommand(uint8_t cmd);
    void writeData(uint8_t data);
    void writeData16(uint16_t data);
    void writeColor(uint16_t color, uint32_t count);
};

#endif // OLED_SSD1333_H
