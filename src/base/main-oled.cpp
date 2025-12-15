/**
 * Coaster-Perso - Base Station
 *
 * ESP32 DevKit + écran OLED SSD1333 (NHD-1.91-176176UBC3)
 * + capteur température/humidité SHT4x
 *
 * Affiche les données environnementales sur l'écran OLED
 */

#include <Arduino.h>
#include "oled-ssd1333.h"
#include "Adafruit_SHT4x.h"

// ============================================
// Configuration des pins ESP32
// ============================================

// Écran OLED SSD1333 (SPI)
#define OLED_CS     5
#define OLED_DC     4
#define OLED_RST    16
#define OLED_SCLK   18
#define OLED_MOSI   23

// Capteur SHT4x (I2C) - pins par défaut ESP32
// SDA = GPIO21, SCL = GPIO22

// ============================================
// Objets globaux
// ============================================

SSD1333 display(OLED_CS, OLED_DC, OLED_RST, OLED_SCLK, OLED_MOSI);
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

// Variables pour les données capteur
float temperature = 0;
float humidity = 0;
bool sensorOk = false;

// Timing
unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 2000;  // 2 secondes

// ============================================
// Prototypes
// ============================================

void initDisplay();
void initSensor();
void readSensor();
void updateDisplay();
void drawHeader();
void drawTemperature();
void drawHumidity();
void drawStatus();

// ============================================
// Setup
// ============================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("================================");
    Serial.println("Coaster-Perso - Base Station");
    Serial.println("================================");

    initDisplay();
    initSensor();

    // Premier affichage
    updateDisplay();
}

// ============================================
// Loop
// ============================================

void loop() {
    unsigned long now = millis();

    if (now - lastUpdate >= UPDATE_INTERVAL) {
        lastUpdate = now;
        readSensor();
        updateDisplay();
    }
}

// ============================================
// Initialisation écran
// ============================================

void initDisplay() {
    Serial.println("Init OLED SSD1333...");

    if (!display.begin()) {
        Serial.println("ERREUR: OLED non detecte!");
        while (1) delay(100);
    }

    display.fillScreen(COLOR_BLACK);
    display.setTextSize(2);
    display.setTextColor(COLOR_CYAN);
    display.setCursor(20, 70);
    display.print("Coaster");
    display.setCursor(35, 95);
    display.print("Perso");

    delay(1500);
    Serial.println("OLED OK");
}

// ============================================
// Initialisation capteur SHT4x
// ============================================

void initSensor() {
    Serial.println("Init SHT4x...");

    if (!sht4.begin()) {
        Serial.println("ERREUR: SHT4x non detecte!");
        sensorOk = false;
        return;
    }

    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);

    sensorOk = true;
    Serial.println("SHT4x OK");
}

// ============================================
// Lecture capteur
// ============================================

void readSensor() {
    if (!sensorOk) return;

    sensors_event_t humidityEvent, tempEvent;

    if (sht4.getEvent(&humidityEvent, &tempEvent)) {
        temperature = tempEvent.temperature;
        humidity = humidityEvent.relative_humidity;

        Serial.printf("T: %.1f C  H: %.0f %%\n", temperature, humidity);
    } else {
        Serial.println("Erreur lecture SHT4x");
    }
}

// ============================================
// Mise à jour affichage
// ============================================

void updateDisplay() {
    display.fillScreen(COLOR_BLACK);

    drawHeader();
    drawTemperature();
    drawHumidity();
    drawStatus();
}

void drawHeader() {
    // Titre
    display.setTextSize(2);
    display.setTextColor(COLOR_CYAN);
    display.setCursor(15, 5);
    display.print("COASTER");

    // Ligne séparatrice
    display.drawFastHLine(10, 28, 156, COLOR_GRAY);
}

void drawTemperature() {
    // Icône thermomètre (rectangle simple)
    display.fillRect(20, 45, 8, 30, COLOR_RED);
    display.fillRect(17, 70, 14, 14, COLOR_RED);

    // Label
    display.setTextSize(1);
    display.setTextColor(COLOR_WHITE);
    display.setCursor(45, 45);
    display.print("Temperature");

    // Valeur
    display.setTextSize(3);
    if (sensorOk) {
        if (temperature < 18) {
            display.setTextColor(COLOR_CYAN);
        } else if (temperature > 28) {
            display.setTextColor(COLOR_RED);
        } else {
            display.setTextColor(COLOR_GREEN);
        }
        char buf[8];
        snprintf(buf, sizeof(buf), "%.1f", temperature);
        display.setCursor(45, 58);
        display.print(buf);

        // Unité
        display.setTextSize(2);
        display.setCursor(130, 60);
        display.print("C");
    } else {
        display.setTextColor(COLOR_GRAY);
        display.setCursor(45, 58);
        display.print("--.-");
    }
}

void drawHumidity() {
    // Icône goutte (triangle + cercle simplifié)
    display.fillRect(20, 105, 10, 20, COLOR_BLUE);
    display.fillRect(17, 120, 16, 10, COLOR_BLUE);

    // Label
    display.setTextSize(1);
    display.setTextColor(COLOR_WHITE);
    display.setCursor(45, 100);
    display.print("Humidite");

    // Valeur
    display.setTextSize(3);
    if (sensorOk) {
        if (humidity < 30) {
            display.setTextColor(COLOR_ORANGE);
        } else if (humidity > 70) {
            display.setTextColor(COLOR_CYAN);
        } else {
            display.setTextColor(COLOR_GREEN);
        }
        char buf[8];
        snprintf(buf, sizeof(buf), "%.0f", humidity);
        display.setCursor(45, 113);
        display.print(buf);

        // Unité
        display.setTextSize(2);
        display.setCursor(115, 115);
        display.print("%");
    } else {
        display.setTextColor(COLOR_GRAY);
        display.setCursor(45, 113);
        display.print("--");
    }
}

void drawStatus() {
    // Ligne séparatrice
    display.drawFastHLine(10, 148, 156, COLOR_GRAY);

    // Status bar
    display.setTextSize(1);

    if (sensorOk) {
        display.setTextColor(COLOR_GREEN);
        display.setCursor(10, 158);
        display.print("SHT4x OK");
    } else {
        display.setTextColor(COLOR_RED);
        display.setCursor(10, 158);
        display.print("NO SENSOR");
    }

    // Uptime
    display.setTextColor(COLOR_GRAY);
    display.setCursor(100, 158);
    unsigned long uptime = millis() / 1000;
    char buf[16];
    snprintf(buf, sizeof(buf), "%lum%02lus", uptime / 60, uptime % 60);
    display.print(buf);
}
