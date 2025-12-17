// ESP32 Base Station Firmware with OLED UI and Rotary Encoder
// Manages hydration goals, displays progress, and provides user interface

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include "Adafruit_SHT4x.h"
#include "RTClib.h"
#include "oled-ssd1333.h"

// External water turnover prediction function
extern float predictWaterTurnoverCH(
    float bodyWeightKg,
    uint8_t sex,
    float humidityPercent,
    float ageYears,
    float temperatureC
);

// ============================================================================
// Pin Definitions (Updated to avoid conflicts)
// ============================================================================
#define PIN_SDA           21      // I2C SDA for SHT41 and RTC
#define PIN_SCL           22      // I2C SCL for SHT41 and RTC
#define PIN_SD_MOSI       23      // SPI MOSI (shared with OLED)
#define PIN_SD_MISO       19      // SPI MISO
#define PIN_SD_SCK        18      // SPI SCK (shared with OLED)
#define PIN_SD_CS         5       // SPI CS for SD card
#define PIN_BUZZER        4       // Active buzzer

// OLED Display (SSD1333) - uses default SPI, only specify CS/DC/RST
#define PIN_OLED_CS       15      // OLED Chip Select
#define PIN_OLED_DC       2       // OLED Data/Command
#define PIN_OLED_RST      16      // OLED Reset

// Rotary Encoder with Push Button
#define PIN_ENCODER_CLK   25      // Rotary encoder clock
#define PIN_ENCODER_DT    26      // Rotary encoder data
#define PIN_ENCODER_SW    34      // Rotary encoder switch (push button)

// ============================================================================
// Debug Configuration
// ============================================================================
#define DEBUG_ENCODER               true    // Set to true to debug encoder issues

// ============================================================================
// Timing Constants
// ============================================================================
#define SENSOR_READ_INTERVAL_MS     (15UL * 60UL * 1000UL)  // 15 minutes
#define BLE_SCAN_DURATION_S         5                        // 5 second scan
#define BLE_SCAN_INTERVAL_MS        (10UL * 1000UL)          // 10 seconds between scans
#define TRAJECTORY_CHECK_INTERVAL_MS (30UL * 60UL * 1000UL)  // 30 minutes
#define ALERT_COOLDOWN_MS           (30UL * 60UL * 1000UL)   // 30 minutes between alerts
#define CONNECTION_TIMEOUT_MS       5000                      // 5 second connection timeout
#define POST_CONNECT_DELAY_MS       2000                      // Wait for data after connect
#define DISPLAY_UPDATE_INTERVAL_MS  1000                      // Update display every second
#define SCREENSAVER_TIMEOUT_MS      (5UL * 60UL * 1000UL)    // 5 minutes of inactivity
#define BUZZER_HIGH_DURATION_MS      800                       // 200ms buzzer on for alerts
#define BUZZER_LOW_DURATION_MS       400                       // 100ms buzzer off between beeps

// ============================================================================
// Hydration Constants
// ============================================================================
#define LIQUID_INTAKE_FACTOR        0.65f    // 65% of water turnover is from liquids
#define BEHIND_TRAJECTORY_THRESHOLD 0.15f    // 15% behind expected pace
#define ACTIVE_HOUR_START           6        // 6:00 AM
#define ACTIVE_HOUR_END             22       // 10:00 PM

// ============================================================================
// UI Configuration
// ============================================================================
#define MAX_SENSOR_READINGS         96       // 24h at 15min intervals

// UI Screens
enum class Screen {
    SETUP_AGE,
    SETUP_SEX,
    SETUP_WEIGHT,
    SETUP_HOUR,
    SETUP_MINUTE,
    MAIN_DISPLAY,
    MENU,
    SETTINGS_AGE,
    SETTINGS_SEX,
    SETTINGS_WEIGHT,
    SETTINGS_HOUR,
    SETTINGS_MINUTE,
    DEBUG_MENU
};

enum class MenuOption {
    BACK_TO_MAIN,
    MODIFY_INFO,
    CHANGE_TIME,
    DEBUG
};

enum class DebugOption {
    BACK_TO_MENU,
    SEND_ALERT,
    SEND_GOAL,
    TRIGGER_DAY_CHANGE
};

// ============================================================================
// BLE Service and Characteristics UUIDs (must match coaster)
// ============================================================================
#define COASTER_SERVICE_UUID        "19B10010-E8F2-537E-4F6C-D104768A1214"
#define CONSUMPTION_CHAR_UUID       "19B10011-E8F2-537E-4F6C-D104768A1214"
#define COMMAND_CHAR_UUID           "19B10012-E8F2-537E-4F6C-D104768A1214"
#define GOAL_CHAR_UUID              "19B10013-E8F2-537E-4F6C-D104768A1214"
#define COASTER_MFG_COMPANY_ID      0xFFFF

// ============================================================================
// Data Structures
// ============================================================================

struct SensorReading {
    uint32_t timestamp;
    float temperature;
    float humidity;
};

struct ConsumptionEvent {
    uint32_t coasterTimestamp;
    uint32_t localTimestamp;
    float amountML;
};

struct HydrationState {
    uint32_t dayStartTimestamp;
    float dailyGoalML;
    float totalConsumedML;
    uint8_t remindersSent;
    bool goalSentToCoaster;
};

struct SensorBuffer {
    SensorReading readings[MAX_SENSOR_READINGS];
    uint8_t head;
    uint8_t count;
};

enum class BLEState {
    IDLE,
    SCANNING,
    CONNECTING,
    CONNECTED,
    DISCONNECTING
};

// User Configuration
struct UserConfig {
    uint8_t age;          // 1-120 years
    uint8_t sex;          // 0 = female, 1 = male
    uint8_t weight;       // 30-200 kg
    bool setupComplete;   // Has initial setup been completed?
    bool timeSet;         // Has time been configured?
};

// Rotary Encoder State
struct EncoderState {
    int16_t position;
    int8_t lastEncoded;         // Last encoded value for state tracking
    int8_t store;               // Accumulated state changes
    bool buttonPressed;
    bool buttonReleased;
    uint32_t lastRotationTime;  // For rotation tracking
    uint32_t lastButtonTime;    // For button debouncing
    bool lastButtonState;
    bool buttonCurrentState;
};

// ============================================================================
// Global State
// ============================================================================

// Hardware
SSD1333 display(PIN_OLED_CS, PIN_OLED_DC, PIN_OLED_RST);
RTC_DS1307 rtc;
Adafruit_SHT4x sht41;
bool rtcInitialized = false;
bool sensorInitialized = false;
bool sdInitialized = false;
bool displayInitialized = false;

// UI State
Screen currentScreen = Screen::SETUP_AGE;
MenuOption selectedMenuOption = MenuOption::BACK_TO_MAIN;
DebugOption selectedDebugOption = DebugOption::BACK_TO_MENU;
UserConfig userConfig = {35, 1, 70, false, false};
EncoderState encoder = {0, 0, 0, false, false, 0, 0, HIGH, HIGH};
uint32_t lastDisplayUpdate = 0;
uint32_t lastActivity = 0;
bool screenSaverActive = false;
uint8_t setupHour = 12;      // For time setup screens
uint8_t setupMinute = 0;     // For time setup screens

// Sensor data
SensorBuffer sensorBuffer = {{}, 0, 0};
uint32_t lastSensorReadTime = 0;
uint8_t lastKnownDay = 0;

// BLE
BLEState bleState = BLEState::IDLE;
BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pConsumptionChar = nullptr;
BLERemoteCharacteristic* pCommandChar = nullptr;
BLERemoteCharacteristic* pGoalChar = nullptr;
BLEScan* pBLEScan = nullptr;
BLEAdvertisedDevice* coasterDevice = nullptr;
volatile bool scanInProgress = false;
volatile bool coasterFound = false;
volatile bool coasterHasData = false;
bool alertPending = false;
bool goalPending = false;
float pendingGoal = 0.0f;
uint32_t connectStartTime = 0;

// Hydration tracking
HydrationState hydrationState = {0, 0.0f, 0.0f, 0, false};
uint32_t lastAlertTime = 0;
bool consumptionReceived = false;
ConsumptionEvent latestConsumption = {0, 0, 0.0f};

// ============================================================================
// Forward Declarations
// ============================================================================
void initHardware();
void initSPI();
void initDisplay();
void initRTC();
void initSensor();
void initSD();
void initBLE();
void initBuzzer();
void initEncoder();
void updateEncoder();
bool readSensor(SensorReading* reading);
void addSensorReading(const SensorReading& reading);
float getAverageTemperature();
float getAverageHumidity();
void logSensorReading(const SensorReading& reading);
void logConsumptionEvent(const ConsumptionEvent& event);
float loadTodayConsumption();
void saveUserConfig();
void loadUserConfig();
void processBLEState();
void startScan();
bool connectToCoaster();
void disconnectFromCoaster();
bool sendCommand(uint8_t cmd);
bool sendGoal(float goalML);
float calculateDailyGoal();
bool checkTrajectory();
void triggerAlert();
void soundBuzzer();
uint32_t getCurrentTimestamp();
uint8_t getHourOfDay();
float getDayProgress();
bool isNewDay();
void onNewDay();
void updateDisplay();
void drawSetupScreen();
void drawMainScreen();
void drawMenuScreen();
void drawDebugMenu();
void handleEncoderInput();
void wakeFromScreenSaver();

// ============================================================================
// BLE Callbacks
// ============================================================================

static void consumptionNotifyCallback(
    BLERemoteCharacteristic* pBLERemoteCharacteristic,
    uint8_t* pData,
    size_t length,
    bool isNotify
) {
    if (length == 8) {
        uint32_t timestamp;
        int32_t scaledValue;
        memcpy(&timestamp, pData, 4);
        memcpy(&scaledValue, pData + 4, 4);

        latestConsumption.coasterTimestamp = timestamp;
        latestConsumption.localTimestamp = getCurrentTimestamp();
        latestConsumption.amountML = scaledValue / 100.0f;
        consumptionReceived = true;

        Serial.println("Consumption event received: " + String(latestConsumption.amountML) + " mL");
    }
}

class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) override {
        Serial.println("BLE: Connected to coaster");
    }

    void onDisconnect(BLEClient* pclient) override {
        Serial.println("BLE: Disconnected from coaster");
        // Clear characteristic pointers - they're invalid after disconnect
        pConsumptionChar = nullptr;
        pCommandChar = nullptr;
        pGoalChar = nullptr;
    }
};

// Static callback instance to avoid memory leak
static MyClientCallback clientCallback;

class MyScanCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        // Check if this is our coaster by service UUID
        if (advertisedDevice.haveServiceUUID() &&
            advertisedDevice.isAdvertisingService(BLEUUID(COASTER_SERVICE_UUID))) {

            // Parse manufacturer data to check data pending flag
            bool hasData = false;
            if (advertisedDevice.haveManufacturerData()) {
                std::string mfgData = advertisedDevice.getManufacturerData();
                if (mfgData.length() >= 3) {
                    uint16_t companyId = (uint8_t)mfgData[0] | ((uint8_t)mfgData[1] << 8);
                    if (companyId == COASTER_MFG_COMPANY_ID) {
                        uint8_t flag = (uint8_t)mfgData[2];
                        hasData = (flag == 0x01);
                    }
                }
            }

            // Update global state
            coasterHasData = hasData;

            // Store device for later connection
            if (coasterDevice != nullptr) {
                delete coasterDevice;
            }
            coasterDevice = new BLEAdvertisedDevice(advertisedDevice);
            coasterFound = true;

            // Only stop scan if we need to connect
            bool shouldConnect = hasData || alertPending || goalPending;
            if (shouldConnect && scanInProgress) {
                Serial.print("BLE: Coaster found, need to connect (data=");
                Serial.print(hasData ? "Y" : "N");
                Serial.print(" alert=");
                Serial.print(alertPending ? "Y" : "N");
                Serial.print(" goal=");
                Serial.print(goalPending ? "Y" : "N");
                Serial.println(")");
                scanInProgress = false;
                pBLEScan->stop();
            }
        }
    }
};

// ============================================================================
// Hardware Initialization
// ============================================================================

void initSPI() {
    // Initialize shared SPI bus for both SD card and OLED
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI);
    Serial.println("SPI bus initialized (shared for SD and OLED)");
}

void initDisplay() {
    Serial.println("Initializing OLED display...");
    if (display.begin()) {
        displayInitialized = true;

        // Show splash screen
        display.fillScreen(COLOR_BLACK);
        display.setTextSize(2);
        display.setTextColor(RGB565(100, 200, 255));
        display.setCursor(20, 70);
        display.print("Coaster");
        display.setCursor(35, 95);
        display.print("Clip");
        delay(1500);

        // Clear screen for later use
        display.fillScreen(COLOR_BLACK);

        Serial.println("OLED initialized");
    } else {
        Serial.println("ERROR: OLED initialization failed!");
        displayInitialized = false;
    }
}

void initRTC() {
    if (rtc.begin()) {
        rtcInitialized = true;
        if (!rtc.isrunning()) {
            Serial.println("RTC is NOT running! Setting to compile time...");
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
        DateTime now = rtc.now();
        lastKnownDay = now.day();
        Serial.println("RTC initialized");
    } else {
        Serial.println("ERROR: RTC not found!");
        rtcInitialized = false;
    }
}

void initSensor() {
    Wire.begin(PIN_SDA, PIN_SCL);
    if (sht41.begin()) {
        sht41.setPrecision(SHT4X_HIGH_PRECISION);
        sht41.setHeater(SHT4X_NO_HEATER);
        sensorInitialized = true;
        Serial.println("SHT41 sensor initialized");
    } else {
        Serial.println("ERROR: SHT41 sensor not found!");
        sensorInitialized = false;
    }
}

void initSD() {
    // SPI bus is already initialized in initSPI()
    if (SD.begin(PIN_SD_CS)) {
        sdInitialized = true;
        Serial.println("SD card initialized");
        if (!SD.exists("/hydration")) {
            SD.mkdir("/hydration");
        }
        if (!SD.exists("/config")) {
            SD.mkdir("/config");
        }
    } else {
        Serial.println("WARNING: SD card initialization failed!");
        sdInitialized = false;
    }
}

void initBLE() {
    BLEDevice::init("CoasterClip-Base");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyScanCallbacks(), true);  // true = allow duplicates
    pBLEScan->setInterval(100);   // 100 * 0.625ms = 62.5ms - fast scanning
    pBLEScan->setWindow(99);      // 99 * 0.625ms = 61.9ms - nearly continuous
    pBLEScan->setActiveScan(true);
    Serial.println("BLE initialized");
}

void initBuzzer() {
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
}

void initEncoder() {
    pinMode(PIN_ENCODER_CLK, INPUT_PULLUP);
    pinMode(PIN_ENCODER_DT, INPUT_PULLUP);
    pinMode(PIN_ENCODER_SW, INPUT_PULLUP);

    // Initialize encoder state
    uint8_t clk = digitalRead(PIN_ENCODER_CLK);
    uint8_t dt = digitalRead(PIN_ENCODER_DT);
    encoder.lastEncoded = (clk << 1) | dt;
    encoder.store = 0;
    encoder.lastButtonState = HIGH;
    encoder.buttonCurrentState = HIGH;
    encoder.position = 0;

    Serial.println("Rotary encoder initialized");
}

void initHardware() {
    initBuzzer();
    initSPI();       // Initialize shared SPI bus first
    initDisplay();   // OLED needs SPI
    initSensor();    // I2C sensor
    initRTC();       // I2C RTC
    initSD();        // SD card needs SPI
    initBLE();
    initEncoder();
}

// ============================================================================
// Encoder Functions
// ============================================================================

void updateEncoder() {
    uint32_t currentTime = millis();

    // Read current encoder pins
    uint8_t clk = digitalRead(PIN_ENCODER_CLK);
    uint8_t dt = digitalRead(PIN_ENCODER_DT);

    // Encode current state as 2-bit value
    int8_t encoded = (clk << 1) | dt;

    // Only process if state has changed
    if (encoded != encoder.lastEncoded) {
        // Lookup table for valid state transitions
        // Returns: 0 = invalid, 1 = CW, -1 = CCW
        // Index: (prevState << 2) | newState
        static const int8_t encStates[] = {
            0,  // 00 -> 00: no change
           -1,  // 00 -> 01: CCW
            1,  // 00 -> 10: CW
            0,  // 00 -> 11: invalid (2 bits changed)
            1,  // 01 -> 00: CW
            0,  // 01 -> 01: no change
            0,  // 01 -> 10: invalid (2 bits changed)
           -1,  // 01 -> 11: CCW
           -1,  // 10 -> 00: CCW
            0,  // 10 -> 01: invalid (2 bits changed)
            0,  // 10 -> 10: no change
            1,  // 10 -> 11: CW
            0,  // 11 -> 00: invalid (2 bits changed)
            1,  // 11 -> 01: CW
           -1,  // 11 -> 10: CCW
            0   // 11 -> 11: no change
        };

        int8_t index = (encoder.lastEncoded << 2) | encoded;
        int8_t dir = encStates[index];

        if (dir != 0) {
            // Valid transition - accumulate
            encoder.store += dir;
            encoder.lastRotationTime = currentTime;

            // Check if we've completed a full detent (2 pulses per detent)
            if (encoder.store >= 2) {
                encoder.position++;
                encoder.store = 0;
                lastActivity = currentTime;

                if (screenSaverActive) {
                    wakeFromScreenSaver();
                }
                #if DEBUG_ENCODER
                Serial.print(">>> DETENT CW, position now: ");
                Serial.println(encoder.position);
                #endif
            } else if (encoder.store <= -2) {
                encoder.position--;
                encoder.store = 0;
                lastActivity = currentTime;

                if (screenSaverActive) {
                    wakeFromScreenSaver();
                }
                #if DEBUG_ENCODER
                Serial.print(">>> DETENT CCW, position now: ");
                Serial.println(encoder.position);
                #endif
            }

            #if DEBUG_ENCODER
            if (dir != 0) {
                Serial.print("State: ");
                Serial.print(encoder.lastEncoded, BIN);
                Serial.print(" -> ");
                Serial.print(encoded, BIN);
                Serial.print(" dir=");
                Serial.print(dir);
                Serial.print(" store=");
                Serial.println(encoder.store);
            }
            #endif
        }

        encoder.lastEncoded = encoded;
    }

    // Read button with simple debouncing
    uint8_t buttonReading = digitalRead(PIN_ENCODER_SW);

    // Reset flags each cycle
    encoder.buttonReleased = false;

    // Detect state change
    if (buttonReading != encoder.lastButtonState) {
        encoder.lastButtonTime = currentTime;
        encoder.lastButtonState = buttonReading;
    }

    // After debounce period (50ms), update actual state
    if (currentTime - encoder.lastButtonTime > 50) {
        if (buttonReading != encoder.buttonCurrentState) {
            encoder.buttonCurrentState = buttonReading;

            if (buttonReading == LOW) {
                // Button pressed
                encoder.buttonPressed = true;
                #if DEBUG_ENCODER
                Serial.println("*** BUTTON PRESSED ***");
                #endif
            } else {
                // Button released
                encoder.buttonPressed = false;
                encoder.buttonReleased = true;  // Set release flag
                lastActivity = currentTime;
                if (screenSaverActive) {
                    wakeFromScreenSaver();
                }
                #if DEBUG_ENCODER
                Serial.println("*** BUTTON RELEASED ***");
                #endif
            }
        }
    }
}

// ============================================================================
// User Configuration Storage
// ============================================================================

void saveUserConfig() {
    if (!sdInitialized) return;

    File file = SD.open("/config/user.cfg", FILE_WRITE);
    if (file) {
        file.write((uint8_t*)&userConfig, sizeof(UserConfig));
        file.close();
        Serial.println("User configuration saved");
    } else {
        Serial.println("ERROR: Failed to save user configuration");
    }
}

void loadUserConfig() {
    if (!sdInitialized) return;

    File file = SD.open("/config/user.cfg", FILE_READ);
    if (file && file.size() == sizeof(UserConfig)) {
        file.read((uint8_t*)&userConfig, sizeof(UserConfig));
        file.close();
        Serial.println("User configuration loaded");
        Serial.printf("  Age: %d, Sex: %s, Weight: %d kg\n",
                      userConfig.age,
                      userConfig.sex ? "Male" : "Female",
                      userConfig.weight);
        Serial.printf("  Setup complete: %s, Time set: %s\n",
                      userConfig.setupComplete ? "Yes" : "No",
                      userConfig.timeSet ? "Yes" : "No");
    } else {
        Serial.println("No saved configuration found, using defaults");
        if (file) file.close();
    }
}

// ============================================================================
// Sensor Functions
// ============================================================================

bool readSensor(SensorReading* reading) {
    if (!sensorInitialized) {
        reading->timestamp = getCurrentTimestamp();
        reading->temperature = 20.0f;
        reading->humidity = 50.0f;
        return false;
    }

    sensors_event_t humidity, temp;
    if (sht41.getEvent(&humidity, &temp)) {
        reading->timestamp = getCurrentTimestamp();
        reading->temperature = temp.temperature;
        reading->humidity = humidity.relative_humidity;
        return true;
    }
    return false;
}

void addSensorReading(const SensorReading& reading) {
    sensorBuffer.readings[sensorBuffer.head] = reading;
    sensorBuffer.head = (sensorBuffer.head + 1) % MAX_SENSOR_READINGS;
    if (sensorBuffer.count < MAX_SENSOR_READINGS) {
        sensorBuffer.count++;
    }
}

float getAverageTemperature() {
    if (sensorBuffer.count == 0) return 20.0f;
    float sum = 0.0f;
    for (uint8_t i = 0; i < sensorBuffer.count; i++) {
        sum += sensorBuffer.readings[i].temperature;
    }
    return sum / sensorBuffer.count;
}

float getAverageHumidity() {
    if (sensorBuffer.count == 0) return 50.0f;
    float sum = 0.0f;
    for (uint8_t i = 0; i < sensorBuffer.count; i++) {
        sum += sensorBuffer.readings[i].humidity;
    }
    return sum / sensorBuffer.count;
}

void logSensorReading(const SensorReading& reading) {
    if (!sdInitialized) return;
    File file = SD.open("/hydration/SENSOR.CSV", FILE_APPEND);
    if (file) {
        file.print(reading.timestamp);
        file.print(",");
        file.print(reading.temperature, 2);
        file.print(",");
        file.println(reading.humidity, 2);
        file.close();
    }
}

void logConsumptionEvent(const ConsumptionEvent& event) {
    if (!sdInitialized) return;
    File file = SD.open("/hydration/CONSUME.CSV", FILE_APPEND);
    if (file) {
        file.print(event.localTimestamp);
        file.print(",");
        file.println(event.amountML, 2);
        file.close();
    }
}

float loadTodayConsumption() {
    if (!sdInitialized || !rtcInitialized) return 0.0f;
    uint32_t now = getCurrentTimestamp();
    uint32_t todayStart = now - (now % 86400);
    float total = 0.0f;

    File file = SD.open("/hydration/CONSUME.CSV", FILE_READ);
    if (file) {
        while (file.available()) {
            String line = file.readStringUntil('\n');
            int commaPos = line.indexOf(',');
            if (commaPos > 0) {
                uint32_t timestamp = line.substring(0, commaPos).toInt();
                float amount = line.substring(commaPos + 1).toFloat();
                if (timestamp >= todayStart) {
                    total += amount;
                }
            }
        }
        file.close();
    }
    return total;
}

// ============================================================================
// RTC Functions
// ============================================================================

uint32_t getCurrentTimestamp() {
    if (!rtcInitialized) {
        return millis() / 1000;
    }
    return rtc.now().unixtime();
}

uint8_t getHourOfDay() {
    if (!rtcInitialized) {
        return 12;
    }
    return rtc.now().hour();
}

float getDayProgress() {
    uint8_t hour = getHourOfDay();
    if (hour < ACTIVE_HOUR_START) return 0.0f;
    if (hour >= ACTIVE_HOUR_END) return 1.0f;
    return (float)(hour - ACTIVE_HOUR_START) / (float)(ACTIVE_HOUR_END - ACTIVE_HOUR_START);
}

bool isNewDay() {
    if (!rtcInitialized) return false;
    DateTime now = rtc.now();
    uint8_t currentDay = now.day();
    if (currentDay != lastKnownDay) {
        lastKnownDay = currentDay;
        return true;
    }
    return false;
}

// ============================================================================
// BLE Functions
// ============================================================================

void startScan() {
    coasterFound = false;
    scanInProgress = true;
    bleState = BLEState::SCANNING;
    // Start continuous scan - runs until we stop it
    pBLEScan->start(0, nullptr, false);
}

bool connectToCoaster() {
    if (coasterDevice == nullptr) return false;

    // Reuse existing client or create new one
    if (pClient == nullptr) {
        pClient = BLEDevice::createClient();
        pClient->setClientCallbacks(&clientCallback);
    }

    Serial.print("BLE: Connecting to ");
    Serial.println(coasterDevice->getAddress().toString().c_str());

    if (!pClient->connect(coasterDevice)) {
        Serial.println("BLE: Connection failed");
        return false;
    }

    pClient->setMTU(517);

    BLERemoteService* pRemoteService = pClient->getService(COASTER_SERVICE_UUID);
    if (pRemoteService == nullptr) {
        Serial.println("BLE: Service not found");
        pClient->disconnect();
        return false;
    }

    pConsumptionChar = pRemoteService->getCharacteristic(CONSUMPTION_CHAR_UUID);
    pCommandChar = pRemoteService->getCharacteristic(COMMAND_CHAR_UUID);
    pGoalChar = pRemoteService->getCharacteristic(GOAL_CHAR_UUID);

    if (pConsumptionChar == nullptr || pCommandChar == nullptr || pGoalChar == nullptr) {
        Serial.println("BLE: Characteristics not found");
        pClient->disconnect();
        return false;
    }

    if (pConsumptionChar->canNotify()) {
        pConsumptionChar->registerForNotify(consumptionNotifyCallback);
    }

    bleState = BLEState::CONNECTED;
    connectStartTime = millis();
    return true;
}

void disconnectFromCoaster() {
    if (pClient != nullptr && pClient->isConnected()) {
        pClient->disconnect();
    }
    bleState = BLEState::DISCONNECTING;
}

bool sendCommand(uint8_t cmd) {
    if (bleState != BLEState::CONNECTED || pCommandChar == nullptr) {
        return false;
    }
    uint8_t data[4] = {cmd, 0, 0, 0};
    pCommandChar->writeValue(data, 4, false);
    return true;
}

bool sendGoal(float goalML) {
    if (bleState != BLEState::CONNECTED || pGoalChar == nullptr) {
        return false;
    }
    uint8_t data[4];
    memcpy(data, &goalML, 4);
    pGoalChar->writeValue(data, 4, false);
    return true;
}

void processBLEState() {
    switch (bleState) {
        case BLEState::IDLE:
            // Start scanning immediately
            startScan();
            break;

        case BLEState::SCANNING:
            // Scan runs continuously. Callback stops it when we need to connect.
            if (!scanInProgress && coasterFound) {
                // Callback stopped the scan because we need to connect
                if (connectToCoaster()) {
                    // Success - state is now CONNECTED
                } else {
                    // Failed - resume scanning
                    startScan();
                }
            }
            break;

        case BLEState::CONNECTING:
            // Not used - connectToCoaster() is synchronous
            // If we somehow end up here, go back to IDLE
            bleState = BLEState::IDLE;
            break;

        case BLEState::CONNECTED:
            // Check for unexpected disconnect
            if (pClient == nullptr || !pClient->isConnected()) {
                Serial.println("BLE: Unexpected disconnect, resuming scan");
                startScan();
                break;
            }

            // Send pending goal first
            if (goalPending) {
                if (sendGoal(pendingGoal)) {
                    Serial.print("BLE: Sent goal ");
                    Serial.println(pendingGoal);
                    goalPending = false;
                    hydrationState.goalSentToCoaster = true;
                }
            }

            // Send pending alert
            if (alertPending) {
                if (sendCommand(0x01)) {
                    Serial.println("BLE: Alert sent to coaster");
                    alertPending = false;
                    lastAlertTime = millis();
                    hydrationState.remindersSent++;
                    soundBuzzer();  // Beep after coaster received the alert
                }
            }

            // Wait for potential incoming data, then disconnect
            if (millis() - connectStartTime > POST_CONNECT_DELAY_MS) {
                if (!goalPending && !alertPending) {
                    Serial.println("BLE: Operations complete, disconnecting");
                    disconnectFromCoaster();
                }
            }
            break;

        case BLEState::DISCONNECTING:
            // Wait for disconnect to complete, then resume scanning
            if (pClient == nullptr || !pClient->isConnected()) {
                startScan();
            }
            break;
    }
}

// ============================================================================
// Hydration Functions
// ============================================================================

float calculateDailyGoal() {
    float avgTemp = getAverageTemperature();
    float avgHumidity = getAverageHumidity();

    float waterTurnover = predictWaterTurnoverCH(
        (float)userConfig.weight,
        userConfig.sex,
        avgHumidity,
        (float)userConfig.age,
        avgTemp
    );

    return waterTurnover * LIQUID_INTAKE_FACTOR;
}

bool checkTrajectory() {
    if (hydrationState.dailyGoalML <= 0) return false;
    float dayProgress = getDayProgress();
    if (dayProgress <= 0) return false;
    float expected = hydrationState.dailyGoalML * dayProgress;
    float deficit = expected - hydrationState.totalConsumedML;
    float deficitRatio = deficit / expected;
    return (deficitRatio >= BEHIND_TRAJECTORY_THRESHOLD);
}

void onNewDay() {
    Serial.println("NEW DAY DETECTED!");
    hydrationState.dayStartTimestamp = getCurrentTimestamp() - (getCurrentTimestamp() % 86400);
    hydrationState.totalConsumedML = 0.0f;
    hydrationState.remindersSent = 0;
    hydrationState.goalSentToCoaster = false;
    hydrationState.dailyGoalML = calculateDailyGoal();
    pendingGoal = hydrationState.dailyGoalML;
    goalPending = true;
}

void soundBuzzer() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_BUZZER, HIGH);
        delay(BUZZER_HIGH_DURATION_MS);
        digitalWrite(PIN_BUZZER, LOW);
        delay(BUZZER_LOW_DURATION_MS);
    }
}

void triggerAlert() {
    Serial.println("HYDRATION ALERT: Queuing alert for coaster");
    alertPending = true;
}

bool canAlert() {
    if (millis() - lastAlertTime < ALERT_COOLDOWN_MS) return false;
    if (hydrationState.remindersSent >= 10) return false;
    return true;
}

// ============================================================================
// UI Drawing Functions
// ============================================================================

void wakeFromScreenSaver() {
    screenSaverActive = false;
    display.displayOn();
    lastDisplayUpdate = 0; // Force immediate redraw
}

void drawSetupScreen() {
    if (!displayInitialized) return;

    display.fillScreen(COLOR_BLACK);

    // Header
    display.fillRect(0, 0, 176, 30, RGB565(40, 80, 120));
    display.setTextSize(2);
    display.setTextColor(COLOR_WHITE);
    display.setCursor(25, 8);
    display.print("SETUP");

    display.setTextSize(1);
    display.setTextColor(RGB565(180, 180, 180));

    int16_t valueY = 80;
    int16_t labelY = 55;

    switch (currentScreen) {
        case Screen::SETUP_AGE:
        case Screen::SETTINGS_AGE: {
            display.setCursor(45, labelY);
            display.print("Your Age");

            display.setTextSize(4);
            display.setTextColor(RGB565(100, 200, 255));
            char buf[8];
            snprintf(buf, sizeof(buf), "%d", userConfig.age);
            int16_t textWidth = strlen(buf) * 24;
            display.setCursor((176 - textWidth) / 2, valueY);
            display.print(buf);

            display.setTextSize(2);
            display.setTextColor(COLOR_WHITE);
            display.setCursor(60, valueY + 45);
            display.print("years");

            // Hints
            display.setTextSize(1);
            display.setTextColor(RGB565(100, 100, 100));
            display.setCursor(25, 150);
            display.print("Rotate to adjust");
            display.setCursor(30, 162);
            display.print("Click to save");
            break;
        }

        case Screen::SETUP_SEX:
        case Screen::SETTINGS_SEX: {
            display.setCursor(50, labelY);
            display.print("Your Sex");

            display.setTextSize(3);
            display.setTextColor(userConfig.sex ? RGB565(100, 200, 255) : RGB565(255, 100, 150));
            const char* sexText = userConfig.sex ? "MALE" : "FEMALE";
            int16_t textWidth = strlen(sexText) * 18;
            display.setCursor((176 - textWidth) / 2, valueY);
            display.print(sexText);

            display.setTextSize(1);
            display.setTextColor(RGB565(100, 100, 100));
            display.setCursor(25, 150);
            display.print("Rotate to change");
            display.setCursor(30, 162);
            display.print("Click to save");
            break;
        }

        case Screen::SETUP_WEIGHT:
        case Screen::SETTINGS_WEIGHT: {
            display.setCursor(35, labelY);
            display.print("Your Weight");

            display.setTextSize(4);
            display.setTextColor(RGB565(100, 200, 255));
            char buf[8];
            snprintf(buf, sizeof(buf), "%d", userConfig.weight);
            int16_t textWidth = strlen(buf) * 24;
            display.setCursor((176 - textWidth) / 2, valueY);
            display.print(buf);

            display.setTextSize(2);
            display.setTextColor(COLOR_WHITE);
            display.setCursor(70, valueY + 45);
            display.print("kg");

            display.setTextSize(1);
            display.setTextColor(RGB565(100, 100, 100));
            display.setCursor(25, 150);
            display.print("Rotate to adjust");
            display.setCursor(30, 162);
            display.print("Click to save");
            break;
        }

        case Screen::SETUP_HOUR:
        case Screen::SETUP_MINUTE:
        case Screen::SETTINGS_HOUR:
        case Screen::SETTINGS_MINUTE: {
            display.setCursor(30, labelY);
            display.print("Set Time");

            if (currentScreen == Screen::SETUP_HOUR || currentScreen == Screen::SETTINGS_HOUR) {
                display.setTextSize(1);
                display.setTextColor(RGB565(180, 180, 180));
                display.setCursor(60, 70);
                display.print("Hour");

                display.setTextSize(4);
                display.setTextColor(RGB565(100, 200, 255));
                char buf[8];
                snprintf(buf, sizeof(buf), "%02d", setupHour);
                display.setCursor(58, 90);
                display.print(buf);
            } else {
                display.setTextSize(1);
                display.setTextColor(RGB565(180, 180, 180));
                display.setCursor(55, 70);
                display.print("Minute");

                display.setTextSize(4);
                display.setTextColor(RGB565(100, 200, 255));
                char buf[8];
                snprintf(buf, sizeof(buf), "%02d", setupMinute);
                display.setCursor(58, 90);
                display.print(buf);
            }

            display.setTextSize(1);
            display.setTextColor(RGB565(100, 100, 100));
            display.setCursor(25, 150);
            display.print("Rotate to adjust");
            display.setCursor(30, 162);
            display.print("Click to save");
            break;
        }

        default:
            break;
    }
}

void drawMainScreen() {
    if (!displayInitialized) {
        Serial.println("drawMainScreen: display not initialized!");
        return;
    }

    //Serial.println("drawMainScreen: filling screen...");
    display.fillScreen(RGB565(15, 20, 30));
    //Serial.println("drawMainScreen: screen filled");

    // Header with gradient effect
    display.fillRect(0, 0, 176, 35, RGB565(30, 60, 100));
    display.drawFastHLine(0, 35, 176, RGB565(100, 150, 200));

    display.setTextSize(2);
    display.setTextColor(COLOR_WHITE);
    display.setCursor(15, 10);
    display.print("HYDRATION");

    // Calculate progress percentage
    float progress = 0.0f;
    if (hydrationState.dailyGoalML > 0) {
        progress = (hydrationState.totalConsumedML / hydrationState.dailyGoalML) * 100.0f;
        if (progress > 100.0f) progress = 100.0f;
    }

    // Progress section
    display.setTextSize(1);
    display.setTextColor(RGB565(200, 200, 200));
    display.setCursor(10, 45);
    display.print("Daily Goal Progress");

    // Large progress display
    display.setTextSize(3);
    uint16_t progressColor;
    if (progress < 30) {
        progressColor = RGB565(255, 100, 100);
    } else if (progress < 70) {
        progressColor = RGB565(255, 200, 100);
    } else {
        progressColor = RGB565(100, 255, 150);
    }
    display.setTextColor(progressColor);

    char buf[16];
    snprintf(buf, sizeof(buf), "%.0f%%", progress);
    int16_t textWidth = strlen(buf) * 18;
    display.setCursor((176 - textWidth) / 2, 62);
    display.print(buf);

    // Progress bar
    int16_t barWidth = 150;
    int16_t barHeight = 12;
    int16_t barX = (176 - barWidth) / 2;
    int16_t barY = 90;

    display.drawRect(barX - 1, barY - 1, barWidth + 2, barHeight + 2, RGB565(100, 150, 200));
    display.fillRect(barX, barY, barWidth, barHeight, RGB565(30, 40, 50));

    int16_t fillWidth = (int16_t)((progress / 100.0f) * barWidth);
    if (fillWidth > 0) {
        display.fillRect(barX, barY, fillWidth, barHeight, progressColor);
    }

    // Consumption details
    display.setTextSize(1);
    display.setTextColor(RGB565(180, 180, 180));
    display.setCursor(10, 110);
    snprintf(buf, sizeof(buf), "%.0f / %.0f mL",
             hydrationState.totalConsumedML,
             hydrationState.dailyGoalML);
    display.print(buf);

    // Temperature and Humidity section
    display.drawFastHLine(5, 128, 166, RGB565(80, 80, 80));

    // Temperature
    SensorReading currentReading;
    readSensor(&currentReading);

    display.setTextSize(1);
    display.setTextColor(RGB565(200, 200, 200));
    display.setCursor(10, 135);
    display.print("Temp:");

    display.setTextSize(2);
    uint16_t tempColor;
    if (currentReading.temperature < 18) {
        tempColor = RGB565(100, 150, 255);
    } else if (currentReading.temperature > 26) {
        tempColor = RGB565(255, 150, 100);
    } else {
        tempColor = RGB565(150, 255, 150);
    }
    display.setTextColor(tempColor);
    display.setCursor(10, 148);
    snprintf(buf, sizeof(buf), "%.1f", currentReading.temperature);
    display.print(buf);
    display.setTextSize(1);
    display.setCursor(52, 152);
    display.print("C");

    // Humidity
    display.setTextSize(1);
    display.setTextColor(RGB565(200, 200, 200));
    display.setCursor(100, 135);
    display.print("Humid:");

    display.setTextSize(2);
    uint16_t humidColor;
    if (currentReading.humidity < 30) {
        humidColor = RGB565(255, 200, 100);
    } else if (currentReading.humidity > 70) {
        humidColor = RGB565(100, 150, 255);
    } else {
        humidColor = RGB565(150, 255, 150);
    }
    display.setTextColor(humidColor);
    display.setCursor(100, 148);
    snprintf(buf, sizeof(buf), "%.0f", currentReading.humidity);
    display.print(buf);
    display.setTextSize(1);
    display.setCursor(134, 152);
    display.print("%");

    // Time display (if RTC available)
    if (rtcInitialized) {
        DateTime now = rtc.now();
        display.setTextSize(1);
        display.setTextColor(RGB565(150, 150, 150));
        display.setCursor(130, 10);
        snprintf(buf, sizeof(buf), "%02d:%02d", now.hour(), now.minute());
        display.print(buf);
    }

    // Footer hint
    display.setTextSize(1);
    display.setTextColor(RGB565(100, 100, 100));
    display.setCursor(35, 167);
    display.print("Click for menu");
}

void drawMenuScreen() {
    if (!displayInitialized) return;

    display.fillScreen(RGB565(15, 20, 30));

    // Header
    display.fillRect(0, 0, 176, 30, RGB565(40, 80, 120));
    display.setTextSize(2);
    display.setTextColor(COLOR_WHITE);
    display.setCursor(45, 8);
    display.print("MENU");

    // Menu options
    const char* options[] = {
        "Back to Main",
        "Personal Info",
        "Change Time",
        "Debug"
    };

    int16_t optionY = 40;
    int16_t optionHeight = 28;

    for (int i = 0; i < 4; i++) {
        bool isSelected = (i == (int)selectedMenuOption);

        if (isSelected) {
            display.fillRect(5, optionY + i * optionHeight, 166, 25, RGB565(50, 100, 150));
            display.drawRect(5, optionY + i * optionHeight, 166, 25, RGB565(100, 200, 255));
        }

        display.setTextSize(1);
        display.setTextColor(isSelected ? COLOR_WHITE : RGB565(180, 180, 180));
        display.setCursor(15, optionY + i * optionHeight + 8);
        display.print(options[i]);
    }

    // Footer hint
    display.setTextSize(1);
    display.setTextColor(RGB565(100, 100, 100));
    display.setCursor(20, 167);
    display.print("Rotate & Click to select");
}

void drawDebugMenu() {
    if (!displayInitialized) return;

    display.fillScreen(RGB565(15, 20, 30));

    // Header
    display.fillRect(0, 0, 176, 30, RGB565(120, 40, 40));
    display.setTextSize(2);
    display.setTextColor(COLOR_WHITE);
    display.setCursor(35, 8);
    display.print("DEBUG");

    // Debug options
    const char* options[] = {
        "Back to Menu",
        "Send Alert",
        "Send Goal",
        "New Day"
    };

    int16_t optionY = 40;
    int16_t optionHeight = 28;

    for (int i = 0; i < 4; i++) {
        bool isSelected = (i == (int)selectedDebugOption);

        if (isSelected) {
            display.fillRect(5, optionY + i * optionHeight, 166, 25, RGB565(150, 50, 50));
            display.drawRect(5, optionY + i * optionHeight, 166, 25, RGB565(255, 100, 100));
        }

        display.setTextSize(1);
        display.setTextColor(isSelected ? COLOR_WHITE : RGB565(180, 180, 180));
        display.setCursor(15, optionY + i * optionHeight + 8);
        display.print(options[i]);
    }

    // Footer hint
    display.setTextSize(1);
    display.setTextColor(RGB565(100, 100, 100));
    display.setCursor(20, 167);
    display.print("Rotate & Click to select");
}

void updateDisplay() {
    if (!displayInitialized) return;

    // Check for screensaver
    if (millis() - lastActivity > SCREENSAVER_TIMEOUT_MS && !screenSaverActive) {
        screenSaverActive = true;
        display.displayOff();
        return;
    }

    if (screenSaverActive) return;

    // Rate limit display updates
    if (millis() - lastDisplayUpdate < DISPLAY_UPDATE_INTERVAL_MS) {
        return;
    }
    lastDisplayUpdate = millis();
    switch (currentScreen) {
        case Screen::SETUP_AGE:
        case Screen::SETUP_SEX:
        case Screen::SETUP_WEIGHT:
        case Screen::SETUP_HOUR:
        case Screen::SETUP_MINUTE:
        case Screen::SETTINGS_AGE:
        case Screen::SETTINGS_SEX:
        case Screen::SETTINGS_WEIGHT:
        case Screen::SETTINGS_HOUR:
        case Screen::SETTINGS_MINUTE:
            drawSetupScreen();
            break;

        case Screen::MAIN_DISPLAY:
            drawMainScreen();
            break;

        case Screen::MENU:
            drawMenuScreen();
            break;

        case Screen::DEBUG_MENU:
            drawDebugMenu();
            break;
    }
}

// ============================================================================
// Input Handling
// ============================================================================

void handleEncoderInput() {
    static int16_t lastPosition = 0;

    // Handle rotation
    int16_t delta = encoder.position - lastPosition;
    if (delta != 0) {
        lastPosition = encoder.position;

        switch (currentScreen) {
            case Screen::SETUP_AGE:
            case Screen::SETTINGS_AGE:
                userConfig.age = constrain(userConfig.age + delta, 1, 120);
                lastDisplayUpdate = 0;
                break;

            case Screen::SETUP_SEX:
            case Screen::SETTINGS_SEX:
                userConfig.sex = (delta > 0) ? 1 : 0;
                lastDisplayUpdate = 0;
                break;

            case Screen::SETUP_WEIGHT:
            case Screen::SETTINGS_WEIGHT:
                userConfig.weight = constrain(userConfig.weight + delta, 30, 200);
                lastDisplayUpdate = 0;
                break;

            case Screen::SETUP_HOUR:
            case Screen::SETTINGS_HOUR:
                setupHour = (setupHour + delta + 24) % 24;
                lastDisplayUpdate = 0;
                break;

            case Screen::SETUP_MINUTE:
            case Screen::SETTINGS_MINUTE:
                setupMinute = (setupMinute + delta + 60) % 60;
                lastDisplayUpdate = 0;
                break;

            case Screen::MAIN_DISPLAY:
                // Main screen: rotation does nothing (use button to access menu)
                break;

            case Screen::MENU:
                if (delta > 0) {
                    selectedMenuOption = (MenuOption)(((int)selectedMenuOption + 1) % 4);
                } else if (delta < 0) {
                    selectedMenuOption = (MenuOption)(((int)selectedMenuOption + 3) % 4);
                }
                lastDisplayUpdate = 0;
                break;

            case Screen::DEBUG_MENU:
                if (delta > 0) {
                    selectedDebugOption = (DebugOption)(((int)selectedDebugOption + 1) % 4);
                } else if (delta < 0) {
                    selectedDebugOption = (DebugOption)(((int)selectedDebugOption + 3) % 4);
                }
                lastDisplayUpdate = 0;
                break;
        }
    }

    // Handle button press
    if (encoder.buttonReleased) {
        encoder.buttonReleased = false;

        switch (currentScreen) {
            case Screen::SETUP_AGE:
                currentScreen = Screen::SETUP_SEX;
                lastDisplayUpdate = 0;
                break;

            case Screen::SETUP_SEX:
                currentScreen = Screen::SETUP_WEIGHT;
                lastDisplayUpdate = 0;
                break;

            case Screen::SETUP_WEIGHT:
                currentScreen = Screen::SETUP_HOUR;
                setupHour = 12;
                lastDisplayUpdate = 0;
                break;

            case Screen::SETUP_HOUR:
                currentScreen = Screen::SETUP_MINUTE;
                setupMinute = 0;
                lastDisplayUpdate = 0;
                break;

            case Screen::SETUP_MINUTE:
                // Save everything and complete setup
                if (rtcInitialized) {
                    DateTime now = rtc.now();
                    rtc.adjust(DateTime(now.year(), now.month(), now.day(),
                                       setupHour, setupMinute, 0));
                    userConfig.timeSet = true;
                }
                userConfig.setupComplete = true;
                saveUserConfig();
                currentScreen = Screen::MAIN_DISPLAY;
                lastDisplayUpdate = 0;

                // Calculate initial daily goal
                hydrationState.dailyGoalML = calculateDailyGoal();
                pendingGoal = hydrationState.dailyGoalML;
                goalPending = true;
                break;

            case Screen::SETTINGS_AGE:
                currentScreen = Screen::SETTINGS_SEX;
                lastDisplayUpdate = 0;
                break;

            case Screen::SETTINGS_SEX:
                currentScreen = Screen::SETTINGS_WEIGHT;
                lastDisplayUpdate = 0;
                break;

            case Screen::SETTINGS_WEIGHT:
                // Save and return to menu
                saveUserConfig();
                currentScreen = Screen::MENU;
                lastDisplayUpdate = 0;
                // Recalculate goal with new parameters
                hydrationState.dailyGoalML = calculateDailyGoal();
                pendingGoal = hydrationState.dailyGoalML;
                goalPending = true;
                break;

            case Screen::SETTINGS_HOUR:
                currentScreen = Screen::SETTINGS_MINUTE;
                lastDisplayUpdate = 0;
                break;

            case Screen::SETTINGS_MINUTE:
                // Save time and return to menu
                if (rtcInitialized) {
                    DateTime now = rtc.now();
                    rtc.adjust(DateTime(now.year(), now.month(), now.day(),
                                       setupHour, setupMinute, 0));
                    userConfig.timeSet = true;
                    saveUserConfig();
                }
                currentScreen = Screen::MENU;
                lastDisplayUpdate = 0;
                break;

            case Screen::MAIN_DISPLAY:
                // Button click on main screen opens menu
                currentScreen = Screen::MENU;
                selectedMenuOption = MenuOption::BACK_TO_MAIN;
                lastDisplayUpdate = 0;
                break;

            case Screen::MENU:
                switch (selectedMenuOption) {
                    case MenuOption::BACK_TO_MAIN:
                        currentScreen = Screen::MAIN_DISPLAY;
                        lastDisplayUpdate = 0;
                        break;

                    case MenuOption::MODIFY_INFO:
                        currentScreen = Screen::SETTINGS_AGE;
                        lastDisplayUpdate = 0;
                        break;

                    case MenuOption::CHANGE_TIME:
                        if (rtcInitialized) {
                            DateTime now = rtc.now();
                            setupHour = now.hour();
                            setupMinute = now.minute();
                        }
                        currentScreen = Screen::SETTINGS_HOUR;
                        lastDisplayUpdate = 0;
                        break;

                    case MenuOption::DEBUG:
                        currentScreen = Screen::DEBUG_MENU;
                        selectedDebugOption = DebugOption::BACK_TO_MENU;
                        lastDisplayUpdate = 0;
                        break;
                }
                break;

            case Screen::DEBUG_MENU:
                switch (selectedDebugOption) {
                    case DebugOption::BACK_TO_MENU:
                        currentScreen = Screen::MENU;
                        lastDisplayUpdate = 0;
                        break;

                    case DebugOption::SEND_ALERT:
                        Serial.println("DEBUG: Triggering alert");
                        alertPending = true;
                        currentScreen = Screen::MAIN_DISPLAY;
                        lastDisplayUpdate = 0;
                        break;

                    case DebugOption::SEND_GOAL:
                        Serial.println("DEBUG: Sending goal to coaster");
                        pendingGoal = hydrationState.dailyGoalML;
                        goalPending = true;
                        currentScreen = Screen::MAIN_DISPLAY;
                        lastDisplayUpdate = 0;
                        break;

                    case DebugOption::TRIGGER_DAY_CHANGE:
                        Serial.println("DEBUG: Triggering new day");
                        onNewDay();
                        currentScreen = Screen::MAIN_DISPLAY;
                        lastDisplayUpdate = 0;
                        break;
                }
                break;

            default:
                break;
        }
    }
}

// ============================================================================
// Setup and Main Loop
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("  CoasterClip Base Station with UI");
    Serial.println("  Version 2.0");
    Serial.println("========================================\n");

    // Initialize all hardware
    initHardware();

    // Load user configuration
    loadUserConfig();

    // Set initial screen based on setup status
    if (!userConfig.setupComplete) {
        currentScreen = Screen::SETUP_AGE;
        Serial.println("Starting initial setup...");
    } else {
        currentScreen = Screen::MAIN_DISPLAY;
        Serial.println("Setup already complete, loading main screen");

        // Load today's consumption
        hydrationState.totalConsumedML = loadTodayConsumption();

        // Calculate daily goal
        hydrationState.dailyGoalML = calculateDailyGoal();
        pendingGoal = hydrationState.dailyGoalML;
        goalPending = true;
    }

    // Take initial sensor reading
    SensorReading reading;
    if (readSensor(&reading)) {
        addSensorReading(reading);
        logSensorReading(reading);
    }

    lastActivity = millis();

    // BLE starts in IDLE, state machine will start scanning immediately

    // Force initial display update
    if (displayInitialized) {
        lastDisplayUpdate = 0;
        updateDisplay();
    }

    Serial.println("Setup complete!\n");
}

void loop() {
    // Update encoder state
    updateEncoder();

    // Handle encoder input
    handleEncoderInput();

    // Update display
    updateDisplay();

    // Only do hydration tracking if setup is complete
    if (userConfig.setupComplete) {
        // Handle day transitions
        if (isNewDay()) {
            onNewDay();
        }

        // Read sensors periodically
        if (millis() - lastSensorReadTime >= SENSOR_READ_INTERVAL_MS) {
            lastSensorReadTime = millis();
            SensorReading reading;
            if (readSensor(&reading)) {
                addSensorReading(reading);
                logSensorReading(reading);
                Serial.printf("Sensor: %.1fC, %.1f%%\n",
                             reading.temperature, reading.humidity);
            }
        }

        // Process BLE state machine
        processBLEState();

        // Process received consumption events
        if (consumptionReceived) {
            consumptionReceived = false;
            hydrationState.totalConsumedML += latestConsumption.amountML;
            logConsumptionEvent(latestConsumption);
            Serial.printf("Total consumed: %.1f / %.1f mL\n",
                         hydrationState.totalConsumedML,
                         hydrationState.dailyGoalML);
            lastDisplayUpdate = 0; // Force display update
        }

        // Queue goal if not yet sent
        if (rtcInitialized && !hydrationState.goalSentToCoaster &&
            hydrationState.dailyGoalML <= 0) {
            hydrationState.dailyGoalML = calculateDailyGoal();
            pendingGoal = hydrationState.dailyGoalML;
            goalPending = true;
        }

        // Check trajectory periodically during active hours
        static uint32_t lastTrajectoryCheck = 0;
        uint8_t hour = getHourOfDay();
        if (hour >= ACTIVE_HOUR_START && hour < ACTIVE_HOUR_END) {
            if (millis() - lastTrajectoryCheck > TRAJECTORY_CHECK_INTERVAL_MS) {
                lastTrajectoryCheck = millis();
                if (hydrationState.dailyGoalML > 0 && checkTrajectory() && canAlert()) {
                    triggerAlert();
                }
            }
        }
    }

    delay(10);
}
