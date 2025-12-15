// ESP32 Base Station Firmware for Coaster-Clip Hydration Tracking System
// Manages hydration goals, receives consumption data, and sends drinking reminders

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include "Adafruit_SHT4x.h"

// External water turnover prediction function
extern float predictWaterTurnoverCH(
    float bodyWeightKg,
    uint8_t sex,
    float humidityPercent,
    float ageYears,
    float temperatureC
);

// ============================================================================
// Pin Definitions
// ============================================================================
#define PIN_SDA           21      // I2C SDA for SHT41
#define PIN_SCL           22      // I2C SCL for SHT41
#define PIN_SD_MOSI       23      // SPI MOSI for SD card
#define PIN_SD_MISO       19      // SPI MISO for SD card
#define PIN_SD_SCK        18      // SPI SCK for SD card
#define PIN_SD_CS         5       // SPI CS for SD card
#define PIN_BUZZER        4       // Active buzzer

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

// ============================================================================
// Hydration Constants
// ============================================================================
#define LIQUID_INTAKE_FACTOR        0.65f    // 65% of water turnover is from liquids
#define BEHIND_TRAJECTORY_THRESHOLD 0.15f    // 15% behind expected pace
#define ACTIVE_HOUR_START           6        // 6:00 AM
#define ACTIVE_HOUR_END             22       // 10:00 PM

// ============================================================================
// Prototype Body Metrics (will be configurable later via rotary encoder)
// ============================================================================
#define BODY_WEIGHT_KG              70.0f
#define AGE_YEARS                   35.0f
#define SEX_MALE                    1

// ============================================================================
// Sensor Buffer Configuration
// ============================================================================
#define MAX_SENSOR_READINGS         96       // 24h at 15min intervals

// ============================================================================
// BLE Service and Characteristics UUIDs (must match coaster)
// ============================================================================
#define COASTER_SERVICE_UUID        "19B10010-E8F2-537E-4F6C-D104768A1214"
#define CONSUMPTION_CHAR_UUID       "19B10011-E8F2-537E-4F6C-D104768A1214"
#define COMMAND_CHAR_UUID           "19B10012-E8F2-537E-4F6C-D104768A1214"
#define GOAL_CHAR_UUID              "19B10013-E8F2-537E-4F6C-D104768A1214"

// Manufacturer data company ID for CoasterClip (0xFFFF = reserved for testing)
#define COASTER_MFG_COMPANY_ID      0xFFFF

// ============================================================================
// Data Structures
// ============================================================================

struct SensorReading {
    uint32_t timestamp;       // Seconds since device boot
    float temperature;        // Celsius
    float humidity;           // Percentage
};

struct ConsumptionEvent {
    uint32_t coasterTimestamp;   // millis() from coaster
    uint32_t localTimestamp;     // Seconds since device boot when received
    float amountML;              // Amount consumed in mL
};

struct HydrationState {
    uint32_t dayStartTimestamp;   // Timestamp of midnight (seconds since boot)
    float dailyGoalML;            // Calculated hydration goal
    float totalConsumedML;        // Running total for today
    uint8_t remindersSent;        // Count of reminders today
    bool goalSentToCoaster;       // Whether today's goal was sent
};

// Circular buffer for sensor readings
struct SensorBuffer {
    SensorReading readings[MAX_SENSOR_READINGS];
    uint8_t head;
    uint8_t count;
};

// BLE connection state
enum class BLEState {
    IDLE,
    SCANNING,
    CONNECTING,
    CONNECTED,
    DISCONNECTING
};

// ============================================================================
// Global State
// ============================================================================

// Time management
uint32_t bootTimestamp = 0;           // Unix timestamp when device booted (set via serial)
bool timeSet = false;                 // Whether time has been set
uint8_t lastKnownDay = 0;             // For detecting day changes

// Sensor management
Adafruit_SHT4x sht41;
SensorBuffer sensorBuffer = {{}, 0, 0};
uint32_t lastSensorReadTime = 0;
bool sensorInitialized = false;

// SD card management
bool sdInitialized = false;

// BLE management
BLEState bleState = BLEState::IDLE;
BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pConsumptionChar = nullptr;
BLERemoteCharacteristic* pCommandChar = nullptr;
BLERemoteCharacteristic* pGoalChar = nullptr;
BLEScan* pBLEScan = nullptr;
BLEAdvertisedDevice* coasterDevice = nullptr;

bool coasterFound = false;
bool coasterHasData = false;          // From manufacturer data
bool alertPending = false;            // Base needs to send alert
bool goalPending = false;             // Base needs to send goal
float pendingGoal = 0.0f;
uint32_t lastScanTime = 0;
uint32_t connectStartTime = 0;
bool scanInProgress = false;          // Track scanning state manually

// Hydration tracking
HydrationState hydrationState = {0, 0.0f, 0.0f, 0, false};

// Alert management
uint32_t lastAlertTime = 0;

// Consumption event queue (simple single-item buffer for now)
bool consumptionReceived = false;
ConsumptionEvent latestConsumption = {0, 0, 0.0f};

// ============================================================================
// Forward Declarations
// ============================================================================
void initSensor();
void initSD();
void initBLE();
void initBuzzer();
bool readSensor(SensorReading* reading);
void addSensorReading(const SensorReading& reading);
float getAverageTemperature();
float getAverageHumidity();
void logSensorReading(const SensorReading& reading);
void logConsumptionEvent(const ConsumptionEvent& event);
float loadTodayConsumption();
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
void checkSerialTimeSync();

// ============================================================================
// BLE Callbacks
// ============================================================================

// Notification callback for consumption data from coaster
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
        latestConsumption.amountML = scaledValue / 100.0f;  // Convert from scaled value
        consumptionReceived = true;

        Serial.println("========================================");
        Serial.println("CONSUMPTION EVENT RECEIVED!");
        Serial.print("  Coaster timestamp: ");
        Serial.println(timestamp);
        Serial.print("  Amount: ");
        Serial.print(latestConsumption.amountML, 2);
        Serial.println(" mL");
        Serial.println("========================================");
    }
}

// BLE client connection callbacks
class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) override {
        Serial.println("BLE: Connected to coaster");
    }

    void onDisconnect(BLEClient* pclient) override {
        Serial.println("BLE: Disconnected from coaster");
        bleState = BLEState::IDLE;
    }
};

// BLE scan callbacks
class MyScanCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        // Check if this is our coaster by service UUID
        if (advertisedDevice.haveServiceUUID() &&
            advertisedDevice.isAdvertisingService(BLEUUID(COASTER_SERVICE_UUID))) {

            Serial.print("Found CoasterClip: ");
            Serial.println(advertisedDevice.getAddress().toString().c_str());

            // Parse manufacturer data to check data pending flag
            coasterHasData = false;
            if (advertisedDevice.haveManufacturerData()) {
                std::string mfgData = advertisedDevice.getManufacturerData();
                if (mfgData.length() >= 3) {
                    // First 2 bytes are company ID (little-endian), third byte is flag
                    uint16_t companyId = (uint8_t)mfgData[0] | ((uint8_t)mfgData[1] << 8);
                    if (companyId == COASTER_MFG_COMPANY_ID) {
                        uint8_t flag = (uint8_t)mfgData[2];
                        coasterHasData = (flag == 0x01);
                        Serial.print("  Data pending flag: ");
                        Serial.println(coasterHasData ? "YES" : "NO");
                    }
                }
            }

            // Store device for later connection
            if (coasterDevice != nullptr) {
                delete coasterDevice;
            }
            coasterDevice = new BLEAdvertisedDevice(advertisedDevice);
            coasterFound = true;
            scanInProgress = false;

            // Stop scanning - we found it
            pBLEScan->stop();
        }
    }
};

// ============================================================================
// Time Management Functions
// ============================================================================

uint32_t getCurrentTimestamp() {
    if (!timeSet) {
        return millis() / 1000;  // Fall back to seconds since boot
    }
    return bootTimestamp + (millis() / 1000);
}

uint8_t getHourOfDay() {
    if (!timeSet) {
        return 12;  // Default to noon if time not set
    }
    uint32_t timestamp = getCurrentTimestamp();
    return (timestamp % 86400) / 3600;  // Seconds in day / seconds in hour
}

float getDayProgress() {
    uint8_t hour = getHourOfDay();
    if (hour < ACTIVE_HOUR_START) return 0.0f;
    if (hour >= ACTIVE_HOUR_END) return 1.0f;
    return (float)(hour - ACTIVE_HOUR_START) / (float)(ACTIVE_HOUR_END - ACTIVE_HOUR_START);
}

bool isNewDay() {
    if (!timeSet) return false;
    uint32_t timestamp = getCurrentTimestamp();
    uint8_t currentDay = (timestamp / 86400) % 31;  // Simplified day tracking
    if (currentDay != lastKnownDay) {
        lastKnownDay = currentDay;
        return true;
    }
    return false;
}

void checkSerialTimeSync() {
    // Check for time sync command: "TIME:<unix_timestamp>"
    if (Serial.available() > 5) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.startsWith("TIME:")) {
            String timestampStr = input.substring(5);
            uint32_t timestamp = timestampStr.toInt();
            if (timestamp > 1700000000) {  // Reasonable Unix timestamp (after 2023)
                bootTimestamp = timestamp - (millis() / 1000);
                timeSet = true;
                lastKnownDay = (timestamp / 86400) % 31;

                Serial.println("========================================");
                Serial.print("Time synchronized! Unix timestamp: ");
                Serial.println(timestamp);
                Serial.print("Current hour: ");
                Serial.println(getHourOfDay());
                Serial.println("========================================");
            }
        }
    }
}

// ============================================================================
// Sensor Management Functions
// ============================================================================

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

bool readSensor(SensorReading* reading) {
    if (!sensorInitialized) {
        // Use default values if sensor not available
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
    if (sensorBuffer.count == 0) {
        return 20.0f;  // Default
    }

    float sum = 0.0f;
    for (uint8_t i = 0; i < sensorBuffer.count; i++) {
        sum += sensorBuffer.readings[i].temperature;
    }
    return sum / sensorBuffer.count;
}

float getAverageHumidity() {
    if (sensorBuffer.count == 0) {
        return 50.0f;  // Default
    }

    float sum = 0.0f;
    for (uint8_t i = 0; i < sensorBuffer.count; i++) {
        sum += sensorBuffer.readings[i].humidity;
    }
    return sum / sensorBuffer.count;
}

// ============================================================================
// SD Card Storage Functions
// ============================================================================

void initSD() {
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

    if (SD.begin(PIN_SD_CS)) {
        sdInitialized = true;
        Serial.println("SD card initialized");

        // Create hydration directory if it doesn't exist
        if (!SD.exists("/hydration")) {
            SD.mkdir("/hydration");
        }
    } else {
        Serial.println("WARNING: SD card initialization failed!");
        sdInitialized = false;
    }
}

void logSensorReading(const SensorReading& reading) {
    if (!sdInitialized) return;

    File file = SD.open("/hydration/SENSOR.CSV", FILE_APPEND);
    if (file) {
        // Format: timestamp,temperature,humidity
        file.print(reading.timestamp);
        file.print(",");
        file.print(reading.temperature, 2);
        file.print(",");
        file.println(reading.humidity, 2);
        file.close();
    } else {
        Serial.println("ERROR: Failed to write sensor log");
    }
}

void logConsumptionEvent(const ConsumptionEvent& event) {
    if (!sdInitialized) return;

    File file = SD.open("/hydration/CONSUME.CSV", FILE_APPEND);
    if (file) {
        // Format: timestamp,amount_ml
        file.print(event.localTimestamp);
        file.print(",");
        file.println(event.amountML, 2);
        file.close();
        Serial.println("Consumption event logged to SD");
    } else {
        Serial.println("ERROR: Failed to write consumption log");
    }
}

float loadTodayConsumption() {
    if (!sdInitialized || !timeSet) return 0.0f;

    // Calculate today's start timestamp (midnight)
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
        Serial.print("Loaded today's consumption from SD: ");
        Serial.print(total, 2);
        Serial.println(" mL");
    }

    return total;
}

// ============================================================================
// BLE Management Functions
// ============================================================================

void initBLE() {
    BLEDevice::init("ESP32-Base");

    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyScanCallbacks());
    pBLEScan->setInterval(1349);  // 1349 * 0.625ms = ~843ms
    pBLEScan->setWindow(449);     // 449 * 0.625ms = ~280ms
    pBLEScan->setActiveScan(true);

    Serial.println("BLE initialized");
}

// Scan complete callback
static void scanCompleteCB(BLEScanResults results) {
    Serial.print("BLE: Scan complete, found ");
    Serial.print(results.getCount());
    Serial.println(" devices");
    scanInProgress = false;
}

void startScan() {
    coasterFound = false;
    coasterHasData = false;
    scanInProgress = true;
    bleState = BLEState::SCANNING;
    pBLEScan->start(BLE_SCAN_DURATION_S, scanCompleteCB, false);
    Serial.println("BLE: Scanning for coaster...");
}

bool connectToCoaster() {
    if (coasterDevice == nullptr) {
        return false;
    }

    Serial.print("BLE: Connecting to ");
    Serial.println(coasterDevice->getAddress().toString().c_str());

    if (pClient != nullptr) {
        delete pClient;
    }

    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());

    if (!pClient->connect(coasterDevice)) {
        Serial.println("BLE: Connection failed");
        return false;
    }

    pClient->setMTU(517);

    // Get service
    BLERemoteService* pRemoteService = pClient->getService(COASTER_SERVICE_UUID);
    if (pRemoteService == nullptr) {
        Serial.println("BLE: Service not found");
        pClient->disconnect();
        return false;
    }

    // Get characteristics
    pConsumptionChar = pRemoteService->getCharacteristic(CONSUMPTION_CHAR_UUID);
    pCommandChar = pRemoteService->getCharacteristic(COMMAND_CHAR_UUID);
    pGoalChar = pRemoteService->getCharacteristic(GOAL_CHAR_UUID);

    if (pConsumptionChar == nullptr || pCommandChar == nullptr || pGoalChar == nullptr) {
        Serial.println("BLE: Characteristics not found");
        pClient->disconnect();
        return false;
    }

    // Register for consumption notifications
    if (pConsumptionChar->canNotify()) {
        pConsumptionChar->registerForNotify(consumptionNotifyCallback);
        Serial.println("BLE: Registered for notifications");
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

    Serial.print("BLE: Sent command 0x");
    Serial.println(cmd, HEX);
    return true;
}

bool sendGoal(float goalML) {
    if (bleState != BLEState::CONNECTED || pGoalChar == nullptr) {
        return false;
    }

    uint8_t data[4];
    memcpy(data, &goalML, 4);
    pGoalChar->writeValue(data, 4, false);

    Serial.print("BLE: Sent hydration goal: ");
    Serial.print(goalML, 1);
    Serial.println(" mL");
    return true;
}

void processBLEState() {
    switch (bleState) {
        case BLEState::IDLE:
            // Check if we should initiate a connection
            if (alertPending || goalPending) {
                // We need to connect to send something
                if (millis() - lastScanTime > BLE_SCAN_INTERVAL_MS) {
                    lastScanTime = millis();
                    startScan();
                }
            } else if (millis() - lastScanTime > BLE_SCAN_INTERVAL_MS) {
                // Regular scanning to check if coaster has data
                lastScanTime = millis();
                startScan();
            }
            break;

        case BLEState::SCANNING:
            // Scanning is handled by callbacks
            // Check if scan is complete
            if (!scanInProgress) {
                if (coasterFound) {
                    // Decide whether to connect
                    bool shouldConnect = coasterHasData || alertPending || goalPending;

                    if (shouldConnect) {
                        bleState = BLEState::CONNECTING;
                        if (connectToCoaster()) {
                            // Connection successful, handled in CONNECTED state
                        } else {
                            bleState = BLEState::IDLE;
                        }
                    } else {
                        Serial.println("BLE: Coaster found but no action needed");
                        bleState = BLEState::IDLE;
                    }
                } else {
                    bleState = BLEState::IDLE;
                }
            }
            break;

        case BLEState::CONNECTING:
            // Connection attempt is synchronous in connectToCoaster()
            // State is set by that function
            break;

        case BLEState::CONNECTED:
            // Handle pending operations
            if (goalPending) {
                if (sendGoal(pendingGoal)) {
                    goalPending = false;
                    hydrationState.goalSentToCoaster = true;
                }
            }

            if (alertPending) {
                if (sendCommand(0x01)) {  // LED alert command
                    alertPending = false;
                    lastAlertTime = millis();
                    hydrationState.remindersSent++;
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
            // Wait for disconnect callback to set state to IDLE
            if (pClient == nullptr || !pClient->isConnected()) {
                bleState = BLEState::IDLE;
            }
            break;
    }
}

// ============================================================================
// Hydration Tracking Functions
// ============================================================================

float calculateDailyGoal() {
    float avgTemp = getAverageTemperature();
    float avgHumidity = getAverageHumidity();

    float waterTurnover = predictWaterTurnoverCH(
        BODY_WEIGHT_KG,
        SEX_MALE,
        avgHumidity,
        AGE_YEARS,
        avgTemp
    );

    float goal = waterTurnover * LIQUID_INTAKE_FACTOR;

    Serial.println("========================================");
    Serial.println("Daily Goal Calculation:");
    Serial.print("  24h Avg Temperature: ");
    Serial.print(avgTemp, 1);
    Serial.println(" C");
    Serial.print("  24h Avg Humidity: ");
    Serial.print(avgHumidity, 1);
    Serial.println(" %");
    Serial.print("  Water Turnover: ");
    Serial.print(waterTurnover, 0);
    Serial.println(" mL");
    Serial.print("  Daily Goal (x0.65): ");
    Serial.print(goal, 0);
    Serial.println(" mL");
    Serial.println("========================================");

    return goal;
}

bool checkTrajectory() {
    if (hydrationState.dailyGoalML <= 0) return false;

    float dayProgress = getDayProgress();
    if (dayProgress <= 0) return false;  // Before active hours

    float expected = hydrationState.dailyGoalML * dayProgress;
    float deficit = expected - hydrationState.totalConsumedML;
    float deficitRatio = deficit / expected;

    Serial.print("Trajectory check: ");
    Serial.print(hydrationState.totalConsumedML, 0);
    Serial.print("/");
    Serial.print(expected, 0);
    Serial.print(" mL (");
    Serial.print((1.0f - deficitRatio) * 100, 0);
    Serial.println("%)");

    return (deficitRatio >= BEHIND_TRAJECTORY_THRESHOLD);
}

void onNewDay() {
    Serial.println("========================================");
    Serial.println("NEW DAY DETECTED!");
    Serial.println("========================================");

    // Reset daily state
    hydrationState.dayStartTimestamp = getCurrentTimestamp() - (getCurrentTimestamp() % 86400);
    hydrationState.totalConsumedML = 0.0f;
    hydrationState.remindersSent = 0;
    hydrationState.goalSentToCoaster = false;

    // Calculate new goal
    hydrationState.dailyGoalML = calculateDailyGoal();

    // Queue goal to be sent to coaster
    pendingGoal = hydrationState.dailyGoalML;
    goalPending = true;
}

// ============================================================================
// Alert Management Functions
// ============================================================================

void initBuzzer() {
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
}

void soundBuzzer() {
    // Three short beeps
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_BUZZER, HIGH);
        delay(100);
        digitalWrite(PIN_BUZZER, LOW);
        delay(100);
    }
}

void triggerAlert() {
    Serial.println("========================================");
    Serial.println("HYDRATION ALERT!");
    Serial.print("You are behind by ");
    float expected = hydrationState.dailyGoalML * getDayProgress();
    float deficit = expected - hydrationState.totalConsumedML;
    Serial.print(deficit, 0);
    Serial.println(" mL");
    Serial.println("========================================");

    // Sound local buzzer
    soundBuzzer();

    // Queue BLE alert to coaster
    alertPending = true;
}

bool canAlert() {
    // Check cooldown
    if (millis() - lastAlertTime < ALERT_COOLDOWN_MS) {
        return false;
    }
    // Limit alerts per day
    if (hydrationState.remindersSent >= 10) {
        return false;
    }
    return true;
}

// ============================================================================
// Setup and Main Loop
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("  Coaster-Clip Base Station v1.0");
    Serial.println("========================================\n");

    // Initialize hardware
    initBuzzer();
    initSensor();
    initSD();
    initBLE();

    // Try to recover today's consumption from SD
    hydrationState.totalConsumedML = loadTodayConsumption();

    // Take initial sensor reading
    SensorReading reading;
    if (readSensor(&reading)) {
        addSensorReading(reading);
        logSensorReading(reading);
        Serial.print("Initial reading - Temp: ");
        Serial.print(reading.temperature, 1);
        Serial.print("C, Humidity: ");
        Serial.print(reading.humidity, 1);
        Serial.println("%");
    }

    Serial.println("\n========================================");
    Serial.println("Setup complete!");
    Serial.println("Send TIME:<unix_timestamp> to set time");
    Serial.println("Example: TIME:1702636800");
    Serial.println("========================================\n");

    // Start first scan
    lastScanTime = millis() - BLE_SCAN_INTERVAL_MS;
}

void loop() {
    // Check for serial time sync
    checkSerialTimeSync();

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

            Serial.print("Sensor: ");
            Serial.print(reading.temperature, 1);
            Serial.print("C, ");
            Serial.print(reading.humidity, 1);
            Serial.print("% (buffer: ");
            Serial.print(sensorBuffer.count);
            Serial.println(" readings)");
        }
    }

    // Process BLE state machine
    processBLEState();

    // Process received consumption events
    if (consumptionReceived) {
        consumptionReceived = false;

        // Update total
        hydrationState.totalConsumedML += latestConsumption.amountML;

        // Log to SD
        logConsumptionEvent(latestConsumption);

        Serial.print("Total consumed today: ");
        Serial.print(hydrationState.totalConsumedML, 1);
        Serial.print(" / ");
        Serial.print(hydrationState.dailyGoalML, 1);
        Serial.println(" mL");
    }

    // Queue goal if not yet sent and time is set
    if (timeSet && !hydrationState.goalSentToCoaster && hydrationState.dailyGoalML <= 0) {
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

    // Small delay to prevent CPU hogging
    delay(100);
}
