// ESP32 Base Station - Calibration Controller (Load Cell + Hall Sensor)
//
// Serial Commands:
//   LOAD CELL:
//     W <value>  - Set reference weight (grams), then request measurement
//     M          - Measure load cell only (don't save)
//     P          - Print load cell calibration CSV
//     C          - Clear load cell calibration data
//
//   HALL SENSOR:
//     HG         - Hall measurement WITH glass (save to CSV)
//     HN         - Hall measurement WITHOUT glass (save to CSV)
//     HM         - Hall measure only (don't save)
//     HP         - Print hall sensor calibration CSV
//     HC         - Clear hall sensor calibration data
//
//   GENERAL:
//     S          - Scan for coaster
//     ?          - Help
//
// Workflow (Load Cell):
//   1. Place known weight on coaster
//   2. Enter "W 100.5" to set reference weight and trigger measurement
//   3. ESP32 connects to coaster, requests raw reading, saves to CSV
//   4. Repeat for all reference weights
//   5. Enter "P" to print the calibration data CSV
//
// Workflow (Hall Sensor):
//   1. Place glass on coaster, enter "HG" to measure with glass
//   2. Remove glass, enter "HN" to measure without glass
//   3. Repeat several times for each state
//   4. Enter "HP" to print hall sensor calibration data

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEScan.h>

// BLE Service and Characteristics UUIDs (must match coaster calibration firmware)
#define CALIBRATION_SERVICE_UUID    "19B10020-E8F2-537E-4F6C-D104768A1214"
#define CALIBRATION_DATA_CHAR_UUID  "19B10021-E8F2-537E-4F6C-D104768A1214"
#define CALIBRATION_CMD_CHAR_UUID   "19B10022-E8F2-537E-4F6C-D104768A1214"

// SD Card Pins
#define PIN_SD_MOSI       23
#define PIN_SD_MISO       19
#define PIN_SD_SCK        18
#define PIN_SD_CS         5

// Calibration file paths
#define LOADCELL_CALIB_FILE  "/CALIB.CSV"
#define HALL_CALIB_FILE      "/HALL.CSV"

// Command codes (must match coaster firmware)
#define CMD_LOADCELL  0x10
#define CMD_HALL      0x11

// BLE Configuration
#define BLE_SCAN_DURATION_S     5
#define CONNECTION_TIMEOUT_MS   10000
#define RESPONSE_TIMEOUT_MS     15000

// BLE State
enum class BLEState {
    IDLE,
    SCANNING,
    CONNECTING,
    CONNECTED,
    WAITING_FOR_DATA
};

BLEState bleState = BLEState::IDLE;
BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pDataChar = nullptr;
BLERemoteCharacteristic* pCmdChar = nullptr;
BLEScan* pBLEScan = nullptr;
BLEAdvertisedDevice* coasterDevice = nullptr;

bool coasterFound = false;
bool sdInitialized = false;

// Calibration state
float pendingReferenceWeight = 0.0f;
bool measurementRequested = false;
bool saveAfterMeasurement = false;
volatile bool dataReceived = false;
volatile int32_t receivedRawValue = 0;
uint32_t measurementStartTime = 0;

// Hall sensor calibration state
enum class HallState { NONE, WITH_GLASS, WITHOUT_GLASS };
HallState pendingHallState = HallState::NONE;
bool hallMeasurementRequested = false;
bool saveHallAfterMeasurement = false;
volatile bool hallDataReceived = false;
volatile int32_t receivedHallMin = 0;
volatile int32_t receivedHallMax = 0;
volatile int32_t receivedHallAvg = 0;

// Forward declarations
void printHelp();
void processSerialCommand();
void startScan();
bool connectToCoaster();
void disconnectFromCoaster();
bool requestLoadcellMeasurement();
bool requestHallMeasurement();
void saveLoadcellPoint(float referenceWeight, int32_t rawValue);
void saveHallPoint(HallState state, int32_t minVal, int32_t maxVal, int32_t avgVal);
void printLoadcellCSV();
void printHallCSV();
void clearLoadcellData();
void clearHallData();
void initSD();

// Notification callback for calibration data from coaster
static void calibrationNotifyCallback(
    BLERemoteCharacteristic* pBLERemoteCharacteristic,
    uint8_t* pData,
    size_t length,
    bool isNotify
) {
    if (length < 1) return;

    uint8_t responseType = pData[0];

    if (responseType == CMD_LOADCELL && length == 5) {
        // Load cell response: [0x10][avg:4 bytes]
        memcpy((void*)&receivedRawValue, &pData[1], 4);
        dataReceived = true;

        Serial.println("\n========================================");
        Serial.println("LOAD CELL DATA RECEIVED!");
        Serial.print("  Raw value: ");
        Serial.println(receivedRawValue);
        Serial.println("========================================");
    }
    else if (responseType == CMD_HALL && length == 13) {
        // Hall sensor response: [0x11][min:4][max:4][avg:4]
        memcpy((void*)&receivedHallMin, &pData[1], 4);
        memcpy((void*)&receivedHallMax, &pData[5], 4);
        memcpy((void*)&receivedHallAvg, &pData[9], 4);
        hallDataReceived = true;

        Serial.println("\n========================================");
        Serial.println("HALL SENSOR DATA RECEIVED!");
        Serial.print("  Min: ");
        Serial.println(receivedHallMin);
        Serial.print("  Max: ");
        Serial.println(receivedHallMax);
        Serial.print("  Avg: ");
        Serial.println(receivedHallAvg);
        Serial.print("  Range: ");
        Serial.println(receivedHallMax - receivedHallMin);
        Serial.println("========================================");
    }
    else {
        Serial.print("Unknown response type: 0x");
        Serial.print(responseType, HEX);
        Serial.print(" length: ");
        Serial.println(length);
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
        // Check if this is our calibration coaster by service UUID
        if (advertisedDevice.haveServiceUUID() &&
            advertisedDevice.isAdvertisingService(BLEUUID(CALIBRATION_SERVICE_UUID))) {

            Serial.print("\nFound CoasterCalib: ");
            Serial.println(advertisedDevice.getAddress().toString().c_str());

            if (coasterDevice != nullptr) {
                delete coasterDevice;
            }
            coasterDevice = new BLEAdvertisedDevice(advertisedDevice);
            coasterFound = true;

            pBLEScan->stop();
        }
    }
};

void initSD() {
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

    if (SD.begin(PIN_SD_CS)) {
        sdInitialized = true;
        Serial.println("SD card initialized");

        // Check if load cell calibration file exists
        if (SD.exists(LOADCELL_CALIB_FILE)) {
            File file = SD.open(LOADCELL_CALIB_FILE, FILE_READ);
            if (file) {
                Serial.print("Existing load cell calibration file (");
                Serial.print(file.size());
                Serial.println(" bytes)");
                file.close();
            }
        } else {
            File file = SD.open(LOADCELL_CALIB_FILE, FILE_WRITE);
            if (file) {
                file.println("reference_g,raw_value");
                file.close();
                Serial.println("Created new load cell calibration file");
            }
        }

        // Check if hall sensor calibration file exists
        if (SD.exists(HALL_CALIB_FILE)) {
            File file = SD.open(HALL_CALIB_FILE, FILE_READ);
            if (file) {
                Serial.print("Existing hall sensor calibration file (");
                Serial.print(file.size());
                Serial.println(" bytes)");
                file.close();
            }
        } else {
            File file = SD.open(HALL_CALIB_FILE, FILE_WRITE);
            if (file) {
                file.println("state,min,max,avg");
                file.close();
                Serial.println("Created new hall sensor calibration file");
            }
        }
    } else {
        Serial.println("WARNING: SD card initialization failed!");
        Serial.println("Calibration data will NOT be saved!");
        sdInitialized = false;
    }
}

void initBLE() {
    BLEDevice::init("ESP32-Calib");

    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyScanCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);

    Serial.println("BLE initialized");
}

void startScan() {
    coasterFound = false;
    bleState = BLEState::SCANNING;

    Serial.println("Scanning for CoasterCalib...");
    pBLEScan->start(BLE_SCAN_DURATION_S, false);

    // Wait for scan to complete
    delay(BLE_SCAN_DURATION_S * 1000 + 500);

    if (coasterFound) {
        Serial.println("Coaster found!");
    } else {
        Serial.println("Coaster NOT found. Make sure calibration firmware is running.");
        bleState = BLEState::IDLE;
    }
}

bool connectToCoaster() {
    if (coasterDevice == nullptr) {
        Serial.println("No coaster device to connect to");
        return false;
    }

    Serial.print("Connecting to ");
    Serial.println(coasterDevice->getAddress().toString().c_str());

    if (pClient != nullptr) {
        if (pClient->isConnected()) {
            pClient->disconnect();
        }
        delete pClient;
    }

    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());

    bleState = BLEState::CONNECTING;

    if (!pClient->connect(coasterDevice)) {
        Serial.println("Connection failed!");
        bleState = BLEState::IDLE;
        return false;
    }

    pClient->setMTU(517);

    // Get calibration service
    BLERemoteService* pRemoteService = pClient->getService(CALIBRATION_SERVICE_UUID);
    if (pRemoteService == nullptr) {
        Serial.println("Calibration service not found!");
        pClient->disconnect();
        bleState = BLEState::IDLE;
        return false;
    }

    // Get characteristics
    pDataChar = pRemoteService->getCharacteristic(CALIBRATION_DATA_CHAR_UUID);
    pCmdChar = pRemoteService->getCharacteristic(CALIBRATION_CMD_CHAR_UUID);

    if (pDataChar == nullptr || pCmdChar == nullptr) {
        Serial.println("Characteristics not found!");
        pClient->disconnect();
        bleState = BLEState::IDLE;
        return false;
    }

    // Register for notifications
    if (pDataChar->canNotify()) {
        pDataChar->registerForNotify(calibrationNotifyCallback);
        Serial.println("Registered for calibration notifications");
    }

    bleState = BLEState::CONNECTED;
    return true;
}

void disconnectFromCoaster() {
    if (pClient != nullptr && pClient->isConnected()) {
        pClient->disconnect();
    }
    bleState = BLEState::IDLE;
}

bool requestLoadcellMeasurement() {
    if (bleState != BLEState::CONNECTED || pCmdChar == nullptr) {
        Serial.println("Not connected!");
        return false;
    }

    // Clear previous data
    dataReceived = false;
    receivedRawValue = 0;

    // Send load cell calibration command (0x10)
    uint8_t cmd[4] = {CMD_LOADCELL, 0, 0, 0};
    pCmdChar->writeValue(cmd, 4, false);

    Serial.println("Load cell measurement requested, waiting for data...");
    bleState = BLEState::WAITING_FOR_DATA;
    measurementStartTime = millis();

    return true;
}

bool requestHallMeasurement() {
    if (bleState != BLEState::CONNECTED || pCmdChar == nullptr) {
        Serial.println("Not connected!");
        return false;
    }

    // Clear previous data
    hallDataReceived = false;
    receivedHallMin = 0;
    receivedHallMax = 0;
    receivedHallAvg = 0;

    // Send hall sensor calibration command (0x11)
    uint8_t cmd[4] = {CMD_HALL, 0, 0, 0};
    pCmdChar->writeValue(cmd, 4, false);

    Serial.println("Hall sensor measurement requested, waiting for data...");
    bleState = BLEState::WAITING_FOR_DATA;
    measurementStartTime = millis();

    return true;
}

void saveLoadcellPoint(float referenceWeight, int32_t rawValue) {
    if (!sdInitialized) {
        Serial.println("SD card not available - cannot save!");
        return;
    }

    File file = SD.open(LOADCELL_CALIB_FILE, FILE_APPEND);
    if (file) {
        file.print(referenceWeight, 2);
        file.print(",");
        file.println(rawValue);
        file.close();

        Serial.println("----------------------------------------");
        Serial.println("Load cell calibration point saved:");
        Serial.print("  Reference: ");
        Serial.print(referenceWeight, 2);
        Serial.println(" g");
        Serial.print("  Raw value: ");
        Serial.println(rawValue);
        Serial.println("----------------------------------------");
    } else {
        Serial.println("ERROR: Failed to write to load cell calibration file!");
    }
}

void saveHallPoint(HallState state, int32_t minVal, int32_t maxVal, int32_t avgVal) {
    if (!sdInitialized) {
        Serial.println("SD card not available - cannot save!");
        return;
    }

    File file = SD.open(HALL_CALIB_FILE, FILE_APPEND);
    if (file) {
        const char* stateStr = (state == HallState::WITH_GLASS) ? "with_glass" : "no_glass";
        file.print(stateStr);
        file.print(",");
        file.print(minVal);
        file.print(",");
        file.print(maxVal);
        file.print(",");
        file.println(avgVal);
        file.close();

        Serial.println("----------------------------------------");
        Serial.println("Hall sensor calibration point saved:");
        Serial.print("  State: ");
        Serial.println(stateStr);
        Serial.print("  Min: ");
        Serial.println(minVal);
        Serial.print("  Max: ");
        Serial.println(maxVal);
        Serial.print("  Avg: ");
        Serial.println(avgVal);
        Serial.println("----------------------------------------");
    } else {
        Serial.println("ERROR: Failed to write to hall sensor calibration file!");
    }
}

void printLoadcellCSV() {
    if (!sdInitialized) {
        Serial.println("SD card not available!");
        return;
    }

    File file = SD.open(LOADCELL_CALIB_FILE, FILE_READ);
    if (!file) {
        Serial.println("Cannot open load cell calibration file!");
        return;
    }

    Serial.println("\n========================================");
    Serial.println("LOAD CELL CALIBRATION DATA (CSV):");
    Serial.println("========================================");

    while (file.available()) {
        String line = file.readStringUntil('\n');
        Serial.println(line);
    }

    Serial.println("========================================");
    Serial.print("File size: ");
    Serial.print(file.size());
    Serial.println(" bytes");
    Serial.println("========================================\n");

    file.close();
}

void printHallCSV() {
    if (!sdInitialized) {
        Serial.println("SD card not available!");
        return;
    }

    File file = SD.open(HALL_CALIB_FILE, FILE_READ);
    if (!file) {
        Serial.println("Cannot open hall sensor calibration file!");
        return;
    }

    Serial.println("\n========================================");
    Serial.println("HALL SENSOR CALIBRATION DATA (CSV):");
    Serial.println("========================================");

    while (file.available()) {
        String line = file.readStringUntil('\n');
        Serial.println(line);
    }

    Serial.println("========================================");
    Serial.print("File size: ");
    Serial.print(file.size());
    Serial.println(" bytes");
    Serial.println("========================================\n");

    file.close();
}

void clearLoadcellData() {
    if (!sdInitialized) {
        Serial.println("SD card not available!");
        return;
    }

    if (SD.exists(LOADCELL_CALIB_FILE)) {
        SD.remove(LOADCELL_CALIB_FILE);
    }

    File file = SD.open(LOADCELL_CALIB_FILE, FILE_WRITE);
    if (file) {
        file.println("reference_g,raw_value");
        file.close();
        Serial.println("Load cell calibration data cleared!");
    } else {
        Serial.println("ERROR: Failed to create new load cell calibration file!");
    }
}

void clearHallData() {
    if (!sdInitialized) {
        Serial.println("SD card not available!");
        return;
    }

    if (SD.exists(HALL_CALIB_FILE)) {
        SD.remove(HALL_CALIB_FILE);
    }

    File file = SD.open(HALL_CALIB_FILE, FILE_WRITE);
    if (file) {
        file.println("state,min,max,avg");
        file.close();
        Serial.println("Hall sensor calibration data cleared!");
    } else {
        Serial.println("ERROR: Failed to create new hall sensor calibration file!");
    }
}

void printHelp() {
    Serial.println("\n========================================");
    Serial.println("CALIBRATION CONTROLLER");
    Serial.println("========================================");
    Serial.println("");
    Serial.println("LOAD CELL COMMANDS:");
    Serial.println("  W <grams>  - Set weight & measure (e.g., 'W 100.5')");
    Serial.println("  M          - Measure load cell only (don't save)");
    Serial.println("  P          - Print load cell CSV");
    Serial.println("  C          - Clear load cell data");
    Serial.println("");
    Serial.println("HALL SENSOR COMMANDS:");
    Serial.println("  HG         - Measure WITH glass (save to CSV)");
    Serial.println("  HN         - Measure WITHOUT glass (save to CSV)");
    Serial.println("  HM         - Measure hall only (don't save)");
    Serial.println("  HP         - Print hall sensor CSV");
    Serial.println("  HC         - Clear hall sensor data");
    Serial.println("");
    Serial.println("GENERAL:");
    Serial.println("  S          - Scan for coaster");
    Serial.println("  ?          - Show this help");
    Serial.println("");
    Serial.println("LOAD CELL WORKFLOW:");
    Serial.println("  1. Place known weight on coaster");
    Serial.println("  2. Enter 'W <weight>' to measure");
    Serial.println("  3. Repeat for all reference weights");
    Serial.println("  4. Enter 'P' to print data");
    Serial.println("");
    Serial.println("HALL SENSOR WORKFLOW:");
    Serial.println("  1. Place glass on coaster, enter 'HG'");
    Serial.println("  2. Remove glass, enter 'HN'");
    Serial.println("  3. Repeat several times for each state");
    Serial.println("  4. Enter 'HP' to print data");
    Serial.println("========================================\n");
}

void processSerialCommand() {
    if (!Serial.available()) return;

    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();

    if (input.length() == 0) return;

    // Hall sensor commands (start with 'H' followed by second char)
    if (input.startsWith("HG")) {
        // HG - Hall measurement WITH glass
        pendingHallState = HallState::WITH_GLASS;
        saveHallAfterMeasurement = true;
        hallMeasurementRequested = true;
        Serial.println("Hall measurement WITH glass requested...");
        return;
    }
    else if (input.startsWith("HN")) {
        // HN - Hall measurement WITHOUT glass
        pendingHallState = HallState::WITHOUT_GLASS;
        saveHallAfterMeasurement = true;
        hallMeasurementRequested = true;
        Serial.println("Hall measurement WITHOUT glass requested...");
        return;
    }
    else if (input.startsWith("HM")) {
        // HM - Hall measure only (don't save)
        pendingHallState = HallState::NONE;
        saveHallAfterMeasurement = false;
        hallMeasurementRequested = true;
        Serial.println("Hall measurement requested (not saving)...");
        return;
    }
    else if (input.startsWith("HP")) {
        // HP - Print hall CSV
        printHallCSV();
        return;
    }
    else if (input.startsWith("HC")) {
        // HC - Clear hall data
        clearHallData();
        return;
    }

    char cmd = input.charAt(0);

    switch (cmd) {
        case 'W': {
            // W <weight> - Set reference weight and request measurement
            if (input.length() > 2) {
                String weightStr = input.substring(2);
                weightStr.trim();
                float weight = weightStr.toFloat();

                if (weight >= 0) {
                    pendingReferenceWeight = weight;
                    saveAfterMeasurement = true;
                    measurementRequested = true;

                    Serial.print("Reference weight set to: ");
                    Serial.print(weight, 2);
                    Serial.println(" g");
                    Serial.println("Requesting load cell measurement...");
                } else {
                    Serial.println("Invalid weight value!");
                }
            } else {
                Serial.println("Usage: W <weight_in_grams>");
                Serial.println("Example: W 100.5");
            }
            break;
        }

        case 'M':
            // M - Measure load cell only
            saveAfterMeasurement = false;
            measurementRequested = true;
            Serial.println("Requesting load cell measurement (not saving)...");
            break;

        case 'P':
            // P - Print load cell CSV
            printLoadcellCSV();
            break;

        case 'C':
            // C - Clear load cell data
            clearLoadcellData();
            break;

        case 'S':
            // S - Scan
            startScan();
            break;

        case '?':
            printHelp();
            break;

        default:
            Serial.print("Unknown command: ");
            Serial.println(input);
            Serial.println("Enter '?' for help");
            break;
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("  ESP32 Calibration Controller");
    Serial.println("  (Load Cell + Hall Sensor)");
    Serial.println("========================================\n");

    // Initialize SD card
    initSD();

    // Initialize BLE
    initBLE();

    // Initial scan
    Serial.println("\nPerforming initial scan...");
    startScan();

    printHelp();
}

// Helper to ensure BLE connection is established
bool ensureConnected() {
    if (bleState == BLEState::CONNECTED) {
        return true;
    }

    if (!coasterFound) {
        Serial.println("Coaster not found, scanning...");
        startScan();
    }

    if (coasterFound) {
        if (!connectToCoaster()) {
            Serial.println("Failed to connect!");
            return false;
        }
        delay(500);  // Give time for connection to stabilize
        return true;
    }

    Serial.println("Cannot proceed - coaster not found");
    return false;
}

void loop() {
    // Process serial commands
    processSerialCommand();

    // Handle load cell measurement request
    if (measurementRequested) {
        measurementRequested = false;

        if (ensureConnected()) {
            requestLoadcellMeasurement();
        }
    }

    // Handle hall sensor measurement request
    if (hallMeasurementRequested) {
        hallMeasurementRequested = false;

        if (ensureConnected()) {
            requestHallMeasurement();
        }
    }

    // Handle waiting for data
    if (bleState == BLEState::WAITING_FOR_DATA) {
        // Check for load cell data
        if (dataReceived) {
            dataReceived = false;
            bleState = BLEState::CONNECTED;

            // Save if requested
            if (saveAfterMeasurement) {
                saveLoadcellPoint(pendingReferenceWeight, receivedRawValue);
                saveAfterMeasurement = false;
            }

            // Disconnect to save power on coaster
            Serial.println("Load cell measurement complete, disconnecting...");
            disconnectFromCoaster();
        }
        // Check for hall sensor data
        else if (hallDataReceived) {
            hallDataReceived = false;
            bleState = BLEState::CONNECTED;

            // Save if requested
            if (saveHallAfterMeasurement && pendingHallState != HallState::NONE) {
                saveHallPoint(pendingHallState, receivedHallMin, receivedHallMax, receivedHallAvg);
                saveHallAfterMeasurement = false;
            }

            // Disconnect to save power on coaster
            Serial.println("Hall sensor measurement complete, disconnecting...");
            disconnectFromCoaster();
        }
        // Check for timeout
        else if (millis() - measurementStartTime > RESPONSE_TIMEOUT_MS) {
            Serial.println("Timeout waiting for measurement data!");
            bleState = BLEState::CONNECTED;
            disconnectFromCoaster();
        }
    }

    delay(10);
}
