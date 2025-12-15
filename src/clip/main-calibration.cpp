// Coaster Calibration Firmware (Load Cell + Hall Sensor)
// For Seeed XIAO nRF52840 Sense
//
// BLE Protocol:
// - Command 0x10: Request load cell reading (returns avg raw ADC value)
// - Command 0x11: Request hall sensor reading (returns min, max, avg)
//
// Response format:
// - Load cell:   [0x10][avg:4 bytes] = 5 bytes
// - Hall sensor: [0x11][min:4 bytes][max:4 bytes][avg:4 bytes] = 13 bytes

#include <Arduino.h>
#include <bluefruit.h>
#include <HX711.h>
#include <Adafruit_FlashTransport.h>

Adafruit_FlashTransport_QSPI flashTransport;

// BLE Service and Characteristics UUIDs
#define CALIBRATION_SERVICE_UUID    "19B10020-E8F2-537E-4F6C-D104768A1214"
#define CALIBRATION_DATA_CHAR_UUID  "19B10021-E8F2-537E-4F6C-D104768A1214"  // Notify readings
#define CALIBRATION_CMD_CHAR_UUID   "19B10022-E8F2-537E-4F6C-D104768A1214"  // Write commands

// Hardware Pins
const int HALL_SENSOR_PIN = A0;
const int LOADCELL_DOUT_PIN = D2;
const int LOADCELL_SCK_PIN = D3;

// Calibration configuration
const int LOADCELL_SAMPLES = 50;     // Number of load cell readings to average
const int HALL_SAMPLES = 100;        // Number of hall sensor readings
const int SAMPLE_DELAY_MS = 50;      // Delay between load cell samples
const int HALL_SAMPLE_DELAY_MS = 20; // Delay between hall sensor samples

// Command codes
const uint8_t CMD_LOADCELL = 0x10;
const uint8_t CMD_HALL = 0x11;

// Load Cell
HX711 loadcell;

// BLE Objects
BLEService calibrationService(CALIBRATION_SERVICE_UUID);
BLECharacteristic calibrationDataChar(CALIBRATION_DATA_CHAR_UUID);
BLECharacteristic calibrationCmdChar(CALIBRATION_CMD_CHAR_UUID);

bool bleConnected = false;
volatile uint8_t pendingCommand = 0;  // 0 = none, 0x10 = loadcell, 0x11 = hall

// Forward declarations
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void command_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);

void connect_callback(uint16_t conn_handle) {
    BLEConnection* connection = Bluefruit.Connection(conn_handle);
    char central_name[32] = { 0 };
    connection->getPeerName(central_name, sizeof(central_name));

    Serial.print("Connected to ");
    Serial.println(central_name);

    bleConnected = true;
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    (void)conn_handle;
    Serial.print("Disconnected, reason = 0x");
    Serial.println(reason, HEX);
    bleConnected = false;
}

void command_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    (void)conn_hdl;
    (void)chr;

    if (len < 1) return;

    uint8_t command = data[0];
    Serial.print("Command received: 0x");
    Serial.println(command, HEX);

    if (command == CMD_LOADCELL) {
        pendingCommand = CMD_LOADCELL;
        Serial.println("Load cell reading requested!");
    } else if (command == CMD_HALL) {
        pendingCommand = CMD_HALL;
        Serial.println("Hall sensor reading requested!");
    }
}

void setupBLE() {
    Bluefruit.begin();
    Bluefruit.setTxPower(4);
    Bluefruit.setName("CoasterCalib");

    // Set connection callbacks
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    // Configure and Add Service
    calibrationService.begin();

    // Configure Calibration Data Characteristic (Notify, variable length)
    calibrationDataChar.setProperties(CHR_PROPS_NOTIFY);
    calibrationDataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    calibrationDataChar.setMaxLen(20);  // Max 20 bytes for responses
    calibrationDataChar.begin();

    // Configure Command Characteristic (Write)
    calibrationCmdChar.setProperties(CHR_PROPS_WRITE);
    calibrationCmdChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
    calibrationCmdChar.setWriteCallback(command_write_callback);
    calibrationCmdChar.setMaxLen(4);
    calibrationCmdChar.begin();

    // Setup advertising
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(calibrationService);
    Bluefruit.Advertising.addName();

    // Set advertising interval
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(160, 320);
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);

    Serial.println("BLE Calibration Service started");
    Serial.println("Advertising as 'CoasterCalib'");
}

// Get raw load cell value (no calibration applied)
int32_t getRawLoadcellReading() {
    if (loadcell.is_ready()) {
        return loadcell.read();
    }
    return 0;
}

// Get hall sensor reading
int32_t getHallReading() {
    return analogRead(HALL_SENSOR_PIN);
}

// Perform load cell calibration measurement
int32_t performLoadcellMeasurement() {
    Serial.println("\n========================================");
    Serial.println("LOAD CELL MEASUREMENT");
    Serial.print("Taking ");
    Serial.print(LOADCELL_SAMPLES);
    Serial.println(" samples...");

    int64_t sum = 0;
    int validSamples = 0;

    for (int i = 0; i < LOADCELL_SAMPLES; i++) {
        if (loadcell.is_ready()) {
            int32_t raw = loadcell.read();
            sum += raw;
            validSamples++;

            if ((i + 1) % 10 == 0) {
                Serial.print("  Progress: ");
                Serial.print(i + 1);
                Serial.print("/");
                Serial.println(LOADCELL_SAMPLES);
            }
        }
        delay(SAMPLE_DELAY_MS);
    }

    if (validSamples == 0) {
        Serial.println("ERROR: No valid readings!");
        return 0;
    }

    int32_t average = (int32_t)(sum / validSamples);

    Serial.println("----------------------------------------");
    Serial.print("Valid samples: ");
    Serial.println(validSamples);
    Serial.print("Average: ");
    Serial.println(average);
    Serial.println("========================================\n");

    return average;
}

// Perform hall sensor calibration measurement
// Returns min, max, avg via output parameters
void performHallMeasurement(int32_t* outMin, int32_t* outMax, int32_t* outAvg) {
    Serial.println("\n========================================");
    Serial.println("HALL SENSOR MEASUREMENT");
    Serial.print("Taking ");
    Serial.print(HALL_SAMPLES);
    Serial.println(" samples...");

    int32_t minVal = INT32_MAX;
    int32_t maxVal = INT32_MIN;
    int64_t sum = 0;

    for (int i = 0; i < HALL_SAMPLES; i++) {
        int32_t reading = analogRead(HALL_SENSOR_PIN);

        if (reading < minVal) minVal = reading;
        if (reading > maxVal) maxVal = reading;
        sum += reading;

        if ((i + 1) % 20 == 0) {
            Serial.print("  Progress: ");
            Serial.print(i + 1);
            Serial.print("/");
            Serial.print(HALL_SAMPLES);
            Serial.print(" (current: ");
            Serial.print(reading);
            Serial.println(")");
        }

        delay(HALL_SAMPLE_DELAY_MS);
    }

    *outMin = minVal;
    *outMax = maxVal;
    *outAvg = (int32_t)(sum / HALL_SAMPLES);

    Serial.println("----------------------------------------");
    Serial.print("Min: ");
    Serial.println(*outMin);
    Serial.print("Max: ");
    Serial.println(*outMax);
    Serial.print("Avg: ");
    Serial.println(*outAvg);
    Serial.print("Range: ");
    Serial.println(*outMax - *outMin);
    Serial.println("========================================\n");
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    delay(500);

    Serial.println("\n========================================");
    Serial.println("  Coaster Calibration Mode");
    Serial.println("  (Load Cell + Hall Sensor)");
    Serial.println("========================================\n");

    // Configure battery charging current to 100mA
    pinMode(22, OUTPUT);
    digitalWrite(22, LOW);

    // Initialize SPI Flash and enter deep power-down mode
    flashTransport.begin();
    flashTransport.runCommand(0xB9);
    delayMicroseconds(5);
    flashTransport.end();

    // Initialize hall sensor pin
    pinMode(HALL_SENSOR_PIN, INPUT);
    Serial.print("Hall sensor initial reading: ");
    Serial.println(analogRead(HALL_SENSOR_PIN));

    // Initialize load cell (no scale/offset - we want RAW values)
    Serial.println("Initializing load cell...");
    loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

    // Reset the HX711
    digitalWrite(LOADCELL_SCK_PIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(LOADCELL_SCK_PIN, LOW);

    // Wait for load cell to be ready
    Serial.print("Waiting for load cell...");
    while (!loadcell.is_ready()) {
        delay(100);
        Serial.print(".");
    }
    Serial.println(" Ready!");

    // Important: Do NOT apply any scale or offset
    loadcell.set_scale(1.0);
    loadcell.set_offset(0);

    Serial.print("Load cell initial reading: ");
    Serial.println(getRawLoadcellReading());

    // Initialize BLE
    setupBLE();

    Serial.println("\n----------------------------------------");
    Serial.println("Ready for calibration!");
    Serial.println("BLE Commands:");
    Serial.println("  0x10 - Load cell measurement");
    Serial.println("  0x11 - Hall sensor measurement");
    Serial.println("Serial Commands:");
    Serial.println("  L - Load cell measurement");
    Serial.println("  H - Hall sensor measurement");
    Serial.println("  R - Single readings (both)");
    Serial.println("----------------------------------------\n");
}

void loop() {
    // Process pending BLE commands
    if (pendingCommand != 0 && bleConnected) {
        uint8_t cmd = pendingCommand;
        pendingCommand = 0;

        if (cmd == CMD_LOADCELL) {
            // Load cell measurement
            int32_t avg = performLoadcellMeasurement();

            // Send response: [type][avg]
            uint8_t data[5];
            data[0] = CMD_LOADCELL;
            memcpy(&data[1], &avg, 4);

            if (calibrationDataChar.notify(data, sizeof(data))) {
                Serial.println("Load cell data sent via BLE");
            } else {
                Serial.println("Failed to send load cell data!");
            }
        }
        else if (cmd == CMD_HALL) {
            // Hall sensor measurement
            int32_t minVal, maxVal, avgVal;
            performHallMeasurement(&minVal, &maxVal, &avgVal);

            // Send response: [type][min][max][avg]
            uint8_t data[13];
            data[0] = CMD_HALL;
            memcpy(&data[1], &minVal, 4);
            memcpy(&data[5], &maxVal, 4);
            memcpy(&data[9], &avgVal, 4);

            if (calibrationDataChar.notify(data, sizeof(data))) {
                Serial.println("Hall sensor data sent via BLE");
            } else {
                Serial.println("Failed to send hall sensor data!");
            }
        }
    }

    // Serial commands for manual testing
    if (Serial.available()) {
        char cmd = Serial.read();

        if (cmd == 'l' || cmd == 'L') {
            // Manual load cell measurement
            int32_t avg = performLoadcellMeasurement();
            Serial.print("Load cell average: ");
            Serial.println(avg);
        }
        else if (cmd == 'h' || cmd == 'H') {
            // Manual hall sensor measurement
            int32_t minVal, maxVal, avgVal;
            performHallMeasurement(&minVal, &maxVal, &avgVal);
        }
        else if (cmd == 'r' || cmd == 'R') {
            // Single readings
            Serial.println("\n--- Single Readings ---");
            Serial.print("Load cell: ");
            Serial.println(getRawLoadcellReading());
            Serial.print("Hall sensor: ");
            Serial.println(getHallReading());
            Serial.println("-----------------------\n");
        }
    }

    delay(10);
}
