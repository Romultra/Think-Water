#include <Arduino.h>
#include <bluefruit.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <math.h>
#include <HX711.h>

// BLE Service and Characteristics UUIDs
#define COASTER_SERVICE_UUID        "19B10010-E8F2-537E-4F6C-D104768A1214"
#define CONSUMPTION_CHAR_UUID       "19B10011-E8F2-537E-4F6C-D104768A1214"  // Notify consumption events
#define COMMAND_CHAR_UUID           "19B10012-E8F2-537E-4F6C-D104768A1214"  // Write commands from base
#define GOAL_CHAR_UUID              "19B10013-E8F2-537E-4F6C-D104768A1214"  // Write hydration goal

const int HALL_SENSOR_PIN = A0;
const int IMU_INT1_PIN = PIN_LSM6DS3TR_C_INT1; // LSM6DS3TR-C INT1 pin on XIAO nrf52840 Sense
const int LOADCELL_DOUT_PIN = 9;
const int LOADCELL_SCK_PIN = 8;

// Accelerometer Configuration
LSM6DS3 imu(I2C_MODE, 0x6A); // I2C address for Seeed XIAO nrf52840 Sense

// NOTE: the IMU returns accelerations in units of g (1g = 9.80665 m/s^2)
// Convert the previous m/s^2 thresholds into g-units here.
const float ACCEL_STABILITY_THRESHOLD = 0.051f; // ~0.5 m/s^2 in g (max deviation for stability)
const float UPRIGHT_Z_MIN = 0.867f; // ~8.5 m/s^2 in g (minimum Z acceleration for upright)
const float UPRIGHT_Z_MAX = 1.071f; // ~10.5 m/s^2 in g (maximum Z acceleration for upright)
const int STABILITY_SAMPLES = 5; // Number of consecutive samples to confirm stability

// Load Cell Configuration
HX711 loadcell;
const float LOADCELL_CALIBRATION_FACTOR = 1.0; // Placeholder - needs calibration

// BLE Objects
BLEService coasterService(COASTER_SERVICE_UUID);
BLECharacteristic consumptionChar(CONSUMPTION_CHAR_UUID);
BLECharacteristic commandChar(COMMAND_CHAR_UUID);
BLECharacteristic goalChar(GOAL_CHAR_UUID);

bool bleConnected = false;
volatile bool dataPendingToSend = false;
volatile bool shouldAdvertise = false;
uint32_t lastConsumptionValue = 0;
float hydrationGoal = 2000.0; // Default 2000ml

// Hysteresis prevents "flickering" events
// Glass is present if value > 600
// Glass is removed if value < 500
// Between 500-600, we keep the previous state
const int THRESHOLD_HIGH = 600; 
const int THRESHOLD_LOW = 500;

// Duty Cycle Intervals
const int FAST_POLL_MS = 50;   // Check fast when interacting
const int SLOW_POLL_MS = 2000; // Check slow when idle (Deep Sleep Duty Cycle)

enum class EventType {
  HEARTBEAT_TICK,
  GLASS_PLACED,
  GLASS_REMOVED,
  DRINK_DETECTED,
  MOTION_DETECTED,
  BLE_COMMAND_RECEIVED,
  ALERT_TIMEOUT,
  WAKEUP
};

struct Event {
  EventType type;
  uint32_t value;
  uint32_t timestamp;
};

enum class DeviceState {
  IDLE_WITH_GLASS,
  IDLE_NO_GLASS,
  MOVING_WITH_GLASS,
  MOVING_NO_GLASS,
  ALERTING,
  TEST
};

DeviceState currentState = DeviceState::TEST;

QueueHandle_t eventQueue;
TimerHandle_t heartbeatTimer;

void heartbeatCallback(TimerHandle_t xTimer) {
  Event evt;
  evt.type = EventType::HEARTBEAT_TICK;
  xQueueSend(eventQueue, &evt, 0);
}

bool isGlassPresent() {
  int val = analogRead(HALL_SENSOR_PIN);
  if (val > THRESHOLD_HIGH) {
    return true;
  } else if (val < THRESHOLD_LOW) {
    return false;
  }
  // In hysteresis zone, keep previous state
  static bool lastState = false;
  return lastState;
}

float getWeight() {
  // Check if load cell is ready
  if (loadcell.is_ready()) {
    // Get average of 10 readings (already calibrated by set_scale)
    float weight = loadcell.get_units(10);
    return weight;
  } else {
    Serial.println("Load cell not ready");
    return -1.0; // Return -1 to indicate error
  }
}

bool isUprightStable() {
  // Collect multiple samples to check for stability
  static float lastZ = 0.97f; // Initialize to middle of upright range (g)
  float sumDeviation = 0;
  bool isStable = true;

  for (int i = 0; i < STABILITY_SAMPLES; i++) {
    // Read accelerometer data
    float accelX = imu.readFloatAccelX();
    float accelY = imu.readFloatAccelY();
    float accelZ = imu.readFloatAccelZ();

    // Check if Z-axis acceleration is in upright range
    if (accelZ < UPRIGHT_Z_MIN || accelZ > UPRIGHT_Z_MAX) {
      return false; // Not in upright orientation
    }

    // Check for stability: deviation from expected Z value
    float deviation = abs(accelZ - lastZ);
    sumDeviation += deviation;

    // Check that X and Y are close to zero (minimal lateral acceleration)
    if (abs(accelX) > ACCEL_STABILITY_THRESHOLD || abs(accelY) > ACCEL_STABILITY_THRESHOLD) {
      isStable = false;
    }

    lastZ = accelZ;
    delay(10); // Small delay between samples
  }

  // Average deviation across samples
  float avgDeviation = sumDeviation / STABILITY_SAMPLES;
  
  return isStable && (avgDeviation < ACCEL_STABILITY_THRESHOLD);
}

// ISR for IMU wake-up interrupt (FreeRTOS-safe)
void imuInterruptHandler() {
  printf("IMU Interrupt Triggered\n");
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  Event motionEvt;
  motionEvt.type = EventType::MOTION_DETECTED;
  motionEvt.timestamp = millis();
  // Send event to queue from ISR
  if (eventQueue != NULL) {
    xQueueSendFromISR(eventQueue, &motionEvt, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

// Configure LSM6DS3TR-C wake-up interrupt
void setupIMUWakeup() {
  // Configure wake-up threshold (WK_THS register 0x5B)
  // Bits [5:0] = threshold, 1 LSB = FS_XL/64
  // At 2g full scale: 1 LSB = 2g/64 = 31.25mg
  // Setting to 0x02 = ~62.5mg threshold
  // imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x);
  imu.writeRegister(0x5B, 0x02);
  
  // Configure wake-up duration (WAKE_UP_DUR register 0x5C)
  // WAKE_DUR[1:0] = duration in ODR cycles before wake-up
  // Set to 0x00 for immediate wake-up
  imu.writeRegister(0x5C, 0x00);
  
  // Enable wake-up interrupt on INT1 (MD1_CFG register 0x5E)
  // Bit 5 (INT1_WU) = 1 to route wake-up to INT1
  imu.writeRegister(0x5E, 0x20);
  
  // Enable interrupts (CTRL4_C register 0x13)
  // Bit 4 (INT1_ON_INT1) = 1 to enable INT1 pin
  uint8_t ctrl4c;
  imu.readRegister(&ctrl4c, 0x13);
  ctrl4c |= 0x10;
  imu.writeRegister(0x13, ctrl4c);
  
  // Configure interrupt pin as input with pull-down
  pinMode(IMU_INT1_PIN, INPUT_PULLDOWN);
  
  // Attach interrupt handler (rising edge)
  attachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN), imuInterruptHandler, RISING);
  
  Serial.println("IMU wake-up interrupt configured");
}

// Clear the wake-up interrupt source
void clearIMUInterrupt() {
  uint8_t wakeupSrc;
  imu.readRegister(&wakeupSrc, 0x1B); // Read WAKE_UP_SRC to clear interrupt
}

bool isStable() {
  // Collect multiple samples to check for stability (any orientation)
  static float lastX = 0;
  static float lastY = 0;
  static float lastZ = 0.97f; // g-units
  float sumDeviation = 0;
  bool isStable = true;

  for (int i = 0; i < STABILITY_SAMPLES; i++) {
    // Read accelerometer data
    float accelX = imu.readFloatAccelX();
    float accelY = imu.readFloatAccelY();
    float accelZ = imu.readFloatAccelZ();

    // Check for stability: deviation from previous values
    float deviationX = abs(accelX - lastX);
    float deviationY = abs(accelY - lastY);
    float deviationZ = abs(accelZ - lastZ);
    sumDeviation += (deviationX + deviationY + deviationZ);

    // Check that all axes have minimal acceleration (low movement)
    if (abs(accelX) > ACCEL_STABILITY_THRESHOLD || 
        abs(accelY) > ACCEL_STABILITY_THRESHOLD || 
        abs(accelZ) > ACCEL_STABILITY_THRESHOLD) {
      isStable = false;
    }

    lastX = accelX;
    lastY = accelY;
    lastZ = accelZ;
    delay(10); // Small delay between samples
  }

  // Average deviation across samples
  float avgDeviation = sumDeviation / (STABILITY_SAMPLES * 3);
  
  return isStable && (avgDeviation < ACCEL_STABILITY_THRESHOLD);
}

// BLE Connection Callbacks
void connect_callback(uint16_t conn_handle) {
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));
  
  Serial.print("Connected to ");
  Serial.println(central_name);
  
  bleConnected = true;
  
  // If we have pending data, it will be sent from main loop
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  Serial.print("Disconnected, reason = 0x");
  Serial.println(reason, HEX);
  
  bleConnected = false;
  
  // If we still have data to send or need to advertise, restart advertising
  if (dataPendingToSend || shouldAdvertise) {
    Bluefruit.Advertising.start(0); // Advertise indefinitely until connected
    Serial.println("Restarting advertising (data pending)");
  }
}

// Command Characteristic Write Callback
void command_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len < 1) return;
  
  uint8_t command = data[0];
  Serial.print("Command received: 0x");
  Serial.println(command, HEX);
  
  // Command codes:
  // 0x01 = LED Alert Request
  // 0x02 = Query Status
  // 0xFF = Disconnect after sending data
  
  Event evt;
  evt.type = EventType::BLE_COMMAND_RECEIVED;
  evt.value = command;
  evt.timestamp = millis();
  xQueueSend(eventQueue, &evt, 0);
}

// Goal Characteristic Write Callback
void goal_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len == 4) {
    // Expect 4 bytes for float (hydration goal in ml)
    memcpy(&hydrationGoal, data, 4);
    Serial.print("Hydration goal updated: ");
    Serial.print(hydrationGoal);
    Serial.println(" ml");
  }
}

// Setup BLE
void setupBLE() {
  // Initialize Bluefruit
  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Reduce power for shorter range, save battery
  Bluefruit.setName("CoasterClip");
  
  // Set connection callbacks
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  
  // Configure and Add Service
  coasterService.begin();
  
  // Configure Consumption Characteristic (Notify only)
  consumptionChar.setProperties(CHR_PROPS_NOTIFY);
  consumptionChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  consumptionChar.setFixedLen(8); // timestamp(4) + value(4)
  consumptionChar.begin();
  
  // Configure Command Characteristic (Write only)
  commandChar.setProperties(CHR_PROPS_WRITE);
  commandChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  commandChar.setWriteCallback(command_write_callback);
  commandChar.setFixedLen(4); // command code + optional data
  commandChar.begin();
  
  // Configure Goal Characteristic (Write only)
  goalChar.setProperties(CHR_PROPS_WRITE);
  goalChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  goalChar.setWriteCallback(goal_write_callback);
  goalChar.setFixedLen(4); // float value (ml)
  goalChar.begin();
  
  // Setup advertising (but don't start yet - only when we have data)
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(coasterService);
  Bluefruit.Advertising.addName();
  
  // Set advertising interval (units of 0.625ms)
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 320);    // 100-200ms (balance power/discovery)
  Bluefruit.Advertising.setFastTimeout(30);       // Fast advertising for 30s, then slow
  
  Serial.println("BLE initialized (not advertising yet)");
}

// Start BLE advertising when we have data to send
void startBLEAdvertising() {
  if (!Bluefruit.Advertising.isRunning()) {
    Bluefruit.Advertising.start(0); // Advertise indefinitely
    Serial.println("BLE advertising started");
  }
}

// Stop BLE advertising to save power
void stopBLEAdvertising() {
  if (Bluefruit.Advertising.isRunning()) {
    Bluefruit.Advertising.stop();
    Serial.println("BLE advertising stopped");
  }
}

// Send consumption event via BLE
bool sendConsumptionEvent(uint32_t timestamp, uint32_t value) {
  if (!bleConnected) {
    // Not connected, mark data as pending and start advertising
    dataPendingToSend = true;
    lastConsumptionValue = value;
    startBLEAdvertising();
    Serial.println("Not connected - advertising to send data");
    return false;
  }
  
  // Pack data: timestamp(4 bytes) + value(4 bytes)
  uint8_t data[8];
  memcpy(data, &timestamp, 4);
  memcpy(data + 4, &value, 4);
  
  if (consumptionChar.notify(data, sizeof(data))) {
    Serial.println("Consumption event sent via BLE");
    dataPendingToSend = false;
    
    // Disconnect after sending to save power
    // The base station will disconnect, or we can request it
    return true;
  } else {
    Serial.println("Failed to send consumption event");
    dataPendingToSend = true;
    return false;
  }
}

void setup() {
  Serial.begin(115200);
  //while (!Serial) { delay(10); }

  // Create a Queue that can hold 10 events (must exist before ISR runs)
  eventQueue = xQueueCreate(10, sizeof(Event));

  // Initialize internal I2C bus for IMU (Wire1 on XIAO nRF52840 Sense)
  Wire1.begin();
  delay(10); // Allow I2C to stabilize

  // Initialize accelerometer
  if (imu.begin() != 0) {
    Serial.println("Failed to initialize IMU!");
  } else {
    Serial.println("IMU initialized successfully");
    // Setup wake-up interrupt after IMU is initialized
    setupIMUWakeup();
    clearIMUInterrupt(); // Clear any pending interrupts
  }
  
  // Initialize load cell
  /*
  loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  loadcell.set_scale(LOADCELL_CALIBRATION_FACTOR); // Apply calibration factor
  loadcell.tare(); // Reset the scale to 
  Serial.println("Load cell initialized");
  */

  // Initialize BLE
  setupBLE();
  startBLEAdvertising();

  // Start with SLOW polling (Hibernation)
  heartbeatTimer = xTimerCreate("Heartbeat", pdMS_TO_TICKS(SLOW_POLL_MS), pdTRUE, 0, heartbeatCallback);
  xTimerStart(heartbeatTimer, 0);

  Serial.println("Coaster Clip Main Setup");
  delay(500);
}

void loop() {
  Event evt;

  if (xQueueReceive(eventQueue, &evt, portMAX_DELAY) == pdTRUE) {
    switch (currentState) {
      case DeviceState::IDLE_WITH_GLASS:
        // Monitor for drinking activity
        Serial.println("State: IDLE_WITH_GLASS");
        
        // Wait for waik-up interupt from accellerometer
        if (evt.type == EventType::MOTION_DETECTED) {
          currentState = DeviceState::MOVING_WITH_GLASS;
          Serial.println("Transition to MOVING_WITH_GLASS");
        }

        // On heartbeat: check for glass removal and send pending BLE data
        if (evt.type == EventType::HEARTBEAT_TICK) {
          // If connected and have pending data, send it
          if (bleConnected && dataPendingToSend) {
            sendConsumptionEvent(millis(), lastConsumptionValue);
          }
          
          bool glassPresent = isGlassPresent();
          if (!glassPresent) {
            currentState = DeviceState::IDLE_NO_GLASS;
            Serial.println("Transition to IDLE_NO_GLASS");
            // Create GLASS_REMOVED event
            Event removeEvt;
            removeEvt.type = EventType::GLASS_REMOVED;
            xQueueSend(eventQueue, &removeEvt, 0);
          }
        }
        
        // Handle BLE commands (LED alerts, status queries, etc.)
        if (evt.type == EventType::BLE_COMMAND_RECEIVED) {
          uint8_t command = (uint8_t)evt.value;
          // Command 0x01 = LED Alert (not implemented yet)
          // Command 0x02 = Status Query (could send current state)
          // For now, just acknowledge
          Serial.print("BLE Command in IDLE_WITH_GLASS: 0x");
          Serial.println(command, HEX);
        }

        break;

      case DeviceState::IDLE_NO_GLASS:
        // Monitor for glass placement
        Serial.println("State: IDLE_NO_GLASS");
        
        // Wait for waik-up interupt from accellerometer
        if (evt.type == EventType::MOTION_DETECTED) {
          currentState = DeviceState::MOVING_NO_GLASS;
          Serial.println("Transition to MOVING_NO_GLASS");
        }

        // On heartbeat: check for glass placement and send pending BLE data
        if (evt.type == EventType::HEARTBEAT_TICK) {
          // If connected and have pending data, send it
          if (bleConnected && dataPendingToSend) {
            sendConsumptionEvent(millis(), lastConsumptionValue);
          }
          
          bool glassPresent = isGlassPresent();
          if (glassPresent) {
            currentState = DeviceState::IDLE_WITH_GLASS;
            Serial.println("Transition to IDLE_WITH_GLASS");
            // Create GLASS_PLACED event
            Event placeEvt;
            placeEvt.type = EventType::GLASS_PLACED;
            xQueueSend(eventQueue, &placeEvt, 0);
          }
        }
        
        // Handle BLE commands
        if (evt.type == EventType::BLE_COMMAND_RECEIVED) {
          uint8_t command = (uint8_t)evt.value;
          Serial.print("BLE Command in IDLE_NO_GLASS: 0x");
          Serial.println(command, HEX);
        }
        break;
      
      case DeviceState::MOVING_WITH_GLASS:
        // Handle movement with glass
        Serial.println("State: MOVING_WITH_GLASS");

        // One heartbeat: check accel for upright stable position and for glass removal
        if (evt.type == EventType::HEARTBEAT_TICK) {
          bool isStableUpright = isUprightStable();
          bool glassPresent = isGlassPresent();

          if (isStableUpright) {
            if (glassPresent) {
              currentState = DeviceState::IDLE_WITH_GLASS;
              Serial.println("Transition to IDLE_WITH_GLASS");
              // Create DRINK_DETECTED event
              Event drinkEvt;
              drinkEvt.type = EventType::DRINK_DETECTED;
              xQueueSend(eventQueue, &drinkEvt, 0);
              
              // Send consumption event via BLE
              float weight = getWeight();
              if (weight > 0) {
                uint32_t weightValue = (uint32_t)(weight * 100); // Convert to int (0.01g precision)
                sendConsumptionEvent(millis(), weightValue);
              }

            } else {
              currentState = DeviceState::IDLE_NO_GLASS;
              Serial.println("Transition to IDLE_NO_GLASS");
              // Create GLASS_REMOVED event
              Event removeEvt;
              removeEvt.type = EventType::GLASS_REMOVED;
              xQueueSend(eventQueue, &removeEvt, 0);
            }          
          } else if (!glassPresent) {
            currentState = DeviceState::MOVING_NO_GLASS;
            Serial.println("Transition to MOVING_NO_GLASS");
            // Create GLASS_REMOVED event
            Event removeEvt;
            removeEvt.type = EventType::GLASS_REMOVED;
            xQueueSend(eventQueue, &removeEvt, 0);
          }
        }
        break;
      
      case DeviceState::MOVING_NO_GLASS:
        // Handle movement without glass
        Serial.println("State: MOVING_NO_GLASS");

        // On heartbeat, check accel for any stable position and for glass placement
        if (evt.type == EventType::HEARTBEAT_TICK) {
          bool isDeviceStable = isStable();
          bool glassNowPresent = isGlassPresent();
          if (isDeviceStable) {
            if (glassNowPresent) {
              currentState = DeviceState::IDLE_WITH_GLASS;
              Serial.println("Transition to IDLE_WITH_GLASS");
              // Create GLASS_PLACED event
              Event placeEvt;
              placeEvt.type = EventType::GLASS_PLACED;
              xQueueSend(eventQueue, &placeEvt, 0);
            } else {
              currentState = DeviceState::IDLE_NO_GLASS;
              Serial.println("Transition to IDLE_NO_GLASS");
              // Only create event when changing glass state
            }
          } else if (glassNowPresent) {
            currentState = DeviceState::MOVING_WITH_GLASS;
            Serial.println("Transition to MOVING_WITH_GLASS");
            // Create GLASS_PLACED event
            Event placeEvt;
            placeEvt.type = EventType::GLASS_PLACED;
            xQueueSend(eventQueue, &placeEvt, 0);
          }
        }
        break;
      
      case DeviceState::ALERTING:
        // Alert user if needed
        Serial.println("State: ALERTING");
        // Transition logic here
        break;
      
      case DeviceState::TEST:
        // Testing state
        Serial.println("State: TEST");
        
        if (evt.type == EventType::HEARTBEAT_TICK) {
          /*
          // Read raw accelerometer values
          float accelX = imu.readFloatAccelX();
          float accelY = imu.readFloatAccelY();
          float accelZ = imu.readFloatAccelZ();
          
          // Calculate magnitude of acceleration
          float accelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
          
          // Test isUprightStable function
          bool isUpright = isUprightStable();
          // Test isStable (any orientation)
          bool isAnyStable = isStable();

          // Test load cell
          float weight = getWeight();
          
          // Test hall sensor
          int hallValue = analogRead(HALL_SENSOR_PIN);
          bool glassPresent = isGlassPresent();

          // Print all values to serial monitor
          Serial.print("AccelX: ");
          Serial.print(accelX, 3);
          Serial.print(" | AccelY: ");
          Serial.print(accelY, 3);
          Serial.print(" | AccelZ: ");
          Serial.print(accelZ, 3);
          Serial.print(" | Magnitude: ");
          Serial.print(accelMagnitude, 3);
          Serial.print(" | Upright+Stable: ");
          Serial.print(isUpright ? "YES" : "NO");
          Serial.print(" | Stable(any): ");
          Serial.println(isAnyStable ? "YES" : "NO");
          
          Serial.print("Z Range Check (g): [");
          Serial.print(UPRIGHT_Z_MIN, 3);
          Serial.print(" - ");
          Serial.print(UPRIGHT_Z_MAX, 3);
          Serial.print("] = ");
          Serial.println((accelZ >= UPRIGHT_Z_MIN && accelZ <= UPRIGHT_Z_MAX) ? "IN RANGE" : "OUT OF RANGE");
          
          Serial.print("Load Cell Weight: ");
          if (weight >= 0) {
            Serial.print(weight, 2);
            Serial.println(" units");
          } else {
            Serial.println("ERROR");
          }
          
          Serial.print("Hall Sensor: ");
          Serial.print(hallValue);
          Serial.print(" | Glass Present: ");
          Serial.println(glassPresent ? "YES" : "NO");*/
          
          // BLE Test Status
          Serial.print("BLE Connected: ");
          Serial.print(bleConnected ? "YES" : "NO");
          Serial.print(" | Advertising: ");
          Serial.print(Bluefruit.Advertising.isRunning() ? "YES" : "NO");
          Serial.print(" | Data Pending: ");
          Serial.println(dataPendingToSend ? "YES" : "NO");
          
          Serial.println("---");
          
          // BLE Test: If connected, send test consumption data
          if (bleConnected && dataPendingToSend) {
            Serial.println("BLE TEST: Sending pending consumption data...");
            sendConsumptionEvent(millis(), lastConsumptionValue);
          }
        }

        if (evt.type == EventType::MOTION_DETECTED) {
          Serial.println("Motion Detected Event Received");
          clearIMUInterrupt(); // Clear interrupt source
        }
        
        // BLE Test: Handle commands in TEST mode
        if (evt.type == EventType::BLE_COMMAND_RECEIVED) {
          uint8_t command = (uint8_t)evt.value;
          Serial.println("========================================");
          Serial.print("BLE TEST: Command Received: 0x");
          Serial.println(command, HEX);
          
          switch(command) {
            case 0x01:
              Serial.println("  -> LED Alert Command (not implemented)");
              break;
            case 0x02: {
              Serial.println("  -> Status Query Command");
              Serial.print("     Current State: TEST, Weight: ");
              float testWeight = getWeight();
              Serial.print(testWeight, 2);
              Serial.println(" units");
              break;
            }
            case 0xFF:
              Serial.println("  -> Disconnect Request");
              // ESP32 will disconnect
              break;
            default:
              Serial.println("  -> Unknown Command");
              break;
          }
          Serial.println("========================================");
        }
        
        // BLE Test: Simulate consumption event on serial input
        if (Serial.available()) {
          char cmd = Serial.read();
          if (cmd == 't' || cmd == 'T') {
            Serial.println("\n*** BLE TEST: Sending test consumption event ***");
            float testWeight = getWeight();
            if (testWeight > 0) {
              uint32_t weightValue = (uint32_t)(testWeight * 100);
              sendConsumptionEvent(millis(), weightValue);
            } else {
              // Send dummy data for testing
              sendConsumptionEvent(millis(), 15000); // 150.00 grams
            }
          } else if (cmd == 'a' || cmd == 'A') {
            Serial.println("\n*** BLE TEST: Starting advertising ***");
            startBLEAdvertising();
          } else if (cmd == 's' || cmd == 'S') {
            Serial.println("\n*** BLE TEST: Stopping advertising ***");
            stopBLEAdvertising();
          }
        }
        break;
    }
  }
  yield(); // Allow other tasks to run
} 