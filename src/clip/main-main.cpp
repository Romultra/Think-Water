#include <Arduino.h>
#include <bluefruit.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <math.h>

const int HALL_SENSOR_PIN = A0;

// Accelerometer Configuration
LSM6DS3 imu(I2C_MODE, 0x6A); // I2C address for Seeed XIAO nrf52840 Sense
const float ACCEL_STABILITY_THRESHOLD = 0.5; // m/s^2 - max deviation for stability
const float UPRIGHT_Z_MIN = 8.5; // m/s^2 - minimum Z acceleration (upright)
const float UPRIGHT_Z_MAX = 10.5; // m/s^2 - maximum Z acceleration (upright)
const int STABILITY_SAMPLES = 5; // Number of consecutive samples to confirm stability

// Hysteresis prevents "flickering" events
// Glass is present if value > 600
// Glass is removed if value < 500
// Between 500-600, we keep the previous state
const int THRESHOLD_HIGH = 600; 
const int THRESHOLD_LOW = 500;

// Duty Cycle Intervals
const int FAST_POLL_MS = 50;   // Check fast when interacting
const int SLOW_POLL_MS = 2000; // Check slow when idle (Deep Sleep Duty Cycle)

BLEService        coasterService = BLEService(0x1234); // Custom UUID
BLECharacteristic measurementChar = BLECharacteristic(0x2A37); // Standard HR or Custom
BLECharacteristic commandChar     = BLECharacteristic(0x2A38);
BLECharacteristic goalChar        = BLECharacteristic(0x2A39);

bool dataPending = false; // Do we have unsent data?

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

bool isUprightStable() {
  // Collect multiple samples to check for stability
  static float lastZ = 9.5; // Initialize to middle of upright range
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

bool isStable() {
  // Collect multiple samples to check for stability (any orientation)
  static float lastX = 0;
  static float lastY = 0;
  static float lastZ = 9.5;
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

void startAdvertising(bool fastMode) {
  Bluefruit.Advertising.stop(); // Stop current adv
  
  // Clean Setup
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(coasterService);

  // MANUFACTURE DATA TRICK: Put status in the packet!
  uint8_t statusByte = dataPending ? 0x01 : 0x00; 
  Bluefruit.Advertising.addManufacturerData(&statusByte, 1);

  if (fastMode) {
    // "I have data! Pick me up!"
    Bluefruit.Advertising.setInterval(32, 244); // 20ms - 150ms
    Bluefruit.Advertising.setFastTimeout(10);   // High speed for 10s
  } else {
    // "I'm just sleeping... zzz"
    Bluefruit.Advertising.setInterval(3200, 3200); // 2000ms fixed
  }
  
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.start(0);
}

void onCommandReceived(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    // Convert BLE write to FSM Event
    Event evt;
    evt.type = EventType::BLE_COMMAND_RECEIVED;
    evt.value = data[0]; // e.g., 1 = LED RED, 2 = LED GREEN
    xQueueSend(eventQueue, &evt, 0);
}


void setupBLE() {
  Bluefruit.begin();
  Bluefruit.setName("SmartCoaster");

  // Define Service & Characteristics
  coasterService.begin();

  measurementChar.setProperties(CHR_PROPS_READ);
  measurementChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  measurementChar.setFixedLen(4); // Float or Uint32
  measurementChar.begin();

  commandChar.setProperties(CHR_PROPS_WRITE);
  commandChar.setWriteCallback(onCommandReceived); // Trigger event on write!
  commandChar.begin();

  goalChar.setProperties(CHR_PROPS_WRITE);
  goalChar.begin();
  
  // Setup Advertising (Heartbeat by default)
  startAdvertising(false); // false = Slow/Heartbeat mode
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Initialize accelerometer
  if (!imu.begin()) {
    Serial.println("Failed to initialize IMU!");
  } else {
    Serial.println("IMU initialized successfully");
  }

  // 1. Setup BLE (Standard Adafruit Code)
  setupBLE();

  // Create a Queue that can hold 10 events
  eventQueue = xQueueCreate(10, sizeof(Event));

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

        // On heartbeat: check for glass removal
        if (evt.type == EventType::HEARTBEAT_TICK) {
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

        break;

      case DeviceState::IDLE_NO_GLASS:
        // Monitor for glass placement
        Serial.println("State: IDLE_NO_GLASS");
        
        // Wait for waik-up interupt from accellerometer
        if (evt.type == EventType::MOTION_DETECTED) {
          currentState = DeviceState::MOVING_NO_GLASS;
          Serial.println("Transition to MOVING_NO_GLASS");
        }

        // On heartbeat: check for glass placement
        if (evt.type == EventType::HEARTBEAT_TICK) {
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
          // Read raw accelerometer values
          float accelX = imu.readFloatAccelX();
          float accelY = imu.readFloatAccelY();
          float accelZ = imu.readFloatAccelZ();
          
          // Calculate magnitude of acceleration
          float accelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
          
          // Test isUprightStable function
          bool isUpright = isUprightStable();
          
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
          Serial.println(isUpright ? "YES" : "NO");
          
          Serial.print("Z Range Check: [");
          Serial.print(UPRIGHT_Z_MIN, 1);
          Serial.print(" - ");
          Serial.print(UPRIGHT_Z_MAX, 1);
          Serial.print("] = ");
          Serial.println((accelZ >= UPRIGHT_Z_MIN && accelZ <= UPRIGHT_Z_MAX) ? "IN RANGE" : "OUT OF RANGE");
          
          Serial.println("---");
        }
        break;
    }
  }
  yield(); // Allow other tasks to run
} 