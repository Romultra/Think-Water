#include <Arduino.h>
#include <bluefruit.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <Wire.h>
#include <math.h>
#include <climits>
#include <HX711.h>
#include <Adafruit_FlashTransport.h>
#include <Adafruit_NeoPixel.h>

Adafruit_FlashTransport_QSPI flashTransport;

// BLE Service and Characteristics UUIDs
#define COASTER_SERVICE_UUID        "19B10010-E8F2-537E-4F6C-D104768A1214"
#define CONSUMPTION_CHAR_UUID       "19B10011-E8F2-537E-4F6C-D104768A1214"  // Notify consumption events
#define COMMAND_CHAR_UUID           "19B10012-E8F2-537E-4F6C-D104768A1214"  // Write commands from base
#define GOAL_CHAR_UUID              "19B10013-E8F2-537E-4F6C-D104768A1214"  // Write hydration goal

const int HALL_SENSOR_PIN = A0;
const int LOADCELL_DOUT_PIN = D2;
const int LOADCELL_SCK_PIN = D3;
const int NEO_PIXEL_PIN = D7;

// NeoPixel configuration
#define NUM_NEOPIXELS 10
Adafruit_NeoPixel strip(NUM_NEOPIXELS, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Load Cell Configuration
HX711 loadcell;
const float LOADCELL_SCALE = -373.3692f;  // Calibrated: raw units per gram
const long LOADCELL_OFFSET = -57432;       // Calibrated: raw value at 0g
const float THRESHOLD_DRINK_AMOUNT = 5.0; // Minimum weight change (grams) to register a drink event
float prevWeight = NAN; // NAN until first valid measurement

// Loadcell stability results (updated by background task)
// NOTE: Volatile variables for cross-task communication (ISR-safe for bool/int types).
// For float values, reading may not be atomic on all platforms. In this application,
// slightly stale or partially-read float values don't compromise safety (worst case:
// one incorrect stability reading). For critical applications, use FreeRTOS mutexes
// or atomic types. The loadcellStabilityTask writes, main loop reads.
volatile bool loadcellStable = false;        // Full stability (magnitude + deviation)
volatile bool loadcellMagnitudeOk = false;   // Quick magnitude check (on table)
volatile float loadcellLastWeight = 0.0f;
volatile bool fastPeriodActive = false;      // Track timer period for stability task

// Thresholds for loadcell-based stability
const int STABILITY_SAMPLES = 6;
const float LOADCELL_BOTTOM_PLATE_THRESHOLD = -54.0f; // grams: weight of bottom plate/contact to consider "on table"
const long LOADCELL_RAW_DEVIATION_THRESHOLD = 100; // raw units: max avg deviation to consider stable (~0.5g)

// BLE Objects
BLEService coasterService(COASTER_SERVICE_UUID);
BLECharacteristic consumptionChar(CONSUMPTION_CHAR_UUID);
BLECharacteristic commandChar(COMMAND_CHAR_UUID);
BLECharacteristic goalChar(GOAL_CHAR_UUID);

bool bleConnected = false;
volatile bool dataPendingToSend = false;
int32_t lastConsumptionValue = 0;  // Scaled by 100 (e.g., 1550 = 15.50g)

// Manufacturer data for advertising (Company ID 0xFFFF + 1-byte data pending flag)
uint8_t mfgData[3] = {0xFF, 0xFF, 0x00};

// Forward declaration for advertising update function
void updateAdvertisingData(bool hasPendingData);

bool glassChanged = false;
float hydrationGoal = 2000.0; // Default 2000ml
float totalConsumed = 2000.0; // Total consumed in grams (ml) TODO: Remove dummy value

// Hysteresis prevents "flickering" events
// TODO: Calibrate these thresholds experimentally
const int THRESHOLD_HIGH = 464;  // Above this = no glass (calibrated)
const int THRESHOLD_LOW = 460;   // Below this = glass present (calibrated)

// Duty Cycle Intervals
const int FAST_POLL_MS = 200;   // Check fast when interacting
const int SLOW_POLL_MS = 2000; // Check slow when idle (Deep Sleep Duty Cycle)

enum class EventType {
  HEARTBEAT_TICK,
  CHECK_FOR_DRINK,
  BLE_COMMAND_RECEIVED
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
  TEST
};

//DeviceState currentState = DeviceState::TEST;
DeviceState currentState;

QueueHandle_t eventQueue;
TimerHandle_t heartbeatTimer;
SemaphoreHandle_t ledMutex;

TaskHandle_t ledAlertTaskHandle;
TaskHandle_t ledFeedbackTaskHandle;
TaskHandle_t loadcellStabilityTaskHandle;

void heartbeatCallback(TimerHandle_t xTimer) {
  (void)xTimer;
  // Notify the loadcell stability task to perform the time-wise check.
  // That task will enqueue the actual HEARTBEAT_TICK event after completing checks.
  if (loadcellStabilityTaskHandle) {
    xTaskNotifyGive(loadcellStabilityTaskHandle);
  } else {
    // Fallback: if task not ready, still send heartbeat event
    Event evt;
    evt.type = EventType::HEARTBEAT_TICK;
    evt.timestamp = millis();
    evt.value = 0;
    xQueueSend(eventQueue, &evt, 0);
  }
}

// Wait for load cell to be ready with timeout, returns true if ready
bool waitForLoadcell(int timeoutMs = 200) {
  int waited = 0;
  while (!loadcell.is_ready() && waited < timeoutMs) {
    vTaskDelay(pdMS_TO_TICKS(10));
    waited += 10;
  }
  return loadcell.is_ready();
}

// Get raw ADC reading from load cell (for precision stability checks)
long getRawReading() {
  if (!waitForLoadcell()) {
    return LONG_MIN; // Error value
  }
  return loadcell.read();
}

float getWeight() {
  // Wait for load cell to be ready (with timeout)
  // HX711 needs time between conversions (10Hz = 100ms, 80Hz = 12.5ms)
  if (!waitForLoadcell()) {
    Serial.println("Load cell not ready");
    return NAN; // Return NaN to indicate error/unavailable
  }

  // Get weight in grams
  float weight = loadcell.get_units();
  return weight;
}

// Quick magnitude check - single reading to see if coaster is on table.
// Fast enough for high-frequency polling during MOVING states.
static bool performQuickMagnitudeCheck(float &outWeight) {
  float weight = getWeight();
  if (isnan(weight)) { outWeight = NAN; return false; }
  outWeight = weight;

  bool magOk = (weight >= LOADCELL_BOTTOM_PLATE_THRESHOLD);

  Serial.print("MagCheck: weight=");
  Serial.print(weight, 1);
  Serial.print("g magOk=");
  Serial.println(magOk ? "YES" : "NO");

  return magOk;
}

// Full stability check with deviation sampling. Takes multiple readings to
// detect water sloshing. Only call when magnitude already passes (on table).
// Should be executed from the dedicated FreeRTOS task (not from ISRs).
static bool performDeviationCheck(float &outWeight) {
  // First do magnitude check
  float weight = getWeight();
  if (isnan(weight)) { outWeight = NAN; return false; }
  outWeight = weight;

  bool magOk = (weight >= LOADCELL_BOTTOM_PLATE_THRESHOLD);
  if (!magOk) {
    Serial.print("Stability: weight=");
    Serial.print(weight, 1);
    Serial.println("g magOk=NO");
    return false;
  }

  // Magnitude OK - sample for deviation
  long rawLast = getRawReading();
  if (rawLast == LONG_MIN) { return false; }
  long rawSumDelta = 0;

  for (int i = 1; i < STABILITY_SAMPLES; i++) {
    long rawNow = getRawReading();
    if (rawNow == LONG_MIN) { return false; }
    rawSumDelta += labs(rawNow - rawLast);
    rawLast = rawNow;
  }

  long avgRawDelta = rawSumDelta / (STABILITY_SAMPLES - 1);
  bool deviationOk = (avgRawDelta <= LOADCELL_RAW_DEVIATION_THRESHOLD);

  Serial.print("Stability: delta=");
  Serial.print(avgRawDelta);
  Serial.print(" weight=");
  Serial.print(weight, 1);
  Serial.print("g devOk=");
  Serial.print(deviationOk ? "YES" : "NO");
  Serial.print(" stable=");
  Serial.println(deviationOk ? "YES" : "NO");

  return deviationOk;
}

// FreeRTOS task: waits for notification from heartbeat timer, performs the
// loadcell stability sampling, updates cached state, then enqueues HEARTBEAT_TICK.
// Check type depends on timer period:
// - Fast period: quick magnitude check only (coaster in air detection)
// - Slow period: full deviation check (water settling detection)
void loadcellStabilityTask(void *pvParameters) {
  (void)pvParameters;

  for (;;) {
    // Wait until heartbeat timer notifies us
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    float measuredWeight = -1.0f;
    bool magOk = false;
    bool stable = false;

    if (fastPeriodActive) {
      // Fast period: quick magnitude check only
      magOk = performQuickMagnitudeCheck(measuredWeight);
      // Can't be stable without full deviation check
      stable = false;
    } else {
      // Slow period: full stability check with deviation sampling
      stable = performDeviationCheck(measuredWeight);
      magOk = (measuredWeight >= LOADCELL_BOTTOM_PLATE_THRESHOLD);
    }

    // Update shared state
    loadcellLastWeight = measuredWeight;
    loadcellMagnitudeOk = magOk;
    loadcellStable = stable;

    // Enqueue heartbeat event for the main loop now that we have fresh loadcell data
    Event evt;
    evt.type = EventType::HEARTBEAT_TICK;
    evt.timestamp = millis();
    evt.value = (uint32_t)(measuredWeight >= 0 ? (uint32_t)measuredWeight : 0);
    xQueueSend(eventQueue, &evt, 0);
  }
}

bool isGlassPresent() {
  int val = analogRead(HALL_SENSOR_PIN);
  // In hysteresis zone, keep previous state. For the inital call, use middle of threshold as a binary state.
  static bool lastState = (val < (THRESHOLD_HIGH + THRESHOLD_LOW) / 2);

  if (val > THRESHOLD_HIGH) lastState = false;
  else if (val < THRESHOLD_LOW) lastState = true;

  return lastState;
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

  // Always restart advertising after disconnect, with appropriate data pending flag
  updateAdvertisingData(dataPendingToSend);
  if (dataPendingToSend) {
    Serial.println("Restarting advertising (data pending)");
  } else {
    Serial.println("Restarting advertising (available for commands)");
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

// Update advertising data with current data pending flag
// This allows the base station to know if the coaster has data to send
void updateAdvertisingData(bool hasPendingData) {
  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();

  // Update manufacturer data flag
  mfgData[2] = hasPendingData ? 0x01 : 0x00;

  // Re-add all advertising data
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(coasterService);
  Bluefruit.Advertising.addManufacturerData(mfgData, sizeof(mfgData));
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.start(0);
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
  commandChar.setMaxLen(4); // command code + optional data
  commandChar.begin();
  
  // Configure Goal Characteristic (Write only)
  goalChar.setProperties(CHR_PROPS_WRITE);
  goalChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  goalChar.setWriteCallback(goal_write_callback);
  goalChar.setFixedLen(4); // float value (ml)
  goalChar.begin();
  
  // Setup advertising
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(coasterService);
  Bluefruit.Advertising.addManufacturerData(mfgData, sizeof(mfgData));  // Data pending flag
  Bluefruit.Advertising.addName();
  
  // Set advertising interval (units of 0.625ms)
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 320);    // 100-200ms (balance power/discovery)
  Bluefruit.Advertising.setFastTimeout(10);       // Fast advertising for 10s, then slow (power saving)
  
  Serial.println("BLE initialized");
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
// value is scaled by 100 (e.g., 1550 = 15.50g)
bool sendConsumptionEvent(uint32_t timestamp, int32_t value) {
  if (!bleConnected) {
    // Not connected, mark data as pending and update advertising to signal base
    dataPendingToSend = true;
    lastConsumptionValue = value;
    updateAdvertisingData(true);  // Signal that we have data to send
    Serial.println("Not connected - advertising with data pending flag");
    return false;
  }

  // Pack data: timestamp(4 bytes) + value(4 bytes, signed)
  uint8_t data[8];
  memcpy(data, &timestamp, 4);
  memcpy(data + 4, &value, 4);

  if (consumptionChar.notify(data, sizeof(data))) {
    Serial.println("Consumption event sent via BLE");
    dataPendingToSend = false;
    updateAdvertisingData(false);  // Clear the data pending flag

    // Disconnect after sending to save power
    // The base station will disconnect, or we can request it
    return true;
  } else {
    Serial.println("Failed to send consumption event");
    dataPendingToSend = true;
    updateAdvertisingData(true);  // Ensure flag is set on failure
    return false;
  }
}

void ledAlertTask(void *pvParameters) {
  (void)pvParameters;

  // Use NeoPixel strip for alerts instead of built-in LED
  // Warm red color (tweak as desired)
  const uint8_t baseR = 255;
  const uint8_t baseG = 40;
  const uint8_t baseB = 20;

  const int blinkCount = 3;
  const int blinkPeriodMs = 600; // total period per blink (rise+fall)
  const int stepMs = 20; // time per step for smoothness
  const int steps = blinkPeriodMs / stepMs; // e.g., 600/20 = 30

  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for notification

    // Acquire mutex to prevent concurrent LED access
    if (xSemaphoreTake(ledMutex, portMAX_DELAY) != pdTRUE) continue;

    for (int b = 0; b < blinkCount; b++) {
      for (int s = 0; s < steps; s++) {
        // theta goes 0..2PI to produce rise then fall
        float theta = (2.0f * 3.14159265f * (float)s) / (float)(steps - 1);
        float br = (1.0f - cosf(theta)) * 0.5f; // 0..1..0
        uint8_t bri = (uint8_t)(br * 255.0f);

        // scale color by brightness
        uint8_t r = (uint8_t)((baseR * (uint16_t)bri) / 255);
        uint8_t g = (uint8_t)((baseG * (uint16_t)bri) / 255);
        uint8_t bl = (uint8_t)((baseB * (uint16_t)bri) / 255);

        // Set all pixels to the same warm red at computed brightness
        for (int i = 0; i < NUM_NEOPIXELS; i++) {
          strip.setPixelColor(i, strip.Color(r, g, bl));
        }
        strip.show();

        vTaskDelay(pdMS_TO_TICKS(stepMs));
      }
    }

    // Turn off after sequence
    strip.clear();
    strip.show();

    xSemaphoreGive(ledMutex);
  }
}

void ledFeedbackTask(void *pvParameters) {
  (void)pvParameters;

  // Use NeoPixel strip to display progress towards hydration goal
  const uint8_t baseR = 30;   // encouraging green
  const uint8_t baseG = 200;
  const uint8_t baseB = 80;
  const int fullDurationMs = 1100; // total time to animate to 100%
  const int stepMs = 20; // animation timestep

  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for notification

    // Acquire mutex to prevent concurrent LED access
    if (xSemaphoreTake(ledMutex, portMAX_DELAY) != pdTRUE) continue;

    float progress = totalConsumed / hydrationGoal;
    if (!isfinite(progress)) progress = 0.0f;
    progress = constrain(progress, 0.0f, 1.0f);

    // If nothing to show, clear and release mutex
    if (progress <= 0.0f) {
      strip.clear();
      strip.show();
      xSemaphoreGive(ledMutex);
      continue;
    }

    // Determine per-LED final target brightness (0..1)
    float target = progress * (float)NUM_NEOPIXELS;
    int full = (int)floor(target);
    float frac = target - (float)full; // partial brightness for next LED

    // Total animation duration proportional to progress
    int T = max((int)(fullDurationMs * progress), stepMs);
    int steps = max(1, T / stepMs);

    // Each LED's start is staggered across total duration
    float segTime = (float)T / (float)NUM_NEOPIXELS;

    for (int s = 0; s <= steps; s++) {
      int currentMs = s * stepMs;

      for (int i = 0; i < NUM_NEOPIXELS; i++) {
        // Map logical LED position to physical position
        // Position 0 starts at 7th LED (index 6) and rotates in reverse
        int physicalLed = (6 - i + NUM_NEOPIXELS) % NUM_NEOPIXELS;

        // desired final brightness for this LED (0..1)
        float tgtBr = 0.0f;
        if (i < full) tgtBr = 1.0f;
        else if (i == full) tgtBr = frac;
        else tgtBr = 0.0f;

        if (tgtBr <= 0.0f) {
          strip.setPixelColor(physicalLed, 0);
          continue;
        }

        // local progress for this LED from its staggered start
        float startMs = i * segTime;
        float local = (currentMs - startMs) / segTime;
        local = constrain(local, 0.0f, 1.0f);

        // smooth ease (0..1)
        float ease = (1.0f - cosf(local * 3.14159265f)) * 0.5f;

        // actual brightness scales to target brightness
        float bri = tgtBr * ease;

        uint8_t r = (uint8_t)((float)baseR * bri + 0.5f);
        uint8_t g = (uint8_t)((float)baseG * bri + 0.5f);
        uint8_t b = (uint8_t)((float)baseB * bri + 0.5f);

        strip.setPixelColor(physicalLed, strip.Color(r, g, b));
      }

      strip.show();
      vTaskDelay(pdMS_TO_TICKS(stepMs));
    }

    // Ensure final exact state matches progress
    for (int i = 0; i < NUM_NEOPIXELS; i++) {
      // Map logical LED position to physical position
      int physicalLed = (6 - i + NUM_NEOPIXELS) % NUM_NEOPIXELS;

      if (i < full) {
        strip.setPixelColor(physicalLed, strip.Color(baseR, baseG, baseB));
      } else if (i == full && frac > 0.0f) {
        uint8_t r = (uint8_t)((float)baseR * frac + 0.5f);
        uint8_t g = (uint8_t)((float)baseG * frac + 0.5f);
        uint8_t b = (uint8_t)((float)baseB * frac + 0.5f);
        strip.setPixelColor(physicalLed, strip.Color(r, g, b));
      } else {
        strip.setPixelColor(physicalLed, 0);
      }
    }
    strip.show();

    // Hold the display for 2 seconds
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Smooth fade out over 600ms
    const int fadeOutMs = 600;
    const int fadeSteps = fadeOutMs / stepMs;
    for (int s = 0; s <= fadeSteps; s++) {
      float fadeProgress = (float)s / (float)fadeSteps; // 0..1
      float brightness = 1.0f - fadeProgress; // 1..0

      for (int i = 0; i < NUM_NEOPIXELS; i++) {
        int physicalLed = (6 - i + NUM_NEOPIXELS) % NUM_NEOPIXELS;

        if (i < full) {
          uint8_t r = (uint8_t)((float)baseR * brightness + 0.5f);
          uint8_t g = (uint8_t)((float)baseG * brightness + 0.5f);
          uint8_t b = (uint8_t)((float)baseB * brightness + 0.5f);
          strip.setPixelColor(physicalLed, strip.Color(r, g, b));
        } else if (i == full && frac > 0.0f) {
          uint8_t r = (uint8_t)((float)baseR * frac * brightness + 0.5f);
          uint8_t g = (uint8_t)((float)baseG * frac * brightness + 0.5f);
          uint8_t b = (uint8_t)((float)baseB * frac * brightness + 0.5f);
          strip.setPixelColor(physicalLed, strip.Color(r, g, b));
        } else {
          strip.setPixelColor(physicalLed, 0);
        }
      }
      strip.show();
      vTaskDelay(pdMS_TO_TICKS(stepMs));
    }

    // Ensure fully off
    strip.clear();
    strip.show();

    xSemaphoreGive(ledMutex);
  }
}

// Determine initial state based on glass presence and stability
void determineInitialState() {
  bool glassPresent = isGlassPresent();
  // Use loadcell-based stability check (cached by background task)
  if (glassPresent && loadcellStable) {
    currentState = DeviceState::IDLE_WITH_GLASS;
  } else if (!glassPresent && loadcellStable) {
    currentState = DeviceState::IDLE_NO_GLASS;
  } else if (glassPresent && !loadcellStable) {
    currentState = DeviceState::MOVING_WITH_GLASS;
  } else {
    currentState = DeviceState::MOVING_NO_GLASS;
  }

  Serial.print("Initial State: ");
  switch (currentState) {
    case DeviceState::IDLE_WITH_GLASS:
      Serial.println("IDLE_WITH_GLASS");
      break;
    case DeviceState::IDLE_NO_GLASS:
      Serial.println("IDLE_NO_GLASS");
      break;
    case DeviceState::MOVING_WITH_GLASS:
      Serial.println("MOVING_WITH_GLASS");
      break;
    case DeviceState::MOVING_NO_GLASS:
      Serial.println("MOVING_NO_GLASS");
      break;
    default:
      Serial.println("UNKNOWN");
      break;
  }
}


void setup() {
  Serial.begin(115200);
  //while (!Serial) { delay(10); } // TODO: Remove for production
  //delay(500);

  // Configure battery charging current to 100mA
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);

  // Initialize SPI Flash and enter deep power-down mode to save power
  flashTransport.begin();
  flashTransport.runCommand(0xB9);  // enter deep power-down mode
  delayMicroseconds(5);             // tDP=3uS
  flashTransport.end();

  // Create a Queue that can hold 10 events (must exist before ISR runs)
  eventQueue = xQueueCreate(10, sizeof(Event));

  // Create mutex for LED strip access
  ledMutex = xSemaphoreCreateMutex();

  // Initialize load cell
  loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  digitalWrite(LOADCELL_SCK_PIN, HIGH);
  delayMicroseconds(100); // Short pulse
  digitalWrite(LOADCELL_SCK_PIN, LOW);
  while (!loadcell.is_ready()) {
    delay(10);
  }
  loadcell.set_scale(LOADCELL_SCALE);   // Apply calibration scale factor
  loadcell.set_offset(LOADCELL_OFFSET); // Apply calibration zero offset
  Serial.println("Load cell initialized");
  // Initialize NeoPixel strip
  strip.begin();
  strip.setBrightness(255);
  strip.show();
  

  // Initialize BLE
  setupBLE();
  startBLEAdvertising();

  // Start with SLOW polling (Hibernation)
  // Create tasks that will be notified by the heartbeat timer.
  xTaskCreate(ledAlertTask, "LED Alert Task", 256, NULL, 1, &ledAlertTaskHandle);
  xTaskCreate(ledFeedbackTask, "LED Feedback Task", 256, NULL, 1, &ledFeedbackTaskHandle);
  xTaskCreate(loadcellStabilityTask, "Loadcell Stability", 512, NULL, 2, &loadcellStabilityTaskHandle);

  // Start with SLOW polling (Hibernation). The timer now notifies the loadcell task
  // which will enqueue the HEARTBEAT_TICK after sampling.
  heartbeatTimer = xTimerCreate("Heartbeat", pdMS_TO_TICKS(SLOW_POLL_MS), pdTRUE, 0, heartbeatCallback);
  xTimerStart(heartbeatTimer, 0);

  // Disable when testing, TODO: Enable when ready
  determineInitialState(); 

  Serial.println("Coaster Clip Main Setup");
  delay(500);
}

void loop() {
  Event evt;

  if (xQueueReceive(eventQueue, &evt, portMAX_DELAY) == pdTRUE) {
    bool eventConsumed = false;
    bool glassPresent = false;
    Serial.print("Event received: ");
    switch (evt.type) {
      case EventType::BLE_COMMAND_RECEIVED:
        Serial.println("BLE_COMMAND_RECEIVED");
        break;
      case EventType::HEARTBEAT_TICK:
        Serial.println("HEARTBEAT_TICK");
        break;
      case EventType::CHECK_FOR_DRINK:
        Serial.println("CHECK_FOR_DRINK");
        break;
      default:
        Serial.println("UNKNOWN");
        break;
    }
    if (evt.type == EventType::BLE_COMMAND_RECEIVED) {
      uint8_t command = (uint8_t)evt.value;
      if (command == 0x01) { // LED Alert Request
        // Notify LED Alert Task
        xTaskNotifyGive(ledAlertTaskHandle);
        eventConsumed = true;
      }
    }

    if (evt.type == EventType::HEARTBEAT_TICK) {
      // On heartbeat: check if we have pending data to send
      if (bleConnected && dataPendingToSend) {
        sendConsumptionEvent(millis(), lastConsumptionValue);
      }
      glassPresent = isGlassPresent();
    }

    if (!eventConsumed) {
    switch (currentState) {
      case DeviceState::IDLE_WITH_GLASS:
        // Monitor for drinking activity
        Serial.println("State: IDLE_WITH_GLASS");

        if (evt.type == EventType::CHECK_FOR_DRINK) {
          float weight = getWeight();
          if (!isnan(weight)) {
            // Initialize prevWeight on first valid reading or after glass change
            if (isnan(prevWeight) || glassChanged) {
              prevWeight = weight;
              glassChanged = false;
              Serial.print("Baseline weight set to ");
              Serial.print(prevWeight);
              Serial.println(" g");
            } else {
              float drinkAmount = prevWeight - weight;
              if (drinkAmount > THRESHOLD_DRINK_AMOUNT) {
                totalConsumed += drinkAmount;
                Serial.print("Drink detected: ");
                Serial.print(drinkAmount);
                Serial.println(" g");

                prevWeight = weight;

                // Send consumption event via BLE (scaled by 100 for decimal precision)
                int32_t scaledAmount = (int32_t)(drinkAmount * 100.0f + 0.5f);
                sendConsumptionEvent(millis(), scaledAmount);
              }
            }
          }
        }

        // On heartbeat: check for instability (movement) or glass removal
        else if (evt.type == EventType::HEARTBEAT_TICK) {
          // Detect movement via loadcell instability
          if (!loadcellStable) {
            currentState = DeviceState::MOVING_WITH_GLASS;
            Serial.println("Transition to MOVING_WITH_GLASS");
            // Switch to faster heartbeat tick for active state
            xTimerChangePeriod(heartbeatTimer, pdMS_TO_TICKS(FAST_POLL_MS), 0);
            fastPeriodActive = true;

            // Give feedback on progress towards hydration goal when glass is picked up
            xTaskNotifyGive(ledFeedbackTaskHandle);
          } else if (!glassPresent) {
            currentState = DeviceState::IDLE_NO_GLASS;
            Serial.println("Transition to IDLE_NO_GLASS");

            glassChanged = true;
          } else {
            // Use cached weight from stability task
            float weight = loadcellLastWeight;
            if (!isnan(weight)) {
              prevWeight = weight;
            }
          }
        }

        break;

      case DeviceState::IDLE_NO_GLASS:
        // Monitor for glass placement
        Serial.println("State: IDLE_NO_GLASS");

        // On heartbeat: check for instability (movement) or glass placement
        if (evt.type == EventType::HEARTBEAT_TICK) {
          if (!loadcellStable) {
            currentState = DeviceState::MOVING_NO_GLASS;
            Serial.println("Transition to MOVING_NO_GLASS");
            // Switch to faster heartbeat tick for active state
            xTimerChangePeriod(heartbeatTimer, pdMS_TO_TICKS(FAST_POLL_MS), 0);
            fastPeriodActive = true;
          } else if (glassPresent) {
            currentState = DeviceState::IDLE_WITH_GLASS;
            Serial.println("Transition to IDLE_WITH_GLASS");

            glassChanged = true;
          }
        }
        break;
      
      case DeviceState::MOVING_WITH_GLASS:
        // Handle movement with glass
        // Two-phase approach:
        // - Fast period: quick magnitude check (is coaster on table?)
        // - Slow period: full deviation check (is water settled?)
        Serial.println("State: MOVING_WITH_GLASS");

        if (evt.type == EventType::HEARTBEAT_TICK) {
          if (fastPeriodActive) {
            // Fast period: only magnitude check was performed
            if (loadcellMagnitudeOk) {
              // Coaster landed on table - switch to slow period for deviation check
              Serial.println("Magnitude OK - switching to slow period for deviation check");
              xTimerChangePeriod(heartbeatTimer, pdMS_TO_TICKS(SLOW_POLL_MS), 0);
              fastPeriodActive = false;
              // Stay in MOVING_WITH_GLASS until deviation check passes
            } else if (!glassPresent) {
              currentState = DeviceState::MOVING_NO_GLASS;
              Serial.println("Transition to MOVING_NO_GLASS");
              glassChanged = true;
            }
            // else: still in air, stay in fast period
          } else {
            // Slow period: full stability check was performed
            if (loadcellStable && glassPresent) {
              currentState = DeviceState::IDLE_WITH_GLASS;
              Serial.println("Transition to IDLE_WITH_GLASS");
              // Already on slow period

              // Queue CHECK_FOR_DRINK event
              Event e;
              e.type = EventType::CHECK_FOR_DRINK;
              e.timestamp = millis();
              e.value = 0;
              xQueueSend(eventQueue, &e, 0);
            } else if (loadcellStable && !glassPresent) {
              currentState = DeviceState::IDLE_NO_GLASS;
              Serial.println("Transition to IDLE_NO_GLASS");
              // Already on slow period
              glassChanged = true;
            } else if (!loadcellMagnitudeOk) {
              // Coaster picked up again - back to fast period
              Serial.println("Magnitude failed - switching back to fast period");
              xTimerChangePeriod(heartbeatTimer, pdMS_TO_TICKS(FAST_POLL_MS), 0);
              fastPeriodActive = true;
              if (!glassPresent) {
                currentState = DeviceState::MOVING_NO_GLASS;
                Serial.println("Transition to MOVING_NO_GLASS");
                glassChanged = true;
              }
            }
            // else: deviation check failed but magnitude OK (water sloshing) - stay in slow period, retry
          }
        }
        break;
      
      case DeviceState::MOVING_NO_GLASS:
        // Handle movement without glass
        // Two-phase approach (same as MOVING_WITH_GLASS)
        Serial.println("State: MOVING_NO_GLASS");

        if (evt.type == EventType::HEARTBEAT_TICK) {
          if (fastPeriodActive) {
            // Fast period: only magnitude check was performed
            if (loadcellMagnitudeOk) {
              // Coaster landed on table - switch to slow period for deviation check
              Serial.println("Magnitude OK - switching to slow period for deviation check");
              xTimerChangePeriod(heartbeatTimer, pdMS_TO_TICKS(SLOW_POLL_MS), 0);
              fastPeriodActive = false;
            } else if (glassPresent) {
              currentState = DeviceState::MOVING_WITH_GLASS;
              Serial.println("Transition to MOVING_WITH_GLASS");
              glassChanged = true;
            }
            // else: still in air, stay in fast period
          } else {
            // Slow period: full stability check was performed
            if (loadcellStable && glassPresent) {
              currentState = DeviceState::IDLE_WITH_GLASS;
              Serial.println("Transition to IDLE_WITH_GLASS");
              // Already on slow period
              glassChanged = true;
            } else if (loadcellStable && !glassPresent) {
              currentState = DeviceState::IDLE_NO_GLASS;
              Serial.println("Transition to IDLE_NO_GLASS");
              // Already on slow period
            } else if (!loadcellMagnitudeOk) {
              // Coaster picked up again - back to fast period
              Serial.println("Magnitude failed - switching back to fast period");
              xTimerChangePeriod(heartbeatTimer, pdMS_TO_TICKS(FAST_POLL_MS), 0);
              fastPeriodActive = true;
              if (glassPresent) {
                currentState = DeviceState::MOVING_WITH_GLASS;
                Serial.println("Transition to MOVING_WITH_GLASS");
                glassChanged = true;
              }
            }
            // else: deviation check failed but magnitude OK - stay in slow period, retry
          }
        }
        break;
      
      case DeviceState::TEST:
        // Testing state
        Serial.println("State: TEST");
        
        if (evt.type == EventType::HEARTBEAT_TICK) {          
          // Loadcell cached stability
          bool isAnyStable = loadcellStable;
          float weight = loadcellLastWeight;
          
          // Test hall sensor
          int hallValue = analogRead(HALL_SENSOR_PIN);
          bool glassPresent = isGlassPresent();
          
          // Print all values to serial monitor
          Serial.print(" | Loadcell Stable: ");
          Serial.print(isAnyStable ? "YES" : "NO");
          Serial.print(" | Loadcell Weight: ");
          Serial.print(weight, 2);
          Serial.println(" g");
          
          Serial.print("Hall Sensor: ");
          Serial.print(hallValue);
          Serial.print(" | Glass Present: ");
          Serial.println(glassPresent ? "YES" : "NO");
          /*
          // BLE Test Status
          Serial.print("BLE Connected: ");
          Serial.print(bleConnected ? "YES" : "NO");
          Serial.print(" | Advertising: ");
          Serial.print(Bluefruit.Advertising.isRunning() ? "YES" : "NO");
          Serial.print(" | Data Pending: ");
          Serial.println(dataPendingToSend ? "YES" : "NO");
          
          Serial.println("---");*/
          
          // BLE Test: If connected, send test consumption data
          if (bleConnected && dataPendingToSend) {
            Serial.println("BLE TEST: Sending pending consumption data...");
            sendConsumptionEvent(millis(), lastConsumptionValue);
          }
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
              int32_t weightValue = (int32_t)(testWeight * 100.0f + 0.5f);
              sendConsumptionEvent(millis(), weightValue);
            } else {
              // Send dummy data for testing
              sendConsumptionEvent(millis(), 15000); // 150.00 grams (scaled)
            }
          } else if (cmd == 'a' || cmd == 'A') {
            Serial.println("\n*** BLE TEST: Starting advertising ***");
            startBLEAdvertising();
          } else if (cmd == 's' || cmd == 'S') {
            Serial.println("\n*** BLE TEST: Stopping advertising ***");
            stopBLEAdvertising();
          } else if (cmd == 'f' || cmd == 'F') {
            Serial.println("\n*** LED TEST: Triggering feedback animation ***");
            xTaskNotifyGive(ledFeedbackTaskHandle);
          }
        }
        break;
    }
  }
  }
  yield(); // Allow other tasks to run
} 