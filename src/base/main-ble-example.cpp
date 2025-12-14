#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>

// BLE Service and Characteristics UUIDs (must match coaster)
#define COASTER_SERVICE_UUID        "19B10010-E8F2-537E-4F6C-D104768A1214"
#define CONSUMPTION_CHAR_UUID       "19B10011-E8F2-537E-4F6C-D104768A1214"  // Notify consumption events
#define COMMAND_CHAR_UUID           "19B10012-E8F2-537E-4F6C-D104768A1214"  // Write commands to coaster
#define GOAL_CHAR_UUID              "19B10013-E8F2-537E-4F6C-D104768A1214"  // Write hydration goal

// BLE objects
static BLEAdvertisedDevice* myDevice = nullptr;
static BLEClient* pClient = nullptr;
static BLERemoteCharacteristic* pConsumptionChar = nullptr;
static BLERemoteCharacteristic* pCommandChar = nullptr;
static BLERemoteCharacteristic* pGoalChar = nullptr;

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;

// Consumption data tracking
struct ConsumptionData {
  uint32_t timestamp;
  uint32_t value;
};

ConsumptionData latestConsumption = {0, 0};
bool newConsumptionReceived = false;

// Notification callback for consumption data
static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                           uint8_t* pData, size_t length, bool isNotify) {
  if (length == 8) {
    // Parse timestamp and value
    memcpy(&latestConsumption.timestamp, pData, 4);
    memcpy(&latestConsumption.value, pData + 4, 4);
    
    newConsumptionReceived = true;
    
    Serial.println("========================================");
    Serial.println("CONSUMPTION EVENT RECEIVED!");
    Serial.print("  Timestamp: ");
    Serial.println(latestConsumption.timestamp);
    Serial.print("  Weight: ");
    Serial.print(latestConsumption.value / 100.0, 2);
    Serial.println(" grams");
    Serial.println("========================================");
  }
}

// Connection callbacks
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println("ESP32: Connected to coaster");
    connected = true;
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("ESP32: Disconnected from coaster");
    doScan = true; // Start scanning again
  }
};

// Connect to the BLE Server
bool connectToServer() {
  Serial.print("Forming connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());
  
  pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remote BLE Server
  pClient->connect(myDevice);
  Serial.println(" - Connected to server");
  pClient->setMTU(517); // Set client MTU

  // Obtain a reference to the service we are after in the remote BLE server
  BLERemoteService* pRemoteService = pClient->getService(COASTER_SERVICE_UUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(COASTER_SERVICE_UUID);
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found coaster service");

  // Obtain references to the characteristics
  pConsumptionChar = pRemoteService->getCharacteristic(CONSUMPTION_CHAR_UUID);
  pCommandChar = pRemoteService->getCharacteristic(COMMAND_CHAR_UUID);
  pGoalChar = pRemoteService->getCharacteristic(GOAL_CHAR_UUID);

  if (pConsumptionChar == nullptr || pCommandChar == nullptr || pGoalChar == nullptr) {
    Serial.println("Failed to find one or more characteristics");
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found all characteristics");

  // Register for notifications on consumption characteristic
  if (pConsumptionChar->canNotify()) {
    pConsumptionChar->registerForNotify(notifyCallback);
    Serial.println(" - Registered for consumption notifications");
  }

  connected = true;
  return true;
}

// Scan for BLE servers and find the coaster
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // Check if this is the coaster
    if (advertisedDevice.haveServiceUUID() && 
        advertisedDevice.isAdvertisingService(BLEUUID(COASTER_SERVICE_UUID))) {
      
      Serial.println("*** Found CoasterClip device! ***");
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = false;
    }
  }
};

// Send a command to the coaster
void sendCommand(uint8_t cmd) {
  if (connected && pCommandChar != nullptr) {
    uint8_t data[4] = {cmd, 0, 0, 0};
    pCommandChar->writeValue(data, 4, false);
    Serial.print("Command sent: 0x");
    Serial.println(cmd, HEX);
  } else {
    Serial.println("Not connected - cannot send command");
  }
}

// Send hydration goal to the coaster
void sendHydrationGoal(float goalML) {
  if (connected && pGoalChar != nullptr) {
    uint8_t data[4];
    memcpy(data, &goalML, 4);
    pGoalChar->writeValue(data, 4, false);
    Serial.print("Hydration goal sent: ");
    Serial.print(goalML);
    Serial.println(" ml");
  } else {
    Serial.println("Not connected - cannot send goal");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 BLE Base Station Test ===");
  Serial.println("Starting BLE Client...");

  BLEDevice::init("ESP32-Base");

  // Start scanning for BLE devices
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
  
  doScan = true;
  
  Serial.println("Scanning for CoasterClip...");
  Serial.println("\nTest Commands:");
  Serial.println("  '1' - Send LED Alert Command (0x01)");
  Serial.println("  '2' - Send Status Query Command (0x02)");
  Serial.println("  '3' - Send Hydration Goal (2500ml)");
  Serial.println("  'd' - Disconnect from coaster");
  Serial.println("  's' - Start scanning");
}

void loop() {
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("Successfully connected to coaster!");
      
      // Test: Send a hydration goal after connection
      delay(1000);
      sendHydrationGoal(2000.0);
    } else {
      Serial.println("Failed to connect to coaster");
    }
    doConnect = false;
  }

  // Check for serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch(cmd) {
      case '1':
        Serial.println("\nSending LED Alert Command...");
        sendCommand(0x01);
        break;
      
      case '2':
        Serial.println("\nSending Status Query Command...");
        sendCommand(0x02);
        break;
      
      case '3':
        Serial.println("\nSending Hydration Goal...");
        sendHydrationGoal(2500.0);
        break;
      
      case 'd':
        if (connected) {
          Serial.println("\nDisconnecting...");
          pClient->disconnect();
        }
        break;
      
      case 's':
        if (!connected) {
          Serial.println("\nStarting scan...");
          BLEDevice::getScan()->start(5, false);
          doScan = true;
        }
        break;
    }
  }

  // If we should be scanning, check if scan has completed
  if (doScan) {
    // Scanning is already happening in background
    delay(1000);
  }

  // Display consumption data if received
  if (newConsumptionReceived) {
    newConsumptionReceived = false;
    
    // After receiving data, we can disconnect to save power on coaster
    delay(500);
    Serial.println("Data received - disconnecting to save coaster power...");
    if (connected) {
      pClient->disconnect();
    }
  }

  delay(100);
}
