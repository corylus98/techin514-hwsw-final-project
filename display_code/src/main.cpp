#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <AccelStepper.h>

#define MOTOR_PIN_1 5
#define MOTOR_PIN_2 6
#define LED_PIN 3

#define STEP_20_DEGREES 60  // Steps required to rotate 90 degrees
#define STEP_180_DEGREES 540 // Steps required to rotate 180 degrees
#define ORIGIN 0

AccelStepper stepper(AccelStepper::FULL2WIRE, MOTOR_PIN_1, MOTOR_PIN_2);
bool hasMoved180 = false;

static BLEUUID serviceUUID("04faf1c3-e3d5-49c0-9ead-e628bc35a133");
static BLEUUID charUUID("90705d32-c022-4a47-bda4-8ff642549198");

static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;
static boolean connected = false;
static boolean doConnect = false;
static boolean doScan = false;

// BLE Scanning
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    }
  }
};

// Distance tracking
unsigned long distanceStartTime = 0;
bool isTrackingDistance = false;

// Function to blink LED
void blinkLED(int duration) {
    Serial.println("LED");
    digitalWrite(LED_PIN, HIGH);
    delay(duration);
    digitalWrite(LED_PIN, LOW);
}

// Function to move the stepper motor
void moveStepper(int targetPosition) {
    Serial.print("Moving stepper to position: ");
    Serial.println(targetPosition);

    stepper.moveTo(targetPosition);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }

    // Check if motor reached 180 degrees
    if (targetPosition == STEP_180_DEGREES) {
        hasMoved180 = true;
    }
}

// BLE Data Processing
void processBLEData(String data) {
    Serial.print("Received BLE Data: ");
    Serial.println(data);

    // Check for frequency match signal
    if (data == "FREQ_MATCH") {
        moveStepper(STEP_20_DEGREES);
        Serial.println("Peak frequency detected. Moving stepper motor.");
    }

    // Check for proximity alert signal
    if (data == "PROXIMITY_ALERT") {
        blinkLED(5000);
        Serial.println("Proximity alert received. Blinking LED for 5 seconds.");
    }
}

// BLE Notification Callback
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    String receivedString = String((char*)pData).substring(0, length);
    Serial.print("Raw BLE Data: ");
    Serial.println(receivedString);
    
    processBLEData(receivedString);
}

// BLE Connection Callback
std::string serverDeviceName = ""; 
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    serverDeviceName = myDevice->getName();
    Serial.println("✅ Connected to BLE Server.");
    Serial.println(serverDeviceName.c_str());
  }
  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("❌ Disconnected from BLE Server.");
  }
};

// Connect to BLE Server
bool connectToServer() {
  Serial.print("Connecting to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient* pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  pClient->connect(myDevice);
  Serial.println("Connected to BLE server.");
  pClient->setMTU(517);

  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.println("Failed to find service UUID.");
    pClient->disconnect();
    return false;
  }

  serverDeviceName = myDevice->getName();

  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("Failed to find characteristic UUID.");
    pClient->disconnect();
    return false;
  }

  if(pRemoteCharacteristic->canRead()) {
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  }

  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
  }

  connected = true;
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE Display Controller...");

  BLEDevice::init("");

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);

  stepper.setMaxSpeed(500);
  stepper.setAcceleration(300);

  // Initialize LED pin
  pinMode (LED_PIN, OUTPUT);

  stepper.setCurrentPosition(ORIGIN);
  Serial.println("Stepper initialized at origin.");
}

void loop() {
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("Connected to BLE Server.");
    } else {
      Serial.println("Failed to connect.");
    }
    doConnect = false;
  }

  if (connected) {
    String newValue = "Time since boot: " + String(millis()/1000);
    Serial.println("Setting new characteristic value to \"" + newValue  + "\"");
    pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
  } else if (doScan) {
    BLEDevice::getScan()->start(0);
  }

  if (hasMoved180) {
      Serial.println("Stepper moved 180 degrees. Returning to origin...");
      moveStepper(ORIGIN);
      hasMoved180 = false;
  }

  delay(1000);
}