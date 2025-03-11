// #include <Arduino.h>
// #include <BLEDevice.h>
// #include <BLEUtils.h>
// #include <BLEScan.h>
// #include <BLEAdvertisedDevice.h>

// // Pin definitions for X27 stepper motor
// #define PIN_1 9    // Orange wire
// #define PIN_2 10   // Yellow wire
// #define PIN_3 11   // Pink wire
// #define PIN_4 12   // Blue wire

// // LED Pin
// #define LED_PIN 7 

// // X27 stepper motor has 315 steps per revolution
// const int STEPS_PER_REVOLUTION = 315;
// const int DELAY_BETWEEN_STEPS = 3; // milliseconds
// int currentStep = 0;

// static BLEUUID serviceUUID("04faf1c3-e3d5-49c0-9ead-e628bc35a133");
// static BLEUUID charUUID("90705d32-c022-4a47-bda4-8ff642549198");

// static BLERemoteCharacteristic* pRemoteCharacteristic;
// static BLEAdvertisedDevice* myDevice;
// static boolean connected = false;
// static boolean doConnect = false;
// static boolean doScan = false;

// // Full step sequence
// const byte stepSequence[4][4] = {
//   {1, 0, 0, 0},
//   {0, 1, 0, 0},
//   {0, 0, 1, 0},
//   {0, 0, 0, 1}
// };

// void setup() {
//   Serial.begin(115200);
//   Serial.println("Starting BLE Display Controller...");

//   // Initialize motor control pins as outputs
//   pinMode(PIN_1, OUTPUT);
//   pinMode(PIN_2, OUTPUT);
//   pinMode(PIN_3, OUTPUT);
//   pinMode(PIN_4, OUTPUT);

//   // Initialize LED pin
//   pinMode(LED_PIN, OUTPUT);
//   digitalWrite(LED_PIN, LOW);

//   BLEDevice::init("");

//   BLEScan* pBLEScan = BLEDevice::getScan();
//   pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
//   pBLEScan->setActiveScan(true);
//   pBLEScan->start(5, false);
// }

// // Distance tracking
// unsigned long distanceStartTime = 0;
// bool isTrackingDistance = false;

// // Function to set pins according to sequence
// void setPins(int step) {
//   digitalWrite(PIN_1, stepSequence[step][0]);
//   digitalWrite(PIN_2, stepSequence[step][1]);
//   digitalWrite(PIN_3, stepSequence[step][2]);
//   digitalWrite(PIN_4, stepSequence[step][3]);
// }

// // Function to turn off all coils
// void releaseMotor() {
//   digitalWrite(PIN_1, LOW);
//   digitalWrite(PIN_2, LOW);
//   digitalWrite(PIN_3, LOW);
//   digitalWrite(PIN_4, LOW);
// }

// // Function to rotate by specified angle in degrees
// void rotateByDegrees(float degrees) {
//   // Calculate steps needed for the angle
//   int steps = (degrees * STEPS_PER_REVOLUTION) / 360;
  
//   Serial.print("Rotating ");
//   Serial.print(degrees);
//   Serial.print(" degrees (");
//   Serial.print(steps);
//   Serial.println(" steps)");
  
//   // Perform the steps
//   for (int i = 0; i < steps; i++) {
//     int stepIndex = i % 4;
//     setPins(stepIndex);
//     delay(DELAY_BETWEEN_STEPS);
//   }
  
//   // Release motor to prevent heating
//   releaseMotor();
//   currentStep += degrees;
// }

// // Function to blink LED
// void blinkLED(int duration) {
//   for (int i = 0; i < (duration / 500); i++) {
//     digitalWrite(LED_PIN, HIGH);
//     delay(250);
//     digitalWrite(LED_PIN, LOW);
//     delay(250);
//   }
// }

// // BLE Data Processing
// void processBLEData(String data) {
//     Serial.print("Received BLE Data: ");
//     Serial.println(data);

//     // Check for frequency match signal
//     if (data == "FREQ_MATCH") {
//         Serial.println("Peak frequency detected. Moving stepper motor by 1 degree.");
//         rotateByDegrees(1);
//     }

//     // Check for proximity alert signal
//     if (data == "PROXIMITY_ALERT") {
//         Serial.println("Proximity alert received. Blinking LED for 5 seconds.");
//         blinkLED(5000);
//     }

//     // Check if stepper motor has reached 180 degrees
//     if (currentStep >= 180) {
//         Serial.println("Stepper motor reached 180 degrees. Blinking LED for 10 seconds and resetting motor.");
//         blinkLED(10000);
//         currentStep = 0;
//     }
// }

// // BLE Notification Callback
// static void notifyCallback(
//   BLERemoteCharacteristic* pBLERemoteCharacteristic,
//   uint8_t* pData,
//   size_t length,
//   bool isNotify) {

//   String receivedString = String((char*)pData).substring(0, length);
//   processBLEData(receivedString);
// }

// // BLE Connection Callback
// std::string serverDeviceName = ""; 
// class MyClientCallback : public BLEClientCallbacks {
//   void onConnect(BLEClient* pclient) {
//     Serial.println("✅ Connected to BLE Server.");
//     Serial.println(serverDeviceName.c_str());
//   }
//   void onDisconnect(BLEClient* pclient) {
//     connected = false;
//     Serial.println("❌ Disconnected from BLE Server.");
//   }
// };

// // Connect to BLE Server
// bool connectToServer() {
//   Serial.print("Connecting to ");
//   Serial.println(myDevice->getAddress().toString().c_str());

//   BLEClient* pClient = BLEDevice::createClient();
//   pClient->setClientCallbacks(new MyClientCallback());

//   pClient->connect(myDevice);
//   Serial.println("Connected to BLE server.");

//   BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
//   if (pRemoteService == nullptr) {
//     Serial.println("Failed to find service UUID.");
//     pClient->disconnect();
//     return false;
//   }

//   pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
//   if (pRemoteCharacteristic == nullptr) {
//     Serial.println("Failed to find characteristic UUID.");
//     pClient->disconnect();
//     return false;
//   }

//   if (pRemoteCharacteristic->canNotify()) {
//     pRemoteCharacteristic->registerForNotify(notifyCallback);
//   }

//   connected = true;
//   return true;
// }

// // BLE Scanning
// class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
//   void onResult(BLEAdvertisedDevice advertisedDevice) {
//     if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
//       BLEDevice::getScan()->stop();
//       myDevice = new BLEAdvertisedDevice(advertisedDevice);
//       doConnect = true;
//       doScan = true;
//     }
//   }
// };

// void loop() {
//   if (doConnect) {
//     if (connectToServer()) {
//       Serial.println("Connected to BLE Server.");
//     } else {
//       Serial.println("Failed to connect.");
//     }
//     doConnect = false;
//   }

//   if (connected) {
//     String newValue = "Time since boot: " + String(millis()/1000);
//     Serial.println("Setting new characteristic value to \"" + newValue  + "\"");
//     pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
//   } else if (doScan) {
//     BLEDevice::getScan()->start(0);
//   }

//   delay(1000);
// }

#include <Arduino.h>
#include <AccelStepper.h>

#define MOTOR_PIN_1 5
#define MOTOR_PIN_2 6
#define LED_PIN 3


AccelStepper stepper(AccelStepper::FULL2WIRE, MOTOR_PIN_1, MOTOR_PIN_2);
bool direction = false;

void setup() {
    Serial.begin(115200);
    
    stepper.setMaxSpeed(500);
    stepper.setAcceleration(300);

    pinMode (LED_PIN, OUTPUT);
    
    Serial.println("Stepper Motor Test Started...");
}

void blinkLED(int duration) {
    Serial.println("LED");
    digitalWrite(LED_PIN, HIGH);
    delay(duration);
    digitalWrite(LED_PIN, LOW);
}

void loop() {
    blinkLED(1000);
    if (direction) {
        stepper.moveTo(0);  // Move back to 0
        Serial.println("Moving to 0");
    } else {
        stepper.moveTo(540);  // Move forward to 540
        Serial.println("Moving to 540");
    }
    direction = !direction;  // Toggle direction
    
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }    

    delay(500);
}


