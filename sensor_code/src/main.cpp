#include <Arduino.h>
#include <Wire.h>
#include <driver/i2s.h>
#include "arduinoFFT.h" 
#include <U8g2lib.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <stdlib.h>

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
unsigned long previousMillis = 0;
const long interval = 1000;
unsigned long lastTimeBelowThreshold = 0;
bool trackingProximity = false;

#define I2S_WS   9  // LRCLK (Word Select)
#define I2S_DIN  8  // Data In (DOUT from mic)
#define I2S_BCLK 20  // Bit Clock
#define TIME_THRESHOLD 120000
#define TARGET_FREQUENCY 62.5
#define DISTANCE_THRESHOLD 100
const int trigPin = 2;
const int echoPin = 3;

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02                                 
#define SCL_PLOT 0x03                                                       
                                                     
#define BUFFER_SIZE 512  // Buffer for storing audio samples

#define SAMPLES 512  // Number of samples for FFT, must be a power of 2
#define SAMPLING_FREQUENCY 16000  // Adjust based on your microphone's sampling rate

double vReal[SAMPLES];  // Array to store the real part of the FFT (audio data)
double vImag[SAMPLES];  // Array to store the imaginary part (set to 0)
float duration, distance;

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);
double FindPeakFrequency(double* fftData, int numSamples);
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 7, /* data=*/ 6, /* reset=*/ U8X8_PIN_NONE);

#define SERVICE_UUID        "04faf1c3-e3d5-49c0-9ead-e628bc35a133"
#define CHARACTERISTIC_UUID "90705d32-c022-4a47-bda4-8ff642549198"

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("✅ BLE Device Connected");
    };
 
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("❌ BLE Device Disconnected");
    }
};

void setup() {

    Serial.begin(115200);
    pinMode(trigPin, OUTPUT);                 
    pinMode(echoPin, INPUT);                                                        
    Wire.begin(); 
    u8g2.begin();
    u8g2.setFlipMode(1);

    // Configure I2S          
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 16000,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Using left channel
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 512
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DIN
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);

    // // Name server device
    // BLEDevice::init("SWHW_Sensor🌲🌲🌲");
    // pServer = BLEDevice::createServer();
    // pServer->setCallbacks(new MyServerCallbacks());
    // BLEService *pService = pServer->createService(SERVICE_UUID);
    // pCharacteristic = pService->createCharacteristic(
    //     CHARACTERISTIC_UUID,
    //     BLECharacteristic::PROPERTY_READ |
    //     BLECharacteristic::PROPERTY_WRITE |
    //     BLECharacteristic::PROPERTY_NOTIFY
    // );
    // pCharacteristic->addDescriptor(new BLE2902());
    // pCharacteristic->setValue("👋 Hello World 👋");
    // pService->start();
    // // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this is working for backward compatibility
    // BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    // pAdvertising->addServiceUUID(SERVICE_UUID);
    // pAdvertising->setScanResponse(true);
    // pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    // pAdvertising->setMinPreferred(0x12);
    // BLEDevice::startAdvertising();  
}


void loop() {
    int16_t sampleBuffer[SAMPLES];  // Array to store raw audio samples
    size_t bytesRead;

    // Read I2S audio data
    i2s_read(I2S_NUM_0, sampleBuffer, sizeof(sampleBuffer), &bytesRead, portMAX_DELAY);

    // Convert the raw 16-bit samples to double values for FFT processing
    for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = (double)sampleBuffer[i];  // Assign real part
        vImag[i] = 0.0;  // Imaginary part is zeroed
    }
                                        
    // Perform FFT
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);  // Apply Hamming window
    FFT.compute(FFTDirection::Forward);  // Compute FFT
    FFT.complexToMagnitude();  // Convert to magnitudes
                
    // Get the major peak frequency
    double peakFreq = FindPeakFrequency(vReal, SAMPLES);

    Serial.print("Major peak frequency: ");
    Serial.print(peakFreq);
    Serial.println(" Hz");

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = (duration*.0343)/2;
    Serial.print("Distance: ");
    Serial.println(distance);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);                  

    // Display Peak Frequency
    u8g2.setCursor(10, 20);
    if (peakFreq == 62.5) {
      u8g2.print("Typing...");
    }
    // u8g2.print("Freq: ");
    // u8g2.print(peakFreq);
    // u8g2.print(" Hz");

    // Display Distance
    u8g2.setCursor(10, 40);
    if (distance < 100) {
      u8g2.print("At screen");
    } else {
      u8g2.print("Off screen");
    }
    // u8g2.print("Dist: ");
    // u8g2.print(distance);
    // u8g2.print(" cm");

    u8g2.sendBuffer();

    unsigned long currentMillis = millis();
  // previousMillis = currentMillis;

    if (deviceConnected) {
        // Send new readings to database
        if (peakFreq == TARGET_FREQUENCY) {
        pCharacteristic->setValue("FREQ_MATCH");
        pCharacteristic->notify();
        Serial.println("🎙️ Send Frequency 🎙️");
        }   
        if (distance < DISTANCE_THRESHOLD) {
        if (!trackingProximity) {
            trackingProximity = true;
            lastTimeBelowThreshold = currentMillis;
        } else if (currentMillis - lastTimeBelowThreshold >= TIME_THRESHOLD) {
            pCharacteristic->setValue("PROXIMITY_ALERT");
            pCharacteristic->notify();
            trackingProximity = false;
            Serial.println("🎯 Send Proximity 🎯");
        }
        } else {
        trackingProximity = false;

      // char buffer[32];
      // snprintf(buffer, sizeof(buffer), "Distance: %.2f cm", distance);
      // pCharacteristic->setValue(buffer);
      // pCharacteristic->notify();
      // Serial.println(buffer);
    }
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500);  // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising();  // advertise again
      Serial.println("Start advertising");
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }

  delay(1000);                       
}

// Function to manually find the peak frequency using only positive frequencies
double FindPeakFrequency(double* fftData, int numSamples) {
    int peakIndex = 0;
    double peakValue = 0.0;

    // Loop through the FFT data to find the maximum value (the peak)
    for (int i = 1; i < numSamples / 2; i++) { // Only consider positive frequencies
        if (fftData[i] > peakValue) {
            peakValue = fftData[i];
            peakIndex = i;
        }
    }

    // Calculate the frequency corresponding to the peak index (using only positive frequencies)
    double peakFrequency = (peakIndex * 1.0 * SAMPLING_FREQUENCY) / numSamples;
    return peakFrequency;
}

