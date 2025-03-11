#include <Arduino.h>
#include <AccelStepper.h>

#define MOTOR_PIN_1 5
#define MOTOR_PIN_2 6
#define LED_PIN 3


AccelStepper stepper(AccelStepper::FULL2WIRE, MOTOR_PIN_1, MOTOR_PIN_2);
bool direction = false;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Setup started...");
    stepper.setMaxSpeed(300);
    stepper.setAcceleration(200);

    pinMode (LED_PIN, OUTPUT);

    Serial.println("Stepper Motor Test Started...");
}

void blinkLED(int duration) {
    Serial.println("LED");
    digitalWrite(LED_PIN, HIGH);
    delay(duration);
    digitalWrite(LED_PIN, LOW);
    delay(duration);
}

void loop() {
    blinkLED(500);
    delay(2000);
    
    stepper.moveTo(300); 
    Serial.println("Rotating 30 degrees...");
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }  

    Serial.println("Stopping motor for 5 seconds...");
    delay(5000);

    stepper.moveTo(600);  // Move to cumulative 80 degrees
    Serial.println("Rotating 50 degrees...");
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }   

    Serial.println("Motor stopped.");

    Serial.println("Blinking LED for 3 seconds...");
    for (int i = 0; i < 6; i++) {
        blinkLED(500);
    }

    Serial.println("Sequence completed.");

    delay(5000);
}