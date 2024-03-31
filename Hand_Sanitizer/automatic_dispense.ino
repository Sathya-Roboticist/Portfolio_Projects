#include <Servo.h>

#define IR_SENSOR_PIN 2
#define SERVO_PIN 9

Servo dispenserServo;

int irState = 0; // variable to store IR sensor state
int previousIrState = 0; // variable to store previous IR sensor state
unsigned long previousMillis = 0; // variable to store previous time
unsigned long interval = 2000; // interval at which to dispense sanitizer (in milliseconds)
bool dispenserActive = false; // flag to indicate if sanitizer dispenser is active

void setup() {
  Serial.begin(9600);
  pinMode(IR_SENSOR_PIN, INPUT);
  dispenserServo.attach(SERVO_PIN);
}

void loop() {
  // read the state of the IR sensor
  irState = digitalRead(IR_SENSOR_PIN);

  // check if the state of the IR sensor has changed
  if (irState != previousIrState) {
    // if the IR sensor is triggered
    if (irState == HIGH) {
      Serial.println("Hand Detected");
      dispenserActive = true; // activate sanitizer dispenser
      previousMillis = millis(); // reset timer
    } else {
      Serial.println("No Hand Detected");
    }
    previousIrState = irState;
  }

  // if sanitizer dispenser is active
  if (dispenserActive) {
    unsigned long currentMillis = millis();
    // check if it's time to dispense sanitizer
    if (currentMillis - previousMillis >= interval) {
      // dispense sanitizer
      Serial.println("Dispensing Sanitizer");
      dispenserServo.write(90); // rotate servo to dispense sanitizer
      delay(1000); // wait for sanitizer to dispense (adjust as needed)
      dispenserServo.write(0); // return servo to initial position
      dispenserActive = false; // deactivate sanitizer dispenser
    }
  }
}
