#include <AccelStepper.h> //requires adding this library in Arduino

// Define pin connections
#define DIR_PIN 9
#define STEP_PIN 8

// Create an instance of AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Steps per revolution for your stepper motor
const float STEPS_PER_REVOLUTION = 200; // Adjust this for your motor

// Screw specifications
const float SCREW_DIAMETER = 6.0; // mm
const float SCREW_PITCH = 1.0;    // mm (T6-1 screw: 1 mm per revolution)

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Set the maximum speed and acceleration for the stepper motor
  stepper.setMaxSpeed(1000); // steps per second
  stepper.setAcceleration(500); // steps per second^2

  // Indicate setup is complete
  Serial.println("Setup complete. Waiting for commands...");
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming distance
    String distance_str = Serial.readStringUntil('\n');
    float distance_mm = distance_str.toFloat();

    // Calculate revolutions and steps
    float revolutions = distance_mm / SCREW_PITCH;
    long steps = revolutions * STEPS_PER_REVOLUTION;

    // Move the stepper motor
    stepper.moveTo(steps);
    stepper.runToPosition();

    // Send a response back
    Serial.print("Moved ");
    Serial.print(distance_mm);
    Serial.println(" mm.");
  }
}
