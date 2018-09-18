
/**
 * This program illustrates basic usage of a stepper motor. The motor
 * is controlled using the Adafruit TB6612 stepper motor driver.
 */

// Include stepper library
#include <Stepper.h>

// defines
#define STEPS 200

// Stepper motor pin assignments
int INA2 = 7;
int INA1 = 8;
int INB1 = 9;
int INB2 = 6;

// define our stepper motor object
Stepper stepper(STEPS, 7, 8, 9, 6);

void setup() {
  stepper.setSpeed(20);
}

void loop() {
  stepper.step(STEPS / 2);
  delay(500);
  stepper.step(-STEPS / 2);
  delay(500);
}
