
// Include stepper library
#include <Stepper.h>

// defines
#define STEPS 200

// DC Motors
int leftMotor1 = 1;    // Left Motor
int leftMotor2 = 12;
int leftMotorPWM = 11;

int rightMotor1 = 4;    // Right Motor
int rightMotor2 = 5;
int rightMotorPWM = 10;

// Sensors
int ir_sensor = A0;

// Stepper motor pin assignments
int INA2 = 7;
int INA1 = 8;
int INB1 = 9;
int INB2 = 6;

// define our stepper motor object
Stepper stepper(STEPS, 7, 8, 9, 6);


void setup() {

}

void loop() {

}
