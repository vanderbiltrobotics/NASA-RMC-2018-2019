/*
 * This program turns both motors on, driving the robot forward
*/

int leftMotor1 = 1;    // Left Motor
int leftMotor2 = 12;
int leftMotorPWM = 11;

int rightMotor1 = 4;    // Right Motor
int rightMotor2 = 5;
int rightMotorPWM = 10;


void setup() {

  // Set motor 1 direction
  pinMode(leftMotor1, LOW);
  pinMode(leftMotor2, HIGH);

  // Set motor 2 direction
  pinMode(rightMotor1, LOW);
  pinMode(rightMotor2, HIGH); 

  // Set motor speeds
  analogWrite(leftMotorPWM, 200);
  analogWrite(rightMotorPWM, 200);
}

void loop() {
    // Do nothing
}

