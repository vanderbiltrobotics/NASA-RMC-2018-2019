
/*
 * This program drives the robot forward, stopping when it gets close to a wall
 */

// motor 1
int leftMotor1=1;
int leftMotor2=12;
int leftMotorPWM =11;

// motor 2
int rightMotor1=4;
int rightMotor2=5;
int rightMotorPWM =10;

// sensor
int sensorPin = A0;

// Other vars
int inputFromSensor;
float voltage;

void setup() {

  // Set pin directions
  pinMode(leftMotor1,OUTPUT); 
  pinMode(leftMotor2,OUTPUT);
  pinMode(leftMotorPWM,OUTPUT);
  
  pinMode(rightMotor1,OUTPUT);
  pinMode(rightMotor2,OUTPUT);
  pinMode(rightMotorPWM,OUTPUT);

  // Motor turning forward
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);

  // Set initial speed
  analogWrite(leftMotorPWM, 100); 
  analogWrite(rightMotorPWM, 100);

}

void loop() {

  // Get new sensor reading, convert to corresponding voltage
  inputFromSensor = analogRead(sensorPin);
  voltage = map(inputFromSensor,0,1023,0,5);  

  // If an object is close, turn off motors
  if(voltage>=1){
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
  }

  // Otherwise, turn motors back on
  else {
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
  }
}
