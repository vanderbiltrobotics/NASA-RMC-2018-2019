
/*
 * This program reads from a range sensor and prints the range information
 */

// sensor
int sensorPin = A0;

// Other vars
int sensorValue;
float voltage;

void setup() {

  // Establish serial connection
  Serial.begin(9600);
  
}

void loop() {

  // Get new sensor reading, convert to corresponding voltage
  inputFromSensor = analogRead(sensorPin);
  voltage = map(sensorValue, 0, 1023, 0, 5);  

  // Print data to serial port
  Serial.print("Quantized value:\t");
  Serial.print(sensorValue);
  Serial.print("\tVoltage:\t");
  Serial.println(voltage);

  // So values are printed at a reasonable rate
  delay(200);
}
