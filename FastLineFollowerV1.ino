#include <QTRSensors.h>

// Definir pines para el controlador de motores TB6612FNG
#define motorAInput1 7
#define motorAInput2 8
#define motorBInput1 5
#define motorBInput2 4
#define motorAPWM 9
#define motorBPWM 3
#define STBY      6  // Pin STBY del TB6612FNG


// Definir pines para el sensor QTR-8A
#define NUM_SENSORS 8
#define NUM_SAMPLES_PER_SENSOR 4
#define EMITTER_PIN 10

// Constantes del control PID
#define KP 0.0035

// Constante proporcional
#define KD 0.0058 // Constante derivativa
#define KI 0.00001  // Constante integral

QTRSensors qtr;

unsigned int sensorValues[NUM_SENSORS];
int lastError = 0;
int integral = 0;

void setup() {
  // Set output pins for TB6612FNG motor controller
  pinMode(motorAInput1, OUTPUT);
  pinMode(motorAInput2, OUTPUT);
  pinMode(motorBInput1, OUTPUT);
  pinMode(motorBInput2, OUTPUT);
  pinMode(motorAPWM, OUTPUT);
  pinMode(motorBPWM, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);
  // Initialize QTR-8A sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, NUM_SENSORS);
  qtr.setEmitterPin(EMITTER_PIN);
  
  digitalWrite(LED_BUILTIN, HIGH); // turn off Arduino's LED to indicate we are through with calibration

  // Start serial communication
  Serial.begin(9600);
  
    for (int i = 0; i < 250; i++) {
    qtr.calibrate();
    delay(10);
  }
    digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

}


void loop() {
  // Read sensor values
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Calculate error (weighted center position)
  int weightedSum = 0;
  int sum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    weightedSum += (i * sensorValues[i]);
    sum += sensorValues[i];
  }
  int error = position - (NUM_SENSORS - 1) * 1000 / 2;

  // Calculate PID control components
  int proportional = KP * error;
  int derivative = KD * (error - lastError);
  integral += error;
  int integralComponent = KI * integral;

  // Calculate control signal
  int controlSignal = proportional + derivative + integralComponent;

  // Control motors using the control signal
  int motorSpeed = 225; // Adjust this value to control the forward speed

 // Control motors using the control signal
int motorSpeedA = motorSpeed - controlSignal;
int motorSpeedB = motorSpeed + controlSignal;

// Limit the motor speed values to the range of 0-255
motorSpeedA = constrain(motorSpeedA, 0, 255);
motorSpeedB = constrain(motorSpeedB, 0, 255);

// Control the motors
if (controlSignal > 4) {
  // Turn right
  digitalWrite(motorBInput1, LOW);
  digitalWrite(motorBInput2, HIGH);
  digitalWrite(motorAInput1, HIGH);//LOW para avanzar hacia adelante
  digitalWrite(motorAInput2, LOW);
  analogWrite(motorAPWM, motorSpeedA);
  analogWrite(motorBPWM, motorSpeedB);
} 
if (controlSignal < -4 ) {
  // Turn left
  digitalWrite(motorBInput1, HIGH);
  digitalWrite(motorBInput2, LOW);
  digitalWrite(motorAInput1, LOW);
  digitalWrite(motorAInput2, HIGH); //LOW para avanzar hacia adelante
  analogWrite(motorAPWM, motorSpeedA);
  analogWrite(motorBPWM, motorSpeedB);
}

if (controlSignal >= -4 && controlSignal <= 4) {
  // Turn left
  digitalWrite(motorBInput1, LOW);
  digitalWrite(motorBInput2, HIGH);
  digitalWrite(motorAInput1, LOW);
  digitalWrite(motorAInput2, HIGH);
  analogWrite(motorAPWM, motorSpeedA);
  analogWrite(motorBPWM, motorSpeedB);
}

  // Update last error
  lastError = error;

  // Print sensor values and position to the serial monitor
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(controlSignal);
  

  //delay(2);
}
