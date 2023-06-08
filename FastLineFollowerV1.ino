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
#define KP 4.6// Constante proporcional
#define KD 19.4 // Constante derivativa 
#define KI 0.009  // Constante integral

QTRSensors qtr;

unsigned int sensorValues[NUM_SENSORS];
int lastError = 0;
int integral = 0;

void setup() {
// Establecer pines de salida para el driver TB6612FNG
  pinMode(motorAInput1, OUTPUT);
  pinMode(motorAInput2, OUTPUT);
  pinMode(motorBInput1, OUTPUT);
  pinMode(motorBInput2, OUTPUT);
  pinMode(motorAPWM, OUTPUT);
  pinMode(motorBPWM, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);
  // Inicializa el sensor QTR-8A 
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, NUM_SENSORS);
  qtr.setEmitterPin(EMITTER_PIN);
  
  digitalWrite(LED_BUILTIN, HIGH); // enciende LED de Arduino para indicar el inicio de la calibracion
 
  Serial.begin(9600);// inicio de comunicacion serial
  
    for (int i = 0; i < 250; i++) {
    qtr.calibrate();
    delay(10);
  }
    digitalWrite(LED_BUILTIN, LOW); apaga el led Arduino para indicar termino de calibracion
}


void loop() {
  // Leer valores del sensor
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Calcular el error (posición central ponderada)
  int weightedSum = 0;
  int sum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    weightedSum += (i * sensorValues[i]);
    sum += sensorValues[i];
  }
  int error = position - (NUM_SENSORS - 1) * 1000 / 2;

  // Calcular componentes de control PID
  int proportional = KP * error;
  int derivative = KD * (error - lastError);
  integral += error;
  int integralComponent = KI * integral;

  // Calcular señal de control
  int controlSignal = proportional + derivative + integralComponent;

  // Controlar motores usando la señal de control
  int motorSpeed = 255; //Ajuste este valor para controlar la velocidad de avance//130

 // Control de motores usando la señal de control
int motorSpeedA = motorSpeed - controlSignal;
int motorSpeedB = motorSpeed + controlSignal;

// Limit the motor speed values to the range of 0-255
motorSpeedA = constrain(motorSpeedA, 0, 255);
motorSpeedB = constrain(motorSpeedB, 0, 255);

// Control de motores
if (controlSignal >2) {
  //gira derecha
  digitalWrite(motorBInput1, LOW);
  digitalWrite(motorBInput2, HIGH);
  digitalWrite(motorAInput1, LOW);//LOW para avanzar hacia adelante
  digitalWrite(motorAInput2, HIGH);
  analogWrite(motorAPWM, motorSpeedA);
  analogWrite(motorBPWM, motorSpeedB);
} 
if (controlSignal < -2 ) {
  // gira izquierda
  digitalWrite(motorBInput1, LOW);
  digitalWrite(motorBInput2, HIGH);
  digitalWrite(motorAInput1, LOW);
  digitalWrite(motorAInput2, HIGH); //LOW para avanzar hacia adelante
  analogWrite(motorAPWM, motorSpeedA);
  analogWrite(motorBPWM, motorSpeedB);
}

}
