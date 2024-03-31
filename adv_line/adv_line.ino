// Length = 5m

/*************************************************************************
 * QRT-X GPIO pins declaration
 *************************************************************************/
#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t SensorCount = 11;
uint16_t sensorValues[SensorCount];

const int oddPin = 2;
const int evenPin = 2;

/*************************************************************************
 * DRV8835 GPIO pins declaration
 *************************************************************************/
// int mode = ; Auto ON

// Left Motor
int aphase = 3;
int aenbl = 5;

// Right Motor
int bphase = 6;
int benbl = 9;

int base_speed = 75;

int max_speed = 120;
int min_speed = -120;

/*************************************************************************
* PID controler variables
*************************************************************************/
float Kp = 0.04;  //related to the proportional control term;
                   //change the value by trial-and-error (ex: 0.07). 0.03
float Ki = 0.0009; //related to the integral control term;
                   //change the value by trial-and-error (ex: 0.0008). 0.00001
float Kd = 0.4;  //related to the derivative control term;
                   //change the value by trial-and-error (ex: 0.6). 0.3

int P = 0;
int I = 0;
int D = 0;

int lastError = 0;

/*************************************************************************
* Buttons pins declaration
*************************************************************************/
int buttoncalibrate = A6;
int startButton = A7;

void setup() {
  Serial.begin(9600);

  // QTRX Set up
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){4, 7, 8, 10, 11, 12, A0, A1, A2, A3, A4}, SensorCount);
  qtr.setEmitterPin(oddPin);

  // Motors Set up
  pinMode(aphase, OUTPUT);
  pinMode(aenbl, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);

  pinMode(13,OUTPUT);

  pinMode(buttoncalibrate, INPUT);
  pinMode(startButton, INPUT);

  digitalWrite(13, LOW);

  while( analogRead(buttoncalibrate) == 0); // Start Calibration A6

  calibration(); // Calibrate the robot for 10 seconds

  while( analogRead(startButton) == 0); // Start Calibration A6

}

void loop() {
  PID_control();
}

void PID_control(){
  uint16_t position = qtr.readLineBlack(sensorValues);

  int error = 5000 - position;

  P = error;
  I = I + error;
  D = error - lastError;

  lastError = error;

  float motor_speed = P*Kp + I*Ki + D*Kd;
  int motor_speed_A = base_speed - motor_speed;
  int motor_speed_B = base_speed + motor_speed;

  // Set max speed
  if (motor_speed_A > max_speed) { motor_speed_A = max_speed; }
  if (motor_speed_B > max_speed) { motor_speed_B = max_speed; }

  // Set min speed
  if (motor_speed_A < min_speed) { motor_speed_A = min_speed; }
  if (motor_speed_B < min_speed) { motor_speed_B = min_speed; }

  // pid_test( position, motor_speed);

  forward_movement(motor_speed_A, motor_speed_B);

  // Serial.print('\n');
  // Serial.print("A: ");
  // Serial.print(motor_speed_A);
  // Serial.print('\n');
  // Serial.print("B: ");
  // Serial.print(motor_speed_B);

  // Serial.print('\n');
}

void forward_movement(int speedA, int speedB) {
  if (speedA < 0) {
    speedA = 0 - speedA;
    digitalWrite(aphase, LOW);
  }
  else {
    digitalWrite(aphase, HIGH);
  }

  if (speedB < 0) {
    speedB = 0 - speedB;
    digitalWrite(bphase, HIGH);
  }
  else {
    digitalWrite(bphase, LOW);
  }

  analogWrite(aenbl, speedA);
  analogWrite(benbl, speedB);
}

void calibration() {
  digitalWrite(13,HIGH);
  for (uint16_t i = 0; i < 400; i++) { qtr.calibrate(); }
  digitalWrite(13,LOW);
}


//  Testing functions
void qtrx_test(){
  uint16_t position = qtr.readLineBlack(sensorValues);

  for (uint8_t i = 0; i < SensorCount; i++)
  {

    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  delay(250);
}

void pid_test(int position,int motor_speed){

  Serial.print("\n");
  Serial.print("Line: ");
  Serial.println(position);
  Serial.print("\n");

  Serial.print("P: ");
  Serial.print(P);
  Serial.print('\t');
  Serial.print("I: ");
  Serial.print(I);
  Serial.print('\t');
  Serial.print("D: ");
  Serial.print(D);
  Serial.print('\n');

  Serial.print("Kp: ");
  Serial.print(Kp);
  Serial.print('\t');
  Serial.print("Ki: ");
  Serial.print(Ki);
  Serial.print('\t');
  Serial.print("Kd: ");
  Serial.print(Kd);
  Serial.print('\t');

  Serial.print("sum: ");
  Serial.print(motor_speed);
  Serial.print('\n');
  Serial.print('\n');

  delay(500);
}