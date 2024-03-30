#include <QTRSensors.h> //Make sure to install the library

/*************************************************************************
* QTR GPIO pins declaration
*************************************************************************/

QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];
int button = 12;

const int oddPin = 2;

/*************************************************************************
* DRV8835 GPIO pins declaration
*************************************************************************/
int mode = 13;
int aphase = 7; //old 10
int aenbl = 10; // old 7
int bphase = 4;
int benbl = 9;
int led_pin = 3;

int max_speed = 255;
int min_speed = -255;

/*************************************************************************
* PID controler variables
*************************************************************************/
float Kp = 0.495;  //related to the proportional control term;
                   //change the value by trial-and-error (ex: 0.07).
float Ki = 0.00006; //related to the integral control term;
                   //change the value by trial-and-error (ex: 0.0008).
float Kd = 1.05; //related to the derivative control term;
                   //change the value by trial-and-error (ex: 0.6).

int P = 0;
int I = 0;
int D = 0;

int lastError = 0;


int base_speed = 255;

void setup() {
  // Serial.begin(9600);
  pinMode(led_pin, OUTPUT);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ A5, A4, A3, A2, A1, A0 }, SensorCount);
  qtr.setEmitterPin(oddPin);

  pinMode(mode, OUTPUT);
  pinMode(aphase, OUTPUT);
  pinMode(aenbl, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);
  pinMode(button, INPUT);

  digitalWrite(mode, HIGH); //one of the two control interfaces

  while(digitalRead(button) == 0);
  digitalWrite(led_pin, HIGH);  // turn the LED on (HIGH is the voltage level)
  calibration(); // Calibrate the robot for 10 seconds
  digitalWrite(led_pin, LOW);   // turn the LED off by making the voltage LOW
  while(digitalRead(button) == 0);
}

void loop() {
  // forward_movement(100, 100);

  PID_control();
}

void PID_control(){
  uint16_t position = qtr.readLineBlack(sensorValues);
  //  for (uint8_t i = 0; i < SensorCount; i++)
  // {

  //   Serial.print(sensorValues[i]);
  //   Serial.print('\t');
  // }
  // Serial.println(position);

  // if(position > 2400 and position < 2600 )
  //   position = 2500;

  int error = 2500 - position;
  // Serial.print(((position / 100) * 100));
  // Serial.print('\n');
  // Serial.print('\n');
  P = error;
  I = I + error;
  D = error - lastError;

  lastError = error;

  float motor_speed = P*Kp + I*Ki + D*Kd;
  int motor_speed_A = base_speed + motor_speed;
  int motor_speed_B = base_speed - motor_speed;

  // Set max speed
  if (motor_speed_A > max_speed) { motor_speed_A = max_speed; }
  if (motor_speed_B > max_speed) { motor_speed_B = max_speed; }

  // Set min speed
  if (motor_speed_A < min_speed) { motor_speed_A = min_speed; }
  if (motor_speed_B < min_speed) { motor_speed_B = min_speed; }

  // Serial.print("\n");
  // Serial.print("Line: ");
  // Serial.println(position);
  // Serial.print("\n");

  // Serial.print("P: ");
  // Serial.print(P);
  // Serial.print('\t');
  // Serial.print("I: ");
  // Serial.print(I);
  // Serial.print('\t');
  // Serial.print("D: ");
  // Serial.print(D);
  // Serial.print('\n');

  // Serial.print("Kp: ");
  // Serial.print(Kp);
  // Serial.print('\t');
  // Serial.print("Ki: ");
  // Serial.print(Ki);
  // Serial.print('\t');
  // Serial.print("Kd: ");
  // Serial.print(Kd);
  // Serial.print('\t');

  // Serial.print("sum: ");
  // Serial.print(motor_speed);
  // Serial.print('\n');
  // Serial.print('\n');
  forward_movement(motor_speed_A, motor_speed_B);

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

  // digitalWrite(13, HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  // digitalWrite(13, LOW);
}