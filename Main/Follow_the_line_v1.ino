#include <QTRSensors.h> //Make sure to install the library

/*************************************************************************
* QTR GPIO pins declaration
*************************************************************************/

QTRSensors qtr;
const uint8_t SensorCount = 11;
uint16_t sensorValues[SensorCount];

const int oddPin = 2;
const int evenPin = 1;

/*************************************************************************
* DRV8835 GPIO pins declaration
*************************************************************************/
int mode = 13;
int aphase = 7;
int aenbl = 10;
int bphase = 4;
int benbl = 9;

/*************************************************************************
* Buttons pins declaration
*************************************************************************/
int buttoncalibrate = 12; //or pin A3
int buttonstart = 12;

/*************************************************************************
* PID controler variables
*************************************************************************/
float Kp = 0.0007; //related to the proportional control term; 
              //change the value by trial-and-error (ex: 0.07).
float Ki = 0.001; //related to the integral control term; 
              //change the value by trial-and-error (ex: 0.0008).
float Kd = 0.007; //related to the derivative control term; 
              //change the value by trial-and-error (ex: 0.6).

int P = 0;
int I = 0;
int D = 0;

int lastError = 0;

int base_speed_A = 40;
int base_speed_B = 40;

void setup() {
  Serial.begin(9600);
  
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5, 3, 5, 6, 8, 11  }, SensorCount);
  qtr.setEmitterPins(oddPin, evenPin);

  pinMode(mode, OUTPUT);
  pinMode(aphase, OUTPUT);
  pinMode(aenbl, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);

  digitalWrite(mode, HIGH); //one of the two control interfaces 

  pinMode(LED_BUILTIN, OUTPUT);

  while (digitalRead(buttoncalibrate) == LOW) { } // the main function won't start until the robot is calibrated

  calibration(); //calibrate the robot for 10 seconds

  while (digitalRead(buttoncalibrate) == LOW) { } // the main function won't start until the robot is calibrated

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

  float motor_speed =( P*Kp + I*Ki + D*Kd)/10;
  int motor_speed_A = base_speed_A - motor_speed; 
  int motor_speed_B = base_speed_B + motor_speed;

  if (position > 4500 && position < 5500){
    motor_speed_A = 40; 
    motor_speed_B = 40;

  }

  if (motor_speed_A > 80) {
    motor_speed_A = 80;
  }
  if (motor_speed_B > 80) {
    motor_speed_B = 80;
  }
  if (motor_speed_A < -80) {
    motor_speed_A = -80;
  }
  if (motor_speed_B < -80) {
    motor_speed_B = -80;
  }


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
 
  // Serial.print("A: ");
  // Serial.println(motor_speed_A);
  
  // Serial.print("B: ");
  // Serial.println(motor_speed_B);

  forward_movement(motor_speed_A, motor_speed_B);

  // delay(3000);
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

  Serial.print("A: ");
  Serial.println(speedA);
  
  Serial.print("B: ");
  Serial.println(speedB);
}

void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}