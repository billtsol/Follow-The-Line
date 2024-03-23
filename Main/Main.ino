#include <QTRSensors.h> //Make sure to install the library

/*************************************************************************
 * QTR GPIO pins declaration
 *************************************************************************/

QTRSensors qtr;
const uint8_t SensorCount = 11;
uint16_t sensorValues[SensorCount];

const int oddPin = 2;
const int evenPin = 12;

/*************************************************************************
 * DRV8835 GPIO pins declaration
 *************************************************************************/
int mode = 13;
int aphase = 7; // old 10
int aenbl = 10; // old 7
int bphase = 4;
int benbl = 9;

int max_speed = 250;
int min_speed = -250;

/*************************************************************************
 * Buttons pins declaration
 *************************************************************************/
int buttoncalibrate = 13;

/*************************************************************************
 * PID controler variables
 *************************************************************************/
float Kp = 0.05; // related to the proportional control term;
                 // change the value by trial-and-error (ex: 0.07).
float Ki = 0.00001; // related to the integral control term;
                    // change the value by trial-and-error (ex: 0.0008).
float Kd = 0.5; // related to the derivative control term;
                // change the value by trial-and-error (ex: 0.6).

int P = 0;
int I = 0;
int D = 0;

int lastError = 0;

int base_speed = 200;

void setup()
{
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A5, 3, A4, 5, A3, 8, A2, 6, A1, 11, A0}, SensorCount);
  qtr.setEmitterPins(oddPin, evenPin);

  pinMode(mode, OUTPUT);
  pinMode(aphase, OUTPUT);
  pinMode(aenbl, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);

  digitalWrite(mode, HIGH); // one of the two control interfaces

  delay(2000);

  calibration(); // Calibrate the robot for 10 seconds
}

void loop()
{
  // forward_movement(100, 100);

  PID_control();
}

void PID_control()
{
  uint16_t position = qtr.readLineBlack(sensorValues);

  int error = 5000 - position;

  P = error;
  I = I + error;
  D = error - lastError;

  lastError = error;

  float motor_speed = P * Kp + I * Ki + D * Kd;
  int motor_speed_A = base_speed + motor_speed;
  int motor_speed_B = base_speed - motor_speed;

  // Set max speed
  if (motor_speed_A > max_speed)
  {
    motor_speed_A = max_speed;
  }
  if (motor_speed_B > max_speed)
  {
    motor_speed_B = max_speed;
  }

  // Set min speed
  if (motor_speed_A < min_speed)
  {
    motor_speed_A = min_speed;
  }
  if (motor_speed_B < min_speed)
  {
    motor_speed_B = min_speed;
  }

  forward_movement(motor_speed_A, motor_speed_B);
}

void forward_movement(int speedA, int speedB)
{
  if (speedA < 0)
  {
    speedA = 0 - speedA;
    digitalWrite(aphase, LOW);
  }
  else
  {
    digitalWrite(aphase, HIGH);
  }

  if (speedB < 0)
  {
    speedB = 0 - speedB;
    digitalWrite(bphase, HIGH);
  }
  else
  {
    digitalWrite(bphase, LOW);
  }

  analogWrite(aenbl, speedA);
  analogWrite(benbl, speedB);
}

void calibration()
{

  // digitalWrite(buttoncalibrate, HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  // digitalWrite(buttoncalibrate, LOW);
}
