#include <DRV8835MotorShield.h>
#include <QTRSensors.h>

/*
 * This example uses the DRV8835MotorShield library to drive each motor with the
 * Pololu DRV8835 Dual Motor Driver Shield for Arduino forward, then backward.
 * The yellow user LED is on when a motor is set to a positive speed and off when
 * a motor is set to a negative speed.
 Min speed: 0
 Max speed: 400
*/

// create an object for your type of sensor (RC or Analog)
// in this example we have three sensors on analog inputs 0 - 2 (digital pins 14 - 16)
// Set pins for four RC sensors connected to pins 6, 7, A0, and A1.
// (Most analog pins can also be used as digital pins.)
QTRSensors qtr;

const int SensorCount = 11;
const int oddPin = 2;
const int evenPin = 1;

const int calibratoinLedPin = 13;

uint16_t sensorValues[SensorCount];

void printResults();

#define BTN_PIN 12

int var = 0;

DRV8835MotorShield motors;

void setup()
{
  pinMode(BTN_PIN, INPUT);

  // uncomment one or both of the following lines if your motors' directions need to be flipped
  // motors.flipM1(true);
  // motors.flipM2(true);

  motors.setM1Speed(0);
  motors.setM2Speed(0);

  qtr.setTypeRC(); // 1    2   3   4   5  6   7  8  9  10  11
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, 3, 5, 6, 8, 11}, SensorCount);
  qtr.setEmitterPins(oddPin, evenPin);

  // optional: wait for some input from the user, such as  a button press
  delay(500);
  pinMode(calibratoinLedPin, OUTPUT);
  digitalWrite(calibratoinLedPin, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }

  digitalWrite(calibratoinLedPin, LOW); // turn off Arduino's LED to indicate we are through with calibration

  printResults();
  // optional: signal that the calibration phase is now over and wait for further
  // input from the user, such as a button press

  Serial.begin(9600);
}

void loop()
{

  // QRT
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {

    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  delay(250);

  // run M1 motor with positive speed
  var = digitalRead(BTN_PIN);
  Serial.println(var);

  if (var == HIGH)
  {
    delay(1000);

    motors.setM1Speed(250);
    motors.setM2Speed(250);

    delay(2000);

    motors.setM1Speed(0);
    motors.setM2Speed(0);
  }
  else
  {
    motors.setM1Speed(0);
    motors.setM2Speed(0);
  }
}

void printResults()
{

  // print the calibration minimum values measured when emitters were on
  // Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }

  Serial.println();
  Serial.println();
  delay(1000);
}
