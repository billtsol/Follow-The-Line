#include <DRV8835MotorShield.h>

/*
 * This example uses the DRV8835MotorShield library to drive each motor with the
 * Pololu DRV8835 Dual Motor Driver Shield for Arduino forward, then backward.
 * The yellow user LED is on when a motor is set to a positive speed and off when
 * a motor is set to a negative speed.

 Max speed: 400
 Min speed: 0

 */

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

  Serial.begin(9600);
}

void loop()
{
  // run M1 motor with positive speed
  var = digitalRead(BTN_PIN);
  Serial.println(var);

  if (var == HIGH)
  {
    delay(1000);

    motors.setM1Speed(200);
    motors.setM2Speed(200);

    delay(2000);

    motors.setM1Speed(400);
    motors.setM2Speed(400);

    delay(2000);

    
    motors.setM1Speed(100);
    motors.setM2Speed(400);

    delay(1000);

    
    motors.setM1Speed(400);
    motors.setM2Speed(400);

    delay(5000);

    motors.setM1Speed(0);
    motors.setM2Speed(0);

    delay(1000);
  }
  else
  {
    motors.setM1Speed(0);
    motors.setM2Speed(0);
  }
}