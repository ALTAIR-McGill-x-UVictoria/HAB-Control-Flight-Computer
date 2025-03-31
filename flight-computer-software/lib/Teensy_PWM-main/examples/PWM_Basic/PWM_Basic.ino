/****************************************************************************************************************************
  PWM_Basic.ino

  For Teensy boards (Teensy 2.x, Teensy LC, Teensy 3.x, Teensy 4.x, etc.)
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/Teensy_PWM
  Licensed under MIT license
*****************************************************************************************************************************/

#define _PWM_LOGLEVEL_       4

#include "Teensy_PWM.h"

#define USING_FLEX_TIMERS      true

#if USING_FLEX_TIMERS
  #define pinToUse       5
#else
  #define pinToUse       15
#endif

//creates pwm instance
Teensy_PWM* PWM_Instance;

float frequency = 1000.0f;

float dutyCycle = 0.0f;

void setup()
{
  Serial.begin(115200);

  while (!Serial && millis() < 5000);

  delay(500);

#if USING_FLEX_TIMERS
  Serial.print(F("\nStarting PWM_Basic using FlexTimers on "));
#else
  Serial.print(F("\nStarting PWM_Basic using QuadTimers on "));
#endif

  Serial.println(BOARD_NAME);
  Serial.println(TEENSY_PWM_VERSION);

  //assigns PWM frequency of 1.0 KHz and a duty cycle of 0%
  PWM_Instance = new Teensy_PWM(pinToUse, frequency, dutyCycle);

  if ( (!PWM_Instance) || !PWM_Instance->isPWMEnabled())
  {
    Serial.print(F("Stop here forever"));

    while (true)
      delay(10000);
  }
}

void loop()
{
  // You can change frequency here, anytime
  frequency = 2000.0f;
  //frequency = 20.0f;
  dutyCycle = 20.0f;

  PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);

  delay(10000);

  // You can change frequency here, anytime
  frequency = 5000.0f;
  //frequency = 50.0f;
  dutyCycle = 90.0f;

  PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);

  //while (1)
  delay(10000);
}
