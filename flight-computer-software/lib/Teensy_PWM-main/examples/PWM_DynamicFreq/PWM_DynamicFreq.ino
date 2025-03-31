/****************************************************************************************************************************
  PWM_DynamicFreq.ino

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

float frequency;

char dashLine[] = "=====================================================================================";

void printPWMInfo(Teensy_PWM* PWM_Instance)
{
  Serial.println(dashLine);
  Serial.print("Actual data: pin = ");
  Serial.print(PWM_Instance->getPin());
  Serial.print(", PWM DC = ");
  Serial.print(PWM_Instance->getActualDutyCycle());
  Serial.print(", PWMPeriod = ");
  Serial.print(PWM_Instance->getPWMPeriod());
  Serial.print(", PWM Freq (Hz) = ");
  Serial.println(PWM_Instance->getActualFreq(), 4);
  Serial.println(dashLine);
}

void setup()
{
  Serial.begin(115200);

  while (!Serial && millis() < 5000);

  delay(500);

#if USING_FLEX_TIMERS
  Serial.print(F("\nStarting PWM_DynamicFreq using FlexTimers on "));
#else
  Serial.print(F("\nStarting PWM_DynamicFreq using QuadTimers on "));
#endif

  Serial.println(BOARD_NAME);
  Serial.println(TEENSY_PWM_VERSION);

  frequency = 10000.0f;

  PWM_Instance = new Teensy_PWM(pinToUse, frequency, 50.0f);

  if ( (!PWM_Instance) || !PWM_Instance->isPWMEnabled())
  {
    Serial.print(F("Stop here forever"));

    while (true)
      delay(10000);
  }

  Serial.println(dashLine);
}

void loop()
{
  delay(5000);

  frequency = 20000.0f;

  Serial.print(F("Change PWM Freq to "));
  Serial.println(frequency);
  PWM_Instance->setPWM(pinToUse, frequency, 50.0f);
  printPWMInfo(PWM_Instance);

  delay(5000);

  frequency = 10000.0f;

  Serial.print(F("Change PWM Freq to "));
  Serial.println(frequency);
  PWM_Instance->setPWM(pinToUse, frequency, 50.0f);
  printPWMInfo(PWM_Instance);
}
