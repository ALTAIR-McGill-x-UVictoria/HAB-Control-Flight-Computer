/****************************************************************************************************************************
  PWM_MultiChannel.ino

  For Teensy boards (Teensy 2.x, Teensy LC, Teensy 3.x, Teensy 4.x, etc.)
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/Teensy_PWM
  Licensed under MIT license
*****************************************************************************************************************************/

#define _PWM_LOGLEVEL_       4

#include "Teensy_PWM.h"

#define USING_FLEX_TIMERS      true

// To select correct pins from different timers for different frequencies
// For example, pin 7/FlexPWM1_3_B and pin8/FlexPWM1_3_A will have same frequency, etc.
// For the whole list, check Teensy_PWM.h or README.md

#if USING_FLEX_TIMERS
// Using FlexTimers
uint32_t PWM_Pins[] = { 4, 5, 6, 7 };
#else
// Using QuadTimers
uint32_t PWM_Pins[] = { 10, 11, 14, 15 };
#endif

float dutyCycle[]   = { 10.0f, 30.0f, 50.0f, 90.0f };

#define NUM_OF_PINS       ( sizeof(PWM_Pins) / sizeof(uint32_t) )

// Must be same frequency for same channel
float frequency = 2000.0f;

//creates pwm instances
Teensy_PWM* PWM_Instance[NUM_OF_PINS];

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
  Serial.print(F("\nStarting PWM_MultiChannel using FlexTimers on "));
#else
  Serial.print(F("\nStarting PWM_MultiChannel using QuadTimers on "));
#endif

  Serial.println(BOARD_NAME);
  Serial.println(TEENSY_PWM_VERSION);

  for (uint8_t index = 0; index < NUM_OF_PINS; index++)
  {
    PWM_Instance[index] = new Teensy_PWM(PWM_Pins[index], frequency, dutyCycle[index]);

    if (PWM_Instance[index])
    {
      PWM_Instance[index]->setPWM();
    }
  }

  Serial.println(dashLine);
  Serial.println("Index\tPin\tPWM_freq\tDutyCycle\tActual Freq");
  Serial.println(dashLine);

  for (uint8_t index = 0; index < NUM_OF_PINS; index++)
  {
    if (PWM_Instance[index])
    {
      Serial.print(index);
      Serial.print("\t");
      Serial.print(PWM_Pins[index]);
      Serial.print("\t");
      Serial.print(frequency);
      Serial.print("\t\t");
      Serial.print(dutyCycle[index]);
      Serial.print("\t\t");
      Serial.println(PWM_Instance[index]->getActualFreq(), 4);
    }
    else
    {
      Serial.println();
    }
  }

  for (uint8_t index = 0; index < NUM_OF_PINS; index++)
  {
    printPWMInfo(PWM_Instance[index]);
  }
}

void loop()
{
  //Long delay has no effect on the operation of hardware-based PWM channels
  delay(1000000);
}
