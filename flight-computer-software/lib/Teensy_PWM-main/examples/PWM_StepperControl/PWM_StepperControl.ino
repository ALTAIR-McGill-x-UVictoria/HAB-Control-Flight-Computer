/****************************************************************************************************************************
  PWM_StepperControl.ino
  
  For Teensy boards (Teensy 2.x, Teensy LC, Teensy 3.x, Teensy 4.x, etc.)
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/Teensy_PWM
  Licensed under MIT license

  Credits of Paul van Dinther (https://github.com/dinther). Check https://github.com/khoih-prog/RP2040_PWM/issues/16
*****************************************************************************************************************************/

// Use with Stepper-Motor driver, such as TMC2209

#define _PWM_LOGLEVEL_        4

#include <Teensy_PWM.h>

#define USING_FLEX_TIMERS      true

#if USING_FLEX_TIMERS
  #define STEP_PIN       5
#else
  #define STEP_PIN       15
#endif

#define DIR_PIN         9

float frequency;
float dutyCycle;

Teensy_PWM* stepper;

void setSpeed(int speed)
{
  if (speed == 0)
  {
    // Use DC = 0 to stop stepper
    stepper->setPWM(STEP_PIN, 500, 0);
  }
  else
  {
    //  Set the frequency of the PWM output and a duty cycle of 50%
    digitalWrite(DIR_PIN, (speed < 0));
    stepper->setPWM(STEP_PIN, abs(speed), 50);
  }
}

void setup() 
{
  pinMode(DIR_PIN, OUTPUT);
  
  Serial.begin(115200);

  while (!Serial && millis() < 5000);

  delay(100);

#if USING_FLEX_TIMERS
  Serial.print(F("\nStarting PWM_StepperControl using FlexTimers on "));
#else
  Serial.print(F("\nStarting PWM_StepperControl using QuadTimers on "));
#endif

  Serial.println(BOARD_NAME);
  Serial.println(TEENSY_PWM_VERSION);
  
  //  Create PWM object and passed just a random frequency of 500 in it
  //  The duty cycle is how you turn the motor on and off
  stepper = new Teensy_PWM(STEP_PIN, 500, 0);
}

void loop() 
{
  setSpeed(1000);
  delay(3000);

  // Stop before reversing
  setSpeed(0);
  delay(3000);

  // Reversing
  setSpeed(-500);
  delay(3000);
}
