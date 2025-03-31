/****************************************************************************************************************************
  Teensy_PWM.h

  For Teensy boards (Teensy 2.x, Teensy LC, Teensy 3.x, Teensy 4.x, etc.)
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/Teensy_PWM
  Licensed under MIT license

  Version: 1.1.1

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.0.0   K.Hoang      06/11/2022 Initial coding for Teensy 4.x, etc.
  1.1.0   K.Hoang      06/11/2022 Add support to Teensy 3.x, Teensy LC
  1.1.1   K.Hoang      21/01/2023 Add `PWM_StepperControl` example
*****************************************************************************************************************************/

#pragma once

#ifndef TEENSY_PWM_H
#define TEENSY_PWM_H

////////////////////////////////////////

#if !( defined(CORE_TEENSY) || defined(TEENSYDUINO) )
  #error This code is designed to run on Teensy platform! Please check your Tools->Board setting.

  ////////////////////////////////////////

#elif ( defined(__arm__) && defined(TEENSYDUINO) && defined(__IMXRT1062__) )

  #define USING_TEENSY4_PWM       true

  // For Teensy 4.0/4.1

  #ifndef BOARD_NAME
    #if defined(ARDUINO_TEENSY41)
      #define BOARD_NAME          F("Teensy 4.1")

      #if (_PWM_LOGLEVEL_ > 3)
        #warning Using Teensy 4.1
      #endif

    #elif defined(ARDUINO_TEENSY_MICROMOD)
      #define BOARD_NAME          F("Teensy MicroMod")

      #if (_PWM_LOGLEVEL_ > 3)
        #warning Using Teensy MicroMod
      #endif

    #else
      #define BOARD_NAME          F("Teensy 4.0")

      #if (_PWM_LOGLEVEL_ > 3)
        #warning Using Teensy 4.0
      #endif

    #endif

  #endif

  ////////////////////////////////////////

  // For Teensy 3.x and LC
#elif defined(__arm__) && defined(TEENSYDUINO) && (defined(KINETISK) || defined(KINETISL))

  //#error Not yet support board

  #define USING_TEENSY3_PWM       true

  #ifdef BOARD_NAME
    #undef BOARD_NAME
  #endif

  #if defined(__MK66FX1M0__)
    // For Teensy 3.6
    #warning Using Teensy 3.6
    #define BOARD_NAME  "Teensy 3.6"
  #elif defined(__MK64FX512__)
    // For Teensy 3.5
    #warning Using Teensy 3.5
    #define BOARD_NAME  "Teensy 3.5"
  #elif defined(__MK20DX256__)
    // For Teensy 3.2/3.1
    #warning Using Teensy 3.2/3.1
    #define BOARD_NAME  "Teensy 3.2/3.1"
  #elif defined(__MK20DX128__)
    // For Teensy 3.0
    #warning Using Teensy 3.0
    #define BOARD_NAME  "Teensy 3.0"
  #elif defined(__MKL26Z64__)
    // For Teensy LC
    #warning Using Teensy LC
    #define BOARD_NAME  "Teensy LC"
  #endif

  #if defined(KINETISK)
    #define F_TIMER F_BUS
  #elif defined(KINETISL)
    #define F_TIMER (F_PLL/2)
  #endif

  ////////////////////////////////////////

  // For Teensy 2.0 and Teensy++ 2.0

#elif ( defined(ARDUINO_ARCH_AVR) || defined(__AVR__) )

  #error Not yet support board

  #define USING_TEENSY2_PWM       true

  #ifdef BOARD_NAME
    #undef BOARD_NAME
  #endif

  // For Teensy 2.0 and Teensy++ 2.0
  #warning Using Teensy 2.0 or Teensy++ 2.0
  #define BOARD_NAME  "Teensy 2.0 or Teensy++ 2.0"

  #if defined(KINETISK)
    #define F_TIMER F_BUS
  #elif defined(KINETISL)
    #define F_TIMER (F_PLL/2)
  #endif

  ////////////////////////////////////////

#else

  #error Not support board

#endif

////////////////////////////////////////

#if (_PWM_LOGLEVEL_ > 3)
  #warning Using Teensy Hardware PWM
#endif

////////////////////////////////////////

#ifndef TEENSY_PWM_VERSION
  #define TEENSY_PWM_VERSION             F("Teensy_PWM v1.1.1")

  #define TEENSY_PWM_VERSION_MAJOR       1
  #define TEENSY_PWM_VERSION_MINOR       1
  #define TEENSY_PWM_VERSION_PATCH       1

  #define TEENSY_PWM_VERSION_INT         1001001
#endif

////////////////////////////////////////

#include "Arduino.h"

#include "PWM_Generic_Debug.h"

////////////////////////////////////////

// Teensy4x
/*************************************
  // Known usage of FlexPWM and QuadTimers
  // -------------------------------------
  //   FlexPWM1_0    PWM pin 1, 36(T4.0), 37(T4.0), 44(T4.1), 45(T4.1)
  //   FlexPWM1_1    PWM pin 0, 34(T4.0), 35(T4.0), 42(T4.1), 43(T4.1)
  //   FlexPWM1_2    PWM pin 24, 38(T4.0), 39(T4.0), 46(T4.1), 47(T4.1)
  //   FlexPWM1_3    PWM pin 7, 8, 25
  //   FlexPWM2_0    PWM pin 4, 33
  //   FlexPWM2_1    PWM pin 5, Tlc5940 library
  //   FlexPWM2_2    PWM pin 6, 9
  //   FlexPWM2_3    PWM pin 36(T4.1), 37(T4.1)
  //   FlexPWM3_0    PWM pin 53(T4.1)
  //   FlexPWM3_1    PWM pin 28, 29
  //   FlexPWM3_2
  //   FlexPWM3_3    PWM pin 51(T4.1)
  //   FlexPWM4_0    PWM pin 22
  //   FlexPWM4_1    PWM pin 23
  //   FlexPWM4_2    PWM pin 2, 3, Tlc5940 library
  //   FlexPWM4_3
  //   QuadTimer1_0  PWM pin 10
  //   QuadTimer1_1  PWM pin 12
  //   QuadTimer1_2  PWM pin 11
  //   QuadTimer1_3
  //   QuadTimer2_0  PWM pin 13
  //   QuadTimer2_1
  //   QuadTimer2_2
  //   QuadTimer2_3
  //   QuadTimer3_0  PWM pin 19
  //   QuadTimer3_1  PWM pin 18
  //   QuadTimer3_2  PWM pin 14
  //   QuadTimer3_3  PWM pin 15
  //   QuadTimer4_0  OctoWS2811, ADC library
  //   QuadTimer4_1  OctoWS2811
  //   QuadTimer4_2  OctoWS2811
  //   QuadTimer4_3  AudioInputAnalog, ADC library

  ////////////////////////////////////////

*************************************/

////////////////////////////////////////

#define INVALID_TEENSY_PIN         255

////////////////////////////////////////

#define MAX_COUNT_16BIT           65536UL

#define MAX_16BIT                 65535UL

////////////////////////////////////////
////////////////////////////////////////

#if USING_TEENSY4_PWM

#include "imxrt.h"
#include "core_pins.h"

////////////////////////////////////////

extern "C"
{
  void pwm_init();
  void flexpwmWrite(IMXRT_FLEXPWM_t *p, unsigned int submodule, uint8_t channel, uint16_t val);
  void flexpwmFrequency(IMXRT_FLEXPWM_t *p, unsigned int submodule, uint8_t channel, float frequency);

  void quadtimerWrite(IMXRT_TMR_t *p, unsigned int submodule, uint16_t val);
  void quadtimerFrequency(IMXRT_TMR_t *p, unsigned int submodule, float frequency);
  void analogWriteFrequency(uint8_t pin, float frequency);
}

////////////////////////////////////////

// From teensy/avr/cores/teensy4/pwm.c

typedef struct pwm_pin_info_struct
{
  uint8_t type;    // 0=no pwm, 1=flexpwm, 2=quad
  uint8_t module;  // 0-3, 0-3
  uint8_t channel; // 0=X, 1=A, 2=B
  uint8_t muxval;  //
} PWM_pin_info;

////////////////////////////////////////

#define M(a, b) ((((a) - 1) << 4) | (b))

////////////////////////////////////////

static const PWM_pin_info pwm_pin_info[] =
{
  {1, M(1, 1), 0, 4},  // FlexPWM1_1_X   0  // AD_B0_03
  {1, M(1, 0), 0, 4},  // FlexPWM1_0_X   1  // AD_B0_02
  {1, M(4, 2), 1, 1},  // FlexPWM4_2_A   2  // EMC_04
  {1, M(4, 2), 2, 1},  // FlexPWM4_2_B   3  // EMC_05
  {1, M(2, 0), 1, 1},  // FlexPWM2_0_A   4  // EMC_06
  {1, M(2, 1), 1, 1},  // FlexPWM2_1_A   5  // EMC_08
  {1, M(2, 2), 1, 2},  // FlexPWM2_2_A   6  // B0_10
  {1, M(1, 3), 2, 6},  // FlexPWM1_3_B   7  // B1_01
  {1, M(1, 3), 1, 6},  // FlexPWM1_3_A   8  // B1_00
  {1, M(2, 2), 2, 2},  // FlexPWM2_2_B   9  // B0_11
  {2, M(1, 0), 0, 1},  // QuadTimer1_0  10  // B0_00
  {2, M(1, 2), 0, 1},  // QuadTimer1_2  11  // B0_02
  {2, M(1, 1), 0, 1},  // QuadTimer1_1  12  // B0_01
  {2, M(2, 0), 0, 1},  // QuadTimer2_0  13  // B0_03
  {2, M(3, 2), 0, 1},  // QuadTimer3_2  14  // AD_B1_02
  {2, M(3, 3), 0, 1},  // QuadTimer3_3  15  // AD_B1_03
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {2, M(3, 1), 0, 1},  // QuadTimer3_1  18  // AD_B1_01
  {2, M(3, 0), 0, 1},  // QuadTimer3_0  19  // AD_B1_00
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {1, M(4, 0), 1, 1},  // FlexPWM4_0_A  22  // AD_B1_08
  {1, M(4, 1), 1, 1},  // FlexPWM4_1_A  23  // AD_B1_09
  {1, M(1, 2), 0, 4},  // FlexPWM1_2_X  24  // AD_B0_12
  {1, M(1, 3), 0, 4},  // FlexPWM1_3_X  25  // AD_B0_13
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {1, M(3, 1), 2, 1},  // FlexPWM3_1_B  28  // EMC_32
  {1, M(3, 1), 1, 1},  // FlexPWM3_1_A  29  // EMC_31
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {1, M(2, 0), 2, 1},  // FlexPWM2_0_B  33  // EMC_07
#ifdef ARDUINO_TEENSY40
  {1, M(1, 1), 2, 1},  // FlexPWM1_1_B  34  // SD_B0_03
  {1, M(1, 1), 1, 1},  // FlexPWM1_1_A  35  // SD_B0_02
  {1, M(1, 0), 2, 1},  // FlexPWM1_0_B  36  // SD_B0_01
  {1, M(1, 0), 1, 1},  // FlexPWM1_0_A  37  // SD_B0_00
  {1, M(1, 2), 2, 1},  // FlexPWM1_2_B  38  // SD_B0_05
  {1, M(1, 2), 1, 1},  // FlexPWM1_2_A  39  // SD_B0_04
#endif
#ifdef ARDUINO_TEENSY41
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {1, M(2, 3), 1, 6},  // FlexPWM2_3_A  36  // B1_00
  {1, M(2, 3), 2, 6},  // FlexPWM2_3_B  37  // B1_01
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {1, M(1, 1), 2, 1},  // FlexPWM1_1_B  42  // SD_B0_03
  {1, M(1, 1), 1, 1},  // FlexPWM1_1_A  43  // SD_B0_02
  {1, M(1, 0), 2, 1},  // FlexPWM1_0_B  44  // SD_B0_01
  {1, M(1, 0), 1, 1},  // FlexPWM1_0_A  45  // SD_B0_00
  {1, M(1, 2), 2, 1},  // FlexPWM1_2_B  46  // SD_B0_05
  {1, M(1, 2), 1, 1},  // FlexPWM1_2_A  47  // SD_B0_04
  {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_0_B
  {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_2_A
  {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_2_B
  {1, M(3, 3), 2, 1},  // FlexPWM3_3_B  51  // EMC_22
  {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_1_B
  {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_1_A
  {1, M(3, 0), 1, 1},  // FlexPWM3_0_A  54  // EMC_29
#endif
#ifdef ARDUINO_TEENSY_MICROMOD
  {1, M(1, 1), 2, 1},  // FlexPWM1_1_B  34  // SD_B0_03
  {1, M(1, 1), 1, 1},  // FlexPWM1_1_A  35  // SD_B0_02
  {1, M(1, 0), 2, 1},  // FlexPWM1_0_B  36  // SD_B0_01
  {1, M(1, 0), 1, 1},  // FlexPWM1_0_A  37  // SD_B0_00
  {1, M(1, 2), 1, 1},  // FlexPWM1_2_A  38  // SD_B0_04
  {1, M(1, 2), 2, 1},  // FlexPWM1_2_B  39  // SD_B0_05
  {2, M(2, 1), 0, 1},  // QuadTimer2_1  40  // B0_04
  {2, M(2, 2), 0, 1},  // QuadTimer2_2  41  // B0_05
  {0, M(1, 0), 0, 0},  // duplicate QuadTimer3_0
  {0, M(1, 0), 0, 0},  // duplicate QuadTimer3_1
  {0, M(1, 0), 0, 0},  // duplicate QuadTimer3_2
  {2, M(4, 0), 0, 1},  // QuadTimer4_0  45  // B0_09
#endif
};

////////////////////////////////////////
////////////////////////////////////////

typedef enum
{
  FLEXPWM1_0 = 0,
  FLEXPWM1_1,
  FLEXPWM1_2,
  FLEXPWM1_3,
  FLEXPWM2_0,
  FLEXPWM2_1,
  FLEXPWM2_2,
  FLEXPWM2_3,
  FLEXPWM3_0,
  FLEXPWM3_1,
  FLEXPWM3_2,
  FLEXPWM3_3,
  FLEXPWM4_0,
  FLEXPWM4_1,
  FLEXPWM4_2,
  FLEXPWM4_3,
  MAX_FLEX_PWM_CHANNELS
} FlexChannel;

////////////////////////////////////////

typedef enum
{
  QTMR1_0 = 0,
  QTMR1_1,
  QTMR1_2,
  QTMR2_0,
  QTMR2_1,
  QTMR2_2,
  QTMR3_0,
  QTMR3_1,
  QTMR3_2,
  QTMR4_0,
  QTMR4_1,
  QTMR4_2,
  MAX_QTIMER_PWM_CHANNELS
} QuadChannel;

////////////////////////////////////////

static uint32_t curFlexFrequency  [MAX_FLEX_PWM_CHANNELS]   = { 0 };
static uint32_t curQTimerFrequency[MAX_QTIMER_PWM_CHANNELS] = { 0 };

////////////////////////////////////////

class Teensy_PWM
{
  public:

    // dutycycle = 0.0f - 100.0f => 0-65535
    Teensy_PWM(const uint32_t& pin, const float& frequency, const float& dutycycle)
    {
      _dutycycle  = round(map(dutycycle, 0, 100.0f, 0, MAX_COUNT_16BIT));

      analogWriteRes(_resolution);


      // input DC is 16-bit
      _PWMEnabled = setupPWM(pin, (uint32_t) frequency, _dutycycle);
    }

    ///////////////////////////////////////////

    ~Teensy_PWM()
    {

    }

    ///////////////////////////////////////////
    ///////////////////////////////////////////

  private:

    IMXRT_FLEXPWM_t*  _flexpwm = nullptr;
    IMXRT_TMR_t*      _qtimer  = nullptr;

    ///////////////////////////////////////////

    // input dutycycle is 16-bit. To be mapped to 15 bit
    // TODO: set frequency by calc period + TOP

    bool setupPWM(uint32_t pin, uint32_t frequency, uint16_t dutycycle)
    {
      // Map dutycycle to _resolution
      uint16_t newDC = round(map(dutycycle, 0, MAX_COUNT_16BIT, 0, 1 << _resolution));

      PWM_LOGDEBUG5(F("setupPWM: Mapping dutycycle ="), dutycycle, F("to newDC ="), newDC, F("for _resolution ="),
                    _resolution);

      //////////////////////////////////

      const PWM_pin_info *info;

      if (pin >= CORE_NUM_DIGITAL)
      {
        PWM_LOGDEBUG1(F("setupPWM: Wrong pin ="), pin);

        return false;
      }

      info = pwm_pin_info + pin;

      uint8_t PWMTimer = ((info->module >> 4) & 3);

      uint8_t moduleIndex = ((info->module) & 3);

      if (info->type == 1)
      {
        if (!_flexpwm)
        {
          // FlexPWM pin
          switch (PWMTimer)
          {
            case 0:
              PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer1 moduleIndex ="), moduleIndex, F("for PWM pin ="), pin);

              _flexpwm = &IMXRT_FLEXPWM1;

              if (curFlexFrequency[FLEXPWM1_0 + moduleIndex] == 0)
                curFlexFrequency[FLEXPWM1_0 + moduleIndex] = frequency;
              else
              {
                PWM_LOGERROR1(F("setupPWM: FlexTimer1 already used PWM Frequency ="), curFlexFrequency[FLEXPWM1_0 + moduleIndex]);
              }

              break;

            case 1:
              PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer2 moduleIndex ="), moduleIndex, F("for PWM pin ="), pin);

              _flexpwm = &IMXRT_FLEXPWM2;

              if (curFlexFrequency[FLEXPWM2_0 + moduleIndex] == 0)
                curFlexFrequency[FLEXPWM2_0 + moduleIndex] = frequency;
              else
              {
                PWM_LOGERROR1(F("setupPWM: FlexTimer2 already used PWM Frequency ="), curFlexFrequency[FLEXPWM2_0 + moduleIndex]);
              }

              break;

            case 2:
              PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer3 moduleIndex ="), moduleIndex, F("for PWM pin ="), pin);

              _flexpwm = &IMXRT_FLEXPWM3;

              if (curFlexFrequency[FLEXPWM3_0 + moduleIndex] == 0)
                curFlexFrequency[FLEXPWM3_0 + moduleIndex] = frequency;
              else
              {
                PWM_LOGERROR1(F("setupPWM: FlexTimer3 already used PWM Frequency ="), curFlexFrequency[FLEXPWM3_0 + moduleIndex]);
              }

              break;

            default:
              PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer4 moduleIndex ="), moduleIndex, F("for PWM pin ="), pin);

              _flexpwm = &IMXRT_FLEXPWM4;

              if (curFlexFrequency[FLEXPWM4_0 + moduleIndex] == 0)
                curFlexFrequency[FLEXPWM4_0 + moduleIndex] = frequency;
              else
              {
                PWM_LOGERROR1(F("setupPWM: FlexTimer4 already used PWM Frequency ="), curFlexFrequency[FLEXPWM4_0 + moduleIndex]);
              }
          }
        }

        if (_frequency != frequency)
        {
          // only for new Freq
          flexpwmFrequency(_flexpwm, info->module & 0x03, info->channel, frequency);

          _frequency = frequency;
        }

        flexpwmWrite(_flexpwm, info->module & 0x03, info->channel, newDC);

      }
      else if (info->type == 2)
      {
        if (!_qtimer)
        {
          // QuadTimer pin
          switch (PWMTimer)
          {
            case 0:
              PWM_LOGDEBUG3(F("setupPWM: Using QuadTimer1 moduleIndex ="), moduleIndex, F("for PWM pin ="), pin);

              _qtimer = &IMXRT_TMR1;

              if (curQTimerFrequency[QTMR1_0 + moduleIndex] == 0)
                curQTimerFrequency[QTMR1_0 + moduleIndex] = frequency;
              else
              {
                PWM_LOGERROR1(F("setupPWM: QuadTimer1 already used PWM Frequency ="), curQTimerFrequency[QTMR1_0 + moduleIndex]);
              }

              break;

            case 1:
              PWM_LOGDEBUG3(F("setupPWM: Using QuadTimer2 moduleIndex ="), moduleIndex, F("for PWM pin ="), pin);

              _qtimer = &IMXRT_TMR2;

              if (curQTimerFrequency[QTMR2_0 + moduleIndex] == 0)
                curQTimerFrequency[QTMR2_0 + moduleIndex] = frequency;
              else
              {
                PWM_LOGERROR1(F("setupPWM: QuadTimer2 already used PWM Frequency ="), curQTimerFrequency[QTMR2_0 + moduleIndex]);
              }

              break;

            case 2:
              PWM_LOGDEBUG3(F("setupPWM: Using QuadTimer3 moduleIndex ="), moduleIndex, F("for PWM pin ="), pin);
              _qtimer = &IMXRT_TMR3;

              if (curQTimerFrequency[QTMR3_0 + moduleIndex] == 0)
                curQTimerFrequency[QTMR3_0 + moduleIndex] = frequency;
              else
              {
                PWM_LOGERROR1(F("setupPWM: QuadTimer3 already used PWM Frequency ="), curQTimerFrequency[QTMR3_0 + moduleIndex]);
              }

              break;

            default:
              PWM_LOGDEBUG3(F("setupPWM: Using QuadTimer4 moduleIndex ="), moduleIndex, F("for PWM pin ="), pin);

              _qtimer = &IMXRT_TMR4;

              if (curQTimerFrequency[QTMR4_0 + moduleIndex] == 0)
                curQTimerFrequency[QTMR4_0 + moduleIndex] = frequency;
              else
              {
                PWM_LOGERROR1(F("setupPWM: QuadTimer4 already used PWM Frequency ="), curQTimerFrequency[QTMR4_0 + moduleIndex]);
              }
          }
        }

        if (_frequency != frequency)
        {
          // only for new Freq
          quadtimerFrequency(_qtimer, info->module & 0x03, frequency);

          _frequency = frequency;
        }

        quadtimerWrite(_qtimer, info->module & 0x03, newDC);
      }
      else
      {
        PWM_LOGDEBUG(F("setupPWM: No free PWM timer"));

        return false;
      }

      *(portConfigRegister(pin)) = info->muxval;

      //////////////////////////////////

      _pin = pin;

      return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////

#elif USING_TEENSY3_PWM

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


#include "pins_arduino.h"
#include "core_pins.h"

////////////////////////////////////////

// From teensy/avr/cores/teensy3/pin_teensy.c

/*********************************************************************************
  #if defined(KINETISK)
  #define F_TIMER F_BUS
  #elif defined(KINETISL)

  #if F_CPU > 16000000
    #define F_TIMER (F_PLL/2)
  #else
    #define F_TIMER (F_PLL)
  #endif//Low Power

  #endif

  ////////////////////////////////////////

  #if F_TIMER == 128000000
  #define DEFAULT_FTM_MOD       (0xFFFF)
  #define DEFAULT_FTM_PRESCALE  2
  #elif F_TIMER == 120000000
  #define DEFAULT_FTM_MOD       (0xEFFF)
  #define DEFAULT_FTM_PRESCALE  2
  #elif F_TIMER == 108000000
  #define DEFAULT_FTM_MOD       (0xD7FF)
  #define DEFAULT_FTM_PRESCALE  2
  #elif F_TIMER == 96000000
  #define DEFAULT_FTM_MOD       (0xBFFF)
  #define DEFAULT_FTM_PRESCALE  2
  #elif F_TIMER == 90000000
  #define DEFAULT_FTM_MOD       (0xB3FF)
  #define DEFAULT_FTM_PRESCALE  2
  #elif F_TIMER == 80000000
  #define DEFAULT_FTM_MOD       (0x9FFF)
  #define DEFAULT_FTM_PRESCALE  2
  #elif F_TIMER == 72000000
  #define DEFAULT_FTM_MOD       (0x8FFF)
  #define DEFAULT_FTM_PRESCALE  2
  #elif F_TIMER == 64000000
  #define DEFAULT_FTM_MOD       (0xFFFF)
  #define DEFAULT_FTM_PRESCALE  1
  #elif F_TIMER == 60000000
  #define DEFAULT_FTM_MOD       (0xEFFF)
  #define DEFAULT_FTM_PRESCALE  1
  #elif F_TIMER == 56000000
  #define DEFAULT_FTM_MOD       (0xDFFF)
  #define DEFAULT_FTM_PRESCALE  1
  #elif F_TIMER == 54000000
  #define DEFAULT_FTM_MOD       (0xD7FF)
  #define DEFAULT_FTM_PRESCALE  
  #elif F_TIMER == 48000000
  #define DEFAULT_FTM_MOD       (0xBFFF)
  #define DEFAULT_FTM_PRESCALE  1
  #elif F_TIMER == 40000000
  #define DEFAULT_FTM_MOD       (0x9FFF)
  #define DEFAULT_FTM_PRESCALE  1
  #elif F_TIMER == 36000000
  #define DEFAULT_FTM_MOD       (0x8FFF)
  #define DEFAULT_FTM_PRESCALE  1
  #elif F_TIMER == 24000000
  #define DEFAULT_FTM_MOD       (0xBFFF)
  #define DEFAULT_FTM_PRESCALE  0
  #elif F_TIMER == 16000000
  #define DEFAULT_FTM_MOD       (0x7FFF)
  #define DEFAULT_FTM_PRESCALE  0
  #elif F_TIMER == 8000000
  #define DEFAULT_FTM_MOD       (0x3FFF)
  #define DEFAULT_FTM_PRESCALE  0
  #elif F_TIMER == 4000000
  #define DEFAULT_FTM_MOD       (0x1FFF)
  #define DEFAULT_FTM_PRESCALE  0
  #elif F_TIMER == 2000000
  #define DEFAULT_FTM_MOD       (0x0FFF)
  #define DEFAULT_FTM_PRESCALE  0
  #endif

*********************************************************************************/

////////////////////////////////////////

#if defined(__MK20DX128__)

#define FTM0_CH0_PIN 22
#define FTM0_CH1_PIN 23
#define FTM0_CH2_PIN  9
#define FTM0_CH3_PIN 10
#define FTM0_CH4_PIN  6
#define FTM0_CH5_PIN 20
#define FTM0_CH6_PIN 21
#define FTM0_CH7_PIN  5
#define FTM1_CH0_PIN  3
#define FTM1_CH1_PIN  4

#elif defined(__MK20DX256__)

#define FTM0_CH0_PIN 22
#define FTM0_CH1_PIN 23
#define FTM0_CH2_PIN  9
#define FTM0_CH3_PIN 10
#define FTM0_CH4_PIN  6
#define FTM0_CH5_PIN 20
#define FTM0_CH6_PIN 21
#define FTM0_CH7_PIN  5
#define FTM1_CH0_PIN  3
#define FTM1_CH1_PIN  4
#define FTM2_CH0_PIN 32
#define FTM2_CH1_PIN 25

#elif defined(__MKL26Z64__)

#define FTM0_CH0_PIN 22
#define FTM0_CH1_PIN 23
#define FTM0_CH2_PIN  9
#define FTM0_CH3_PIN 10
#define FTM0_CH4_PIN  6
#define FTM0_CH5_PIN 20
#define FTM1_CH0_PIN 16
#define FTM1_CH1_PIN 17
#define FTM2_CH0_PIN  3
#define FTM2_CH1_PIN  4

#elif defined(__MK64FX512__)

#define FTM0_CH0_PIN 22
#define FTM0_CH1_PIN 23
#define FTM0_CH2_PIN  9
#define FTM0_CH3_PIN 10
#define FTM0_CH4_PIN  6
#define FTM0_CH5_PIN 20
#define FTM0_CH6_PIN 21
#define FTM0_CH7_PIN  5
#define FTM1_CH0_PIN  3
#define FTM1_CH1_PIN  4
#define FTM2_CH0_PIN 29
#define FTM2_CH1_PIN 30
#define FTM3_CH0_PIN  2
#define FTM3_CH1_PIN 14
#define FTM3_CH2_PIN  7
#define FTM3_CH3_PIN  8
#define FTM3_CH4_PIN 35
#define FTM3_CH5_PIN 36
#define FTM3_CH6_PIN 37
#define FTM3_CH7_PIN 38

#elif defined(__MK66FX1M0__)

#define FTM0_CH0_PIN 22
#define FTM0_CH1_PIN 23
#define FTM0_CH2_PIN  9
#define FTM0_CH3_PIN 10
#define FTM0_CH4_PIN  6
#define FTM0_CH5_PIN 20
#define FTM0_CH6_PIN 21
#define FTM0_CH7_PIN  5
#define FTM1_CH0_PIN  3
#define FTM1_CH1_PIN  4
#define FTM2_CH0_PIN 29
#define FTM2_CH1_PIN 30
#define FTM3_CH0_PIN  2
#define FTM3_CH1_PIN 14
#define FTM3_CH2_PIN  7
#define FTM3_CH3_PIN  8
#define FTM3_CH4_PIN 35
#define FTM3_CH5_PIN 36
#define FTM3_CH6_PIN 37
#define FTM3_CH7_PIN 38
#define TPM1_CH0_PIN 16
#define TPM1_CH1_PIN 17

#endif

////////////////////////////////////////

#define FTM_PINCFG(pin)   FTM_PINCFG2(pin)
#define FTM_PINCFG2(pin)  CORE_PIN ## pin ## _CONFIG

////////////////////////////////////////
////////////////////////////////////////

#define Teensy3_PWM       Teensy_PWM

class Teensy3_PWM
{
  public:

    // dutycycle = 0.0f - 100.0f => 0-65535
    Teensy3_PWM(const uint32_t& pin, const float& frequency, const float& dutycycle)
    {
      _dutycycle  = round(map(dutycycle, 0, 100.0f, 0, MAX_COUNT_16BIT));

      analogWriteRes(_resolution);


      // input DC is 16-bit
      _PWMEnabled = setupPWM(pin, (uint32_t) frequency, _dutycycle);
    }

    ///////////////////////////////////////////

    ~Teensy3_PWM()
    {

    }

    ///////////////////////////////////////////
    ///////////////////////////////////////////

  private:

    ///////////////////////////////////////////

    // input dutycycle is 16-bit. To be mapped to 15 bit
    // TODO: set frequency by calc period + TOP

    bool setupPWM(uint32_t pin, uint32_t frequency, uint16_t dutycycle)
    {
      // Map dutycycle to _resolution
      uint32_t newDC = round(map(dutycycle, 0, MAX_COUNT_16BIT, 0, 1 << _resolution));

      PWM_LOGDEBUG5(F("setupPWM: Mapping dutycycle ="), dutycycle, F("to newDC ="), newDC, F("for _resolution ="),
                    _resolution);

      //////////////////////////////////

      if (_frequency != frequency)
      {
        // only for new Freq
        analogWriteFrequency(pin, frequency);

        _frequency = frequency;
      }

      //////////////////////////////////

      uint32_t timerValue;

      if (pin == FTM1_CH0_PIN || pin == FTM1_CH1_PIN)
      {     	
        timerValue = (newDC * (uint32_t)(FTM1_MOD + 1)) >> _resolution;

#if defined(FTM2_CH0_PIN)
      }
      else if (pin == FTM2_CH0_PIN || pin == FTM2_CH1_PIN)
      {      	
        timerValue = (newDC * (uint32_t)(FTM2_MOD + 1)) >> _resolution;
#endif

#if defined(FTM3_CH0_PIN)
      }
      else if (pin == FTM3_CH0_PIN || pin == FTM3_CH1_PIN || pin == FTM3_CH2_PIN ||
               pin == FTM3_CH3_PIN || pin == FTM3_CH4_PIN || pin == FTM3_CH5_PIN ||
               pin == FTM3_CH6_PIN || pin == FTM3_CH7_PIN)
      {     	
        timerValue = (newDC * (uint32_t)(FTM3_MOD + 1)) >> _resolution;
#endif

#if defined(TPM1_CH0_PIN)
      }
      else if (pin == TPM1_CH0_PIN || pin == TPM1_CH1_PIN)
      {
        timerValue = (newDC * (uint32_t)(TPM1_MOD + 1)) >> _resolution;
#endif
      }
      else
      {      	
        timerValue = (newDC * (uint32_t)(FTM0_MOD + 1)) >> _resolution;
      }

      switch (pin)
      {
#ifdef FTM0_CH0_PIN

        case FTM0_CH0_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer0, CH0 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM0_C0V = timerValue;
          FTM_PINCFG(FTM0_CH0_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM0_CH1_PIN

        case FTM0_CH1_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer0, CH1 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM0_C1V = timerValue;
          FTM_PINCFG(FTM0_CH1_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM0_CH2_PIN

        case FTM0_CH2_PIN:
        PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer0, CH2 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM0_C2V = timerValue;
          FTM_PINCFG(FTM0_CH2_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM0_CH3_PIN

        case FTM0_CH3_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer0, CH3 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM0_C3V = timerValue;
          FTM_PINCFG(FTM0_CH3_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM0_CH4_PIN

        case FTM0_CH4_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer0, CH4 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM0_C4V = timerValue;
          FTM_PINCFG(FTM0_CH4_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM0_CH5_PIN

        case FTM0_CH5_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer0, CH5 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM0_C5V = timerValue;
          FTM_PINCFG(FTM0_CH5_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM0_CH6_PIN

        case FTM0_CH6_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer0, CH6 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM0_C6V = timerValue;
          FTM_PINCFG(FTM0_CH6_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM0_CH7_PIN

        case FTM0_CH7_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer0, CH7 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM0_C7V = timerValue;
          FTM_PINCFG(FTM0_CH7_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM1_CH0_PIN

        case FTM1_CH0_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer1, CH0 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM1_C0V = timerValue;
          FTM_PINCFG(FTM1_CH0_PIN) = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM1_CH1_PIN

        case FTM1_CH1_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer1, CH1 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM1_C1V = timerValue;
          FTM_PINCFG(FTM1_CH1_PIN) = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM2_CH0_PIN

        case FTM2_CH0_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer2, CH0 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM2_C0V = timerValue;
          FTM_PINCFG(FTM2_CH0_PIN) = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM2_CH1_PIN

        case FTM2_CH1_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer2, CH1 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM2_C1V = timerValue;
          FTM_PINCFG(FTM2_CH1_PIN) = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM3_CH0_PIN

        case FTM3_CH0_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer3, CH0 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM3_C0V = timerValue;
          FTM_PINCFG(FTM3_CH0_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM3_CH1_PIN

        case FTM3_CH1_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer3, CH1 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM3_C1V = timerValue;
          FTM_PINCFG(FTM3_CH1_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM3_CH2_PIN

        case FTM3_CH2_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer3, CH2 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM3_C2V = timerValue;
          FTM_PINCFG(FTM3_CH2_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM3_CH3_PIN

        case FTM3_CH3_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer3, CH3 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM3_C3V = timerValue;
          FTM_PINCFG(FTM3_CH3_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM3_CH4_PIN

        case FTM3_CH4_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer3, CH4 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM3_C4V = timerValue;
          FTM_PINCFG(FTM3_CH4_PIN) = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM3_CH5_PIN

        case FTM3_CH5_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer3, CH5 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM3_C5V = timerValue;
          FTM_PINCFG(FTM3_CH5_PIN) = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM3_CH6_PIN

        case FTM3_CH6_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer3, CH6 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM3_C6V = timerValue;
          FTM_PINCFG(FTM3_CH6_PIN) = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef FTM3_CH7_PIN

        case FTM3_CH7_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using FlexTimer3, CH7 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          FTM3_C7V = timerValue;
          FTM_PINCFG(FTM3_CH7_PIN) = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef TPM1_CH0_PIN

        case TPM1_CH0_PIN:
         	PWM_LOGDEBUG3(F("setupPWM: Using T_PWM1, CH0 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          TPM1_C0V = timerValue;
          FTM_PINCFG(TPM1_CH0_PIN) = PORT_PCR_MUX(6) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

#ifdef TPM1_CH1_PIN

        case TPM1_CH1_PIN:
        	PWM_LOGDEBUG3(F("setupPWM: Using T_PWM1, CH1 for PWM pin ="), pin, F("timerValue ="), timerValue);
        	
          TPM1_C1V = timerValue;
          FTM_PINCFG(TPM1_CH1_PIN) = PORT_PCR_MUX(6) | PORT_PCR_DSE | PORT_PCR_SRE;
          break;
#endif

        default:
          digitalWrite(pin, (_dutycycle > (MAX_16BIT >> 1) ) ? HIGH : LOW);

          pinMode(pin, OUTPUT);
      }

      //////////////////////////////////

      _pin = pin;

      return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////


#elif USING_TEENSY2_PWM

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#endif      // USING_TEENSY4_PWM

    ///////////////////////////////////////////
    ///////////////////////////////////////////

  public:

    // dutycycle from 0-65535 for 0%-100%
    bool setPWM_Int(const uint8_t& pin, const float& frequency, const uint16_t& dutycycle)
    {
      if (pin != _pin)
        return false;

      PWM_LOGINFO3(F("setPWM_Int: dutycycle ="), dutycycle, F(", frequency ="), frequency);

      return setupPWM(pin, (uint32_t) frequency, dutycycle);
    }

    ///////////////////////////////////////////

    bool setPWM()
    {
      return setPWM_Int(_pin, _frequency, _dutycycle);
    }

    ///////////////////////////////////////////

    bool setPWM(const uint8_t& pin, const float& frequency, const float& dutycycle)
    {
      _dutycycle = round(map(dutycycle, 0, 100.0f, 0, MAX_COUNT_16BIT));

      PWM_LOGDEBUG3(F("setPWM: _dutycycle ="), _dutycycle, F(", frequency ="), frequency);

      return setPWM_Int(pin, frequency, _dutycycle);
    }

    ///////////////////////////////////////////

    bool setPWM_Period(const uint8_t& pin, const float& period_us, const float& dutycycle)
    {
      _dutycycle = round(map(dutycycle, 0, 100.0f, 0, MAX_COUNT_16BIT));

      PWM_LOGDEBUG3(F("setPWM_Period: _dutycycle ="), _dutycycle, F(", period_us ="), period_us);

      return setPWM_Int(pin, round(1000000.0f / period_us), _dutycycle);
    }

    ///////////////////////////////////////////

    // Must have same frequency
    bool setPWM_manual(const uint8_t& pin, const uint16_t& DCValue)
    {
      // Not the same pin or overvalue
      if (_pin != pin)
        return false;

      _dutycycle = DCValue;

      PWM_LOGINFO3(F("setPWM_manual: _dutycycle ="), _dutycycle, F(", frequency ="), _frequency);

      return setupPWM(pin, _frequency, DCValue);
    }

    ///////////////////////////////////////////

    inline float getActualDutyCycle()
    {
      return ( ( (float) _dutycycle + 1 ) * 100 / MAX_16BIT );
    }

    ///////////////////////////////////////////

    inline float getActualFreq()
    {
      return _frequency;
    }

    ///////////////////////////////////////////

    inline float getPWMPeriod()
    {
      return (1000000.0f / _frequency);
    }

    ///////////////////////////////////////////

    inline uint32_t get_freq_CPU()
    {
      return F_CPU;
    }

    ///////////////////////////////////////////

    inline uint32_t getPin()
    {
      return _pin;
    }

    ///////////////////////////////////////////

    void setResolution(const uint8_t& resolution)
    {
      // Must be <= 16-bit and >= 8-bit
      if ( (resolution <= 16) && (resolution >= 8) )
      {
        _resolution = resolution;

        analogWriteRes(_resolution);
      }
    }

    ///////////////////////////////////////////

    bool isPWMEnabled()
    {
      return _PWMEnabled;
    }

    ///////////////////////////////////////////////////////////////////

  private:

    float             _frequency = 0;

    // dutycycle from 0-65535 for 0%-100%
    uint16_t          _dutycycle;

    uint8_t           _pin = INVALID_TEENSY_PIN;

    // In number of bits
    uint8_t           _resolution = 16;

    bool              _PWMEnabled = false;

    ///////////////////////////////////////////
};

///////////////////////////////////////////


#endif    // TEENSY_PWM_H

