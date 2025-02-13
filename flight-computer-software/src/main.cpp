/*
Sources:
  Adafruit_MAX31865 Arduino Library Example
  MS5607 Arduino Library Example
  SparkFun_BNO080_Cortex_Based_IMU Library Example
  ArduTFLite Library Example
*/
#include <Arduino.h>
#include "Sensors.h"

const int led = 13;

void setup() {
  // put your setup code here, to run once:

  pinMode(led, OUTPUT);
  Serial.begin(115200);  // start serial for output
  while (!Serial) delay(10);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(led, HIGH);  
  delay(100);

  Serial.println("\n--------------------------");

  digitalWrite(led, LOW);
  delay(100);
}

