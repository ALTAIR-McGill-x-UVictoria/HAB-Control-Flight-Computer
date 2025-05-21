/*
  Using the BNO080 IMU with helper methods
  By: Guillaume Walck

  Date: September 08th, 2020
  License: This code is public domain.

  This example shows how to access multiple data of one type using helper methods.

  It takes about 1ms at 400kHz I2C to read a record from the sensor, but we are polling the sensor continually
  between updates from the sensor. Use the interrupt pin on the BNO080 breakout to avoid polling.

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 9600 baud to serial monitor.
*/

#include <Arduino.h>
#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

long lastSensorTime = 0;

void printAccuracyLevel(byte accuracyNumber);

void setup()
{
  Serial.println();
  Serial.println("BNO080 Multiple Read Example");

  Wire1.begin();

  //Are you using a ESP? Check this issue for more information: https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/issues/16
//  //=================================
//  delay(100); //  Wait for BNO to boot
//  // Start i2c and BNO080
//  Wire.flush();   // Reset I2C
//  IMU.begin(BNO080_DEFAULT_ADDRESS, Wire);
//  Wire.begin(4, 5);
//  Wire.setClockStretchLimit(4000);
//  //=================================

  if (myIMU.begin(0x4A, Wire1) == false)
  {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }

  Wire1.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableLinearAccelerometer(50);  // m/s^2 no gravity
  myIMU.enableRotationVector(50);  // quat
  myIMU.enableGyro(50);  // rad/s
  //myIMU.enableMagnetometer(50);  // cannot be enabled at the same time as RotationVector (will not produce data)

  Serial.println(F("LinearAccelerometer enabled, Output in form x, y, z, accuracy, in m/s^2"));
  Serial.println(F("Gyro enabled, Output in form x, y, z, accuracy, in radians per second"));
  Serial.println(F("Rotation vector, Output in form i, j, k, real, accuracy"));
  //Serial.println(F("Magnetometer enabled, Output in form x, y, z, accuracy, in uTesla"));

  // Calibrate
  myIMU.calibrateAll();
}


void loop() {

  //Look for data from the IMU
  if (myIMU.dataAvailable() == true)
  {
    // internal copies of the IMU data
    float ax, ay, az, gx, gy, gz, qx, qy, qz, qw; //  mx, my, mz, (qx, qy, qz, qw = i,j,k, real)
    byte linAccuracy = 0;
    byte gyroAccuracy = 0;
    //byte magAccuracy = 0;
    float quatRadianAccuracy = 0;
    byte quatAccuracy = 0;

    // get IMU data in one go for each sensor type
    myIMU.getLinAccel(ax, ay, az, linAccuracy);
    myIMU.getGyro(gx, gy, gz, gyroAccuracy);
    myIMU.getQuat(qx, qy, qz, qw, quatRadianAccuracy, quatAccuracy);
    //myIMU.getMag(mx, my, mz, magAccuracy);

    float roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
    float pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
    float yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees

    if (millis() - lastSensorTime > 1000)
    {
      Serial.println(F("======================================================"));
      Serial.println(F("BNO080 SENSOR DATA"));
      Serial.println(F("======================================================"));

      lastSensorTime = millis();

      Serial.print(F("acc :"));
        Serial.print(ax, 2);
        Serial.print(F(","));
        Serial.print(ay, 2);
        Serial.print(F(","));
        Serial.print(az, 2);
        Serial.print(F(","));
        Serial.print(az, 2);
        Serial.print(F(","));
        printAccuracyLevel(linAccuracy);

        Serial.print(F("gyro:"));
        Serial.print(gx, 2);
        Serial.print(F(","));
        Serial.print(gy, 2);
        Serial.print(F(","));
        Serial.print(gz, 2);
        Serial.print(F(","));
        printAccuracyLevel(gyroAccuracy);
    /*
        Serial.print(F("mag :"));
        Serial.print(mx, 2);
        Serial.print(F(","));
        Serial.print(my, 2);
        Serial.print(F(","));
        Serial.print(mz, 2);
        Serial.print(F(","));
        printAccuracyLevel(magAccuracy);
    */

        Serial.print(F("quat:"));
        Serial.print(qx, 2);
        Serial.print(F(","));
        Serial.print(qy, 2);
        Serial.print(F(","));
        Serial.print(qz, 2);
        Serial.print(F(","));
        Serial.print(qw, 2);
        Serial.print(F(","));
        printAccuracyLevel(quatAccuracy);

        Serial.print(F("angle:"));
        Serial.print(roll, 2);
        Serial.print(F(","));
        Serial.print(pitch, 2);
        Serial.print(F(","));
        Serial.print(yaw, 2);
        Serial.print(F(","));
        printAccuracyLevel(quatRadianAccuracy);
    }
    
  }
}

//Given an accuracy number, print what it means
void printAccuracyLevel(byte accuracyNumber)
{
  if (accuracyNumber == 0) Serial.println(F("Unreliable"));
  else if (accuracyNumber == 1) Serial.println(F("Low"));
  else if (accuracyNumber == 2) Serial.println(F("Medium"));
  else if (accuracyNumber == 3) Serial.println(F("High"));
}
