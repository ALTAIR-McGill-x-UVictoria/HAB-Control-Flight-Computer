/*
 *  This is a test program for Sensors, printing the data from the sensors.
 */

#include <Arduino.h>
#include "../../include/Sensors.h"

Sensors sensors;

void setup()
{
  Serial.begin(115200);
  delay(100);
  while (true)
  {
    sensors.begin();
    if (!sensors.status.imu1)
      Serial.println("Failed to initialize IMU 1.");
    if (!sensors.status.imu2)
      Serial.println("Failed to initialize IMU 2.");
    if (!sensors.status.imu3)
      Serial.println("Failed to initialize IMU 3.");
    if (!sensors.status.pressure)
      Serial.println("Failed to initialize Altimeter.");
    if (!sensors.status.temperature)
      Serial.println("Failed to initialize Temperature Probe.");
    if (sensors.status.imu1 && sensors.status.imu2 && sensors.status.imu3 && sensors.status.pressure && sensors.status.temperature)
    {
      Serial.println("Successfully initialized all sensors.");
      break;
    }
    else
    {
      Serial.println("Retrying sensor initialization.");
      delay(100);
    }
  }
  sensors.start();
}

void loop()
{
  float xLinearAcceleration, yLinearAcceleration, zLinearAcceleration;
  sensors.getFusedLinearAcceleration(xLinearAcceleration, yLinearAcceleration, zLinearAcceleration);

  float xAngularVelocity, yAngularVelocity, zAngularVelocity;
  sensors.getFusedAngularVelocity(xAngularVelocity, yAngularVelocity, zAngularVelocity);

  float yawOrientation, pitchOrientation, rollOrientation;
  sensors.getFusedOrientation(yawOrientation, pitchOrientation, rollOrientation);

  float vx, vy, vz;
  sensors.getRelativeVelocity(vx, vy, vz);

  float px, py, pz;
  sensors.getRelativePosition(px, py, pz);

  // Basic IMU data
  Serial.printf("Linear Acceleration IMU1: (x, y, z)=(%.2f, %.2f, %.2f), accuracy=%d\n", 
    sensors.imu1Data.xLinearAcceleration, sensors.imu1Data.yLinearAcceleration, 
    sensors.imu1Data.zLinearAcceleration, sensors.imu1Data.linearAccuracy);
  
  Serial.printf("Linear Acceleration IMU2: (x, y, z)=(%.2f, %.2f, %.2f), accuracy=%d\n", 
    sensors.imu2Data.xLinearAcceleration, sensors.imu2Data.yLinearAcceleration, 
    sensors.imu2Data.zLinearAcceleration, sensors.imu2Data.linearAccuracy);
  
  Serial.printf("Linear Acceleration IMU3: (x, y, z)=(%.2f, %.2f, %.2f), accuracy=%d\n", 
    sensors.imu3Data.xLinearAcceleration, sensors.imu3Data.yLinearAcceleration, 
    sensors.imu3Data.zLinearAcceleration, sensors.imu3Data.linearAccuracy);

  Serial.printf("Angular Velocity IMU1: (x, y, z)=(%.2f, %.2f, %.2f), accuracy=%d\n", 
    sensors.imu1Data.xAngularVelocity, sensors.imu1Data.yAngularVelocity, 
    sensors.imu1Data.zAngularVelocity, sensors.imu1Data.gyroAccuracy);
  
  Serial.printf("Angular Velocity IMU2: (x, y, z)=(%.2f, %.2f, %.2f), accuracy=%d\n", 
    sensors.imu2Data.xAngularVelocity, sensors.imu2Data.yAngularVelocity, 
    sensors.imu2Data.zAngularVelocity, sensors.imu2Data.gyroAccuracy);
  
  Serial.printf("Angular Velocity IMU3: (x, y, z)=(%.2f, %.2f, %.2f), accuracy=%d\n", 
    sensors.imu3Data.xAngularVelocity, sensors.imu3Data.yAngularVelocity, 
    sensors.imu3Data.zAngularVelocity, sensors.imu3Data.gyroAccuracy);

  Serial.printf("Orientation IMU1: (yaw, pitch, roll)=(%.2f, %.2f, %.2f), degrees of accuracy=%.2f, accuracy=%d\n", 
    sensors.imu1Data.yawOrientation, sensors.imu1Data.pitchOrientation, 
    sensors.imu1Data.rollOrientation, sensors.imu1Data.orientationAccuracy, 
    sensors.imu1Data.rotationAccuracy);
  
  Serial.printf("Orientation IMU2: (yaw, pitch, roll)=(%.2f, %.2f, %.2f), degrees of accuracy=%.2f, accuracy=%d\n", 
    sensors.imu2Data.yawOrientation, sensors.imu2Data.pitchOrientation, 
    sensors.imu2Data.rollOrientation, sensors.imu2Data.orientationAccuracy, 
    sensors.imu2Data.rotationAccuracy);
  
  Serial.printf("Orientation IMU3: (yaw, pitch, roll)=(%.2f, %.2f, %.2f), degrees of accuracy=%.2f, accuracy=%d\n", 
    sensors.imu3Data.yawOrientation, sensors.imu3Data.pitchOrientation, 
    sensors.imu3Data.rollOrientation, sensors.imu3Data.orientationAccuracy, 
    sensors.imu3Data.rotationAccuracy);

  // Sensor Data
  Serial.printf("Temperature: %.2f C\n", sensors.getTemperature());
  Serial.printf("Pressure: %.2f hPa\n", sensors.getPressure());
  Serial.printf("Altitude: %.2f m\n", sensors.getAltitude());

  Serial.printf("Median Linear Acceleration: (x, y, z)=(%.2f, %.2f, %.2f)\n", 
    xLinearAcceleration, yLinearAcceleration, zLinearAcceleration);
  
  Serial.printf("Median Angular Velocity: (x, y, z)=(%.2f, %.2f, %.2f)\n", 
    xAngularVelocity, yAngularVelocity, zAngularVelocity);
  
  Serial.printf("Median Orientation: (yaw, pitch, roll)=(%.2f, %.2f, %.2f)\n", 
    yawOrientation, pitchOrientation, rollOrientation);

  Serial.printf("Relative Velocity: (x, y, z)=(%.2f, %.2f, %.2f)\n", vx, vy, vz);
  Serial.printf("Relative Position: (x, y, z)=(%.2f, %.2f, %.2f)\n", px, py, pz);

/*
  // Set relative linear values using GPS values to counteract the effects of drift
  // Set new relative velocity
  sensors.setRelativeVelocity(0.0, 0.0, 0.0);
  Serial.println("Set new relative velocity to (0.0, 0.0, 0.0)");

  // Set new relative position
  sensors.setRelativePosition(0.0, 0.0, 0.0);
  Serial.println("Set new relative position to (0.0, 0.0, 0.0)");
*/

  Serial.println("--------------------------");
  threads.delay(100);
}
