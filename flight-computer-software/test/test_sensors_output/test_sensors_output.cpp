/*
 *  This is a test program for Sensors, printing the data from the sensors.
 */

#include <Arduino.h>
#include "../../include/Sensors.h"

Sensors sensors;

void setup() {
  Serial.begin(115200);
  SensorStatus status = {false, false, false, false, false};
  delay(100);
  while (true) {
    status = sensors.begin(status);
    if (!status.imu1)
      Serial.println("Failed to initialize IMU 1.");
    if (!status.imu2)
      Serial.println("Failed to initialize IMU 2.");
    if (!status.imu3)
      Serial.println("Failed to initialize IMU 3.");
    if (!status.pressure)
      Serial.println("Failed to initialize Altimeter.");
    if (!status.temperature)
      Serial.println("Failed to initialize Temperature Probe.");
    if (status.imu1 && status.imu2 && status.imu3 && status.pressure && status.temperature) {
      Serial.println("Successfully initialized all sensors.");
      break;
    }
    else {
      Serial.println("Retrying sensor initialization.");
      delay(100);
    }
  }
  sensors.enableIMUReports(100);
  delay(1000);
}

void loop() {

  if (sensors.collectIMUData()){

    // Get temperature
    float temperature = sensors.getTemperature();

    // Get pressure
    float pressure = sensors.getPressure();

    // Get altitude
    float altitude = sensors.getAltitude();


  /*
    float xLinearAcceleration, yLinearAcceleration, zLinearAcceleration;
    byte linearAccuracy;
    sensors.getFusedLinearAcceleration(xLinearAcceleration, yLinearAcceleration, zLinearAcceleration, linearAccuracy);
    
    float xAngularVelocity, yAngularVelocity, zAngularVelocity;
    byte gyroAccuracy;
    sensors.getFusedAngularVelocity(xAngularVelocity, yAngularVelocity, zAngularVelocity, gyroAccuracy);

    float yawOrientation, pitchOrientation, rollOrientation, orientationAccuracy;
    byte rotationAccuracy;
    sensors.getFusedOrientation(yawOrientation, pitchOrientation, rollOrientation, orientationAccuracy, rotationAccuracy);

    // Get relative velocity
    float vx, vy, vz;
    sensors.getRelativeVelocity(vx, vy, vz);

    // Get relative position
    float px, py, pz;
    sensors.getRelativePosition(px, py, pz);
  */
    // Print data
    Serial.println("Temperature: " + String(temperature) + " C");
    Serial.println("Pressure: " + String(pressure) + " hPa");
    Serial.println("Altitude: " + String(altitude) + " m");
    
    Serial.println("Acceleration (IMU 1): X: " + String(sensors.imu1Data.xLinearAcceleration) + " Y: " + String(sensors.imu1Data.yLinearAcceleration) + " Z: " + String(sensors.imu1Data.zLinearAcceleration) + " Accuracy: " + String(sensors.imu1Data.linearAccuracy));
    Serial.println("Acceleration (IMU 2): X: " + String(sensors.imu2Data.xLinearAcceleration) + " Y: " + String(sensors.imu2Data.yLinearAcceleration) + " Z: " + String(sensors.imu2Data.zLinearAcceleration) + " Accuracy: " + String(sensors.imu2Data.linearAccuracy));
    Serial.println("Acceleration (IMU 3): X: " + String(sensors.imu3Data.xLinearAcceleration) + " Y: " + String(sensors.imu3Data.yLinearAcceleration) + " Z: " + String(sensors.imu3Data.zLinearAcceleration) + " Accuracy: " + String(sensors.imu3Data.linearAccuracy));
    
    Serial.println("Angular Velocity (IMU 1): X: " + String(sensors.imu1Data.xAngularVelocity) + " Y: " + String(sensors.imu1Data.yAngularVelocity) + " Z: " + String(sensors.imu1Data.zAngularVelocity) + " Accuracy: " + String(sensors.imu1Data.gyroAccuracy));
    Serial.println("Angular Velocity (IMU 2): X: " + String(sensors.imu2Data.xAngularVelocity) + " Y: " + String(sensors.imu2Data.yAngularVelocity) + " Z: " + String(sensors.imu2Data.zAngularVelocity) + " Accuracy: " + String(sensors.imu2Data.gyroAccuracy));
    Serial.println("Angular Velocity (IMU 3): X: " + String(sensors.imu3Data.xAngularVelocity) + " Y: " + String(sensors.imu3Data.yAngularVelocity) + " Z: " + String(sensors.imu3Data.zAngularVelocity) + " Accuracy: " + String(sensors.imu3Data.gyroAccuracy));
    
    Serial.println("Orientation (IMU 1): Yaw: " + String(sensors.imu1Data.yawOrientation) + " Pitch: " + String(sensors.imu1Data.pitchOrientation) + " Roll: " + String(sensors.imu1Data.rollOrientation) + " Degrees Accuracy: " + String(sensors.imu1Data.orientationAccuracy) + " Accuracy: " + String(sensors.imu1Data.rotationAccuracy));
    Serial.println("Orientation (IMU 2): Yaw: " + String(sensors.imu2Data.yawOrientation) + " Pitch: " + String(sensors.imu2Data.pitchOrientation) + " Roll: " + String(sensors.imu2Data.rollOrientation) + " Degrees Accuracy: " + String(sensors.imu2Data.orientationAccuracy) + " Accuracy: " + String(sensors.imu2Data.rotationAccuracy));
    Serial.println("Orientation (IMU 3): Yaw: " + String(sensors.imu3Data.yawOrientation) + " Pitch: " + String(sensors.imu3Data.pitchOrientation) + " Roll: " + String(sensors.imu3Data.rollOrientation) + " Degrees Accuracy: " + String(sensors.imu3Data.orientationAccuracy) + " Accuracy: " + String(sensors.imu3Data.rotationAccuracy));
  /*
    Serial.println("Median Acceleration: X: " + String(xLinearAcceleration) + " Y: " + String(yLinearAcceleration) + " Z: " + String(zLinearAcceleration) + " Accuracy: " + String(linearAccuracy));
    Serial.println("Median Angular Velocity: X: " + String(xAngularVelocity) + " Y: " + String(yAngularVelocity) + " Z: " + String(zAngularVelocity) + " Accuracy: " + String(gyroAccuracy));
    Serial.println("Median Orientation: Yaw: " + String(yawOrientation) + " Pitch: " + String(pitchOrientation) + " Roll: " + String(rollOrientation) + " Degrees Accuracy: " + String(orientationAccuracy) + " Accuracy: " + String(rotationAccuracy));

    Serial.println("Relative Velocity: Vx: " + String(vx) + " Vy: " + String(vy) + " Vz: " + String(vz));
    Serial.println("Relative Position: Px: " + String(px) + " Py: " + String(py) + " Pz: " + String(pz));

    // Set new relative velocity
    sensors.setRelativeVelocity(0.0, 0.0, 0.0);
    Serial.println("Set new relative velocity to (0.0, 0.0, 0.0)");

    // Set new relative position
    sensors.setRelativePosition(0.0, 0.0, 0.0);
    Serial.println("Set new relative position to (0.0, 0.0, 0.0)");
  */
    Serial.println("--------------------------");
    delay(100);
  }
}
