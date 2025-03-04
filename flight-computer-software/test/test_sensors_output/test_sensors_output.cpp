/*
 *  This is a test program for Sensors, printing the data from the sensors.
 */

#include <Arduino.h>
#include "../../include/Sensors.h"

Sensors sensors;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  while (!sensors.begin()) {
    Serial.println("Retrying sensor initialization.");
    delay(10);
  }
  uint16_t interval = 10;
  sensors.enableReports(sensors.imu1, interval);
  sensors.enableReports(sensors.imu2, interval);
  sensors.enableReports(sensors.imu3, interval);
  delay(1000);
}

void loop() {

  //Serial.println("hello");
  // Get temperature
  float temperature = sensors.getTemperature();

  // Get pressure
  float pressure = sensors.getPressure();

  // Get altitude
  float altitude = sensors.getAltitude();


  //Serial.println("hello2");
  // Get acceleration for IMU 1
  float ax1, ay1, az1, linearaccuracy1;
  sensors.getLinearAcceleration(sensors.imu1, ax1, ay1, az1, linearaccuracy1);
  // Get acceleration for IMU 2
  float ax2, ay2, az2, linearaccuracy2;
  sensors.getLinearAcceleration(sensors.imu2, ax2, ay2, az2, linearaccuracy2);
  // Get acceleration for IMU 3
  float ax3, ay3, az3, linearaccuracy3;
  sensors.getLinearAcceleration(sensors.imu3, ax3, ay3, az3, linearaccuracy3);


  //Serial.println("hello3");
  // Get orientation for IMU 1
  float yaw1, pitch1, roll1, accuracy1;
  sensors.getOrientation(sensors.imu1, yaw1, pitch1, roll1, accuracy1);
  // Get orientation for IMU 2
  float yaw2, pitch2, roll2, accuracy2;
  sensors.getOrientation(sensors.imu2, yaw2, pitch2, roll2, accuracy2);
  // Get orientation for IMU 3
  float yaw3, pitch3, roll3, accuracy3;
  sensors.getOrientation(sensors.imu3, yaw3, pitch3, roll3, accuracy3);

  float ax, ay, az, mylinearaccuracy;
  float yaw, pitch, roll, accuracyDegrees;


  //Serial.println("hello4");
  sensors.getFusedLinearAcceleration(ax, ay, az, mylinearaccuracy);
  
  //Serial.println("hello5");
  sensors.getFusedOrientation(yaw, pitch, roll, accuracyDegrees);

  // Get relative velocity for IMU 1
  float vx1, vy1, vz1;
  sensors.getRelativeVelocity(sensors.imu1, vx1, vy1, vz1);

  // Get relative position for IMU 1
  float px1, py1, pz1;
  sensors.getRelativePosition(sensors.imu1, px1, py1, pz1);

  //get angular velocity for imu1
  float xangularvelocity1, yangularvelocity1, zangularvelocity1;
  float xangularvelocity2, yangularvelocity2, zangularvelocity2;
  float xangularvelocity3, yangularvelocity3, zangularvelocity3;
  sensors.getAngularVelocity(sensors.imu1, xangularvelocity1, yangularvelocity1, zangularvelocity1);
  sensors.getAngularVelocity(sensors.imu2, xangularvelocity2, yangularvelocity2, zangularvelocity2);
  sensors.getAngularVelocity(sensors.imu3, xangularvelocity3, yangularvelocity3, zangularvelocity3);




  float xvelocity1, yvelocity1, zvelocity1;
  float xvelocity2, yvelocity2, zvelocity2;
  float xvelocity3, yvelocity3, zvelocity3;
  sensors.getRelativeVelocity(sensors.imu1, xvelocity1, yvelocity1, zvelocity1);
  sensors.getRelativeVelocity(sensors.imu2, xvelocity2, yvelocity2, zvelocity2);
  sensors.getRelativeVelocity(sensors.imu3, xvelocity3, yvelocity3, zvelocity3);

  // Print data
  Serial.println("Temperature: " + String(temperature) + " C");
  Serial.println("Pressure: " + String(pressure) + " hPa");
  Serial.println("Altitude: " + String(altitude) + " m");
  
  Serial.println("Acceleration (IMU 1): X: " + String(ax1) + " Y: " + String(ay1) + " Z: " + String(az1) + " Accuracy: " + String(linearaccuracy1));
  Serial.println("Acceleration (IMU 2): X: " + String(ax2) + " Y: " + String(ay2) + " Z: " + String(az2) + " Accuracy: " + String(linearaccuracy2));
  Serial.println("Acceleration (IMU 3): X: " + String(ax3) + " Y: " + String(ay3) + " Z: " + String(az3) + " Accuracy: " + String(linearaccuracy3));
  
  Serial.println("Orientation (IMU 1): Yaw: " + String(yaw1) + " Pitch: " + String(pitch1) + " Roll: " + String(roll1) + " Accuracy: " + String(accuracy1));
  Serial.println("Orientation (IMU 2): Yaw: " + String(yaw2) + " Pitch: " + String(pitch2) + " Roll: " + String(roll2) + " Accuracy: " + String(accuracy2));
  Serial.println("Orientation (IMU 3): Yaw: " + String(yaw3) + " Pitch: " + String(pitch3) + " Roll: " + String(roll3) + " Accuracy: " + String(accuracy3));
  
  Serial.println("Median Acceleration: X: " + String(ax) + " Y: " + String(ay) + " Z: " + String(az));
  Serial.println("Median Orientation: Yaw: " + String(yaw) + " Pitch: " + String(pitch) + " Roll: " + String(roll));
  
  Serial.println("Relative Velocity: Vx: " + String(vx1) + " Vy: " + String(vy1) + " Vz: " + String(vz1));
  Serial.println("Relative Position: Px: " + String(px1) + " Py: " + String(py1) + " Pz: " + String(pz1));

  Serial.println("Angular velocity IMU1: X: " + String(xangularvelocity1) + " Y: " + String(yangularvelocity1) + " Z: " + String(zangularvelocity1));
  Serial.println("Angular velocity IMU2: X: " + String(xangularvelocity2) + " Y: " + String(yangularvelocity2) + " Z: " + String(zangularvelocity2));
  Serial.println("Angular velocity IMU3: X: " + String(xangularvelocity3) + " Y: " + String(yangularvelocity3) + " Z: " + String(zangularvelocity3));

  Serial.println("Relative Velocity IMU1: X: " + String(xvelocity1) + " Y: " + String(yvelocity1) + " Z: " + String(zvelocity1));
  Serial.println("Relative Velocity IMU2: X: " + String(xvelocity2) + " Y: " + String(yvelocity2) + " Z: " + String(zvelocity2));
  Serial.println("Relative Velocity IMU3: X: " + String(xvelocity3) + " Y: " + String(yvelocity3) + " Z: " + String(zvelocity3));


  Serial.println("--------------------------");

  // Set new relative velocity
  // sensors.setRelativeVelocity(0.0, 0.0, 0.0);
  // Serial.println("Set new relative velocity to (0.0, 0.0, 0.0)");

  // Set new relative position
  // sensors.setRelativePosition(0.0, 0.0, 0.0);
  // Serial.println("Set new relative position to (0.0, 0.0, 0.0)");

  delay(10);
}
