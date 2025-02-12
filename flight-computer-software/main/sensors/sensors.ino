/*
 *  This is a test program for Sensors, printing the data from the sensors.
 */




#include "Sensors.h"

Sensors sensors;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  if (!sensors.begin()) {
    Serial.println("Stopping program.");
    while (1) {
      delay(10);
    }
  }
  uint16_t interval = 10;
  sensors.enableReports(sensors.imu1, interval);
  sensors.enableReports(sensors.imu2, interval);
  sensors.enableReports(sensors.imu3, interval);
  delay(1000);
}

void loop() {


  // Get temperature
  float temperature = sensors.getTemperature();

  // Get pressure
  float pressure = sensors.getPressure();

  // Get altitude
  float altitude = sensors.getAltitude();


  // Get acceleration for IMU 1
  float ax1, ay1, az1;
  sensors.getLinearAcceleration(sensors.imu1, ax1, ay1, az1);
  // Get acceleration for IMU 2
  float ax2, ay2, az2;
  sensors.getLinearAcceleration(sensors.imu2, ax2, ay2, az2);
  // Get acceleration for IMU 3
  float ax3, ay3, az3;
  sensors.getLinearAcceleration(sensors.imu3, ax3, ay3, az3);

  // Get orientation for IMU 1
  float yaw1, pitch1, roll1, accuracy1;
  sensors.getOrientation(sensors.imu1, yaw1, pitch1, roll1, accuracy1);
  // Get orientation for IMU 2
  float yaw2, pitch2, roll2, accuracy2;
  sensors.getOrientation(sensors.imu2, yaw2, pitch2, roll2, accuracy2);
  // Get orientation for IMU 3
  float yaw3, pitch3, roll3, accuracy3;
  sensors.getOrientation(sensors.imu3, yaw3, pitch3, roll3, accuracy3);

  // Get relative velocity for IMU 1
  float vx1, vy1, vz1;
  sensors.getRelativeVelocity(sensors.imu1, vx1, vy1, vz1);

  // Get relative position for IMU 1
  float px1, py1, pz1;
  sensors.getRelativePosition(sensors.imu1, px1, py1, pz1);

  // Print data
  Serial.println("Temperature: " + String(temperature) + " C");
  Serial.println("Pressure: " + String(pressure) + " hPa");
  Serial.println("Altitude: " + String(altitude) + " m");
  Serial.println("Acceleration (IMU 1): X: " + String(ax1) + " Y: " + String(ay1) + " Z: " + String(az1));
  Serial.println("Orientation (IMU 2): Yaw: " + String(yaw2) + " Pitch: " + String(pitch2) + " Roll: " + String(roll2) + " Accuracy: " + String(accuracy2));
  Serial.println("Orientation (IMU 3): Yaw: " + String(yaw3) + " Pitch: " + String(pitch3) + " Roll: " + String(roll3) + " Accuracy: " + String(accuracy3));
  Serial.println("Relative Velocity: Vx: " + String(vx1) + " Vy: " + String(vy1) + " Vz: " + String(vz1));
  Serial.println("Relative Position: Px: " + String(px1) + " Py: " + String(py1) + " Pz: " + String(pz1));

  // Set new relative velocity
  // sensors.setRelativeVelocity(0.0, 0.0, 0.0);
  // Serial.println("Set new relative velocity to (0.0, 0.0, 0.0)");

  // Set new relative position
  // sensors.setRelativePosition(0.0, 0.0, 0.0);
  // Serial.println("Set new relative position to (0.0, 0.0, 0.0)");

  delay(100);
}
