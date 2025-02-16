/*
Sources:
  Adafruit_MAX31865 Arduino Library Example
  MS5607 Arduino Library Example
  SparkFun_BNO080_Cortex_Based_IMU Library Example
*/

/*
The included sensor libraries (Temperature probe sensor Adafruit_MAX31865,
Pressure sensor MS5607, IMU SparkFun_BNO080_Cortex_Based_IMU) are used by this wrapper
library to provide the following data :
  - Acceleration
  - Relative Velocity (from set start point)
  - Relative Position (from set start point)
  - Temperature
  - Orientation
  - Altitude
  - Pressure
*/

#include <Adafruit_MAX31865.h>
#include <MS5607.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <vector>

// The value of the Rref resistor: 430.0 for PT100
#define RREF 430.0
// The 'nominal' 0-degrees-C resistance of the sensor: 100.0 for PT100
#define RNOMINAL 100.0

class Sensors
{
public:
  Sensors();

  bool begin();
  void enableReports(BNO080 imu, uint16_t interval = 10);
  float getTemperature();
  float getPressure();
  float getAltitude();
  void getLinearAcceleration(BNO080 imu, float &x, float &y, float &z, float &accuracy);
  void getOrientation(BNO080 imu, float &yaw, float &pitch, float &roll, float &accuracyDegrees);
  void getRelativeVelocity(BNO080 imu, float &vx, float &vy, float &vz);
  void getRelativePosition(BNO080 imu, float &px, float &py, float &pz);
  void setRelativeVelocity(float vx, float vy, float vz);
  void setRelativePosition(float px, float py, float pz);
  //void getFusedLinearAcceleration(float &ax, float &ay, float &az);
  void getFusedLinearAcceleration(float &ax, float &ay, float &az, float &axaverage, float &ayaverage, float &azaverage);
  //void getFusedOrientation(float &yaw, float &pitch, float &roll, float &accuracyDegrees);
  void getFusedOrientation(float &yawMedian, float &yawAverage, float &pitchMedian, float &pitchAverage, float &rollMedian, float &rollAverage, float &accuracyDegrees);
  float sensorFusionMedian(std::vector<float> values, std::vector<float> accuracy);
  float sensorFusionWeightedAverage(std::vector<float> values, std::vector<float> accuracy);

  Adafruit_MAX31865 temperatureProbe;
  MS5607 altimeter;
  BNO080 imu1;
  BNO080 imu2;
  BNO080 imu3;
  unsigned long lastVelocityUpdateTime;
  unsigned long lastPositionUpdateTime;
  float vx, vy, vz;
  float px, py, pz;
};
