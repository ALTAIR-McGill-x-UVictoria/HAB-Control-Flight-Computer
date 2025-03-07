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

#include <TeensyThreads.h>
#include <Adafruit_MAX31865.h>
#include <MS5607.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <vector>

// The value of the Rref resistor: 430.0 for PT100
#define RREF 430.0
// The 'nominal' 0-degrees-C resistance of the sensor: 100.0 for PT100
#define RNOMINAL 100.0

struct SensorStatus
{
  bool pressure;
  bool temperature;
  bool imu1;
  bool imu2;
  bool imu3;
};

struct SensorDataIMU
{
  volatile float xLinearAcceleration, yLinearAcceleration, zLinearAcceleration;
  volatile byte linearAccuracy;
  volatile float xAngularVelocity, yAngularVelocity, zAngularVelocity;
  volatile byte gyroAccuracy;
  volatile float yawOrientation, pitchOrientation, rollOrientation, orientationAccuracy;
  volatile byte rotationAccuracy;
};

class Sensors
{
public:
  SensorDataIMU imu1Data;
  SensorDataIMU imu2Data;
  SensorDataIMU imu3Data;
  volatile float temperature, pressure, altitude;

  volatile float xRelativeVelocity, yRelativeVelocity, zRelativeVelocity;
  volatile float xRelativePosition, yRelativePosition, zRelativePosition;

  // Initializes the sensors
  // params: SensorStatus status
  // returns: SensorStatus
  SensorStatus begin(SensorStatus status);

  void enableReports(uint16_t interval = 50);

  void startDataCollection();

  void stopDataCollection();

  float getTemperature();

  float getPressure();

  float getAltitude();

  // Gets the fused linear acceleration, uses median filter to get the most accurate data out of IMU1, IMU2, IMU3
  // params: float &xLinearAcceleration, float &yLinearAcceleration, float &zLinearAcceleration
  void getFusedLinearAcceleration(float &xLinearAcceleration, float &yLinearAcceleration, float &zLinearAcceleration);

  // Gets the fused angular velocity, uses median filter to get the most accurate data out of IMU1, IMU2, IMU3
  // params: float &xAngularVelocity, float &yAngularVelocity, float &zAngularVelocity
  void getFusedAngularVelocity(float &xAngularVelocity, float &yAngularVelocity, float &zAngularVelocity);

  // Gets the fused orientation, uses median filter to get the most accurate data out of IMU1, IMU2, IMU3
  // params: float &yawOrientation, float &pitchOrientation, float &rollOrientation
  void getFusedOrientation(float &yawOrientation, float &pitchOrientation, float &rollOrientation);

  // Gets the relative velocity
  // params: float &vx, float &vy, float &vz
  void getRelativeVelocity(float &vx, float &vy, float &vz);

  // Gets the relative position
  // params: float &px, float &py, float &pz
  void getRelativePosition(float &px, float &py, float &pz);

  // Sets the relative velocity
  // params: float vx, float vy, float vz
  void setRelativeVelocity(float vx, float vy, float vz);

  // Sets the relative position
  // params: float px, float py, float pz
  void setRelativePosition(float px, float py, float pz);

private:
  // The temperature probe sensor
  Adafruit_MAX31865 temperatureProbe = Adafruit_MAX31865(10, 11, 12, 13); // software SPI: CS, DI, DO, CLK

  // The pressure sensor
  MS5607 altimeter = MS5607(0x76); // Set I2C address to 0x76 for MS5607

  // The IMUs, there are 3 IMUs
  BNO080 imu1;
  BNO080 imu2;
  BNO080 imu3;

  uint16_t interval = 50;

  int altimeterSensorThreadId;
  int temperatureSensorThreadId;
  int imuSensorThreadId;
  int computeRelativeLinearThreadId;
  volatile bool running;

  unsigned long lastImu1UpdateTime;
  unsigned long lastImu2UpdateTime;
  unsigned long lastImu3UpdateTime;
  unsigned long lastRelativeLinearUpdateTime;

  // Enables reports for the IMU
  // params: BNO080* imu, uint16_t interval
  void enableReportsForIMU(BNO080 *imu, uint16_t interval = 50);

  bool fetchDataFromIMU(BNO080 *imu, SensorDataIMU *data);
  
  void invalidateIMUData(unsigned long lastImuUpdateTime,BNO080* imu, SensorDataIMU* data, unsigned long timeout=500);

  // Thread function wrappers for TeensyThreads
  static void altimeterSensorThreadWrapper(void *sensorObj);
  static void temperatureSensorThreadWrapper(void *sensorObj);
  static void imuSensorThreadWrapper(void *sensorObj);
  static void computeRelativeLinearThreadWrapper(void *sensorObj);

  // Actual thread implementations
  void altimeterSensorThreadImpl();
  void temperatureSensorThreadImpl();
  void imuSensorThreadImpl();
  void computeRelativeLinearThreadImpl();

  // If 3 sensors are working, get the median value, filters out outliers.
  // If only 2 sensors are working, return the value with best accuracy
  // If only one sensor is working, return that value
  // IF NO SENSORS ARE WORKING, ADD THIS TO THE CODE
  // Used inside of getFusedLinearAcceleration and getFusedOrientation
  // params: std::vector<float> values, std::vector<byte> accuracy, std::vector<float> orientationAccuracy={}
  // returns: float
  float sensorFusion(std::vector<float> values, std::vector<byte> accuracy, std::vector<float> orientationAccuracy = {});
};
