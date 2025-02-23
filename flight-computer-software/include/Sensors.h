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
  //Constructor
  Sensors();

  //Initializes the sensors
  bool begin();
  
  //Enables reports for the IMU
  //params: BNO080 imu, uint16_t interval
  void enableReports(BNO080 imu, uint16_t interval = 10);
  
  //Gets the temperature in degrees Celsius
  //returns: float temperature
  float getTemperature(); 
  
  //Gets the pressure in hPa
  //returns: float pressure 
  float getPressure();

  //Gets the altitude
  //returns: float altitude
  float getAltitude();

  //Gets the linear acceleration
  //If data is not available from a sensor, set the accuracy to -1
  //params: BNO080 imu, float &x, float &y, float &z, float &accuracy
  void getLinearAcceleration(BNO080 imu, float &x, float &y, float &z, float &accuracy);

  //Gets the orientation
  //If data is not available from a sensor, set the accuracy to -1
  //params: BNO080 imu, float &yaw, float &pitch, float &roll, float &accuracyDegrees
  void getOrientation(BNO080 imu, float &yaw, float &pitch, float &roll, float &accuracyDegrees);
  
  //Gets the relative velocity
  //params: BNO080 imu, float &vx, float &vy, float &vz
  void getRelativeVelocity(BNO080 imu, float &vx, float &vy, float &vz);

  //Gets the relative position
  //params: BNO080 imu, float &px, float &py, float &pz
  void getRelativePosition(BNO080 imu, float &px, float &py, float &pz);
  
  //Sets the relative velocity
  //params: float vx, float vy, float vz
  void setRelativeVelocity(float vx, float vy, float vz);

  //Sets the relative position
  //params: float px, float py, float pz
  void setRelativePosition(float px, float py, float pz);

  //Gets the fused linear acceleration, uses median filter to get the most accurate data out of IMU1, IMU2, IMU3
  //params: float &ax, float &ay, float &az, float &linearaccuracy
  void getFusedLinearAcceleration(float &ax, float &ay, float &az, float &linearaccuracy);

  //Gets the fused orientation, uses median filter to get the most accurate data out of IMU1, IMU2, IMU3
  //params: float &yaw, float &pitch, float &roll, float &accuracyDegrees
  void getFusedOrientation(float &yaw, float &pitch, float &roll, float &accuracyDegrees);

  //If 3 sensors are working, get the median value, filters out outliers. 
  //If only 2 sensors are working, return the value with best accuracy
  //If only one sensor is working, return that value
  //Used inside of getFusedLinearAcceleration and getFusedOrientation
  //params: std::vector<float> values, std::vector<float> accuracy, bool accuracyIsDegrees
  float sensorFusion(std::vector<float> values, std::vector<float> accuracy, bool accuracyIsDegrees);

  //The temperature probe sensor
  Adafruit_MAX31865 temperatureProbe;
  
  //The pressure sensor
  MS5607 altimeter;
  
  //The IMUs, there are 3 IMUs
  BNO080 imu1;
  BNO080 imu2;
  BNO080 imu3;

  unsigned long lastVelocityUpdateTime;
  unsigned long lastPositionUpdateTime;
  float vx, vy, vz;
  float px, py, pz;
};
