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

#ifndef SENSORS_H
#define SENSORS_H

#include <TeensyThreads.h>
#include <Adafruit_MAX31865.h>
#include <MS5607.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <vector>
#include <string>
#include <map>

// The value of the Rref resistor: 430.0 for PT100
#define RREF 430.0
// The 'nominal' 0-degrees-C resistance of the sensor: 100.0 for PT100
#define RNOMINAL 100.0

// Timeout constants for sensor data (in milliseconds)
#define ALTIMETER_TIMEOUT 1000
#define TEMPERATURE_TIMEOUT 1000
#define IMU_TIMEOUT 1000

#define FILTER_FACTOR 0.5f  // Adjust between 0.01 (very smooth) and 0.5 (responsive)

// Set to 1 to enable temporal filtering, 0 to disable
#define ENABLE_TEMPORAL_FILTER 0

// Filter algorithm selection (choose only one)
#define FILTER_ALGORITHM_SIMPLE 1      // Simple rate-limiting + exponential smoothing
#define FILTER_ALGORITHM_ADAPTIVE 2    // Adaptive smoothing based on change magnitude
#define FILTER_ALGORITHM_DOUBLE_EXP 3  // Two-stage double exponential smoothing

// Select which algorithm to use (1, 2, or 3)
#define SELECTED_FILTER_ALGORITHM FILTER_ALGORITHM_ADAPTIVE

// Smoothing strength parameters - adjust as needed
#define FILTER_ALPHA_SIMPLE 0.3f      // Simple filter alpha (smaller = more smoothing)
#define FILTER_ALPHA1_DOUBLE 0.4f      // First stage alpha for double exp
#define FILTER_ALPHA2_DOUBLE 0.25f     // Second stage alpha for double exp

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
  volatile float quatI, quatJ, quatK, quatReal;
};

// Add a structure for GPS data
struct GPSData {
  volatile float latitude;         // Degrees (WGS84)
  volatile float longitude;        // Degrees (WGS84)
  volatile float altitude;         // Meters above sea level
  volatile float speed;            // Ground speed in m/s
  volatile float course;           // Course over ground in degrees (0-360, 0=north)
  volatile float vx;               // North velocity component in m/s
  volatile float vy;               // East velocity component in m/s
  volatile float vz;               // Down velocity component in m/s
  volatile uint8_t satellites;     // Number of satellites used for fix
  volatile uint8_t fixQuality;     // Fix quality (0=no fix, 1=GPS, 2=DGPS, etc.)
  volatile uint32_t timestamp;     // Time of last GPS update (milliseconds)
  volatile bool valid;             // Indicates if GPS data is valid
};

class Sensors
{
public:
  SensorStatus status = {false, false, false, false, false};

  SensorDataIMU imu1Data;
  SensorDataIMU imu2Data;
  SensorDataIMU imu3Data;
  volatile float temperature, pressure, altitude;

  volatile float xRelativeVelocity, yRelativeVelocity, zRelativeVelocity;
  volatile float xRelativePosition, yRelativePosition, zRelativePosition;

  // Add GPS data to public section
  GPSData gpsData;

  // Timers
  unsigned long lastIMU1UpdateTime;
  unsigned long lastIMU2UpdateTime;
  unsigned long lastIMU3UpdateTime;
  
  // Initializes the sensors
  void begin();

  //enables the reports for the sensors, starts the threads
  //params: uint16_t interval
  void start(uint16_t interval = 20);

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

  // Gets the relative velocity (in world frame)
  // params: float &vx, float &vy, float &vz
  void getRelativeVelocity(float &vx, float &vy, float &vz);

  // Gets the relative position (in world frame)
  // params: float &px, float &py, float &pz
  void getRelativePosition(float &px, float &py, float &pz);

  // Sets the relative velocity (in world frame)
  // params: float vx, float vy, float vz
  void setRelativeVelocity(float vx, float vy, float vz);

  // Sets the relative position (in world frame)
  // params: float px, float py, float pz
  void setRelativePosition(float px, float py, float pz);

  // Gets the fused linear acceleration in world frame
  // params: float &xLinearAcceleration, float &yLinearAcceleration, float &zLinearAcceleration
  void getFusedWorldLinearAcceleration(float &xLinearAcceleration, float &yLinearAcceleration, float &zLinearAcceleration);
  
  // Gets the fused angular velocity in world frame
  // params: float &xAngularVelocity, float &yAngularVelocity, float &zAngularVelocity
  void getFusedWorldAngularVelocity(float &xAngularVelocity, float &yAngularVelocity, float &zAngularVelocity);

  // Gets the fused quaternion
  // params: float &quatI, float &quatJ, float &quatK, float &quatReal
  void getFusedQuaternion(float &quatI, float &quatJ, float &quatK, float &quatReal);

  // Calibrates the IMUs
  // params: void
  void calibrateAllIMUs();

  // Check if sensors are ready for operation
  bool areSensorsReady();
  
  // Reset and reinitialize a specific IMU
  bool resetIMU(int imuNumber);

  // Add GPS-related methods
  float getLatitude();
  float getLongitude();
  float getGPSAltitude();
  float getGPSSpeed();
  float getGPSCourse();
  uint8_t getGPSSatellites();
  uint8_t getGPSFixQuality();
  bool isGPSValid();
  
  // Methods for future integration
  void updatePositionWithGPS();
  void updateVelocityWithGPS();
  
private:

  Threads::Mutex i2c_mutex;

  // The temperature probe sensor
  Adafruit_MAX31865 temperatureProbe = Adafruit_MAX31865(10, 11, 12, 13); // software SPI: CS, DI, DO, CLK

  // The pressure sensor
  MS5607 altimeter = MS5607(0x76); // Set I2C address to 0x76 for MS5607

  // The IMUs, there are 3 IMUs
  BNO080 imu1;
  BNO080 imu2;
  BNO080 imu3;

  uint16_t interval;

  int altimeterSensorThreadId;
  int temperatureSensorThreadId;
  int imuSensorThreadId;
  int computeRelativeLinearThreadId;
  volatile bool running;

  unsigned long lastImu1UpdateTime;
  unsigned long lastImu2UpdateTime;
  unsigned long lastImu3UpdateTime;
  unsigned long lastAltimeterUpdateTime;
  unsigned long lastRelativeLinearUpdateTime;

  bool sensorsInitialized = false;

  std::map<std::string, float> lastFusedValues; // Key is a unique ID for each measurement type

  // Enables reports for the IMU
  // params: BNO080* imu, uint16_t interval
  void enableReportsForIMU(BNO080 *imu, uint16_t interval);

  bool fetchDataFromIMU(BNO080 *imu, SensorDataIMU *data);

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

  float fuseYaw(std::vector<float> yawValues, std::vector<byte> accuracy, std::vector<float> orientationAccuracy);

  // Helper method to process a single IMU
  void processIMUSensor(BNO080 &imu, SensorDataIMU &data, unsigned long &lastUpdateTime, bool &statusFlag);

  float applyTemporalFilter(const std::string& sensorKey, float newValue, float maxChangeRate);

  float applyAngularFilter(const std::string& sensorKey, float newValue, float maxChangeRate);

  // Transforms a vector from sensor frame to world frame using a quaternion
  // params: float x, float y, float z, float quatI, float quatJ, float quatK, float quatReal, float &xWorld, float &yWorld, float &zWorld
  void transformToWorldFrame(float x, float y, float z, 
                            float quatI, float quatJ, float quatK, float quatReal, 
                            float &xWorld, float &yWorld, float &zWorld);
                            
  // Quaternion multiplication helper
  // params: float a_i, float a_j, float a_k, float a_real, float b_i, float b_j, float b_k, float b_real, 
  //         float &c_i, float &c_j, float &c_k, float &c_real
  void quaternionMultiply(float a_i, float a_j, float a_k, float a_real,
                          float b_i, float b_j, float b_k, float b_real,
                          float &c_i, float &c_j, float &c_k, float &c_real);
                          
  // Get conjugate of quaternion
  // params: float i, float j, float k, float real, float &conj_i, float &conj_j, float &conj_k, float &conj_real
  void quaternionConjugate(float i, float j, float k, float real, 
                          float &conj_i, float &conj_j, float &conj_k, float &conj_real);

  // Helper method to initialize a single IMU with retries
  bool initializeSingleIMU(BNO080 &imu, TwoWire &wire, uint8_t address, int maxRetries = 5);

  void updateAltitudeWithComplementaryFilter();

  void rungeKutta4Integration(float &position, float &velocity, float acceleration, float dt);

  void detectZeroVelocityAndUpdate(float state[9], float P[9][9], float ax, float ay, float az);

  void filterAcceleration(float &ax, float &ay, float &az);

  static float filtered_roll, filtered_pitch, filtered_yaw;
  bool was_stationary = false;

  // For GPS signal simulation in testing (remove when actual GPS is integrated)
  void simulateGPSData();

  // For future GPS integration
  void calculateGPSVelocityComponents();
  void performKalmanFilter();  // Future implementation for sensor fusion

  
};

#endif // SENSORS_H
