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

// TODO: Add accuracy stuff, reset checks, validation, etc.

#include "Sensors.h"

Sensors::Sensors()
    : temperatureProbe(10, 11, 12, 13), // software SPI: CS, DI, DO, CLK
      altimeter(0x76),                  // Set I2C address to 0x76 for MS5607
      imu1(),
      imu2(),
      imu3(),
      lastVelocityUpdateTime(0),
      lastPositionUpdateTime(0),
      vx(0), vy(0), vz(0),
      px(0), py(0), pz(0)
{
}

bool Sensors::begin()
{
  Wire1.begin();
  Wire2.begin();
  int status = true;

  if (!imu1.begin(0x4A, Wire1, -1))
  {
    Serial.println("Failed to initialize IMU 1.");
    status = false;
  }
  else
    Serial.println("Successfully initialized IMU 1.");

  if (!imu2.begin(0x4B, Wire1, -1))
  {
    Serial.println("Failed to initialize IMU 2.");
    status = false;
  }
  else
    Serial.println("Successfully initialized IMU 2.");

  if (!imu3.begin(0x4A, Wire2, -1))
  {
    Serial.println("Failed to initialize IMU 3.");
    status = false;
  }
  else
    Serial.println("Successfully initialized IMU 3.");

  if (!altimeter.begin())
  {
    Serial.println("Failed to initialize Altimeter.");
    status = false;
  }
  else
    Serial.println("Successfully initialized Altimeter.");

  if (!temperatureProbe.begin(MAX31865_3WIRE))
  {
    Serial.println("Failed to initialize Temperature Probe.");
    status = false;
  }
  else
    Serial.println("Successfully initialized Temperature Probe.");

  return status;
}

void Sensors::enableReports(BNO080 imu, uint16_t interval)
{
  imu.enableRotationVector(interval);
  imu.enableLinearAccelerometer(interval);
}

float Sensors::getTemperature()
{
  return temperatureProbe.temperature(RNOMINAL, RREF);
}

float Sensors::getPressure()
{
  altimeter.readDigitalValue();
  return altimeter.getPressure();
}

float Sensors::getAltitude()
{
  altimeter.readDigitalValue();
  return altimeter.getAltitude();
}

void Sensors::getLinearAcceleration(BNO080 imu, float &ax, float &ay, float &az, float &linearaccuracy)
{
  if (imu.hasReset())
  {
    //Serial.println("IMU has reset. Reason: " + imu.resetReason());
    enableReports(imu);
  }
  if (imu.dataAvailable())
  {
    ax = imu.getLinAccelX();
    ay = imu.getLinAccelY();
    az = imu.getLinAccelZ();
    linearaccuracy = imu.getLinAccelAccuracy();
  } 
  else // if data is not available, make the accuracy -1
  {
    linearaccuracy = -1.0; //negative value means accuracy is not available
  }
}

void Sensors::getOrientation(BNO080 imu, float &yaw, float &pitch, float &roll, float &accuracyDegrees)
{
  if (imu.hasReset())
  {
    //Serial.println("IMU has reset. Reason: " + String(imu.resetReason()));
    enableReports(imu);
  }
  if (imu.dataAvailable())
  {
    accuracyDegrees = imu.getQuatRadianAccuracy() * 180.0 / PI;

    // Convert to degrees
    yaw = imu.getYaw() * 180.0 / PI;
    pitch = imu.getPitch() * 180.0 / PI;
    roll = imu.getRoll() * 180.0 / PI;
  } 
  else // if data is not available, make the accuracy negative
  {
    accuracyDegrees = -1.0; //negative value means accuracy is not available
  }
}

void Sensors::getRelativeVelocity(BNO080 imu, float &vx, float &vy, float &vz)
{
  float ax = 0.0, ay = 0.0, az = 0.0, accuracy = 0.0;
  getLinearAcceleration(imu, ax, ay, az, accuracy);

  unsigned long currentTime = millis();
  float dt = (currentTime - lastVelocityUpdateTime) / 1000.0; // Convert to seconds
  lastVelocityUpdateTime = currentTime;

  // Integrate acceleration to get velocity
  vx = this->vx + ax * dt;
  vy = this->vy + ay * dt;
  vz = this->vz + az * dt;

  setRelativeVelocity(vx, vy, vz);
}

void Sensors::getRelativePosition(BNO080 imu, float &px, float &py, float &pz)
{
  float vx = 0.0, vy = 0.0, vz = 0.0;
  getRelativeVelocity(imu, vx, vy, vz);

  unsigned long currentTime = millis();
  float dt = (currentTime - lastPositionUpdateTime) / 1000.0; // Convert to seconds
  lastPositionUpdateTime = currentTime;

  // Integrate velocity to get position
  px = this->px + vx * dt;
  py = this->py + vy * dt;
  pz = this->pz + vz * dt;

  setRelativePosition(px, py, pz);
}

void Sensors::setRelativeVelocity(float vx, float vy, float vz)
{
  this->vx = vx;
  this->vy = vy;
  this->vz = vz;
}

void Sensors::setRelativePosition(float px, float py, float pz)
{
  this->px = px;
  this->py = py;
  this->pz = pz;
}


// use median filter to get the most accurate data out of IMU1, IMU2, IMU3
void Sensors::getFusedLinearAcceleration(float &ax, float &ay, float &az, float &linearaccuracy)
{
  float ax1, ay1, az1, linearaccuracy1;
  float ax2, ay2, az2, linearaccuracy2;
  float ax3, ay3, az3, linearaccuracy3;

  getLinearAcceleration(imu1, ax1, ay1, az1, linearaccuracy1);
  getLinearAcceleration(imu2, ax2, ay2, az2, linearaccuracy2);
  getLinearAcceleration(imu3, ax3, ay3, az3, linearaccuracy3);

  std::vector<float> axValues = {ax1, ax2, ax3};
  std::vector<float> ayValues = {ay1, ay2, ay3};
  std::vector<float> azValues = {az1, az2, az3};
  std::vector<float> accuracyvalues = {linearaccuracy1, linearaccuracy2, linearaccuracy3};

  // call sensorFusion function
  ax = sensorFusion(axValues, accuracyvalues, false);
  ay = sensorFusion(ayValues, accuracyvalues, false);
  az = sensorFusion(azValues, accuracyvalues, false);
}

void Sensors::getFusedOrientation(float &yaw, float &pitch, float &roll, float &accuracyDegrees)
{
  float yaw1, pitch1, roll1, accuracy1;
  float yaw2, pitch2, roll2, accuracy2;
  float yaw3, pitch3, roll3, accuracy3;

  getOrientation(imu1, yaw1, pitch1, roll1, accuracy1);
  getOrientation(imu2, yaw2, pitch2, roll2, accuracy2);
  getOrientation(imu3, yaw3, pitch3, roll3, accuracy3);

  std::vector<float> yawValues = {yaw1, yaw2, yaw3};
  std::vector<float> pitchValues = {pitch1, pitch2, pitch3};
  std::vector<float> rollValues = {roll1, roll2, roll3};
  std::vector<float> accuracyValues = {accuracy1, accuracy2, accuracy3};

  // call sensorFusion function
  yaw = sensorFusion(yawValues, accuracyValues, true);
  pitch = sensorFusion(pitchValues, accuracyValues, true);
  roll = sensorFusion(rollValues, accuracyValues, true);
}

// get the median value of a vector, filters out outliers
//use bool accuracyDegrees to determine type of accuracy (linear or orientation)
float Sensors::sensorFusion(std::vector<float> values, std::vector<float> accuracy, bool accuracyIsDegrees)
{
  //want to ignore any values that have a negative accuracy
  //check if there is a negative value in the accuracy vector
  //want to check if there is a negative value in accuracy vector, if there is, remove corresponding value from values vector
  std::vector<float> new_values;  //create new vectors to store values and accuracy that are not negative
  std::vector<float> new_accuracy;
  int valuesize = values.size();
  for (int i = 0; i < valuesize; i++)
    {
      if (accuracy[i] >= 0) //if accuracy is positive or 0
        {
          new_values.push_back(values[i]);
          new_accuracy.push_back(accuracy[i]);
        }
    }
  
  if (new_values.size() == 3)
    {
      std::sort(new_values.begin(), new_values.end()); //if there are 3 values, then return the median value, this will ignore outliers
      return new_values[1]; //get middle value
    }
  else if (new_values.size() == 2)
    {
      //if only 2 are working, want to return the value with best accuracy
      // note for linear accuracy (0-3), the higher the value the better
      //for orientation accuracy (degrees), the lower the value the better
      if (accuracyIsDegrees == true) //use accuracyIsDegrees to determine type of accuracy (true if in degrees, false if linear (0-3))
        {
          return new_accuracy[0] < new_accuracy[1] ? new_values[0] : new_values[1]; //if in degrees want lowest accuracy
        }    
      else //if accuracy is linear (0-3) want highest accuracy
        {
          return new_accuracy[0] > new_accuracy[1] ? new_values[0] : new_values[1]; 
        }
    }
    else if (new_values.size() == 1)
    {
      return new_values[0]; //if only 1 sensor is working, then return that value
    }
  else
    {
      //add here error if no sensors are working
      Serial.println("No sensors are working.");
      return -1.0;
    }
}
