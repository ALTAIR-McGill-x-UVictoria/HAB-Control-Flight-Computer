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

#include "Sensors.h"

SensorStatus Sensors::begin(SensorStatus status)
{
  if (!status.imu1)
    status.imu1 = imu1.begin(0x4A, Wire1, -1);
  if (!status.imu2)
    status.imu2 = imu2.begin(0x4B, Wire1, -1);
  if (!status.imu3)
    status.imu3 = imu3.begin(0x4A, Wire2, -1);
  if (!status.pressure)
    status.pressure = altimeter.begin();
  if (!status.temperature)
    status.temperature = temperatureProbe.begin(MAX31865_3WIRE);
  return status;
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

void Sensors::enableIMUReports(uint16_t interval)
{
  this->interval = interval;
  imu1.enableLinearAccelerometer(interval); // m/s^2 no gravity
  imu1.enableRotationVector(interval);      // quat or yawOrientation/pitchOrientation/rollOrientation rad
  imu1.enableGyro(interval);                // rad/s
}

bool Sensors::collectIMUData()
{
  if (imu1.hasReset())
  {
    enableIMUReports(interval);
    Serial.println(" ------------------ BNO085 has reset. ------------------ ");
    Serial.print(F(" Reason: "));
    Serial.println(imu1.resetReason());
  }
  if (imu1.dataAvailable())
  {
    imu1Data.xLinearAcceleration = imu1.getLinAccelX();
    imu1Data.yLinearAcceleration = imu1.getLinAccelY();
    imu1Data.zLinearAcceleration = imu1.getLinAccelZ();
    imu1Data.linearAccuracy = imu1.getLinAccelAccuracy();
    imu1Data.xAngularVelocity = imu1.getGyroX() * 180 / PI;
    imu1Data.yAngularVelocity = imu1.getGyroY() * 180 / PI;
    imu1Data.zAngularVelocity = imu1.getGyroZ() * 180 / PI;
    imu1Data.gyroAccuracy = imu1.getGyroAccuracy();
    imu1Data.yawOrientation = imu1.getYaw() * 180 / PI;
    imu1Data.pitchOrientation = imu1.getPitch() * 180 / PI;
    imu1Data.rollOrientation = imu1.getRoll() * 180 / PI;
    imu1Data.orientationAccuracy = imu1.getQuatRadianAccuracy() * 180 / PI;
    imu1Data.rotationAccuracy = imu1.getQuatAccuracy();
    return true;
  }
  return false;
}

/*

// use median filter to get the most accurate data out of IMU1, IMU2, IMU3
void Sensors::getFusedLinearAcceleration(float &xLinearAcceleration, float &yLinearAcceleration, float &zLinearAcceleration, byte &linAccuracy)
{
  //first get info from the struct
  std::vector<float> axValues = {imu1Data.xLinearAcceleration, imu2Data.xLinearAcceleration, imu3Data.xLinearAcceleration};
  std::vector<float> ayValues = {imu1Data.yLinearAcceleration, imu2Data.yLinearAcceleration, imu3Data.yLinearAcceleration};
  std::vector<float> azValues = {imu1Data.zLinearAcceleration, imu2Data.zLinearAcceleration, imu3Data.zLinearAcceleration};
  std::vector<byte> accuracyValues = {imu1Data.linearAccuracy, imu2Data.linearAccuracy, imu3Data.linearAccuracy};

  // call sensorFusion function
  xLinearAcceleration = sensorFusion(axValues, accuracyValues);
  yLinearAcceleration = sensorFusion(ayValues, accuracyValues);
  zLinearAcceleration = sensorFusion(azValues, accuracyValues);
  linAccuracy = 0; //not sure what to do with this yet
}

void Sensors::getFusedAngularVelocity(float &xAngularVelocity, float &yAngularVelocity, float &zAngularVelocity, byte &gyroAccuracy)
{
  std::vector<float> gxValues = {imu1Data.xAngularVelocity, imu2Data.xAngularVelocity, imu3Data.xAngularVelocity};
  std::vector<float> gyValues = {imu1Data.yAngularVelocity, imu2Data.yAngularVelocity, imu3Data.yAngularVelocity};
  std::vector<float> gzValues = {imu1Data.zAngularVelocity, imu2Data.zAngularVelocity, imu3Data.zAngularVelocity};
  std::vector<byte> accuracyvalues = {imu1Data.gyroAccuracy, imu2Data.gyroAccuracy, imu3Data.gyroAccuracy};

  // call sensorFusion function
  xAngularVelocity = sensorFusion(gxValues, accuracyvalues);
  yAngularVelocity = sensorFusion(gyValues, accuracyvalues);
  zAngularVelocity = sensorFusion(gzValues, accuracyvalues);
  gyroAccuracy = 0; //not sure what to do with this yet
}

void Sensors::getFusedOrientation(float &yawOrientation, float &pitchOrientation, float &rollOrientation, float &accuracyDegrees, byte &rotationAccuracy)
{

  std::vector<float> yawValues = {imu1Data.yawOrientation, imu2Data.yawOrientation, imu3Data.yawOrientation};
  std::vector<float> pitchValues = {imu1Data.pitchOrientation, imu2Data.pitchOrientation, imu3Data.pitchOrientation};
  std::vector<float> rollValues = {imu1Data.rollOrientation, imu2Data.rollOrientation, imu3Data.rollOrientation};
  std::vector<byte> accuracyValues = {imu1Data.rotationAccuracy, imu2Data.rotationAccuracy, imu3Data.rotationAccuracy};
  std::vector<float> orientationAccuracy = {imu1Data.orientationAccuracy, imu2Data.orientationAccuracy, imu3Data.orientationAccuracy};


  // call sensorFusion function
  yawOrientation = sensorFusion(yawValues, accuracyValues, orientationAccuracy);
  pitchOrientation = sensorFusion(pitchValues, accuracyValues, orientationAccuracy);
  rollOrientation = sensorFusion(rollValues, accuracyValues, orientationAccuracy);
  accuracyDegrees = 0; //not sure what to do with this yet
  rotationAccuracy = 0; //not sure what to do with this yet

}

// get the median value of a vector, filters out outliers
//use bool accuracyDegrees to determine type of accuracy (linear or orientation)
float Sensors::sensorFusion(std::vector<float> values, std::vector<byte> accuracy, std::vector<float> orientationAccuracy)
{
  //want to ignore any values that have a negative accuracy
  //check if there is a negative value in the accuracy vector
  //want to check if there is a negative value in accuracy vector, if there is, remove corresponding value from values vector
  std::vector<float> new_values;  //create new vectors to store values and accuracy that are not negative
  std::vector<float> new_accuracy;
  std::vector<float> new_orientationAccuracy;
  int valuesize = values.size();
  for (int i = 0; i < valuesize; i++)
    {
      if (accuracy[i] >= 0) //if accuracy is positive or 0
        {
          new_values.push_back(values[i]);
          new_accuracy.push_back(accuracy[i]);
          if (!orientationAccuracy.empty())
            {
              new_orientationAccuracy.push_back(orientationAccuracy[i]);
            }

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
      if (orientationAccuracy.empty()) //use accuracyIsDegrees to determine type of accuracy (1 if is empty, 0 if not)
        {
          return new_accuracy[0] > new_accuracy[1] ? new_values[0] : new_values[1]; //if accuracy is linear (0-3) want highest accuracy
        }
      else
        {
          return new_orientationAccuracy[0] < new_orientationAccuracy[1] ? new_values[0] : new_values[1]; //if in degrees want lowest accuracy
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


void Sensors::getRelativeVelocity(float &vx, float &vy, float &vz)
{
  float ax = 0.0, ay = 0.0, az = 0.0;

  unsigned long currentTime = millis();
  float dt = (currentTime - lastVelocityUpdateTime) / 1000.0; // Convert to seconds
  lastVelocityUpdateTime = currentTime;

  // Integrate acceleration to get velocity
  vx = this->vx + ax * dt;
  vy = this->vy + ay * dt;
  vz = this->vz + az * dt;

  setRelativeVelocity(vx, vy, vz);
}
void Sensors::getRelativePosition(float &px, float &py, float &pz)
{
  float vx = 0.0, vy = 0.0, vz = 0.0;
  getRelativeVelocity(vx, vy, vz);

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
*/