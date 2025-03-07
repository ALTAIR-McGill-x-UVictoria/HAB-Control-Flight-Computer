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
  Wire1.begin();
  Wire2.begin();
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

void Sensors::enableReportsForIMU(BNO080 *imu, uint16_t interval)
{
  imu->enableLinearAccelerometer(interval); // m/s^2 no gravity
  imu->enableGyro(interval);                // rad/s
  imu->enableRotationVector(interval);      // quat or yawOrientation/pitchOrientation/rollOrientation rad
}

void Sensors::enableReports(uint16_t interval)
{
  this->interval = interval;
  Wire1.setClock(400000);
  Wire2.setClock(400000);
  enableReportsForIMU(&imu1, interval);
  enableReportsForIMU(&imu2, interval);
  enableReportsForIMU(&imu3, interval);
}

bool Sensors::fetchDataFromIMU(BNO080 *imu, SensorDataIMU *data)
{
  if (imu->hasReset())
  {
    enableReportsForIMU(imu, interval);
  }
  if (imu->dataAvailable())
  {
    data->xLinearAcceleration = imu->getLinAccelX();
    data->yLinearAcceleration = imu->getLinAccelY();
    data->zLinearAcceleration = imu->getLinAccelZ();
    data->linearAccuracy = imu->getLinAccelAccuracy();
    data->xAngularVelocity = imu->getGyroX() * 180.0 / PI;
    data->yAngularVelocity = imu->getGyroY() * 180.0 / PI;
    data->zAngularVelocity = imu->getGyroZ() * 180.0 / PI;
    data->gyroAccuracy = imu->getGyroAccuracy();
    data->yawOrientation = imu->getYaw() * 180.0 / PI;
    data->pitchOrientation = imu->getPitch() * 180.0 / PI;
    data->rollOrientation = imu->getRoll() * 180.0 / PI;
    data->orientationAccuracy = imu->getQuatRadianAccuracy() * 180.0 / PI;
    data->rotationAccuracy = imu->getQuatAccuracy();
    return true;
  }
  return false;
}

void Sensors::invalidateIMUData(unsigned long lastImuUpdateTime, BNO080 *imu, SensorDataIMU *data, unsigned long timeout=500)
{
  if (millis() - lastImuUpdateTime > timeout)
  {
    imu->softReset();
    data->linearAccuracy = -1;
    data->gyroAccuracy = -1;
    data->rotationAccuracy = -1;
    data->orientationAccuracy = -1;
  }
}

void Sensors::altimeterSensorThreadWrapper(void *sensorObj)
{
  ((Sensors *)sensorObj)->altimeterSensorThreadImpl();
}

void Sensors::temperatureSensorThreadWrapper(void *sensorObj)
{
  ((Sensors *)sensorObj)->temperatureSensorThreadImpl();
}

void Sensors::imuSensorThreadWrapper(void *sensorObj)
{
  ((Sensors *)sensorObj)->imuSensorThreadImpl();
}

void Sensors::computeRelativeLinearThreadWrapper(void *sensorObj)
{
  ((Sensors *)sensorObj)->computeRelativeLinearThreadImpl();
}

void Sensors::altimeterSensorThreadImpl()
{
  while (running)
  {
    // Get pressure
    altimeter.readDigitalValue();
    pressure = altimeter.getPressure();
    // Get altitude
    altitude = altimeter.getAltitude();
    // Wait before next reading
    threads.delay(interval);
  }
}

void Sensors::temperatureSensorThreadImpl()
{
  while (running)
  {
    // Get temperature
    temperature = temperatureProbe.temperature(RNOMINAL, RREF);
    // Wait before next reading
    threads.delay(interval);
  }
}

void Sensors::imuSensorThreadImpl()
{
  while (running)
  {
    // Collect IMU data
    bool fetchedIMU1 = fetchDataFromIMU(&imu1, &imu1Data);
    bool fetchedIMU2 = fetchDataFromIMU(&imu2, &imu2Data);
    bool fetchedIMU3 = fetchDataFromIMU(&imu3, &imu3Data);
    // Update last update time
    if (fetchedIMU1)
      lastImu1UpdateTime = millis();
    else
      invalidateIMUData(lastImu1UpdateTime, &imu1, &imu1Data);
    if (fetchedIMU2)
      lastImu2UpdateTime = millis();
    else
      invalidateIMUData(lastImu2UpdateTime, &imu2, &imu2Data);
    if (fetchedIMU3)
      lastImu3UpdateTime = millis();
    else
      invalidateIMUData(lastImu3UpdateTime, &imu3, &imu3Data);
    threads.yield();
  }
}

void Sensors::computeRelativeLinearThreadImpl()
{
  while (running)
  {
    float ax = 0.0, ay = 0.0, az = 0.0;
    getFusedLinearAcceleration(ax, ay, az);
    unsigned long currentTime = millis();
    float dt = (currentTime - lastRelativeLinearUpdateTime) / 1000.0; // Convert to seconds
    lastRelativeLinearUpdateTime = currentTime;
    // Integrate acceleration to get velocity
    xRelativeVelocity = xRelativeVelocity + ax * dt;
    yRelativeVelocity = yRelativeVelocity + ay * dt;
    zRelativeVelocity = zRelativeVelocity + az * dt;
    // Integrate velocity to get position
    xRelativePosition = xRelativePosition + xRelativeVelocity * dt;
    yRelativePosition = yRelativePosition + yRelativeVelocity * dt;
    zRelativePosition = zRelativePosition + zRelativeVelocity * dt;
    // Wait before next reading
    threads.delay(interval);
  }
}

void Sensors::startDataCollection()
{
  running = true;
  altimeterSensorThreadId = threads.addThread(altimeterSensorThreadWrapper, this);
  temperatureSensorThreadId = threads.addThread(temperatureSensorThreadWrapper, this);
  imuSensorThreadId = threads.addThread(imuSensorThreadWrapper, this);
  computeRelativeLinearThreadId = threads.addThread(computeRelativeLinearThreadWrapper, this);
}

void Sensors::stopDataCollection()
{
  running = false;
  threads.delay(100); // Give threads time to finish
  threads.kill(altimeterSensorThreadId);
  threads.kill(temperatureSensorThreadId);
  threads.kill(imuSensorThreadId);
  threads.kill(computeRelativeLinearThreadId);
}

float Sensors::getTemperature()
{
  return this->temperature;
}

float Sensors::getPressure()
{
  return this->pressure;
}

float Sensors::getAltitude()
{
  return this->altitude;
}

// use median filter to get the most accurate data out of IMU1, IMU2, IMU3
void Sensors::getFusedLinearAcceleration(float &xLinearAcceleration, float &yLinearAcceleration, float &zLinearAcceleration)
{
  // first get info from the struct
  std::vector<float> axValues = {imu1Data.xLinearAcceleration, imu2Data.xLinearAcceleration, imu3Data.xLinearAcceleration};
  std::vector<float> ayValues = {imu1Data.yLinearAcceleration, imu2Data.yLinearAcceleration, imu3Data.yLinearAcceleration};
  std::vector<float> azValues = {imu1Data.zLinearAcceleration, imu2Data.zLinearAcceleration, imu3Data.zLinearAcceleration};
  std::vector<byte> accuracyValues = {imu1Data.linearAccuracy, imu2Data.linearAccuracy, imu3Data.linearAccuracy};

  // call sensorFusion function
  xLinearAcceleration = sensorFusion(axValues, accuracyValues);
  yLinearAcceleration = sensorFusion(ayValues, accuracyValues);
  zLinearAcceleration = sensorFusion(azValues, accuracyValues);
}

void Sensors::getFusedAngularVelocity(float &xAngularVelocity, float &yAngularVelocity, float &zAngularVelocity)
{
  std::vector<float> gxValues = {imu1Data.xAngularVelocity, imu2Data.xAngularVelocity, imu3Data.xAngularVelocity};
  std::vector<float> gyValues = {imu1Data.yAngularVelocity, imu2Data.yAngularVelocity, imu3Data.yAngularVelocity};
  std::vector<float> gzValues = {imu1Data.zAngularVelocity, imu2Data.zAngularVelocity, imu3Data.zAngularVelocity};
  std::vector<byte> accuracyvalues = {imu1Data.gyroAccuracy, imu2Data.gyroAccuracy, imu3Data.gyroAccuracy};

  // call sensorFusion function
  xAngularVelocity = sensorFusion(gxValues, accuracyvalues);
  yAngularVelocity = sensorFusion(gyValues, accuracyvalues);
  zAngularVelocity = sensorFusion(gzValues, accuracyvalues);
}

void Sensors::getFusedOrientation(float &yawOrientation, float &pitchOrientation, float &rollOrientation)
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
}

void Sensors::getRelativeVelocity(float &vx, float &vy, float &vz)
{
  vx = xRelativeVelocity;
  vy = yRelativeVelocity;
  vz = zRelativeVelocity;
}

void Sensors::getRelativePosition(float &px, float &py, float &pz)
{
  px = xRelativePosition;
  py = yRelativePosition;
  pz = zRelativePosition;
}

void Sensors::setRelativeVelocity(float vx, float vy, float vz)
{
  xRelativeVelocity = vx;
  yRelativeVelocity = vy;
  zRelativeVelocity = vz;
}

void Sensors::setRelativePosition(float px, float py, float pz)
{
  xRelativePosition = px;
  yRelativePosition = py;
  zRelativePosition = pz;
}

// get the median value of a vector, filters out outliers
// use bool accuracyDegrees to determine type of accuracy (linear or orientation)
float Sensors::sensorFusion(std::vector<float> values, std::vector<byte> accuracy, std::vector<float> orientationAccuracy)
{
  // want to ignore any values that have a negative accuracy
  // check if there is a negative value in the accuracy vector
  // want to check if there is a negative value in accuracy vector, if there is, remove corresponding value from values vector
  std::vector<float> new_values; // create new vectors to store values and accuracy that are not negative
  std::vector<float> new_accuracy;
  std::vector<float> new_orientationAccuracy;
  int valuesize = values.size();
  for (int i = 0; i < valuesize; i++)
  {
    if (accuracy[i] >= 0) // if accuracy is positive or 0
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
    std::sort(new_values.begin(), new_values.end()); // if there are 3 values, then return the median value, this will ignore outliers
    return new_values[1];                            // get middle value
  }
  else if (new_values.size() == 2)
  {
    // if only 2 are working, want to return the value with best accuracy
    //  note for linear accuracy (0-3), the higher the value the better
    // for orientation accuracy (degrees), the lower the value the better
    if (orientationAccuracy.empty()) // use accuracyIsDegrees to determine type of accuracy (1 if is empty, 0 if not)
    {
      return new_accuracy[0] > new_accuracy[1] ? new_values[0] : new_values[1]; // if accuracy is linear (0-3) want highest accuracy
    }
    else
    {
      return new_orientationAccuracy[0] < new_orientationAccuracy[1] ? new_values[0] : new_values[1]; // if in degrees want lowest accuracy
    }
  }
  else if (new_values.size() == 1)
  {
    return new_values[0]; // if only 1 sensor is working, then return that value
  }
  else
  {
    // add here error if no sensors are working
    Serial.println("No sensors are working.");
    return -1.0;
  }
}
