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

float Sensors::filtered_roll = 0.0f;
float Sensors::filtered_pitch = 0.0f;
float Sensors::filtered_yaw = 0.0f;

void Sensors::begin()
{
  if (!status.imu1 && !status.imu2)
    Wire1.begin();
  if (!status.imu3)
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
}

void Sensors::enableReportsForIMU(BNO080 *imu, uint16_t interval)
{
  imu->enableLinearAccelerometer(interval); // m/s^2 no gravity
  imu->enableGyro(interval);                // rad/s
  imu->enableRotationVector(interval);      // quat or yawOrientation/pitchOrientation/rollOrientation rad
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
    data->quatI = imu->getQuatI();
    data->quatJ = imu->getQuatJ();
    data->quatK = imu->getQuatK();
    data->quatReal = imu->getQuatReal();
    return true;
  }
  return false;
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
  lastAltimeterUpdateTime = millis(); // Initialize last update time
  while (running)
  {
    if (status.pressure && altimeter.readDigitalValue())
    {
      lastAltimeterUpdateTime = millis();
      pressure = altimeter.getPressure();
      altitude = altimeter.getAltitude();
      status.pressure = true;
    }
    else if (millis() - lastAltimeterUpdateTime > ALTIMETER_TIMEOUT)
    {
      // Mark the altimeter as invalid
      altitude = -10000000.0;                    // set to -10000000.0 if no data is fetched
      status.pressure = altimeter.begin(); // Reinitialize the altimeter;
      lastAltimeterUpdateTime = millis();
    }
    threads.delay(interval);
  }
}

void Sensors::temperatureSensorThreadImpl()
{
  unsigned long lastTemperatureUpdateTime = millis();
  while (running)
  {
    // Get temperature and check if it's valid (not NaN)
    temperature = temperatureProbe.temperature(RNOMINAL, RREF);
    if (!isnan(temperature))
    {
      status.temperature = true;
      lastTemperatureUpdateTime = millis();
    }
    else if (millis() - lastTemperatureUpdateTime > TEMPERATURE_TIMEOUT)
    {
      // Mark the temperature probe as invalid
      status.temperature = false;
      lastTemperatureUpdateTime = millis();
    }

    threads.delay(interval);
  }
}

void Sensors::processIMUSensor(BNO080 &imu, SensorDataIMU &data, unsigned long &lastUpdateTime, bool &statusFlag)
{
  if (statusFlag && fetchDataFromIMU(&imu, &data))
  {
    lastUpdateTime = millis();
    statusFlag = true;
  }
  else if (millis() - lastUpdateTime > IMU_TIMEOUT)
  {
    // Mark data as invalid
    data.linearAccuracy = -1;
    data.gyroAccuracy = -1;
    data.rotationAccuracy = -1;
    data.orientationAccuracy = -1;
    statusFlag = false;
    lastUpdateTime = millis();
  }
}

void Sensors::imuSensorThreadImpl()
{
  lastImu1UpdateTime = lastImu2UpdateTime = lastImu3UpdateTime = millis();
  while (running)
  {
    processIMUSensor(imu1, imu1Data, lastImu1UpdateTime, status.imu1);
    processIMUSensor(imu2, imu2Data, lastImu2UpdateTime, status.imu2);
    processIMUSensor(imu3, imu3Data, lastImu3UpdateTime, status.imu3);
    threads.yield();
  }
}

void Sensors::computeRelativeLinearThreadImpl()
{
  while (running)
  {
    // Only get world frame accelerations
    float axWorld = 0.0, ayWorld = 0.0, azWorld = 0.0;
    getFusedWorldLinearAcceleration(axWorld, ayWorld, azWorld);
    
    unsigned long currentTime = millis();
    float dt = (currentTime - lastRelativeLinearUpdateTime) / 1000.0; // Convert to seconds
    lastRelativeLinearUpdateTime = currentTime;
    
    // Integrate world frame acceleration to get velocity in world frame
    xRelativeVelocity = xRelativeVelocity + axWorld * dt;
    yRelativeVelocity = yRelativeVelocity + ayWorld * dt;
    zRelativeVelocity = zRelativeVelocity + azWorld * dt;
    
    // Integrate world frame velocity to get position in world frame
    xRelativePosition = xRelativePosition + xRelativeVelocity * dt;
    yRelativePosition = yRelativePosition + yRelativeVelocity * dt;
    zRelativePosition = zRelativePosition + zRelativeVelocity * dt;
    
    // Wait before next reading
    threads.delay(interval);
  }
}

void Sensors::start(uint16_t interval)
{
  running = true;
  this->interval = interval;
  Wire1.setClock(400000);
  Wire2.setClock(400000);
  setRelativePosition(0.0, 0.0, 0.0);
  setRelativeVelocity(0.0, 0.0, 0.0);
  threads.delay(100);
  enableReportsForIMU(&imu1, interval);
  threads.delay(100);
  enableReportsForIMU(&imu2, interval);
  threads.delay(100);
  enableReportsForIMU(&imu3, interval);
  threads.delay(100);
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
  static bool allSensorsWorking = true;
  std::vector<float> yawValues = {imu1Data.yawOrientation, imu2Data.yawOrientation, imu3Data.yawOrientation};
  std::vector<float> pitchValues = {imu1Data.pitchOrientation, imu2Data.pitchOrientation, imu3Data.pitchOrientation};
  std::vector<float> rollValues = {imu1Data.rollOrientation, imu2Data.rollOrientation, imu3Data.rollOrientation};
  std::vector<byte> accuracyValues = {imu1Data.rotationAccuracy, imu2Data.rotationAccuracy, imu3Data.rotationAccuracy};
  std::vector<float> orientationAccuracy = {imu1Data.orientationAccuracy, imu2Data.orientationAccuracy, imu3Data.orientationAccuracy};

  // Check if any sensors are valid
  bool anySensorValid = false;
  for (byte accuracy : accuracyValues) {
    if (accuracy >= 0) {
      anySensorValid = true;
      break;
    }
  }
  
  if (!anySensorValid && allSensorsWorking) {
    Serial.println("WARNING: All orientation sensors have invalid data!");
    allSensorsWorking = false;
  } else if (anySensorValid && !allSensorsWorking) {
    Serial.println("INFO: At least one orientation sensor is now working");
    allSensorsWorking = true;
  }

  // call sensorFusion function
  yawOrientation = sensorFusion(yawValues, accuracyValues, orientationAccuracy);
  pitchOrientation = sensorFusion(pitchValues, accuracyValues, orientationAccuracy);
  rollOrientation = sensorFusion(rollValues, accuracyValues, orientationAccuracy);
}

void Sensors::getFusedQuaternion(float &quatI, float &quatJ, float &quatK, float &quatReal)
{
  std::vector<float> iValues = {imu1Data.quatI, imu2Data.quatI, imu3Data.quatI};
  std::vector<float> jValues = {imu1Data.quatJ, imu2Data.quatJ, imu3Data.quatJ};
  std::vector<float> kValues = {imu1Data.quatK, imu2Data.quatK, imu3Data.quatK};
  std::vector<float> realValues = {imu1Data.quatReal, imu2Data.quatReal, imu3Data.quatReal};
  std::vector<byte> accuracyValues = {imu1Data.rotationAccuracy, imu2Data.rotationAccuracy, imu3Data.rotationAccuracy};

  quatI = sensorFusion(iValues, accuracyValues);
  quatJ = sensorFusion(jValues, accuracyValues);
  quatK = sensorFusion(kValues, accuracyValues);
  quatReal = sensorFusion(realValues, accuracyValues);
  
  // Normalize the quaternion
  float norm = sqrt(quatI*quatI + quatJ*quatJ + quatK*quatK + quatReal*quatReal);
  if (norm > 0.0001) {
    quatI /= norm;
    quatJ /= norm;
    quatK /= norm;
    quatReal /= norm;
  }
}

void Sensors::getFusedWorldLinearAcceleration(float &xLinearAcceleration, float &yLinearAcceleration, float &zLinearAcceleration)
{
  // First get fused IMU frame accelerations
  float xAcc, yAcc, zAcc;
  getFusedLinearAcceleration(xAcc, yAcc, zAcc);
  
  // Get fused quaternion
  float quatI, quatJ, quatK, quatReal;
  getFusedQuaternion(quatI, quatJ, quatK, quatReal);
  
  // Transform to world frame
  transformToWorldFrame(xAcc, yAcc, zAcc, quatI, quatJ, quatK, quatReal, 
                        xLinearAcceleration, yLinearAcceleration, zLinearAcceleration);
}

void Sensors::getFusedWorldAngularVelocity(float &xAngularVelocity, float &yAngularVelocity, float &zAngularVelocity)
{
  // First get fused IMU frame angular velocities
  float xGyro, yGyro, zGyro;
  getFusedAngularVelocity(xGyro, yGyro, zGyro);
  
  // Get fused quaternion
  float quatI, quatJ, quatK, quatReal;
  getFusedQuaternion(quatI, quatJ, quatK, quatReal);
  
  // Transform to world frame
  transformToWorldFrame(xGyro, yGyro, zGyro, quatI, quatJ, quatK, quatReal, 
                       xAngularVelocity, yAngularVelocity, zAngularVelocity);
}

void Sensors::transformToWorldFrame(float x, float y, float z, 
                                  float quatI, float quatJ, float quatK, float quatReal,
                                  float &xWorld, float &yWorld, float &zWorld)
{
  // To transform a vector from sensor frame to world frame:
  // v_world = q^-1 * v_sensor * q
  // Where q is the quaternion representing rotation from world to sensor frame
  
  // Get quaternion conjugate (inverse for unit quaternions)
  float conjI, conjJ, conjK, conjReal;
  quaternionConjugate(quatI, quatJ, quatK, quatReal, conjI, conjJ, conjK, conjReal);
  
  // Create quaternion for the vector (0, x, y, z)
  float vecI = x;
  float vecJ = y;
  float vecK = z;
  float vecReal = 0.0;
  
  // Compute q^-1 * v_sensor
  float tempI, tempJ, tempK, tempReal;
  quaternionMultiply(conjI, conjJ, conjK, conjReal,
                    vecI, vecJ, vecK, vecReal,
                    tempI, tempJ, tempK, tempReal);
  
  // Compute (q^-1 * v_sensor) * q
  quaternionMultiply(tempI, tempJ, tempK, tempReal,
                    quatI, quatJ, quatK, quatReal,
                    vecI, vecJ, vecK, vecReal);
  
  // The result is (0, xWorld, yWorld, zWorld)
  xWorld = vecI;
  yWorld = vecJ;
  zWorld = vecK;
}

void Sensors::quaternionMultiply(float a_i, float a_j, float a_k, float a_real,
                               float b_i, float b_j, float b_k, float b_real,
                               float &c_i, float &c_j, float &c_k, float &c_real)
{
  // Standard quaternion multiplication: c = a * b
  c_real = a_real*b_real - a_i*b_i - a_j*b_j - a_k*b_k;
  c_i = a_real*b_i + a_i*b_real + a_j*b_k - a_k*b_j;
  c_j = a_real*b_j - a_i*b_k + a_j*b_real + a_k*b_i;
  c_k = a_real*b_k + a_i*b_j - a_j*b_i + a_k*b_real;
}

void Sensors::quaternionConjugate(float i, float j, float k, float real, 
                                float &conj_i, float &conj_j, float &conj_k, float &conj_real)
{
  // The conjugate of a quaternion (i, j, k, real) is (-i, -j, -k, real)
  conj_i = -i;
  conj_j = -j;
  conj_k = -k;
  conj_real = real;
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
    // add here if no sensors are working
    return 0.0;
  }
}