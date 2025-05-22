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

// std::map<std::string, float> Sensors::lastFusedValues; // Key is a unique ID for each measurement type

void Sensors::begin()
{
  // Always initialize I2C buses
  Wire1.begin();
  Wire2.begin();
  
  // Set higher clock speed for I2C
  Wire1.setClock(400000);
  Wire2.setClock(400000);
  
  // Wait for buses to stabilize
  delay(100);
  
  Serial.println("Initializing sensors...");
  
  // First properly initialize temperature and pressure sensors
  status.temperature = temperatureProbe.begin(MAX31865_3WIRE);
  if (status.temperature) {
    Serial.println("Temperature sensor initialized successfully");
  } else {
    Serial.println("Failed to initialize temperature sensor");
  }
  
  status.pressure = altimeter.begin();
  if (status.pressure) {
    Serial.println("Pressure sensor initialized successfully");
  } else {
    Serial.println("Failed to initialize pressure sensor");
  }
  
  // Then initialize IMUs with retry logic
  Serial.println("Initializing IMU1...");
  status.imu1 = initializeSingleIMU(imu1, Wire1, 0x4A);
  
  Serial.println("Initializing IMU2...");
  status.imu2 = initializeSingleIMU(imu2, Wire1, 0x4B);
  
  Serial.println("Initializing IMU3...");
  status.imu3 = initializeSingleIMU(imu3, Wire2, 0x4A);
  
  // Summary of initialization results
  Serial.println("\nSensor initialization complete:");
  Serial.print("- Temperature sensor: "); Serial.println(status.temperature ? "OK" : "FAILED");
  Serial.print("- Pressure sensor: "); Serial.println(status.pressure ? "OK" : "FAILED");
  Serial.print("- IMU1: "); Serial.println(status.imu1 ? "OK" : "FAILED");
  Serial.print("- IMU2: "); Serial.println(status.imu2 ? "OK" : "FAILED");
  Serial.print("- IMU3: "); Serial.println(status.imu3 ? "OK" : "FAILED");
  
  // Initialize GPS data with default values
  gpsData.latitude = 0.0f;
  gpsData.longitude = 0.0f;
  gpsData.altitude = 0.0f;
  gpsData.speed = 0.0f;
  gpsData.course = 0.0f;
  gpsData.vx = 0.0f;
  gpsData.vy = 0.0f;
  gpsData.vz = 0.0f;
  gpsData.satellites = 0;
  gpsData.fixQuality = 0;
  gpsData.timestamp = 0;
  gpsData.valid = false;
  
  sensorsInitialized = true;
}

bool Sensors::initializeSingleIMU(BNO080 &imu, TwoWire &wire, uint8_t address, int maxRetries) 
{
  for (int attempt = 1; attempt <= maxRetries; attempt++) {
    Serial.printf("  Attempt %d of %d...", attempt, maxRetries);
    
    // Try to initialize the IMU
    if (imu.begin(address, wire, -1)) {
      Serial.println("SUCCESS");
      
      // Configure the IMU after successful initialization
      delay(50);  // Brief stabilization delay
      
      // Enable basic reports to check communication
      imu.enableRotationVector(50);
      
      // Wait for data to verify connection
      unsigned long startTime = millis();
      while (millis() - startTime < 300) {  // Short timeout
        if (imu.dataAvailable()) {
          // Read a value to confirm communication works
          imu.getQuatI();
          Serial.println("  Communication verified");
          return true;
        }
        delay(10);
      }
      
      Serial.println("  Communication check failed, retrying...");
    } else {
      Serial.println("FAILED");
    }
    
    // Reset connection before retry
    wire.end();
    delay(100);
    wire.begin();
    wire.setClock(400000);
    delay(100);
  }
  
  Serial.println("  All attempts failed");
  return false;
}

bool Sensors::areSensorsReady() {
  if (!sensorsInitialized) return false;
  
  // Required sensors - check that at least two IMUs are working
  // (we can function with two of three IMUs)
  int workingIMUs = 0;
  if (status.imu1) workingIMUs++;
  if (status.imu2) workingIMUs++;
  if (status.imu3) workingIMUs++;
  
  // Minimum 2 IMUs required for reliable operation
  return (workingIMUs >= 2) && status.pressure;
}

bool Sensors::resetIMU(int imuNumber) {
  bool result = false;
  
  switch(imuNumber) {
    case 1:
      Serial.println("Resetting IMU1...");
      status.imu1 = initializeSingleIMU(imu1, Wire1, 0x4A);
      result = status.imu1;
      break;
    case 2:
      Serial.println("Resetting IMU2...");
      status.imu2 = initializeSingleIMU(imu2, Wire1, 0x4B);
      result = status.imu2;
      break;
    case 3:
      Serial.println("Resetting IMU3...");
      status.imu3 = initializeSingleIMU(imu3, Wire2, 0x4A);
      result = status.imu3;
      break;
  }
  
  return result;
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
  if (statusFlag) {
    i2c_mutex.lock();
    bool success = fetchDataFromIMU(&imu, &data);
    i2c_mutex.unlock();
    
    if (success) {
      lastUpdateTime = millis();
      statusFlag = true;
    }
  }
  else if (millis() - lastUpdateTime > IMU_TIMEOUT) {
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
        // Only process IMUs that were initialized successfully
        if (status.imu1) {
            processIMUSensor(imu1, imu1Data, lastImu1UpdateTime, status.imu1);
        }
        
        if (status.imu2) {
            processIMUSensor(imu2, imu2Data, lastImu2UpdateTime, status.imu2);
        }
        
        if (status.imu3) {
            processIMUSensor(imu3, imu3Data, lastImu3UpdateTime, status.imu3);
        }
        
        // Always yield to give other threads a chance
        threads.yield();
        
        // Short delay to prevent tight loop hammering the I2C bus
        threads.delay(1);
    }
}

void Sensors::computeRelativeLinearThreadImpl()
{
  // State vector: [position, velocity, acceleration, bias]
  float state[9] = {0}; // [px, py, pz, vx, vy, vz, ax_bias, ay_bias, az_bias]
  float P[9][9] = {0};  // Covariance matrix
  
  // Initialize covariance matrix with uncertainty
  for (int i = 0; i < 9; i++) P[i][i] = 1.0;
  
  // For GPS-based corrections
  unsigned long lastGpsUpdateTime = 0;
  const unsigned long GPS_UPDATE_INTERVAL = 1000; // Check GPS every 1 second
  
  while (running)
  {
    // Get world frame accelerations
    float axWorld = 0.0f, ayWorld = 0.0f, azWorld = 0.0f;
    getFusedWorldLinearAcceleration(axWorld, ayWorld, azWorld);
    
    // Apply additional filtering to reduce noise
    filterAcceleration(axWorld, ayWorld, azWorld);
    
    unsigned long currentTime = millis();
    float dt = (currentTime - lastRelativeLinearUpdateTime) / 1000.0f;
    lastRelativeLinearUpdateTime = currentTime;
    
    // Limit dt to reasonable values in case of delays
    if (dt > 0.1f) dt = 0.1f;
    
    // 1. Prediction step with bias compensation
    float ax_corrected = axWorld - state[6];
    float ay_corrected = ayWorld - state[7];
    float az_corrected = azWorld - state[8];
    
    // Apply deadband filter to very small accelerations
    // Apply stronger deadband when stationary
    float dynamic_deadband = was_stationary ? 0.5f : 0.30f;
    if (fabs(ax_corrected) < dynamic_deadband) ax_corrected = 0.0f;
    if (fabs(ay_corrected) < dynamic_deadband) ay_corrected = 0.0f;
    if (fabs(az_corrected) < dynamic_deadband) az_corrected = 0.0f;
    
    // Use Runge-Kutta integration for better accuracy
    rungeKutta4Integration(state[0], state[3], ax_corrected, dt); // X position
    rungeKutta4Integration(state[1], state[4], ay_corrected, dt); // Y position
    rungeKutta4Integration(state[2], state[5], az_corrected, dt); // Z position
    
    // 2. Zero-velocity update detection
    detectZeroVelocityAndUpdate(state, P, axWorld, ayWorld, azWorld);
    
    // 3. GPS update when available (placeholder for now)
    if (currentTime - lastGpsUpdateTime >= GPS_UPDATE_INTERVAL) {
      lastGpsUpdateTime = currentTime;
      
      // For testing - simulate GPS data (replace with real GPS reading later)
      simulateGPSData();
      
      // When GPS is valid, use it to correct position and velocity drift
      if (gpsData.valid && gpsData.fixQuality > 0) {
        // Calculate GPS velocity components from course and speed
        calculateGPSVelocityComponents();
        
        // GPS position correction with trust factor based on fix quality
        float positionTrustFactor = 0.0f;
        switch (gpsData.fixQuality) {
          case 1: positionTrustFactor = 0.01f; break;  // Basic GPS (~10m accuracy)
          case 2: positionTrustFactor = 0.03f; break;  // DGPS (~3m accuracy)
          case 4: positionTrustFactor = 0.10f; break;  // RTK (~10cm accuracy)
          case 5: positionTrustFactor = 0.20f; break;  // Float RTK (~20cm accuracy)
          default: positionTrustFactor = 0.005f;       // Low trust for unknown quality
        }
        
        // Only apply horizontal position updates (X/Y) from GPS
        // For Z position, we use the barometer and GPS altitude together
        
        // First GPS fix after startup - initialize position reference
        static bool firstGpsFix = true;
        static float initialLat = 0, initialLon = 0;
        
        if (firstGpsFix) {
          initialLat = gpsData.latitude;
          initialLon = gpsData.longitude;
          firstGpsFix = false;
        } else {
          // Convert GPS lat/lon deltas to meters (simplified flat earth model)
          // More accurate calculation would use Haversine formula
          const float DEG_TO_METERS = 111319.9f; // Approximate meters per degree at equator
          
          float latDelta = (gpsData.latitude - initialLat) * DEG_TO_METERS;
          float lonDelta = (gpsData.longitude - initialLon) * DEG_TO_METERS * 
                          cos(gpsData.latitude * PI/180.0f);
                          
          // Apply correction to position estimates (X/Y only)
          float gpsX = latDelta; // North direction as X
          float gpsY = lonDelta; // East direction as Y
          
          // Complementary filter for position - blend IMU integration with GPS
          state[0] = (1.0f - positionTrustFactor) * state[0] + positionTrustFactor * gpsX;
          state[1] = (1.0f - positionTrustFactor) * state[1] + positionTrustFactor * gpsY;
        }
        
        // Apply velocity corrections from GPS
        float velocityTrustFactor = positionTrustFactor * 2.0f; // Typically trust velocity more
        if (velocityTrustFactor > 0.5f) velocityTrustFactor = 0.5f; // Cap at 0.5
        
        state[3] = (1.0f - velocityTrustFactor) * state[3] + velocityTrustFactor * gpsData.vx;
        state[4] = (1.0f - velocityTrustFactor) * state[4] + velocityTrustFactor * gpsData.vy;
        
        // Z velocity from GPS is usually less accurate, lower weight
        state[5] = (1.0f - velocityTrustFactor * 0.5f) * state[5] + 
                   (velocityTrustFactor * 0.5f) * gpsData.vz;
      }
      
      // Update altitude using barometer and GPS in complementary filter
      updateAltitudeWithComplementaryFilter();
      
      // Override Z position with the filtered altitude value
      state[2] = zRelativePosition;
    }
    
    // Update class variables
    xRelativePosition = state[0];
    yRelativePosition = state[1];
    zRelativePosition = state[2];
    xRelativeVelocity = state[3];
    yRelativeVelocity = state[4];
    zRelativeVelocity = state[5];
    
    threads.delay(interval/2);
  }
}

void Sensors::start(uint16_t interval)
{
  if (!sensorsInitialized) {
    Serial.println("ERROR: Cannot start sensors - not initialized!");
    return;
  }
  
  if (!areSensorsReady()) {
    Serial.println("WARNING: Starting with insufficient sensors!");
    // You could abort here if you want to enforce minimum sensor requirements
  }
  
  running = true;
  this->interval = interval;
  
  // Set reference points
  setRelativePosition(0.0, 0.0, 0.0);
  setRelativeVelocity(0.0, 0.0, 0.0);

  // Start each data stream
  Serial.println("Enabling sensor reports...");
  
  if (status.imu1) {
    enableReportsForIMU(&imu1, interval);
    Serial.println("- IMU1 reports enabled");
    threads.delay(50);
  }
  
  if (status.imu2) {
    enableReportsForIMU(&imu2, interval);
    Serial.println("- IMU2 reports enabled");
    threads.delay(50);
  }
  
  if (status.imu3) {
    enableReportsForIMU(&imu3, interval);
    Serial.println("- IMU3 reports enabled");
    threads.delay(50);
  }

  // Wait for sensors to stabilize further
  Serial.println("Allowing sensors to stabilize...");
  delay(200);

  // Start sensor threads
  Serial.println("Starting sensor threads...");

    // Give all I2C-using threads the same priority to prevent inversion
  imuSensorThreadId = threads.addThread(imuSensorThreadWrapper, this);
  altimeterSensorThreadId = threads.addThread(altimeterSensorThreadWrapper, this);
  temperatureSensorThreadId = threads.addThread(temperatureSensorThreadWrapper, this);
  computeRelativeLinearThreadId = threads.addThread(computeRelativeLinearThreadWrapper, this);
  
  Serial.println("Sensor system fully started");
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

// Add these updated implementations for all getFused functions

void Sensors::getFusedLinearAcceleration(float &xLinearAcceleration, float &yLinearAcceleration, float &zLinearAcceleration)
{
  static bool allSensorsWorking = true;
  
  // Create vectors with only connected IMUs
  std::vector<float> axValues;
  std::vector<float> ayValues;
  std::vector<float> azValues;
  std::vector<byte> accuracyValues;
  
  // Only add data for connected IMUs
  if (status.imu1 && imu1Data.linearAccuracy < 254) {
    axValues.push_back(static_cast<float>(imu1Data.xLinearAcceleration));
    ayValues.push_back(static_cast<float>(imu1Data.yLinearAcceleration));
    azValues.push_back(static_cast<float>(imu1Data.zLinearAcceleration));
    accuracyValues.push_back(static_cast<byte>(imu1Data.linearAccuracy));
  }
  
  if (status.imu2 && imu2Data.linearAccuracy < 254) {
    axValues.push_back(static_cast<float>(imu2Data.xLinearAcceleration));
    ayValues.push_back(static_cast<float>(imu2Data.yLinearAcceleration));
    azValues.push_back(static_cast<float>(imu2Data.zLinearAcceleration));
    accuracyValues.push_back(static_cast<byte>(imu2Data.linearAccuracy));
  }
  
  if (status.imu3 && imu3Data.linearAccuracy < 254) {
    axValues.push_back(static_cast<float>(imu3Data.xLinearAcceleration));
    ayValues.push_back(static_cast<float>(imu3Data.yLinearAcceleration));
    azValues.push_back(static_cast<float>(imu3Data.zLinearAcceleration));
    accuracyValues.push_back(static_cast<byte>(imu3Data.linearAccuracy));
  }

  // Check if any sensors are valid
  bool anySensorValid = !accuracyValues.empty();
  
  if (!anySensorValid && allSensorsWorking) {
    Serial.println("WARNING: All linear acceleration sensors have invalid data!");
    allSensorsWorking = false;
  } else if (anySensorValid && !allSensorsWorking) {
    Serial.println("INFO: At least one linear acceleration sensor is now working");
    allSensorsWorking = true;
  }

  // Default values in case no sensors are valid
  xLinearAcceleration = 0.0f;
  yLinearAcceleration = 0.0f;
  zLinearAcceleration = 0.0f;
  
  // Only call sensorFusion if we have valid sensors
  if (anySensorValid) {
    // Get raw fusion results
    float rawX = sensorFusion(axValues, accuracyValues);
    float rawY = sensorFusion(ayValues, accuracyValues);
    float rawZ = sensorFusion(azValues, accuracyValues);
    
    // Apply temporal filtering with appropriate max change rates
    xLinearAcceleration = applyTemporalFilter("accel_x", rawX, 20.0f); // Allow up to 20 m/s² change per sec
    yLinearAcceleration = applyTemporalFilter("accel_y", rawY, 20.0f);
    zLinearAcceleration = applyTemporalFilter("accel_z", rawZ, 20.0f);
  }
}

void Sensors::getFusedAngularVelocity(float &xAngularVelocity, float &yAngularVelocity, float &zAngularVelocity)
{
  static bool allSensorsWorking = true;
  
  // Create vectors with only connected IMUs
  std::vector<float> gxValues;
  std::vector<float> gyValues;
  std::vector<float> gzValues;
  std::vector<byte> accuracyValues;
  
  // Only add data for connected IMUs
  if (status.imu1 && imu1Data.gyroAccuracy < 254) {
    gxValues.push_back(static_cast<float>(imu1Data.xAngularVelocity));
    gyValues.push_back(static_cast<float>(imu1Data.yAngularVelocity));
    gzValues.push_back(static_cast<float>(imu1Data.zAngularVelocity));
    accuracyValues.push_back(static_cast<byte>(imu1Data.gyroAccuracy));
  }
  
  if (status.imu2 && imu2Data.gyroAccuracy < 254) {
    gxValues.push_back(static_cast<float>(imu2Data.xAngularVelocity));
    gyValues.push_back(static_cast<float>(imu2Data.yAngularVelocity));
    gzValues.push_back(static_cast<float>(imu2Data.zAngularVelocity));
    accuracyValues.push_back(static_cast<byte>(imu2Data.gyroAccuracy));
  }
  
  if (status.imu3 && imu3Data.gyroAccuracy < 254) {
    gxValues.push_back(static_cast<float>(imu3Data.xAngularVelocity));
    gyValues.push_back(static_cast<float>(imu3Data.yAngularVelocity));
    gzValues.push_back(static_cast<float>(imu3Data.zAngularVelocity));
    accuracyValues.push_back(static_cast<byte>(imu3Data.gyroAccuracy));
  }

  // Check if any sensors are valid
  bool anySensorValid = !accuracyValues.empty();
  
  if (!anySensorValid && allSensorsWorking) {
    Serial.println("WARNING: All angular velocity sensors have invalid data!");
    allSensorsWorking = false;
  } else if (anySensorValid && !allSensorsWorking) {
    Serial.println("INFO: At least one angular velocity sensor is now working");
    allSensorsWorking = true;
  }

  // Default values in case no sensors are valid
  xAngularVelocity = 0.0f;
  yAngularVelocity = 0.0f;
  zAngularVelocity = 0.0f;
  
  // Only call sensorFusion if we have valid sensors
  if (anySensorValid) {
    // Get raw fusion results
    float rawX = sensorFusion(gxValues, accuracyValues);
    float rawY = sensorFusion(gyValues, accuracyValues);
    float rawZ = sensorFusion(gzValues, accuracyValues);
    
    // Apply temporal filtering with appropriate max change rates (degrees/sec²)
    xAngularVelocity = applyTemporalFilter("gyro_x", rawX, 180.0f);
    yAngularVelocity = applyTemporalFilter("gyro_y", rawY, 180.0f);
    zAngularVelocity = applyTemporalFilter("gyro_z", rawZ, 180.0f);
  }
}

void Sensors::getFusedOrientation(float &yawOrientation, float &pitchOrientation, float &rollOrientation)
{
  static bool allSensorsWorking = true;
  
  // Create vectors with only connected IMUs
  std::vector<float> yawValues;
  std::vector<float> pitchValues;
  std::vector<float> rollValues;
  std::vector<byte> accuracyValues;
  std::vector<float> orientationAccuracyValues;
  
  // Only add data for connected IMUs
  if (status.imu1 && imu1Data.rotationAccuracy < 254) {
    yawValues.push_back(static_cast<float>(imu1Data.yawOrientation));
    pitchValues.push_back(static_cast<float>(imu1Data.pitchOrientation));
    rollValues.push_back(static_cast<float>(imu1Data.rollOrientation));
    accuracyValues.push_back(static_cast<byte>(imu1Data.rotationAccuracy));
    orientationAccuracyValues.push_back(static_cast<float>(imu1Data.orientationAccuracy));
  }
  
  if (status.imu2 && imu2Data.rotationAccuracy < 254) {
    yawValues.push_back(static_cast<float>(imu2Data.yawOrientation));
    pitchValues.push_back(static_cast<float>(imu2Data.pitchOrientation));
    rollValues.push_back(static_cast<float>(imu2Data.rollOrientation));
    accuracyValues.push_back(static_cast<byte>(imu2Data.rotationAccuracy));
    orientationAccuracyValues.push_back(static_cast<float>(imu2Data.orientationAccuracy));
  }
  
  if (status.imu3 && imu3Data.rotationAccuracy < 254) {
    yawValues.push_back(static_cast<float>(imu3Data.yawOrientation));
    pitchValues.push_back(static_cast<float>(imu3Data.pitchOrientation));
    rollValues.push_back(static_cast<float>(imu3Data.rollOrientation));
    accuracyValues.push_back(static_cast<byte>(imu3Data.rotationAccuracy));
    orientationAccuracyValues.push_back(static_cast<float>(imu3Data.orientationAccuracy));
  }

  // Check if any sensors are valid
  bool anySensorValid = !accuracyValues.empty();
  
  if (!anySensorValid && allSensorsWorking) {
    Serial.println("WARNING: All orientation sensors have invalid data!");
    allSensorsWorking = false;
  } else if (anySensorValid && !allSensorsWorking) {
    Serial.println("INFO: At least one orientation sensor is now working");
    allSensorsWorking = true;
  }

  // Default values in case no sensors are valid
  yawOrientation = 0.0f;
  pitchOrientation = 0.0f;
  rollOrientation = 0.0f;
  
  // Only call sensorFusion if we have valid sensors
  if (anySensorValid) {
    // Get raw fusion results
    float rawYaw = fuseYaw(yawValues, accuracyValues, orientationAccuracyValues);
    float rawPitch = sensorFusion(pitchValues, accuracyValues, orientationAccuracyValues);
    float rawRoll = sensorFusion(rollValues, accuracyValues, orientationAccuracyValues);
    
    // Apply angular filtering to handle wrapping correctly
    yawOrientation = applyAngularFilter("yaw", rawYaw, 120.0f);
    
    // Pitch and roll don't wrap in the same way, use standard filter
    pitchOrientation = applyTemporalFilter("pitch", rawPitch, 60.0f);
    rollOrientation = applyTemporalFilter("roll", rawRoll, 60.0f);
  }
}

void Sensors::getFusedQuaternion(float &quatI, float &quatJ, float &quatK, float &quatReal)
{
  static bool allSensorsWorking = true;
  
  // Create vectors with only connected IMUs
  std::vector<float> iValues;
  std::vector<float> jValues;
  std::vector<float> kValues;
  std::vector<float> realValues;
  std::vector<byte> accuracyValues;
  
  // Only add data for connected IMUs
  if (status.imu1 && imu1Data.rotationAccuracy < 254) {
    iValues.push_back(static_cast<float>(imu1Data.quatI));
    jValues.push_back(static_cast<float>(imu1Data.quatJ));
    kValues.push_back(static_cast<float>(imu1Data.quatK));
    realValues.push_back(static_cast<float>(imu1Data.quatReal));
    accuracyValues.push_back(static_cast<byte>(imu1Data.rotationAccuracy));
  }
  
  if (status.imu2 && imu2Data.rotationAccuracy < 254) {
    iValues.push_back(static_cast<float>(imu2Data.quatI));
    jValues.push_back(static_cast<float>(imu2Data.quatJ));
    kValues.push_back(static_cast<float>(imu2Data.quatK));
    realValues.push_back(static_cast<float>(imu2Data.quatReal));
    accuracyValues.push_back(static_cast<byte>(imu2Data.rotationAccuracy));
  }
  
  if (status.imu3 && imu3Data.rotationAccuracy < 254) {
    iValues.push_back(static_cast<float>(imu3Data.quatI));
    jValues.push_back(static_cast<float>(imu3Data.quatJ));
    kValues.push_back(static_cast<float>(imu3Data.quatK));
    realValues.push_back(static_cast<float>(imu3Data.quatReal));
    accuracyValues.push_back(static_cast<byte>(imu3Data.rotationAccuracy));
  }

  // Check if any sensors are valid
  bool anySensorValid = !accuracyValues.empty();
  
  if (!anySensorValid && allSensorsWorking) {
    Serial.println("WARNING: All quaternion sensors have invalid data!");
    allSensorsWorking = false;
  } else if (anySensorValid && !allSensorsWorking) {
    Serial.println("INFO: At least one quaternion sensor is now working");
    allSensorsWorking = true;
  }

  // Default values in case no sensors are valid
  quatI = 0.0f;
  quatJ = 0.0f;
  quatK = 0.0f;
  quatReal = 1.0f;  // Default to identity quaternion
  
  // Only call sensorFusion if we have valid sensors
  if (anySensorValid) {
    // Get raw fusion results
    float rawI = sensorFusion(iValues, accuracyValues);
    float rawJ = sensorFusion(jValues, accuracyValues);
    float rawK = sensorFusion(kValues, accuracyValues);
    float rawReal = sensorFusion(realValues, accuracyValues);
    
    // Apply temporal filtering with appropriate max change rates
    // Quaternions change more slowly than Euler angles
    quatI = applyTemporalFilter("quat_i", rawI, 0.5f);
    quatJ = applyTemporalFilter("quat_j", rawJ, 0.5f);
    quatK = applyTemporalFilter("quat_k", rawK, 0.5f);
    quatReal = applyTemporalFilter("quat_real", rawReal, 0.5f);
    
    // Normalize the quaternion
    float norm = sqrt(quatI*quatI + quatJ*quatJ + quatK*quatK + quatReal*quatReal);
    if (norm > 0.0001) {
      quatI /= norm;
      quatJ /= norm;
      quatK /= norm;
      quatReal /= norm;
    }
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
  if (values.empty()) return 0.0f;
  if (values.size() == 1) return values[0];
  
  // Special case for quaternion components where values may be near zero
  bool isNearZeroData = true;
  for (float val : values) {
    if (fabs(val) > 0.01f) {
      isNearZeroData = false;
      break;
    }
  }
  
  // For very small values (like quat components), use median to avoid division issues
  if (isNearZeroData) {
    std::vector<float> sortedValues = values;
    std::sort(sortedValues.begin(), sortedValues.end());
    return sortedValues[sortedValues.size() / 2];
  }
  
  // Check for outliers when we have 3 sensors
  if (values.size() == 3) {
    // Find maximum difference between any pair
    float maxDiff = 0.0f;
    int outlierIdx = -1;
    
    for (int i = 0; i < 3; i++) {
      for (int j = i+1; j < 3; j++) {
        float diff = fabs(values[i] - values[j]);
        if (diff > maxDiff) {
          maxDiff = diff;
          
          // The point furthest from the cluster is likely the outlier
          float avg = (values[0] + values[1] + values[2]) / 3.0f;
          outlierIdx = (fabs(values[i] - avg) > fabs(values[j] - avg)) ? i : j;
        }
      }
    }
    
    // If we have a significant outlier, remove it
    if (maxDiff > 10.0f) { // Threshold for what constitutes an outlier
      values.erase(values.begin() + outlierIdx);
      if (!orientationAccuracy.empty()) orientationAccuracy.erase(orientationAccuracy.begin() + outlierIdx);
      accuracy.erase(accuracy.begin() + outlierIdx);
    }
  }
  
  // Weighted average based on accuracy
  float sum = 0.0f;
  float weightSum = 0.0f;
  
  for (size_t i = 0; i < values.size(); i++) {
    // Calculate weight based on accuracy
    float weight;
    if (!orientationAccuracy.empty()) {
      // For orientation accuracy (lower is better)
      weight = 1.0f / (orientationAccuracy[i] + 0.1f); // Add small value to avoid division by zero
    } else {
      // For regular accuracy (higher is better)
      weight = static_cast<float>(accuracy[i]) + 0.1f; // Add small value for acc=0 case
    }
    
    sum += values[i] * weight;
    weightSum += weight;
  }
  
  // Avoid division by zero
  if (weightSum < 0.001f) return values[0];
  
  return sum / weightSum;
}

void Sensors::calibrateAllIMUs(){
  // Calibrate all IMUs
  if (status.imu1)
  {
    imu1.calibrateAll();
  }
  if (status.imu2)
  {
    imu2.calibrateAll();
  }
  if (status.imu3)
  {
    imu3.calibrateAll();
  }
}

void Sensors::updateAltitudeWithComplementaryFilter()
{
  // Get altitude from barometer
  float baroAltitude = getAltitude();
  
  // Vertical acceleration component (world frame)
  float azWorld = 0.0f;
  float axWorld = 0.0f, ayWorld = 0.0f;
  getFusedWorldLinearAcceleration(axWorld, ayWorld, azWorld);
  
  // Static values to maintain state between calls
  static float prevAltitude = baroAltitude;
  static float integratedAltitude = baroAltitude;
  static unsigned long prevTime = millis();
  
  // Calculate time delta
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0f;
  prevTime = currentTime;
  
  // Limit dt to prevent large jumps
  if (dt > 0.1f) dt = 0.1f;
  
  // First integrate vertical acceleration to update altitude
  // Use gravitational constant to apply gravity compensation correction
  // (although BNO080's linear acceleration should already exclude gravity)
  float azWithoutGravity = azWorld;
  
  // Double integration for altitude from acceleration
  integratedAltitude += zRelativeVelocity * dt + 0.5f * azWithoutGravity * dt * dt;
  
  // Now combine with barometric altitude
  const float ALPHA_BARO = 0.05f; // Weight for barometer (low-pass filter)
  
  // Check if GPS data is valid to include it
  if (gpsData.valid && gpsData.fixQuality > 0) {
    // Three-way complementary filter with GPS altitude included
    // Weight depends on GPS fix quality
    float gpsWeight = 0.0f;
    switch (gpsData.fixQuality) {
      case 1: gpsWeight = 0.01f; break;  // Basic GPS
      case 2: gpsWeight = 0.02f; break;  // DGPS
      case 4: gpsWeight = 0.05f; break;  // RTK
      case 5: gpsWeight = 0.05f; break;  // Float RTK
      default: gpsWeight = 0.005f;       // Low trust for unknown quality
    }
    
    // Adjust baro weight to maintain proper sum
    float accelWeight = 1.0f - ALPHA_BARO - gpsWeight;
    
    // Apply three-source complementary filter
    zRelativePosition = 
        accelWeight * integratedAltitude + 
        ALPHA_BARO * baroAltitude + 
        gpsWeight * gpsData.altitude;
  } 
  else {
    // If no GPS, fall back to two-source filter
    float accelWeight = 1.0f - ALPHA_BARO;
    
    // Apply two-source complementary filter
    zRelativePosition = accelWeight * integratedAltitude + ALPHA_BARO * baroAltitude;
  }
  
  // Update integrated altitude based on the filtered value to reduce drift
  // This pulls the integration back toward the absolute references
  integratedAltitude = zRelativePosition;
  
  // Also update the vertical velocity estimate using barometer
  float baroVelocity = (baroAltitude - prevAltitude) / dt;
  prevAltitude = baroAltitude;
  
  // Filter the velocity estimate
  const float VEL_ALPHA = 0.1f; // Weight for barometer-derived velocity
  zRelativeVelocity = (1.0f - VEL_ALPHA) * zRelativeVelocity + VEL_ALPHA * baroVelocity;
}

void Sensors::rungeKutta4Integration(float &position, float &velocity, float acceleration, float dt)
{
  // RK4 for velocity
  float k1v = acceleration;
  float k2v = acceleration;
  float k3v = acceleration;
  float k4v = acceleration;
  
  float velocity_next = velocity + (dt/6.0) * (k1v + 2*k2v + 2*k3v + k4v);
  
  // RK4 for position
  float k1p = velocity;
  float k2p = velocity + 0.5*dt*k1v;
  float k3p = velocity + 0.5*dt*k2v;
  float k4p = velocity + dt*k3v;
  
  float position_next = position + (dt/6.0) * (k1p + 2*k2p + 2*k3p + k4p);
  
  // Update values
  position = position_next;
  velocity = velocity_next;
}

void Sensors::detectZeroVelocityAndUpdate(float state[9], float P[9][9], 
                                        float ax, float ay, float az)
{
  // Calculate acceleration magnitude (without gravity)
  float accel_magnitude = sqrt(ax*ax + ay*ay + az*az);
  
  // Get angular velocity for stationary detection
  float wx, wy, wz;
  getFusedAngularVelocity(wx, wy, wz);
  float angular_velocity_magnitude = sqrt(wx*wx + wy*wy + wz*wz);
  
  // More sensitive thresholds for zero-velocity detection
  const float ACCEL_THRESHOLD = 0.03f;
  const float GYRO_THRESHOLD = 0.03f;
  const float VELOCITY_THRESHOLD = 0.1f;
  
  // Calculate velocity magnitude
  float velocity_magnitude = sqrt(state[3]*state[3] + state[4]*state[4] + state[5]*state[5]);
  
  // Incorporate hysteresis for stability
  static int stationary_count = 0;
  static int moving_count = 0;
  
  const int MIN_STATIONARY_SAMPLES = 8;
  const int MIN_MOVING_SAMPLES = 3;
  
  bool stationary_condition = (accel_magnitude < ACCEL_THRESHOLD) && 
                              (angular_velocity_magnitude < GYRO_THRESHOLD) &&
                              (velocity_magnitude < VELOCITY_THRESHOLD);
  
  if (stationary_condition) {
    // Rest of function remains the same...
    
    if (stationary_count >= MIN_STATIONARY_SAMPLES) {
      was_stationary = true;
      
      // Existing code...
    }
  } else {
    // Existing code...
    
    if (moving_count >= MIN_MOVING_SAMPLES) {
      stationary_count = 0;
      if (was_stationary) {
        was_stationary = false;
        Serial.println("Motion detected - exiting zero velocity state");
      }
    }
  }
}

// Add this method to Sensors class
void Sensors::filterAcceleration(float &ax, float &ay, float &az)
{
  static float prev_ax = 0, prev_ay = 0, prev_az = 0;
  static float filt_ax = 0, filt_ay = 0, filt_az = 0;
  static bool first_call = true;
  
  if (first_call) {
    prev_ax = ax; prev_ay = ay; prev_az = az;
    filt_ax = ax; filt_ay = ay; filt_az = az;
    first_call = false;
    return;
  }
  
  // Two-stage filter: first stage (fast response)
  const float ALPHA1 = 0.7f;
  filt_ax = ALPHA1 * filt_ax + (1.0f - ALPHA1) * ax;
  filt_ay = ALPHA1 * filt_ay + (1.0f - ALPHA1) * ay;
  filt_az = ALPHA1 * filt_az + (1.0f - ALPHA1) * az;
  
  // Second stage (further smoothing for steady-state)
  const float ALPHA2 = 0.9f;
  ax = ALPHA2 * prev_ax + (1.0f - ALPHA2) * filt_ax;
  ay = ALPHA2 * prev_ay + (1.0f - ALPHA2) * filt_ay;
  az = ALPHA2 * prev_az + (1.0f - ALPHA2) * filt_az;
  
  // Update previous values
  prev_ax = ax; prev_ay = ay; prev_az = az;
}

float Sensors::getLatitude() {
  return gpsData.latitude;
}

float Sensors::getLongitude() {
  return gpsData.longitude;
}

float Sensors::getGPSAltitude() {
  return gpsData.altitude;
}

float Sensors::getGPSSpeed() {
  return gpsData.speed;
}

float Sensors::getGPSCourse() {
  return gpsData.course;
}

uint8_t Sensors::getGPSSatellites() {
  return gpsData.satellites;
}

uint8_t Sensors::getGPSFixQuality() {
  return gpsData.fixQuality;
}

bool Sensors::isGPSValid() {
  return gpsData.valid;
}

// Methods for future GPS integration
void Sensors::updatePositionWithGPS() {
  // This will be implemented when GPS hardware is integrated
  // For now, just a placeholder that does nothing
  
  // Future implementation would:
  // 1. Check if GPS data is valid
  // 2. Apply Kalman filtering to combine IMU and GPS data
  // 3. Correct position drift using absolute GPS position
}

void Sensors::updateVelocityWithGPS() {
  // This will be implemented when GPS hardware is integrated
  // For now, just a placeholder that does nothing
  
  // Future implementation would:
  // 1. Check if GPS data is valid
  // 2. Apply Kalman filtering to combine IMU and GPS data
  // 3. Correct velocity drift using GPS velocity
}

// For simulating GPS data during testing
void Sensors::simulateGPSData() {
  // Only for testing - will be removed when actual GPS is integrated
  // This method could be called periodically to simulate GPS updates
  
  // For now, just simulate a fixed position with some simple altitude change
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 2000) { // Simulate 1Hz update rate
    lastUpdate = millis();
    
    // Example fixed position (McGill University coordinates)
    gpsData.latitude = 45.5048;
    gpsData.longitude = -73.5772;
    
    // Simulate altitude based on barometer with some offset
    gpsData.altitude = getAltitude() + 2.0f; // 2m offset from barometric altitude
    
    // Set other fields with reasonable values
    // gpsData.speed = sqrt(xRelativeVelocity*xRelativeVelocity + 
                        //  yRelativeVelocity*yRelativeVelocity +
                        //  zRelativeVelocity*zRelativeVelocity);
    gpsData.speed = 0.0f; // Placeholder
    gpsData.course = 0.0f; // North
    // gpsData.vx = xRelativeVelocity;
    // gpsData.vy = yRelativeVelocity;
    // gpsData.vz = zRelativeVelocity;
    gpsData.vx = 0.0f; // Placeholder
    gpsData.vy = 0.0f; // Placeholder
    gpsData.vz = 0.0f; // Placeholder
    gpsData.satellites = 8;
    gpsData.fixQuality = 2; // 0=no fix, 1=GPS, 2=DGPS, etc.
    gpsData.timestamp = millis();
    gpsData.valid = true;
  }
}

// Calculate velocity components from speed and course
void Sensors::calculateGPSVelocityComponents() {
  // This will be implemented when GPS hardware is integrated
  // For now, just a placeholder that does nothing
  
  // Future implementation would calculate vx, vy, vz from speed and course:
  // gpsData.vx = gpsData.speed * cos(gpsData.course * PI/180.0);
  // gpsData.vy = gpsData.speed * sin(gpsData.course * PI/180.0);
  // gpsData.vz would come from change in altitude
}

float Sensors::applyTemporalFilter(const std::string& sensorKey, float newValue, float maxChangeRate) {
#if ENABLE_TEMPORAL_FILTER
  auto it = lastFusedValues.find(sensorKey);
  
  // First measurement handling - same for all algorithms
  if (it == lastFusedValues.end()) {
    // First measurement for this sensor
    lastFusedValues[sensorKey] = newValue;
    return newValue;
  }
  
  float lastValue = it->second;
  float result = newValue; // Default to new value

  // Common rate limiting for all algorithms
  float maxChange = maxChangeRate * (interval / 1000.0f); // Scale by time step
  
  // Apply rate limiting - common to all algorithms
  if (fabs(newValue - lastValue) > maxChange) {
    if (newValue > lastValue)
      newValue = lastValue + maxChange;
    else
      newValue = lastValue - maxChange;
  }
  
#if SELECTED_FILTER_ALGORITHM == FILTER_ALGORITHM_SIMPLE
  // ALGORITHM 1: Simple exponential smoothing
  result = FILTER_ALPHA_SIMPLE * newValue + (1.0f - FILTER_ALPHA_SIMPLE) * lastValue;

#elif SELECTED_FILTER_ALGORITHM == FILTER_ALGORITHM_ADAPTIVE
  // ALGORITHM 2: Adaptive smoothing based on change magnitude
  // Calculate normalized change ratio
  float changeRatio = fabs(newValue - lastValue) / (maxChange > 0.0001f ? maxChange : 0.0001f);
  changeRatio = min(1.0f, changeRatio);
  
  // Calculate adaptive alpha - smaller for small changes (more smoothing)
  // and larger for big changes (less smoothing)
  float adaptiveAlpha = 0.05f + 0.25f * changeRatio;
  
  // Apply exponential smoothing with adaptive alpha
  result = adaptiveAlpha * newValue + (1.0f - adaptiveAlpha) * lastValue;

#elif SELECTED_FILTER_ALGORITHM == FILTER_ALGORITHM_DOUBLE_EXP
  // ALGORITHM 3: Double exponential smoothing (two-stage)
  static std::map<std::string, float> intermediateValues;
  
  auto intermediate_it = intermediateValues.find(sensorKey);
  float intermValue = (intermediate_it != intermediateValues.end()) ? 
                      intermediate_it->second : newValue;
  
  // Two-stage filtering for stronger smoothing
  // First filter stage
  intermValue = FILTER_ALPHA1_DOUBLE * newValue + (1.0f - FILTER_ALPHA1_DOUBLE) * intermValue;
  
  // Second filter stage
  result = FILTER_ALPHA2_DOUBLE * intermValue + (1.0f - FILTER_ALPHA2_DOUBLE) * lastValue;
  
  // Update intermediate value
  intermediateValues[sensorKey] = intermValue;
#endif
  
  // Update stored value for next time
  lastFusedValues[sensorKey] = result;
  return result;
  
#else
  // When filtering is disabled, just pass through the raw value
  return newValue;
#endif
}

float Sensors::fuseYaw(std::vector<float> yawValues, std::vector<byte> accuracy, std::vector<float> orientationAccuracy) {
  // Convert all yaw values to a consistent range
  for (size_t i = 0; i < yawValues.size(); i++) {
    // Normalize to 0-360 range
    while (yawValues[i] < 0) yawValues[i] += 360.0f;
    while (yawValues[i] >= 360.0f) yawValues[i] -= 360.0f;
  }
  
  // Special case for sensors reporting values across the 0/360 boundary
  // For example, sensor 1 reporting 358° and sensor 2 reporting 2°
  if (yawValues.size() >= 2) {
    bool needsUnwrapping = false;
    
    for (size_t i = 0; i < yawValues.size() - 1; i++) {
      for (size_t j = i + 1; j < yawValues.size(); j++) {
        float diff = fabs(yawValues[i] - yawValues[j]);
        if (diff > 180.0f) {
          needsUnwrapping = true;
          break;
        }
      }
    }
    
    if (needsUnwrapping) {
      // Unwrap by adding 360° to small values
      for (size_t i = 0; i < yawValues.size(); i++) {
        if (yawValues[i] < 90.0f)  // If close to 0
          yawValues[i] += 360.0f;
      }
      
      // Now fuse with regular method
      float result = sensorFusion(yawValues, accuracy, orientationAccuracy);
      
      // Re-wrap to 0-360
      while (result >= 360.0f) result -= 360.0f;
      return result;
    }
  }
  
  // Standard fusion for non-boundary cases
  return sensorFusion(yawValues, accuracy, orientationAccuracy);
}

// Add this at the end

// Specialized filter for angular values (handles 0-360° wrapping)
float Sensors::applyAngularFilter(const std::string& sensorKey, float newValue, float maxChangeRate) {
#if ENABLE_TEMPORAL_FILTER
  auto it = lastFusedValues.find(sensorKey);
  
  if (it == lastFusedValues.end()) {
    lastFusedValues[sensorKey] = newValue;
    return newValue;
  }
  
  float lastValue = it->second;
  
  // Handle angle wrapping (e.g., 359° -> 1° is a 2° change, not 358°)
  float diff = newValue - lastValue;
  if (diff > 180.0f) diff -= 360.0f;
  if (diff < -180.0f) diff += 360.0f;
  
  float maxChange = maxChangeRate * (interval / 1000.0f);
  
  // Apply rate limiting
  if (fabs(diff) > maxChange) {
    if (diff > 0)
      newValue = lastValue + maxChange;
    else
      newValue = lastValue - maxChange;
  }
  
  float result = 0.0f;
  
#if SELECTED_FILTER_ALGORITHM == FILTER_ALGORITHM_SIMPLE
  // Simple smoothing for angles
  result = lastValue + FILTER_ALPHA_SIMPLE * diff;
  
#elif SELECTED_FILTER_ALGORITHM == FILTER_ALGORITHM_ADAPTIVE
  // Adaptive smoothing for angles
  float changeRatio = fabs(diff) / (maxChange > 0.0001f ? maxChange : 0.0001f);
  changeRatio = min(1.0f, changeRatio);
  float adaptiveAlpha = 0.05f + 0.25f * changeRatio;
  result = lastValue + adaptiveAlpha * diff;
  
#elif SELECTED_FILTER_ALGORITHM == FILTER_ALGORITHM_DOUBLE_EXP
  // Double exponential for angles (using intermediate value)
  static std::map<std::string, float> intermediateAngles;
  
  auto interm_it = intermediateAngles.find(sensorKey);
  float intermValue = (interm_it != intermediateAngles.end()) ? 
                     interm_it->second : lastValue;
  
  // First stage: adjust intermediate value
  intermValue = intermValue + FILTER_ALPHA1_DOUBLE * diff;
  
  // Second stage: calculate result using intermediate value
  float diffInterm = intermValue - lastValue;
  if (diffInterm > 180.0f) diffInterm -= 360.0f;
  if (diffInterm < -180.0f) diffInterm += 360.0f;
  
  result = lastValue + FILTER_ALPHA2_DOUBLE * diffInterm;
  
  // Store intermediate value
  intermediateAngles[sensorKey] = intermValue;
#endif
  
  // Normalize to 0-360 range
  while (result >= 360.0f) result -= 360.0f;
  while (result < 0.0f) result += 360.0f;
  
  // Update stored value
  lastFusedValues[sensorKey] = result;
  return result;
  
#else
  return newValue;
#endif
}