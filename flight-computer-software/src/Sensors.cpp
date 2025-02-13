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
  //imu.enableMagnometer(interval);
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

void Sensors::getLinearAcceleration(BNO080 imu, float &ax, float &ay, float &az)
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
  }
}

void Sensors::getOrientation(BNO080 imu, float &yaw, float &pitch, float &roll, float &accuracyDegrees)
{
  if (imu.hasReset())
  {
    //Serial.println("IMU has reset. Reason: " + imu.resetReason());
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
}

void Sensors::getRelativeVelocity(BNO080 imu, float &vx, float &vy, float &vz)
{
  float ax = 0.0, ay = 0.0, az = 0.0;
  getLinearAcceleration(imu, ax, ay, az);

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
