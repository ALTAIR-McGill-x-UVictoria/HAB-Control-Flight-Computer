#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

BNO080 imu1;

// Variables to store latest sensor readings
float accelX, accelY, accelZ, accelAccuracy;
float gyroX, gyroY, gyroZ, gyroAccuracy;
float magX, magY, magZ, magAccuracy;
float quatI, quatJ, quatK, quatReal, quatAccuracy;
float roll, pitch, yaw;
float linAccelX, linAccelY, linAccelZ, linAccelAccuracy;
float gravityX, gravityY, gravityZ;

// Counters to track which sensors are reporting
uint32_t accelCount = 0;
uint32_t gyroCount = 0;
uint32_t magCount = 0;
uint32_t quatCount = 0;
uint32_t linAccelCount = 0;
uint32_t gravityCount = 0;

// Print interval
const unsigned long PRINT_INTERVAL = 500; // Extended to 2 seconds
unsigned long lastPrintTime = 0;

// Add these variables to track real timing between sensor reports
unsigned long lastAccelTime = 0;
unsigned long lastGyroTime = 0;
unsigned long lastMagTime = 0;
unsigned long lastQuatTime = 0;
unsigned long lastLinAccelTime = 0;
unsigned long lastGravityTime = 0;

// Arrays to store timing statistics
#define TIMING_SAMPLES 20
unsigned long accelIntervals[TIMING_SAMPLES] = {0};
unsigned long gyroIntervals[TIMING_SAMPLES] = {0};
unsigned long magIntervals[TIMING_SAMPLES] = {0};
unsigned long quatIntervals[TIMING_SAMPLES] = {0};
unsigned long linAccelIntervals[TIMING_SAMPLES] = {0};
unsigned long gravityIntervals[TIMING_SAMPLES] = {0};

// Array indices
uint8_t accelIndex = 0;
uint8_t gyroIndex = 0;
uint8_t magIndex = 0;
uint8_t quatIndex = 0;
uint8_t linAccelIndex = 0;
uint8_t gravityIndex = 0;

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  Serial.println("\nBNO080 IMU Comprehensive Debugging");
  
  // Initialize I2C
  Wire1.begin();
  Wire1.setClock(400000); // Try with 400kHz for better performance
  
  // Give the IMU time to boot
  delay(500); // Increased boot time
  
  // Initialize IMU with specified address and Wire1
  Serial.println("Initializing IMU...");
  bool status = imu1.begin(0x4A, Wire1, -1);
  
  if (!status) {
    Serial.println("First address failed. Trying alternate address...");
    status = imu1.begin(0x4B, Wire1, -1);
    
    if (!status) {
      Serial.println("ERROR: IMU initialization failed. Check connections.");
      Serial.println("Verify the following:");
      Serial.println("1. Power connections to the IMU");
      Serial.println("2. I2C connections (SDA, SCL)");
      Serial.println("3. I2C pullup resistors are present");
      while (1) {
        delay(100);
      }
    }
  }
  
  Serial.println("IMU initialized successfully!");
  
  // Try a soft reset first
  Serial.println("Performing soft reset...");
  imu1.softReset();
  delay(500);
  
  Serial.println("Configuring IMU...");
  
  // // Try different update rates for the problematic sensors
  Serial.print("Enabling accelerometer... ");
  imu1.enableAccelerometer(50); // Try faster update rate (25ms)
  Serial.println("DONE");
  
  delay(100);
  Serial.print("Enabling gyroscope... ");
  imu1.enableGyro(50); // Keep original rate for comparison
  Serial.println("DONE");
  
  delay(100);
  Serial.print("Enabling magnetometer... ");
  imu1.enableMagnetometer(50); // Try faster update rate (25ms)
  Serial.println("DONE");
  
  delay(100);
  Serial.print("Enabling rotation vector... ");
  imu1.enableRotationVector(50); // Keep original rate for comparison
  Serial.println("DONE");
  
  delay(100);
  Serial.print("Enabling linear accelerometer... ");
  imu1.enableLinearAccelerometer(50); // Try faster update rate (25ms)
  Serial.println("DONE");
  
  delay(100);
  Serial.print("Enabling gravity... ");
  imu1.enableGravity(50); // Try faster update rate (25ms)
  Serial.println("DONE");
  
  Serial.println("All sensors configured, beginning data stream...");
  delay(1000); // Longer delay to ensure sensors initialize fully
}

void loop() {
  static unsigned long lastSensorTime = 0;
  static uint8_t lastSensorType = 0;
  
  // Check if new IMU data is available and update our stored values
  if (imu1.dataAvailable()) {
    // Get current sensor type that has new data
    uint8_t sensorType = imu1.getReadings();
    unsigned long currentTime = millis();
    
    // Update the appropriate sensor values based on which sensor had new data
    switch (sensorType) {
      case SENSOR_REPORTID_ACCELEROMETER:
        accelX = imu1.getAccelX();
        accelY = imu1.getAccelY();
        accelZ = imu1.getAccelZ();
        accelAccuracy = imu1.getAccelAccuracy();
        
        // Track actual timing between accelerometer readings
        if (lastAccelTime > 0) {
          accelIntervals[accelIndex] = currentTime - lastAccelTime;
          accelIndex = (accelIndex + 1) % TIMING_SAMPLES;
        }
        lastAccelTime = currentTime;
        accelCount++;
        if (accelX == 0.0 && accelY == 0.0 && accelZ == 0.0) {
          Serial.println("WARNING: All accelerometer values are zero!");
        }
        break;
        
      case SENSOR_REPORTID_GYROSCOPE:
        gyroX = imu1.getGyroX();
        gyroY = imu1.getGyroY();
        gyroZ = imu1.getGyroZ();
        gyroAccuracy = imu1.getGyroAccuracy();
        
        // Track actual timing between gyroscope readings
        if (lastGyroTime > 0) {
          gyroIntervals[gyroIndex] = currentTime - lastGyroTime;
          gyroIndex = (gyroIndex + 1) % TIMING_SAMPLES;
        }
        lastGyroTime = currentTime;
        gyroCount++;
        break;
        
      case SENSOR_REPORTID_MAGNETIC_FIELD:
        magX = imu1.getMagX();
        magY = imu1.getMagY();
        magZ = imu1.getMagZ();
        magAccuracy = imu1.getMagAccuracy();
        
        // Track actual timing between magnetometer readings
        if (lastMagTime > 0) {
          magIntervals[magIndex] = currentTime - lastMagTime;
          magIndex = (magIndex + 1) % TIMING_SAMPLES;
        }
        lastMagTime = currentTime;
        magCount++;
        break;
        
      case SENSOR_REPORTID_ROTATION_VECTOR:
        quatI = imu1.getQuatI();
        quatJ = imu1.getQuatJ();
        quatK = imu1.getQuatK();
        quatReal = imu1.getQuatReal();
        quatAccuracy = imu1.getQuatRadianAccuracy();
        
        // Convert quaternion to euler angles (in degrees)
        roll = imu1.getRoll() * 180.0 / PI;
        pitch = imu1.getPitch() * 180.0 / PI;
        yaw = imu1.getYaw() * 180.0 / PI;
        
        // Track actual timing between rotation vector readings
        if (lastQuatTime > 0) {
          quatIntervals[quatIndex] = currentTime - lastQuatTime;
          quatIndex = (quatIndex + 1) % TIMING_SAMPLES;
        }
        lastQuatTime = currentTime;
        quatCount++;
        break;
        
      case SENSOR_REPORTID_LINEAR_ACCELERATION:
        linAccelX = imu1.getLinAccelX();
        linAccelY = imu1.getLinAccelY();
        linAccelZ = imu1.getLinAccelZ();
        linAccelAccuracy = imu1.getLinAccelAccuracy();
        
        // Track actual timing between linear acceleration readings
        if (lastLinAccelTime > 0) {
          linAccelIntervals[linAccelIndex] = currentTime - lastLinAccelTime;
          linAccelIndex = (linAccelIndex + 1) % TIMING_SAMPLES;
        }
        lastLinAccelTime = currentTime;
        linAccelCount++;
        break;
        
      case SENSOR_REPORTID_GRAVITY:
        gravityX = imu1.getGravityX();
        gravityY = imu1.getGravityY();
        gravityZ = imu1.getGravityZ();
        
        // Track actual timing between gravity readings
        if (lastGravityTime > 0) {
          gravityIntervals[gravityIndex] = currentTime - lastGravityTime;
          gravityIndex = (gravityIndex + 1) % TIMING_SAMPLES;
        }
        lastGravityTime = currentTime;
        gravityCount++;
        break;
        
      default:
        // Serial.print("Unknown sensor report ID: 0x");
        // Serial.println(sensorType, HEX);
        break;
    }
  }
  
  // Print all sensor data at the specified interval
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    Serial.println("\n=====================================================");
    Serial.println("BNO080 SENSOR DATA");
    Serial.println("=====================================================");
    
    // Report counts of each sensor type
    Serial.println("--- SENSOR REPORT COUNTS ---");
    Serial.print("Accelerometer: "); Serial.println(accelCount);
    Serial.print("Gyroscope: "); Serial.println(gyroCount);
    Serial.print("Magnetometer: "); Serial.println(magCount);
    Serial.print("Rotation Vector: "); Serial.println(quatCount);
    Serial.print("Linear Acceleration: "); Serial.println(linAccelCount);
    Serial.print("Gravity: "); Serial.println(gravityCount);
    Serial.println();
    
    // Accelerometer data
    Serial.println("--- ACCELEROMETER (m/s²) ---");
    Serial.print("X: "); Serial.print(accelX, 2);
    Serial.print(" Y: "); Serial.print(accelY, 2);
    Serial.print(" Z: "); Serial.print(accelZ, 2);
    Serial.print(" Accuracy: "); Serial.println(accelAccuracy, 2);
    
    // Gyroscope data
    Serial.println("--- GYROSCOPE (rad/s) ---");
    Serial.print("X: "); Serial.print(gyroX, 2);
    Serial.print(" Y: "); Serial.print(gyroY, 2);
    Serial.print(" Z: "); Serial.print(gyroZ, 2);
    Serial.print(" Accuracy: "); Serial.println(gyroAccuracy, 2);
    
    // Magnetometer data
    Serial.println("--- MAGNETOMETER (μT) ---");
    Serial.print("X: "); Serial.print(magX, 2);
    Serial.print(" Y: "); Serial.print(magY, 2);
    Serial.print(" Z: "); Serial.print(magZ, 2);
    Serial.print(" Accuracy: "); Serial.println(magAccuracy, 2);
    
    // Rotation vector / orientation data
    Serial.println("--- ROTATION VECTOR (Quaternion) ---");
    Serial.print("I: "); Serial.print(quatI, 3);
    Serial.print(" J: "); Serial.print(quatJ, 3);
    Serial.print(" K: "); Serial.print(quatK, 3);
    Serial.print(" Real: "); Serial.print(quatReal, 3);
    Serial.print(" Accuracy: "); Serial.println(quatAccuracy, 3);
    
    Serial.println("--- EULER ANGLES (deg) ---");
    Serial.print("Roll: "); Serial.print(roll, 1);
    Serial.print(" Pitch: "); Serial.print(pitch, 1);
    Serial.print(" Yaw: "); Serial.println(yaw, 1);
    
    // Linear acceleration data
    Serial.println("--- LINEAR ACCELERATION (m/s²) ---");
    Serial.print("X: "); Serial.print(linAccelX, 2);
    Serial.print(" Y: "); Serial.print(linAccelY, 2);
    Serial.print(" Z: "); Serial.print(linAccelZ, 2);
    Serial.print(" Accuracy: "); Serial.println(linAccelAccuracy, 2);
    
    // Gravity vector data
    Serial.println("--- GRAVITY VECTOR (m/s²) ---");
    Serial.print("X: "); Serial.print(gravityX, 2);
    Serial.print(" Y: "); Serial.print(gravityY, 2);
    Serial.print(" Z: "); Serial.println(gravityZ, 2);
    
    // Add timing statistics
    Serial.println("\n--- SENSOR TIMING STATISTICS ---");
    
    // Calculate average intervals
    unsigned long accelSum = 0, gyroSum = 0, magSum = 0;
    unsigned long quatSum = 0, linAccelSum = 0, gravitySum = 0;
    int accelValid = 0, gyroValid = 0, magValid = 0;
    int quatValid = 0, linAccelValid = 0, gravityValid = 0;
    
    for (int i = 0; i < TIMING_SAMPLES; i++) {
      if (accelIntervals[i] > 0) { accelSum += accelIntervals[i]; accelValid++; }
      if (gyroIntervals[i] > 0) { gyroSum += gyroIntervals[i]; gyroValid++; }
      if (magIntervals[i] > 0) { magSum += magIntervals[i]; magValid++; }
      if (quatIntervals[i] > 0) { quatSum += quatIntervals[i]; quatValid++; }
      if (linAccelIntervals[i] > 0) { linAccelSum += linAccelIntervals[i]; linAccelValid++; }
      if (gravityIntervals[i] > 0) { gravitySum += gravityIntervals[i]; gravityValid++; }
    }
    
    // Accelerometer timing
    Serial.print("Accel avg interval: "); 
    if (accelValid > 0) {
      float accelAvg = (float)accelSum / accelValid;
      Serial.print(accelAvg, 1);
      Serial.print("ms, Rate: ");
      Serial.print(1000.0 / accelAvg, 1);
      Serial.print("Hz, Samples: ");
      Serial.println(accelValid);
    } else {
      Serial.println("No data");
    }
    
    // Gyroscope timing
    Serial.print("Gyro avg interval: "); 
    if (gyroValid > 0) {
      float gyroAvg = (float)gyroSum / gyroValid;
      Serial.print(gyroAvg, 1);
      Serial.print("ms, Rate: ");
      Serial.print(1000.0 / gyroAvg, 1);
      Serial.print("Hz, Samples: ");
      Serial.println(gyroValid);
    } else {
      Serial.println("No data");
    }
    
    // Magnetometer timing
    Serial.print("Mag avg interval: "); 
    if (magValid > 0) {
      float magAvg = (float)magSum / magValid;
      Serial.print(magAvg, 1);
      Serial.print("ms, Rate: ");
      Serial.print(1000.0 / magAvg, 1);
      Serial.print("Hz, Samples: ");
      Serial.println(magValid);
    } else {
      Serial.println("No data");
    }
    
    // Quaternion/Rotation vector timing
    Serial.print("Quat avg interval: "); 
    if (quatValid > 0) {
      float quatAvg = (float)quatSum / quatValid;
      Serial.print(quatAvg, 1);
      Serial.print("ms, Rate: ");
      Serial.print(1000.0 / quatAvg, 1);
      Serial.print("Hz, Samples: ");
      Serial.println(quatValid);
    } else {
      Serial.println("No data");
    }
    
    // Linear acceleration timing
    Serial.print("LinAccel avg interval: "); 
    if (linAccelValid > 0) {
      float linAccelAvg = (float)linAccelSum / linAccelValid;
      Serial.print(linAccelAvg, 1);
      Serial.print("ms, Rate: ");
      Serial.print(1000.0 / linAccelAvg, 1);
      Serial.print("Hz, Samples: ");
      Serial.println(linAccelValid);
    } else {
      Serial.println("No data");
    }
    
    // Gravity vector timing
    Serial.print("Gravity avg interval: "); 
    if (gravityValid > 0) {
      float gravityAvg = (float)gravitySum / gravityValid;
      Serial.print(gravityAvg, 1);
      Serial.print("ms, Rate: ");
      Serial.print(1000.0 / gravityAvg, 1);
      Serial.print("Hz, Samples: ");
      Serial.println(gravityValid);
    } else {
      Serial.println("No data");
    }
    
    lastPrintTime = currentTime;
  }
  
  // Brief delay to prevent hammering the I2C bus
  delay(1);
}