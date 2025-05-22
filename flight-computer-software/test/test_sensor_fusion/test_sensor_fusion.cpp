/*
  Testing the Sensor Fusion algorithm with multiple BNO080 IMUs
  
  This test demonstrates how the sensorFusion function works with orientation data
  from multiple IMUs. It shows:
  1. Raw orientation values from each IMU
  2. Accuracy values for each IMU
  3. The fused result and which sensor(s) data was selected
  
  Hardware setup:
  - IMU1: 0x4A on Wire1
  - IMU2: 0x4B on Wire1 
  - IMU3: 0x4A on Wire2
*/

#include <Arduino.h>
#include <Wire.h>
#include "../../include/Sensors.h"

Sensors sensors;

// For manual sensor fusion testing
std::vector<float> yawValues;
std::vector<float> pitchValues;
std::vector<float> rollValues;
std::vector<byte> accuracyValues;
std::vector<float> orientationAccuracyValues;

// For storing raw IMU data for display
struct IMUDisplayData {
  float roll;
  float pitch;
  float yaw;
  byte rotationAccuracy;
  float orientationAccuracy;
  bool valid;
};

IMUDisplayData imu1Display;
IMUDisplayData imu2Display;
IMUDisplayData imu3Display;

unsigned long lastDisplayTime = 0;
const unsigned long DISPLAY_INTERVAL = 1000; // 1 second refresh

void displaySensorData() {
  Serial.println("\n-------------------------------------------------");
  Serial.println("IMU ORIENTATION DATA TEST");
  Serial.println("-------------------------------------------------");
  
  // Print status of each IMU
  Serial.println("IMU Status:");
  Serial.print("  IMU1: "); Serial.println(sensors.status.imu1 ? "CONNECTED" : "DISCONNECTED");
  Serial.print("  IMU2: "); Serial.println(sensors.status.imu2 ? "CONNECTED" : "DISCONNECTED");
  Serial.print("  IMU3: "); Serial.println(sensors.status.imu3 ? "CONNECTED" : "DISCONNECTED");
  
  // Print raw data from each IMU
  Serial.println("\nRAW IMU DATA:");
  
  // IMU1 data
  Serial.println("IMU1:");
  if (imu1Display.valid) {
    Serial.print("  Roll: "); Serial.print(imu1Display.roll, 2);
    Serial.print("° | Pitch: "); Serial.print(imu1Display.pitch, 2);
    Serial.print("° | Yaw: "); Serial.print(imu1Display.yaw, 2);
    Serial.print("° | Accuracy: "); Serial.print(imu1Display.rotationAccuracy);
    Serial.print(" | Orient Acc: "); Serial.print(imu1Display.orientationAccuracy, 3); Serial.println("°");
  } else {
    Serial.println("  NO VALID DATA");
  }
  
  // IMU2 data
  Serial.println("IMU2:");
  if (imu2Display.valid) {
    Serial.print("  Roll: "); Serial.print(imu2Display.roll, 2);
    Serial.print("° | Pitch: "); Serial.print(imu2Display.pitch, 2);
    Serial.print("° | Yaw: "); Serial.print(imu2Display.yaw, 2);
    Serial.print("° | Accuracy: "); Serial.print(imu2Display.rotationAccuracy);
    Serial.print(" | Orient Acc: "); Serial.print(imu2Display.orientationAccuracy, 3); Serial.println("°");
  } else {
    Serial.println("  NO VALID DATA");
  }
  
  // IMU3 data
  Serial.println("IMU3:");
  if (imu3Display.valid) {
    Serial.print("  Roll: "); Serial.print(imu3Display.roll, 2);
    Serial.print("° | Pitch: "); Serial.print(imu3Display.pitch, 2);
    Serial.print("° | Yaw: "); Serial.print(imu3Display.yaw, 2);
    Serial.print("° | Accuracy: "); Serial.print(imu3Display.rotationAccuracy);
    Serial.print(" | Orient Acc: "); Serial.print(imu3Display.orientationAccuracy, 3); Serial.println("°");
  } else {
    Serial.println("  NO VALID DATA");
  }
  
  // Print fused results
  float fusedYaw, fusedPitch, fusedRoll;
  sensors.getFusedOrientation(fusedYaw, fusedPitch, fusedRoll);
  
  Serial.println("\nFUSED ORIENTATION DATA:");
  Serial.print("  Roll: "); Serial.print(fusedRoll, 2);
  Serial.print("° | Pitch: "); Serial.print(fusedPitch, 2);
  Serial.print("° | Yaw: "); Serial.print(fusedYaw, 2); Serial.println("°");
  
  // Show how sensor fusion is working
  Serial.println("\nDEBUGGING SENSOR FUSION:");
  Serial.println("Raw Yaw Values Vector:");
  int validCount = 0;
  for (size_t i = 0; i < yawValues.size(); i++) {
    if (accuracyValues[i] >= 0) {
      validCount++;
      Serial.print("  IMU"); Serial.print(i+1);
      Serial.print(": "); Serial.print(yawValues[i], 2);
      Serial.print("° (Accuracy: "); Serial.print((int)accuracyValues[i]);
      Serial.print(", Orient Acc: "); Serial.print(orientationAccuracyValues[i], 3);
      Serial.println("°)");
    } else {
      Serial.print("  IMU"); Serial.print(i+1);
      Serial.println(": INVALID DATA");
    }
  }
  
  // Explain fusion decision
  Serial.print("\nFusion Decision: ");
  if (validCount == 3) {
    Serial.println("Using MEDIAN value (rejecting outliers)");
  } else if (validCount == 2) {
    Serial.println("Using value with BEST ACCURACY");
  } else if (validCount == 1) {
    Serial.println("Using SINGLE WORKING SENSOR");
  } else {
    Serial.println("NO VALID SENSORS - using default value (0)");
  }
}

// Update your setup function:

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000); // Wait up to 5 seconds for serial connection
  
  Serial.println("Sensor Fusion Test - Initializing...");
  
  // Initialize sensors with retry mechanism
  sensors.begin();
  
  // Check if we have enough sensors to proceed
  if (!sensors.areSensorsReady()) {
    Serial.println("\nERROR: Not enough sensors are functioning correctly!");
    Serial.println("Please check connections and try again.");
    
    // Try to recover failed IMUs
    if (!sensors.status.imu1) sensors.resetIMU(1);
    if (!sensors.status.imu2) sensors.resetIMU(2);
    if (!sensors.status.imu3) sensors.resetIMU(3);
    
    // Check again after recovery attempts
    if (!sensors.areSensorsReady()) {
      Serial.println("Recovery failed. System cannot function reliably.");
      while(1) { 
        delay(1000); 
        Serial.println("System halted - please reset board");
      } // Halt
    } else {
      Serial.println("Recovery successful! Continuing...");
    }
  }
  
  // Start sensors with 50ms interval
  sensors.start(50);
  
  // Bonus: Calibrate IMUs for better accuracy
  Serial.println("Calibrating IMUs...");
  sensors.calibrateAllIMUs();
  
  Serial.println("Setup complete! Beginning sensor fusion test...");
}

void loop() {
  // Update our vectors with latest raw data for display - but only consider connected IMUs
  yawValues.clear();
  pitchValues.clear();
  rollValues.clear();
  accuracyValues.clear();
  orientationAccuracyValues.clear();
  
  // Add only connected IMU data to vectors
  if (sensors.status.imu1 && sensors.imu1Data.rotationAccuracy < 254) {
    yawValues.push_back(static_cast<float>(sensors.imu1Data.yawOrientation));
    pitchValues.push_back(static_cast<float>(sensors.imu1Data.pitchOrientation));
    rollValues.push_back(static_cast<float>(sensors.imu1Data.rollOrientation));
    accuracyValues.push_back(static_cast<byte>(sensors.imu1Data.rotationAccuracy));
    orientationAccuracyValues.push_back(static_cast<float>(sensors.imu1Data.orientationAccuracy));
  }
  
  if (sensors.status.imu2 && sensors.imu2Data.rotationAccuracy < 254) {
    yawValues.push_back(static_cast<float>(sensors.imu2Data.yawOrientation));
    pitchValues.push_back(static_cast<float>(sensors.imu2Data.pitchOrientation));
    rollValues.push_back(static_cast<float>(sensors.imu2Data.rollOrientation));
    accuracyValues.push_back(static_cast<byte>(sensors.imu2Data.rotationAccuracy));
    orientationAccuracyValues.push_back(static_cast<float>(sensors.imu2Data.orientationAccuracy));
  }
  
  if (sensors.status.imu3 && sensors.imu3Data.rotationAccuracy < 254) {
    yawValues.push_back(static_cast<float>(sensors.imu3Data.yawOrientation));
    pitchValues.push_back(static_cast<float>(sensors.imu3Data.pitchOrientation));
    rollValues.push_back(static_cast<float>(sensors.imu3Data.rollOrientation));
    accuracyValues.push_back(static_cast<byte>(sensors.imu3Data.rotationAccuracy));
    orientationAccuracyValues.push_back(static_cast<float>(sensors.imu3Data.orientationAccuracy));
  }
  
  // Update display data structures
  imu1Display = {
    .roll = sensors.imu1Data.rollOrientation,
    .pitch = sensors.imu1Data.pitchOrientation,
    .yaw = sensors.imu1Data.yawOrientation,
    .rotationAccuracy = sensors.imu1Data.rotationAccuracy,
    .orientationAccuracy = sensors.imu1Data.orientationAccuracy,
    .valid = (sensors.imu1Data.rotationAccuracy >= 0)
  };
  
  imu2Display = {
    .roll = sensors.imu2Data.rollOrientation,
    .pitch = sensors.imu2Data.pitchOrientation,
    .yaw = sensors.imu2Data.yawOrientation,
    .rotationAccuracy = sensors.imu2Data.rotationAccuracy,
    .orientationAccuracy = sensors.imu2Data.orientationAccuracy,
    .valid = (sensors.imu2Data.rotationAccuracy >= 0)
  };
  
  imu3Display = {
    .roll = sensors.imu3Data.rollOrientation,
    .pitch = sensors.imu3Data.pitchOrientation,
    .yaw = sensors.imu3Data.yawOrientation,
    .rotationAccuracy = sensors.imu3Data.rotationAccuracy,
    .orientationAccuracy = sensors.imu3Data.orientationAccuracy,
    .valid = (sensors.imu3Data.rotationAccuracy >= 0)
  };
  
  // Display information periodically
  if (millis() - lastDisplayTime >= DISPLAY_INTERVAL) {
    displaySensorData();
    lastDisplayTime = millis();
  }
}
