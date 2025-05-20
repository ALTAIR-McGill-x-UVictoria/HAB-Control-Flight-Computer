/**
 * Test Single IMU - Comprehensive debugging tool
 * 
 * This test program initializes a single BNO080 IMU and prints all 
 * available data to the terminal for debugging purposes.
 */

#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

// Create IMU instance
BNO080 imu;

// Constants
#define IMU_UPDATE_RATE 50  // Hz (how often to read from IMU)
#define SERIAL_BAUD_RATE 115200
#define I2C_CLOCK_SPEED 40000  // Further reduced for reliability (from 100000)
#define IMU_1_ADDRESS 0x4A   // IMU1 address (on Wire1)
#define IMU_2_ADDRESS 0x4B   // IMU2 address (on Wire1)
#define IMU_3_ADDRESS 0x4A   // IMU3 address (on Wire2)

// Hardware reset pin definitions - adjust according to your hardware
#define IMU_RESET_PIN 14     // Connected to RESET pin on BNO080 (if available)
#define IMU_PS0_PIN 15       // Connected to PS0 pin for I2C address control (if available)
#define IMU_PS1_PIN 16       // Connected to PS1 pin for I2C address control (if available)
#define HARDWARE_RESET_AVAILABLE true  // Set to true if reset pin is connected

// Statistics tracking
unsigned long resetCount = 0;
unsigned long packetsReceived = 0;
unsigned long lastPacketTime = 0;
unsigned long startTime = 0;
bool imuConnected = false;

// Debugging counters
unsigned long loopCount = 0;
unsigned long dataAvailableCount = 0;
unsigned long lastDebugTime = 0;

// Rotation vector accuracy values
const char* getAccuracyString(byte accuracy) {
  switch(accuracy) {
    case 0: return "Unreliable";
    case 1: return "Low";
    case 2: return "Medium";
    case 3: return "High";
    default: return "Unknown";
  }
}

// Global variables for I2C scanning
byte foundDevices = 0;

// Function to scan I2C bus and find connected devices
void scanI2CBus(TwoWire &wire) {
  foundDevices = 0;
  Serial.println("\n=== I2C BUS SCAN ===");
  Serial.print("Scanning I2C bus on ");
  if (&wire == &Wire) Serial.print("Wire (I2C0)");
  else if (&wire == &Wire1) Serial.print("Wire1 (I2C1)");
  else Serial.print("Wire2 (I2C2)");
  Serial.println("...");
  
  for (byte address = 1; address < 127; address++) {
    wire.beginTransmission(address);
    byte error = wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.print(" (");
      Serial.print(address);
      Serial.print(")");
      
      // Identify known IMU configurations
      if (&wire == &Wire1 && address == IMU_1_ADDRESS) 
        Serial.print(" - Matches IMU1 configuration!");
      else if (&wire == &Wire1 && address == IMU_2_ADDRESS)
        Serial.print(" - Matches IMU2 configuration!");
      else if (&wire == &Wire2 && address == IMU_3_ADDRESS)
        Serial.print(" - Matches IMU3 configuration!");
        
      Serial.println();
      foundDevices++;
    }
  }
  
  if (foundDevices == 0) {
    Serial.println("No I2C devices found on this bus");
  } else {
    Serial.print("Found ");
    Serial.print(foundDevices);
    Serial.println(" device(s)");
  }
  Serial.println("=====================");
}

// Function to perform hardware reset of the IMU if pins are available
void hardwareResetIMU() {
  if (!HARDWARE_RESET_AVAILABLE) {
    Serial.println("Hardware reset not configured - skipping");
    return;
  }
  
  Serial.println("Performing hardware reset of IMU");
  
  // Configure reset pin
  pinMode(IMU_RESET_PIN, OUTPUT);
  
  // Configure PS0/PS1 pins if they're connected
  pinMode(IMU_PS0_PIN, OUTPUT);
  pinMode(IMU_PS1_PIN, OUTPUT);
  
  // Reset sequence
  digitalWrite(IMU_RESET_PIN, HIGH);
  delay(10);
  digitalWrite(IMU_RESET_PIN, LOW); // Active low reset
  delay(50);  // Hold in reset for 50ms
  digitalWrite(IMU_RESET_PIN, HIGH);
  delay(500); // Give IMU time to boot up after reset
  
  Serial.println("Hardware reset completed");
}

// Function to cycle I2C power if possible
void cycleI2CPower() {
  Serial.println("Cycling I2C power if supported...");
  
  // Turn off I2C - Note: implementation depends on hardware
  Wire.end();
  Wire1.end();
  #if defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY36)
  Wire2.end();
  #endif
  
  delay(500);
  
  // Restart I2C buses
  Wire.begin();
  Wire1.begin();
  Wire.setClock(I2C_CLOCK_SPEED);
  Wire1.setClock(I2C_CLOCK_SPEED);
  
  #if defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY36)
  Wire2.begin();
  Wire2.setClock(I2C_CLOCK_SPEED);
  #endif
  
  Serial.println("I2C bus restarted");
}

// Function to initialize the IMU with basic reports first
bool initIMU() {
  // First try hardware reset if pins are connected
  hardwareResetIMU();
  
  // Cycle I2C power if supported
  cycleI2CPower();
  
  // Scan all available I2C buses to see what's connected
  Wire1.begin();
  Wire1.setClock(I2C_CLOCK_SPEED);
  scanI2CBus(Wire1);
  
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);
  scanI2CBus(Wire);
  
  // Also check Wire2 if it's available on this platform
  #if defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY36)
  Wire2.begin();
  Wire2.setClock(I2C_CLOCK_SPEED);
  scanI2CBus(Wire2);
  #endif
  
  Serial.println("Attempting direct hardware reset sequence...");
  
  // Try all known IMU configurations from the system
  // Try IMU1 configuration (Wire1, 0x4A)
  Serial.println("Trying IMU1 configuration (Wire1, 0x4A)...");
  Wire1.begin();
  Wire1.setClock(I2C_CLOCK_SPEED);
  
  // Reset I2C bus by toggling clock (advanced fix)
  Serial.println("Resetting I2C bus state...");
  #if defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY40)
  const int SCL1_PIN = 16; // Wire1 SCL pin on Teensy 4.0/4.1
  #elif defined(ARDUINO_TEENSY36)
  const int SCL1_PIN = 37; // Wire1 SCL pin on Teensy 3.6
  #else
  const int SCL1_PIN = SCL; // Default to standard SCL pin
  #endif
  
  pinMode(SCL1_PIN, OUTPUT); 
  digitalWrite(SCL1_PIN, LOW);
  delay(100);
  digitalWrite(SCL1_PIN, HIGH);
  delay(100);
  pinMode(SCL1_PIN, INPUT);
  delay(100);
  
  // Try with retry pattern
  bool imuSuccess = false;
  for (int retry = 0; retry < 3 && !imuSuccess; retry++) {
    Serial.print("Connection attempt #");
    Serial.println(retry + 1);
    
    if (imu.begin(IMU_1_ADDRESS, Wire1)) {
      Serial.println("IMU connected as IMU1 (Wire1, 0x4A)");
      
      // Use full reset sequence
      Serial.println("Performing complete reset sequence...");
      imu.softReset();
      delay(100);
      imu.resetReason(); // Clear any reset reason
      delay(100);
      
      // Set to default power mode
      Serial.println("Setting power mode...");
      delay(100);
      
      // Try enabling just one feature
      Serial.println("Enabling rotation vector...");
      imu.enableRotationVector(100); // Try lower report rate
      delay(300);
      
      // Check for data with longer timeout
      Serial.println("Checking for data (extended wait)...");
      unsigned long startCheck = millis();
      while (millis() - startCheck < 5000) { // Longer 5-second timeout
        if (imu.dataAvailable()) {
          Serial.println("Data confirmed available from IMU1!");
          imuSuccess = true;
          break;
        }
        
        // Print status info
        if ((millis() - startCheck) % 1000 == 0) {
          Serial.print(".");
        }
        delay(10);
      }
      
      if (imuSuccess) {
        Serial.println("\nIMU successfully providing data!");
        return true;
      } else {
        Serial.println("\nConnected to IMU1 but no data available after extended wait.");
        // Try another approach if this one failed
        imu.softReset();
        delay(300);
      }
    }
  }
  
  // Try IMU2 configuration (Wire1, 0x4B)
  Serial.println("Trying IMU2 configuration (Wire1, 0x4B)...");
  for (int retry = 0; retry < 3 && !imuSuccess; retry++) {
    Serial.print("Connection attempt #");
    Serial.println(retry + 1);
    
    if (imu.begin(IMU_2_ADDRESS, Wire1)) {
      Serial.println("IMU connected as IMU2 (Wire1, 0x4B)");
      // Similar initialization sequence as above
      // ...
      
      // Check for data with longer timeout
      Serial.println("Checking for data (extended wait)...");
      unsigned long startCheck = millis();
      while (millis() - startCheck < 5000) {
        if (imu.dataAvailable()) {
          Serial.println("Data confirmed available from IMU2!");
          imuSuccess = true;
          break;
        }
        
        if ((millis() - startCheck) % 1000 == 0) {
          Serial.print(".");
        }
        delay(10);
      }
      
      if (imuSuccess) {
        Serial.println("\nIMU2 successfully providing data!");
        return true;
      }
    }
  }
  
  // Try IMU3 configuration (Wire2, 0x4A) if available
  #if defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY36)
  Serial.println("Trying IMU3 configuration (Wire2, 0x4A)...");
  Wire2.begin();
  Wire2.setClock(I2C_CLOCK_SPEED);
  
  if (imu.begin(IMU_3_ADDRESS, Wire2)) {
    Serial.println("IMU connected as IMU3 (Wire2, 0x4A)");
    imu.softReset();
    delay(500);
    imu.enableRotationVector(50);
    delay(300);
    
    // Check for data
    Serial.println("Checking for data availability...");
    unsigned long startCheck = millis();
    while (millis() - startCheck < 2000) {
      if (imu.dataAvailable()) {
        Serial.println("Data confirmed available from IMU3!");
        return true;
      }
      delay(10);
    }
    
    Serial.println("Connected to IMU3 but no data available. Proceeding anyway.");
    return true;
  }
  #endif
  
  // Final fallback - if IMU is connected but not responding, proceed anyway
  Serial.println("All recovery methods exhausted.");
  if (imu.begin(IMU_1_ADDRESS, Wire1) || imu.begin(IMU_2_ADDRESS, Wire1)) {
    Serial.println("IMU connected but not responding - proceeding anyway.");
    imu.enableRotationVector(10); // Try ultra-low rate
    return true;
  }
  
  Serial.println("IMU initialization failed on all configurations");
  return false;
}

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD_RATE);
  delay(500); // Wait for serial to initialize
  
  Serial.println("\n=== BNO080 IMU Test and Debug Tool ===");
  
  // Try to initialize the IMU
  Serial.println("Initializing IMU...");
  imuConnected = initIMU();

  if (imuConnected) {
    Serial.println("IMU initialization successful!");
    startTime = millis();
  } else {
    Serial.println("Could not connect to IMU. Will retry in loop()");
  }
  
  Serial.println("Setup complete. Monitoring IMU data...");
  Serial.println("============================================");
  lastDebugTime = millis();
}

void loop() {
  loopCount++;
  
  // Print diagnostic info every second even if no data available
  if (millis() - lastDebugTime >= 1000) {
    lastDebugTime = millis();
    Serial.printf("\nDiagnostics: Loop count=%lu, Data packets=%lu, Connection status=%s\n",
                  loopCount, dataAvailableCount, imuConnected ? "Connected" : "Disconnected");
    Serial.printf("Time since last packet: %lu ms\n", 
                  lastPacketTime > 0 ? millis() - lastPacketTime : 0);
    loopCount = 0; // Reset for the next interval
    
    // Attempt recovery if needed
    if (lastPacketTime == 0 || millis() - lastPacketTime > 3000) {
      Serial.println("No data received recently. Attempting recovery...");
      
      // If we've never received data or it's been too long
      if (!imuConnected || millis() - lastPacketTime > 5000) {
        Serial.println("Reinitializing IMU with full detection sequence...");
        imu.softReset();
        delay(300); // Longer delay for soft reset
        imuConnected = initIMU();
        
        // Try forcing a different I2C speed as a last resort
        if (!imuConnected) {
          Serial.println("Trying with very slow I2C clock...");
          Wire1.setClock(10000); // Ultra slow I2C
          Wire.setClock(10000);  // Ultra slow I2C
          imuConnected = initIMU();
        }
      } else {
        // Try resetting and re-enabling reports
        Serial.println("Soft-resetting IMU...");
        imu.softReset();
        delay(300); // Longer delay
        imu.enableRotationVector(IMU_UPDATE_RATE);
        delay(100);
      }
    }
  }

  // If we're not connected, keep trying to initialize
  if (!imuConnected) {
    static unsigned long lastRetry = 0;
    if (millis() - lastRetry > 5000) {
      lastRetry = millis();
      Serial.println("Retrying IMU connection...");
      imuConnected = initIMU();
    }
    delay(10);
    return;
  }

  // Check if IMU has new data (this is key for diagnostics)
  if (imu.dataAvailable()) {
    dataAvailableCount++;
    lastPacketTime = millis();
    packetsReceived++;
    
    // Check for resets
    if (imu.hasReset()) {
      resetCount++;
      Serial.println("\n!!! IMU RESET DETECTED !!!");
      
      // Re-enable reports
      Serial.println("Re-enabling reports after reset");
      delay(50);
      imu.enableRotationVector(IMU_UPDATE_RATE);
      delay(50);
      
      // Only gradually add other reports after we verify the IMU is stable
      if (packetsReceived > 100) {
        imu.enableAccelerometer(IMU_UPDATE_RATE);
        imu.enableGyro(IMU_UPDATE_RATE);
        delay(50);
      }
    }
    
    // After receiving enough packets successfully, enable all reports
    static bool allReportsEnabled = false;
    if (!allReportsEnabled && packetsReceived > 50) {
      Serial.println("\nIMU stable, enabling additional reports...");
      imu.enableAccelerometer(IMU_UPDATE_RATE);
      delay(50);
      imu.enableGyro(IMU_UPDATE_RATE);
      delay(50);
      imu.enableLinearAccelerometer(IMU_UPDATE_RATE);
      delay(50);
      imu.enableMagnetometer(IMU_UPDATE_RATE);
      delay(50);
      allReportsEnabled = true;
    }
    
    // ROTATION VECTOR (orientation)
    float quatI = imu.getQuatI();
    float quatJ = imu.getQuatJ();
    float quatK = imu.getQuatK();
    float quatReal = imu.getQuatReal();
    byte quatAccuracy = imu.getQuatRadianAccuracy();
    
    // Convert quaternion to Euler angles (roll, pitch, yaw)
    float roll = atan2(2.0f * (quatReal * quatI + quatJ * quatK), 
                      1.0f - 2.0f * (quatI * quatI + quatJ * quatJ)) * 180.0f / PI;
    float pitch = asin(2.0f * (quatReal * quatJ - quatK * quatI)) * 180.0f / PI;
    float yaw = atan2(2.0f * (quatReal * quatK + quatI * quatJ), 
                     1.0f - 2.0f * (quatJ * quatJ + quatK * quatK)) * 180.0f / PI;

    // Simplified output to reduce serial overhead
    static unsigned long lastOutputTime = 0;
    if (millis() - lastOutputTime >= 250) {
      lastOutputTime = millis();
      Serial.println("\n=== IMU DATA ===");
      Serial.printf("Packets: %lu, Resets: %lu\n", packetsReceived, resetCount);
      Serial.printf("Euler: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f° (Accuracy: %s)\n", 
                   roll, pitch, yaw, getAccuracyString(quatAccuracy));

      // Show other data if available
      if (allReportsEnabled) {
        Serial.printf("Accel: X=%.1f, Y=%.1f, Z=%.1f m/s²\n", 
                     imu.getAccelX(), imu.getAccelY(), imu.getAccelZ());
        Serial.printf("Gyro: X=%.1f, Y=%.1f, Z=%.1f °/s\n", 
                     imu.getGyroX() * 180.0f / PI, imu.getGyroY() * 180.0f / PI, imu.getGyroZ() * 180.0f / PI);
      }
    }
  }
  
  delay(5); // Small delay for stability
}