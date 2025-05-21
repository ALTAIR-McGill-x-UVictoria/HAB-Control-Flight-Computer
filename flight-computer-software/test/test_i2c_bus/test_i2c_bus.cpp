#include <Wire.h>
#include <Arduino.h>

void scanI2CBus(TwoWire &wire, const char* busName) {
  byte error, address;
  int deviceCount = 0;
  
  Serial.print("\nScanning I2C Bus: ");
  Serial.println(busName);
  Serial.println("------------------------------");
  
  for(address = 1; address < 127; address++) {
    wire.beginTransmission(address);
    error = wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Try to identify common I2C devices
      if (address == 0x4A || address == 0x4B) {
        Serial.print(" (Possibly BNO080 IMU)");
      }
      
      Serial.println();
      deviceCount++;
    }
    else if (error == 4) {
      Serial.print("Error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found on this bus");
  } else {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.print(" device");
    if (deviceCount > 1) Serial.print("s");
    Serial.println(" on this bus");
  }
  
  Serial.println("------------------------------");
}

void setup() {
  // Initialize serial and wait for connection
  Serial.begin(115200);
  delay(2000); // Give serial time to initialize
  
  Serial.println("\n=== I2C MULTI-BUS SCANNER ===");
  
  // Initialize both I2C buses
  Wire1.begin();
  Wire1.setClock(100000); // 100 kHz
  
  Wire2.begin();
  Wire2.setClock(100000); // 100 kHz
  
  // Scan both buses
  scanI2CBus(Wire1, "Wire1");
  scanI2CBus(Wire2, "Wire2");
  
  Serial.println("\nScanning complete!");
}

void loop() {
  // Nothing to do here
  delay(5000);
  
  // Optionally, you could periodically rescan:
  // scanI2CBus(Wire1, "Wire1");
  // scanI2CBus(Wire2, "Wire2");
  // Serial.println("\n");
}