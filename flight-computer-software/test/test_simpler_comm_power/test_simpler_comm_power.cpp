#include <Arduino.h>
#include "../../include/SerialCommunication.h"

// Serial communication between two Teensy 4.1 boards
// Pins to use for serial:
// Teensy 1 TX1 → Teensy 2 RX1
// Teensy 1 RX1 → Teensy 2 TX1
// Teensy 1 GND → Teensy 2 GND

// Create the communication object
SerialCommunication comm(Serial1, BoardType::POWER_BOARD);

// Data structures
ControlBoardData rxData;   // Data to receive
PowerBoardData txData;     // Data to send back

// Simulated sensor readings (in a real application, these would come from actual sensors)
float getBatteryVoltage() {
    return 11.5 + (random(0, 10) / 10.0);  // 11.5 to 12.4 volts
}

float getLatitude() {
    return 37.7749 + (random(-100, 100) / 10000.0);  // Simulated location near San Francisco
}

float getLongitude() {
    return -122.4194 + (random(-100, 100) / 10000.0);  // Simulated location near San Francisco
}

void setup() {
    // Initialize serial interfaces
    Serial.begin(9600);
    
    // Wait until the serial interfaces are active
    while (!Serial && millis() < 5000);  // Wait for Serial (USB) to become active
    
    comm.begin();
    Serial.println("Receiver Board initialized");
    Serial.printf("Size of ControlBoardData: %d bytes\n", sizeof(ControlBoardData));
    Serial.printf("Size of PowerBoardData: %d bytes\n", sizeof(PowerBoardData));
}

void loop() {
    // Debug output (once per second)
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 1000) {
        Serial.println("Waiting for controller data...");
        lastDebugTime = millis();
    }
    
    // Try to receive data from controller
    if (comm.receiveData(rxData, 100)) { // Short timeout for responsive loop
        // or use comm.receiveBoardData(rxData, 100);
        Serial.println("RECEIVED DATA:");
        comm.printControlBoardData(rxData);
        
        // Prepare response data
        txData.transponderTimestamp = millis();
        txData.batteryVoltage = getBatteryVoltage();
        txData.latitude = getLatitude();
        txData.longitude = getLongitude();
        txData.abortCommand = false; // Default to false
        
        // Add a status message
        strcpy(txData.statusMsg, "Power board operating normally");
        txData.statusMsgLength = strlen(txData.statusMsg);
        
        // Send response
        comm.sendData(txData);  // or use comm.sendBoardData(txData);
        Serial.println("SENT DATA:");
        comm.printPowerBoardData(txData);
    }
}