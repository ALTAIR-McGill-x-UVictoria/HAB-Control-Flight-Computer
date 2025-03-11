#include <Arduino.h>
#include "../../include/SerialCommunication.h"

// Serial communication between two Teensy 4.1 boards
// Pins to use for serial:
// Teensy 1 TX1 → Teensy 2 RX1
// Teensy 1 RX1 → Teensy 2 TX1
// Teensy 1 GND → Teensy 2 GND

// Create the communication object
SerialCommunication comm(Serial1, BoardType::CONTROL_BOARD);

// Data structures
ControlBoardData txData;   // Data to send
PowerBoardData rxData;     // Data to receive

void setup() {
    // Initialize debug serial interface
    Serial.begin(9600);
    
    // Wait until the serial interface is active
    while (!Serial && millis() < 5000);  // Wait for Serial (USB) to become active
    
    // Initialize the communication system (Serial1 will be initialized in begin())
    comm.begin();
    Serial.println("Controller Board initialized");
    Serial.printf("Size of ControlBoardData: %d bytes\n", sizeof(ControlBoardData));
    Serial.printf("Size of PowerBoardData: %d bytes\n", sizeof(PowerBoardData));

    delay(2000);    // Allow more sync time between Teensys

    // Send initial packet to establish communication
    txData.timestamp = millis();
    txData.pressure = 1000;
    txData.altitude = 0;
    txData.temperature = 25;
    strcpy(txData.statusMsg, "Initial message from Control Board");
    txData.statusMsgLength = strlen(txData.statusMsg);
    
    // Initialize the rest of the fields to prevent random values
    txData.accelX = 0;
    txData.accelY = 0;
    txData.accelZ = 0;
    txData.angularVelocityX = 0;
    txData.angularVelocityY = 0;
    txData.angularVelocityZ = 0;
    txData.orientationYaw = 0;
    txData.orientationPitch = 0;
    txData.orientationRoll = 0;
    
    // Send initial packet
    Serial.println("Sending initial data packet...");
    comm.sendData(txData);
    Serial.println("SENT INITIAL DATA:");
    comm.printControlBoardData(txData);
}

// Timer for periodic sending
elapsedMillis sendTimer;
#define SEND_INTERVAL 5000  // Send data every 5 seconds

// Timeout for receiving response
#define RECEIVE_TIMEOUT 2000

void loop() {
    // Send data every 5 seconds
    if (sendTimer >= SEND_INTERVAL) {
        sendTimer = 0;
        
        // Update data values (in a real application, you would get these from sensors)
        txData.timestamp = millis();
        txData.pressure = random(950, 1050);      // Simulated pressure data (in hPa)
        txData.altitude = random(0, 10000);       // Simulated altitude (in meters)
        txData.temperature = random(15, 35);      // Simulated temperature (in Celsius)
        txData.accelX = random(-100, 100) / 10.0f;  // Simulated acceleration
        txData.accelY = random(-100, 100) / 10.0f;
        txData.accelZ = random(-100, 100) / 10.0f;
        txData.angularVelocityX = random(-500, 500) / 10.0f; // Simulated angular velocity (deg/s)
        txData.angularVelocityY = random(-500, 500) / 10.0f;
        txData.angularVelocityZ = random(-500, 500) / 10.0f;
        txData.orientationYaw = random(0, 360);    // Simulated orientation (deg)
        txData.orientationPitch = random(-90, 90);
        txData.orientationRoll = random(-180, 180);
        sprintf(txData.statusMsg, "Status update at %lu ms", txData.timestamp);
        txData.statusMsgLength = strlen(txData.statusMsg);
        
        // Send the data packet
        Serial.println("Sending data packet...");
        if (comm.sendData(txData)) {
            Serial.println("SENT DATA:");
            comm.printControlBoardData(txData);
        } else {
            Serial.println("ERROR: Failed to send data");
        }
        
        // Wait for response
        Serial.println("Waiting for response...");
        if (comm.receiveData(rxData, RECEIVE_TIMEOUT)) {
            Serial.println("RECEIVED DATA:");
            comm.printPowerBoardData(rxData);
            
            // Check for abort command
            if (rxData.abortCommand) {
                Serial.println("!!! ABORT COMMAND RECEIVED !!!");
                // In a real application, you would handle the abort here
            }
        } else {
            Serial.println("Timeout waiting for response data");
            comm.clearBuffers();
        }
    }
}