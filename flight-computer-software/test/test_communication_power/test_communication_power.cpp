// slave_spi.cpp
#include <SPI.h>
#include "SPI_communication.h"

volatile bool dataReceived = false;
TelemetryPacket receivedPacket;

void processReceivedTelemetryPacket(const TelemetryPacket& packet) {
    // Implementation for processing received telemetry data
    // Print out received data in console
    delay(1000);
    Serial.println("Received Telemetry Data:");
    // Serial.println("Received Telemetry Data:");
    // Serial.print("Timestamp: " + String(packet.timestamp) + " ms");
    // Serial.print("Pressure: " + String(packet.pressure) + " Pa");
    // Serial.print("Temperature: " + String(packet.temperature) + " C");
    // // Print out other telemetry data fields...
    // // (Add similar print statements for accelerations, velocities, battery voltage, etc.)
    // Serial.print("Latitude: " + String(packet.latitude));
    // Serial.print("Longitude: " + String(packet.longitude));
    // Serial.print("Status Message: ");
    // Serial.println(String(reinterpret_cast<char*>(packet.statusMsg)));
    // Serial.println();
}

// Slave setup function
void setup() {
    // Configure SPI pins for slave mode
    pinMode(cs, INPUT);
    pinMode(mosi, INPUT);
    pinMode(miso, OUTPUT);
    pinMode(sck, INPUT);

    // Initialize SPI in slave mode
    SPI.begin();
}

void handleHandshake() {
    uint8_t receivedToken = SPI.transfer(COMM_ACK_SUCCESS);
    
    // Check if handshake token is correct
    if (receivedToken == 0xAB) {
        SPI.transfer(COMM_ACK_SUCCESS);
    } else {
        SPI.transfer(COMM_ACK_FAILURE);
    }
}

bool receiveTelemetryPacket() {
    uint8_t buffer[sizeof(TelemetryPacket) + MAX_STATUS_MSG_LENGTH];
    size_t bufferIndex = 0;

    // Receive entire packet
    while (bufferIndex < sizeof(buffer)) {
        buffer[bufferIndex++] = SPI.transfer(0x00);
    }

    // Extract fixed-length fields
    size_t offset = 0;
    memcpy(&receivedPacket.timestamp, buffer + offset, sizeof(receivedPacket.timestamp));
    offset += sizeof(receivedPacket.timestamp);

    memcpy(&receivedPacket.pressure, buffer + offset, sizeof(receivedPacket.pressure));
    offset += sizeof(receivedPacket.pressure);

    // Continue extracting other fixed-length fields...
    // (Add similar memcpy for temperature, accelerations, velocities, etc.)

    // Extract status message
    receivedPacket.statusMsgLength = buffer[offset++];
    receivedPacket.statusMsg = new uint8_t[receivedPacket.statusMsgLength];
    memcpy(receivedPacket.statusMsg, buffer + offset, receivedPacket.statusMsgLength);
    offset += receivedPacket.statusMsgLength;

    // Extract and validate checksum
    memcpy(&receivedPacket.checksum, buffer + offset, sizeof(receivedPacket.checksum));
    
    // Verify checksum
    bool checksumValid = validateChecksum(buffer, offset, receivedPacket.checksum);
    
    if (!checksumValid) {
        delete[] receivedPacket.statusMsg;
        return false;
    }

    dataReceived = true;
    return true;
}

void loop() {
    // Handle incoming SPI communication
    if (digitalRead(cs) == LOW) {
        // Detect if this is a handshake or data transfer
        uint8_t firstByte = SPI.transfer(0x00);
        
        if (firstByte == 0xAB) {
            handleHandshake();
        } else {
            // Assume it's a data transfer
            receiveTelemetryPacket();
        }
    }

    // Process received data when available
    if (dataReceived) {
        // Process telemetry data
        // Example: Log received data, update system state
        processReceivedTelemetryPacket(receivedPacket);
        
        // Clean up
        delete[] receivedPacket.statusMsg;
        dataReceived = false;
    }
}