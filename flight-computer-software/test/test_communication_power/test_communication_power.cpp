// slave_spi.cpp
#include <SPI.h>
#include "SPI_communication.h"

volatile bool dataReceived = false;
TelemetryPacket receivedPacket;

void processReceivedTelemetryPacket(const TelemetryPacket& packet) {
    // Implementation for processing received telemetry data
    // Print out received data in console
    delay(1000);
    Serial.println("Processed Telemetry Data:");
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
    // Delay to allow master to initialize
    delay(5000); 
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
        // Initialize a dummy byte to send to allow for the reading of the master value
        const uint8_t receivedFirstToken = SPI.transfer(0x00);
        if (receivedFirstToken == 0xAB) {
            Serial.println("Received first token: " + String(receivedFirstToken));
            const uint8_t receivedSecondToken = SPI.transfer(COMM_ACK_SUCCESS);
            if (receivedSecondToken == 0x0A) {
                Serial.println("Handshake successful, communication established.");
            }
        } else {
            Serial.println("Handshake failed, communication not established.\n");
            Serial.println("Received unexpected first token: " + String(receivedFirstToken));
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

    while(true) {
        Serial.println("Test completed for handshake and test packet reception.");
        delay(100000);
    }
}