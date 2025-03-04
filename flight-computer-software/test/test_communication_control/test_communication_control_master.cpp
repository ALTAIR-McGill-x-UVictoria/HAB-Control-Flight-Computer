// master_spi.cpp
#include <SPI.h>
#include "SPI_communication.h"

void setupMasterSPI() {
    // Configure SPI pins
    pinMode(CS, OUTPUT);
    pinMode(MOSI, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(SCK, OUTPUT);
    
    // Initialize SPI
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16);  // 1 MHz clock
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
}

bool performHandshake() {
    const uint8_t handshakeToken = 0xAB;
    uint8_t response = 0;

    // Assert Chip Select
    digitalWrite(CS, LOW);
    
    // Send handshake token
    SPI.transfer(handshakeToken);
    
    // Read response
    response = SPI.transfer(0x00);
    
    // Deassert Chip Select
    digitalWrite(CS, HIGH);

    // Check if slave responded correctly
    return (response == COMM_ACK_SUCCESS);
}

bool sendTelemetryPacket(const TelemetryPacket& packet) {
    // Verify status message length
    if (packet.statusMsgLength > MAX_STATUS_MSG_LENGTH) {
        return false;
    }

    // Prepare transfer buffer
    uint8_t buffer[sizeof(TelemetryPacket) + MAX_STATUS_MSG_LENGTH];
    size_t bufferIndex = 0;

    // Copy fixed-length fields
    memcpy(buffer + bufferIndex, &packet.timestamp, sizeof(packet.timestamp));
    bufferIndex += sizeof(packet.timestamp);
    
    memcpy(buffer + bufferIndex, &packet.pressure, sizeof(packet.pressure));
    bufferIndex += sizeof(packet.pressure);

    // Continue copying other fixed-length fields...
    // (Add similar memcpy for temperature, accelerations, velocities, etc.)

    // Copy status message
    buffer[bufferIndex++] = packet.statusMsgLength;
    memcpy(buffer + bufferIndex, packet.statusMsg, packet.statusMsgLength);
    bufferIndex += packet.statusMsgLength;

    // Calculate and add checksum
    uint16_t calculatedChecksum = calculateChecksum(buffer, bufferIndex);
    memcpy(buffer + bufferIndex, &calculatedChecksum, sizeof(calculatedChecksum));
    bufferIndex += sizeof(calculatedChecksum);

    // Transfer packet
    digitalWrite(CS, LOW);
    for (size_t i = 0; i < bufferIndex; i++) {
        SPI.transfer(buffer[i]);
    }
    digitalWrite(CS, HIGH);

    return true;
}

void loop() {
    // Example usage in main loop
    if (performHandshake()) {
        TelemetryPacket packet;
        // Populate packet with test sensor data from a pre-made function
        populateTelemetryPacket(packet);
        // Send packet over SPI
        sendTelemetryPacket(packet);
    }
}