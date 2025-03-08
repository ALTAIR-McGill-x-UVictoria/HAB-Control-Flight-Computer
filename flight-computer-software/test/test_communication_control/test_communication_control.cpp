// master_spi.cpp
#include <SPI.h>
#include "SPI_communication.h"

// Master setup function
void setup() {
    // Configure SPI pins
    pinMode(cs, OUTPUT);
    pinMode(mosi, OUTPUT);
    pinMode(miso, INPUT);
    pinMode(sck, OUTPUT);
    
    // Initialize SPI
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16);  // 1 MHz clock
    // Teensy 4.1 is ARM-based, thus uses little-endian byte order
    SPI.setBitOrder(LSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    delay(5000);
}

bool performHandshake() {
    Serial.println("Initializing handshake...");
    const uint8_t handshakeToken = 0xAB;

    // Assert Chip Select
    digitalWrite(cs, LOW);
    
    // Send handshake token
    Serial.print("Sending handshake token: " + String(handshakeToken));
    const uint8_t firstResponse = SPI.transfer(handshakeToken);
    
    // Read first response from slave
    if (firstResponse == 0x00) {
        Serial.print("Received first response from slave, awaiting success confirmation.");
        const uint8_t secondResponse = SPI.transfer(0x00);
        if (secondResponse == COMM_ACK_SUCCESS) {
            Serial.print("Received success confirmation from slave.");
            return true;
        } else if (secondResponse == COMM_ACK_FAILURE) {
            Serial.print("Received failure confirmation from slave.");
            return false;
        } else {
            Serial.print("Received unknown second response from slave:" + String(secondResponse));
            return false;
        }
    } else {
        Serial.print("Received unknown first response from slave:" + String(firstResponse));
        return false;
    }
    // Deassert Chip Select
    digitalWrite(cs, HIGH);
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
    digitalWrite(cs, LOW);
    for (size_t i = 0; i < bufferIndex; i++) {
        SPI.transfer(buffer[i]);
    }
    digitalWrite(cs, HIGH);

    return true;
}

void loop() {
    // Example usage in main loop
    if (performHandshake()  == true) {
        TelemetryPacket packet;
        // Populate packet with test sensor data from a pre-made function
        populateTelemetryPacket(packet);
        // Send packet over SPI
        sendTelemetryPacket(packet);
    } else {
        Serial.println("Handshake failed, communication not established.");
    }

    while(true) {
        Serial.println("Test completed for handshake and test packet transmission.");
        // Adding delay to prevent spamming the serial monitor
        delay(100000);
    }
}