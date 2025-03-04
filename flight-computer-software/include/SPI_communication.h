// shared_telemetry.h
#ifndef SPI_communication_h
#define __cplusplus
#include <SPI.h>

// Telemetry packet structure matching the specified schema
struct TelemetryPacket {
    uint32_t timestamp;
    uint32_t pressure;
    uint32_t temperature;
    float accelerationX;
    float accelerationY;
    float accelerationZ;
    float velocityX;
    float velocityY;
    float velocityZ;
    float batteryVoltage;
    float latitude;
    float longitude;
    uint16_t statusMsgLength;
    uint8_t* statusMsg;  // Dynamic length array
    uint16_t checksum;
};

// SPI Communication Pins
const int cs = 0;     // Chip Select
const int miso = 1;   // Master In Slave Out
const int mosi = 26;  // Master Out Slave In
const int sck = 27;   // Serial Clock

// Maximum allowed status message length
const uint16_t MAX_STATUS_MSG_LENGTH = 64;

// Communication acknowledge codes
const uint8_t COMM_ACK_SUCCESS = 0xAA;
const uint8_t COMM_ACK_FAILURE = 0x55;

// Function to calculate 16-bit checksum
uint16_t calculateChecksum(const uint8_t* data, size_t length) {
    uint16_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

// Validate checksum
bool validateChecksum(const uint8_t* data, size_t length, uint16_t expectedChecksum) {
    return calculateChecksum(data, length) == expectedChecksum;
}
// Example of populating a TelemetryPacket
void populateTelemetryPacket(TelemetryPacket& packet) {
    // Timestamp (current system time in milliseconds)
    packet.timestamp = millis();

    // Pressure (in Pascals)
    packet.pressure = 99823; // Slightly below standard atmospheric pressure

    // Temperature (multiplied by 10 for precision)
    packet.temperature = 262; // 26.2 degrees Celsius

    // Acceleration (m/s²)
    packet.accelerationX = 2.45;  // Slight acceleration in X direction
    packet.accelerationY = -0.12; // Minor lateral acceleration 
    packet.accelerationZ = 9.81;  // Gravity reference

    // Velocity (m/s)
    packet.velocityX = 12.3;  // Forward velocity
    packet.velocityY = 0.5;   // Slight lateral movement
    packet.velocityZ = 0.0;   // No vertical velocity

    // Battery voltage
    packet.batteryVoltage = 7.2; // LiPo battery voltage

    // GPS Coordinates
    packet.latitude = 37.7749;   // San Francisco latitude
    packet.longitude = -122.4194; // San Francisco longitude

    // Status message
    const char* statusText = "Flight Mode: Active, Systems Nominal";
    packet.statusMsgLength = strlen(statusText);
    
    // Dynamically allocate memory for status message
    packet.statusMsg = new uint8_t[packet.statusMsgLength];
    memcpy(packet.statusMsg, statusText, packet.statusMsgLength);

    // Calculate checksum (this would typically be done when sending)
    uint8_t checksumBuffer[256]; // Adjust size as needed
    size_t bufferIndex = 0;
    
    // Serialize key packet data for checksum calculation
    memcpy(checksumBuffer + bufferIndex, &packet.timestamp, sizeof(packet.timestamp));
    bufferIndex += sizeof(packet.timestamp);
    
    memcpy(checksumBuffer + bufferIndex, &packet.pressure, sizeof(packet.pressure));
    bufferIndex += sizeof(packet.pressure);
    
    // Continue adding other key fields to checksum calculation
    packet.checksum = calculateChecksum(checksumBuffer, bufferIndex);
}


#endif // SPI_communication_h