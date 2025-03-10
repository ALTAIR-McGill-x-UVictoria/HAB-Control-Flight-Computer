#ifndef SERIAL_COMMUNICATION_H
#define SERIAL_COMMUNICATION_H

#include <Arduino.h>

// Status message buffer size
#define MAX_STATUS_MSG_LENGTH 64

#define baud 230400

// Define a packet header for reliable packet detection
static const uint8_t PACKET_HEADER[] = {0xAA, 0xBB};

// Define board types
enum class BoardType {
    CONTROL_BOARD,
    POWER_BOARD
};

#pragma pack(push,1)
/* Data structure to be sent from control board */
struct ControlBoardData {
    uint32_t timestamp;        // 0x00: Time of data capture
    uint32_t pressure;         // 0x04: Atmospheric pressure
    uint32_t altitude;         // 0x08: Altitude
    uint32_t temperature;      // 0x0C: Temperature probe reading
    float accelX;              // 0x10: X-axis linear acceleration (m/s^2)
    float accelY;              // 0x14: Y-axis linear acceleration (m/s^2)
    float accelZ;              // 0x18: Z-axis linear acceleration (m/s^2)
    float angularVelocityX;    // 0x1C: X-axis angular velocity (deg/s)
    float angularVelocityY;    // 0x20: Y-axis angular velocity (deg/s)
    float angularVelocityZ;    // 0x24: Z-axis angular velocity (deg/s)
    float orientationYaw;      // 0x28: Yaw (deg)
    float orientationPitch;    // 0x2C: Pitch (deg)
    float orientationRoll;     // 0x2E: Roll (deg)
    uint16_t statusMsgLength;  // 0x30: Length of the variable data section
    char statusMsg[MAX_STATUS_MSG_LENGTH]; // 0x32: Extra data with dynamic length
    uint16_t checksum;         // 0x32 + Msg Length: Error detection
};

/* Data structure to be received from power board */
struct PowerBoardData {
    uint32_t transponderTimestamp; // 0x00: Time
    float batteryVoltage;      // 0x04: Voltage value of battery
    float latitude;            // 0x08: GPS latitude
    float longitude;           // 0x0C: GPS longitude
    bool abortCommand;         // 0x10: Command to abort, default false
    uint16_t statusMsgLength;  // 0x11: Length of the variable data section
    char statusMsg[MAX_STATUS_MSG_LENGTH]; // 0x13: Status message and heartbeat
    uint16_t checksum;         // Checksum at the end
};
#pragma pack(pop)

class SerialCommunication {
private:
    Stream& dataSerial;    // Serial for data transmission (e.g., Serial1)
    BoardType boardType;   // Type of board this instance represents
    
    // Internal tracking
    bool headerFound = false;
    size_t bytesRead = 0;
    unsigned long packetStartTime = 0;
    uint8_t* receiveBuffer;
    size_t receiveBufferSize;

public:
    // Constructor
    SerialCommunication(Stream& data, BoardType type = BoardType::CONTROL_BOARD) 
        : dataSerial(data), boardType(type), receiveBuffer(nullptr), receiveBufferSize(0) {
    }
    
    ~SerialCommunication() {
        if (receiveBuffer) {
            delete[] receiveBuffer;
        }
    }

    void begin() {
        // Note: caller must initialize the HardwareSerial objects separately
        Serial.println("Serial Communication initialized");
        if (boardType == BoardType::CONTROL_BOARD) {
            Serial.println("Configured as Control Board");
            // Control board receives PowerBoardData
            prepareReceiveBuffer<PowerBoardData>();
        } else {
            Serial.println("Configured as Power Board");
            // Power board receives ControlBoardData
            prepareReceiveBuffer<ControlBoardData>();
        }
        clearBuffers();
    }

    // Function to calculate simple checksum
    uint16_t calculateChecksum(uint8_t* data, size_t length) {
        uint16_t sum = 0;
        for (size_t i = 0; i < length; i++) {
            sum += data[i];
        }
        return sum;
    }

    void clearBuffers() {
        while (dataSerial.available()) {
            dataSerial.read(); // Discard any data
        }
        delay(100);
    }

    // Prepare a receive buffer for a specific data type
    template<typename T>
    void prepareReceiveBuffer() {
        if (receiveBuffer) {
            delete[] receiveBuffer;
        }
        receiveBufferSize = sizeof(T);
        receiveBuffer = new uint8_t[receiveBufferSize];
        bytesRead = 0;
        headerFound = false;
    }

    // Send data with header and checksum
    template<typename T>
    void sendData(T& data) {
        // Validate that we're sending the right data type based on board type
        if (boardType == BoardType::CONTROL_BOARD && !std::is_same<T, ControlBoardData>::value) {
            Serial.println("Warning: Control board should send ControlBoardData");
        }
        else if (boardType == BoardType::POWER_BOARD && !std::is_same<T, PowerBoardData>::value) {
            Serial.println("Warning: Power board should send PowerBoardData");
        }
        
        // Calculate checksum - exclude the checksum field itself (assuming it's the last field)
        size_t checksumOffset = sizeof(T) - sizeof(uint16_t);
        *reinterpret_cast<uint16_t*>((uint8_t*)&data + checksumOffset) = 
            calculateChecksum((uint8_t*)&data, checksumOffset);
        
        // Send packet
        dataSerial.write(PACKET_HEADER, sizeof(PACKET_HEADER));
        dataSerial.write((uint8_t*)&data, sizeof(T));
        dataSerial.flush();
    }
    
    // Receive data - returns true if a complete packet was received
    template<typename T>
    bool receiveData(T& data, unsigned long timeout = 1000) {
        // Validate that we're receiving the right data type based on board type
        if (boardType == BoardType::CONTROL_BOARD && !std::is_same<T, PowerBoardData>::value) {
            Serial.println("Error: Control board should receive PowerBoardData");
            return false;
        }
        else if (boardType == BoardType::POWER_BOARD && !std::is_same<T, ControlBoardData>::value) {
            Serial.println("Error: Power board should receive ControlBoardData");
            return false;
        }
        
        if (receiveBufferSize != sizeof(T)) {
            prepareReceiveBuffer<T>();
        }
        
        unsigned long startTime = millis();
        
        while (millis() - startTime < timeout) {
            // Look for header bytes first
            if (!headerFound) {
                if (dataSerial.available() >= 2) {
                    uint8_t h1 = dataSerial.read();
                    uint8_t h2 = dataSerial.read();
                    
                    if (h1 == PACKET_HEADER[0] && h2 == PACKET_HEADER[1]) {
                        headerFound = true;
                        packetStartTime = millis();
                        bytesRead = 0;
                        Serial.println("Header found, waiting for data...");
                    }
                }
            }
            // After header is found, collect the packet data
            else {
                // Read available bytes into buffer
                while (dataSerial.available() > 0 && bytesRead < receiveBufferSize) {
                    receiveBuffer[bytesRead++] = dataSerial.read();
                }
                
                // If we have a complete packet
                if (bytesRead == receiveBufferSize) {
                    memcpy(&data, receiveBuffer, receiveBufferSize);
                    
                    // Verify checksum
                    size_t checksumOffset = receiveBufferSize - sizeof(uint16_t);
                    uint16_t receivedChecksum = *reinterpret_cast<uint16_t*>((uint8_t*)&data + checksumOffset);
                    uint16_t calculatedChecksum = calculateChecksum((uint8_t*)&data, checksumOffset);
                    
                    // Reset for next packet
                    headerFound = false;
                    bytesRead = 0;
                    
                    if (receivedChecksum == calculatedChecksum) {
                        return true;  // Valid packet received
                    } else {
                        Serial.println("Invalid checksum");
                                                return false;
                    }
                }
                
                // Timeout if packet doesn't arrive completely within the inner timeout
                if (millis() - packetStartTime > timeout) {
                    Serial.printf("Packet timeout, received %d of %zu bytes\n", bytesRead, receiveBufferSize);
                    headerFound = false;
                    bytesRead = 0;
                    return false;
                }
            }
            
            // Small delay to prevent CPU hogging
            delay(1);
        }
        
        return false; // Timeout
    }
        
    // Print control board data in a formatted way
    void printControlBoardData(const ControlBoardData& data) {
        Serial.println("\n========================== CONTROL BOARD DATA ==========================");
        Serial.println("| Field              | Value                                      |");
        Serial.println("|--------------------|--------------------------------------------|");
        Serial.printf("| Timestamp          | %u ms                                  |\n", data.timestamp);
        Serial.printf("| Pressure           | %u hPa                                 |\n", data.pressure);
        Serial.printf("| Altitude           | %u m                                   |\n", data.altitude);
        Serial.printf("| Temperature        | %u °C                                  |\n", data.temperature);
        Serial.printf("| Accel X            | %.2f m/s²                                |\n", data.accelX);
        Serial.printf("| Accel Y            | %.2f m/s²                                |\n", data.accelY);
        Serial.printf("| Accel Z            | %.2f m/s²                                |\n", data.accelZ);
        Serial.printf("| Angular Velocity X | %.2f deg/s                               |\n", data.angularVelocityX);
        Serial.printf("| Angular Velocity Y | %.2f deg/s                               |\n", data.angularVelocityY);
        Serial.printf("| Angular Velocity Z | %.2f deg/s                               |\n", data.angularVelocityZ);
        Serial.printf("| Yaw                | %.2f deg                                 |\n", data.orientationYaw);
        Serial.printf("| Pitch              | %.2f deg                                 |\n", data.orientationPitch);
        Serial.printf("| Roll               | %.2f deg                                 |\n", data.orientationRoll);
        Serial.printf("| Status Length      | %u bytes                                |\n", data.statusMsgLength);
        Serial.printf("| Status Message     | %s                     |\n", data.statusMsg);
        
        // Calculate checksum offset (the checksum field itself)
        size_t checksumOffset = sizeof(data) - sizeof(data.checksum);
        uint16_t calculatedChecksum = calculateChecksum((uint8_t*)&data, checksumOffset);
        bool checksumValid = (calculatedChecksum == data.checksum);
        
        Serial.printf("| Checksum           | 0x%04X (%s)                           |\n", 
                       data.checksum, checksumValid ? "Valid" : "Invalid");
        Serial.println("====================================================================");
    }
    
    // Print power board data in a formatted way
    void printPowerBoardData(const PowerBoardData& data) {
        Serial.println("\n========================== POWER BOARD DATA ==========================");
        Serial.println("| Field                | Value                                   |");
        Serial.println("|----------------------|------------------------------------------|");
        Serial.printf("| Transponder Timestamp | %u ms                               |\n", data.transponderTimestamp);
        Serial.printf("| Battery Voltage       | %.2f V                                  |\n", data.batteryVoltage);
        Serial.printf("| Latitude              | %.6f°                             |\n", data.latitude);
        Serial.printf("| Longitude             | %.6f°                            |\n", data.longitude);
        Serial.printf("| Abort Command         | %s                                    |\n", data.abortCommand ? "TRUE" : "FALSE");
        Serial.printf("| Status Length         | %u bytes                               |\n", data.statusMsgLength);
        Serial.printf("| Status Message        | %s                     |\n", data.statusMsg);

        // Calculate checksum offset (the checksum field itself)
        size_t checksumOffset = sizeof(data) - sizeof(data.checksum);
        uint16_t calculatedChecksum = calculateChecksum((uint8_t*)&data, checksumOffset);
        bool checksumValid = (calculatedChecksum == data.checksum);
        
        Serial.printf("| Checksum              | 0x%04X (%s)                           |\n", 
                       data.checksum, checksumValid ? "Valid" : "Invalid");
        Serial.println("===================================================================");
    }
};

#endif // SERIAL_COMMUNICATION_H
