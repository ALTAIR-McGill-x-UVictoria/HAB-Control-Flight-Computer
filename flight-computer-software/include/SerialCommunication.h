#include <Arduino.h>

#ifndef SERIAL_COMMUNICATION_H
#define SERIAL_COMMUNICATION_H

// Status message buffer size
#define MAX_STATUS_MSG_LENGTH 64
#define baud 115200

// Define a packet header for reliable packet detection
static const uint8_t PACKET_HEADER[] = {0xAA, 0xBB};

// Define board types
enum class BoardType
{
    CONTROL_BOARD,
    POWER_BOARD
};

#pragma pack(push, 1)
struct ControlBoardData
{
    uint32_t timestamp;                    // 0x00: Time of data capture
    float pressure;                        // 0x04: Atmospheric pressure
    float altitude;                        // 0x08: Altitude
    float temperature;                     // 0x0C: Temperature probe reading
    float accelX;                          // 0x10: X-axis linear acceleration (m/s^2)
    float accelY;                          // 0x14: Y-axis linear acceleration (m/s^2)
    float accelZ;                          // 0x18: Z-axis linear acceleration (m/s^2)
    float angularVelocityX;                // 0x1C: X-axis angular velocity (deg/s)
    float angularVelocityY;                // 0x20: Y-axis angular velocity (deg/s)
    float angularVelocityZ;                // 0x24: Z-axis angular velocity (deg/s)
    float orientationYaw;                  // 0x28: Yaw (deg)
    float orientationPitch;                // 0x2C: Pitch (deg)
    float orientationRoll;                 // 0x2E: Roll (deg)
    uint32_t statusMsgLength;              // 0x30: Length of the variable data section
    char statusMsg[MAX_STATUS_MSG_LENGTH]; // 0x34: Extra data with dynamic length
    uint16_t checksum;                     // 0x36 + Msg Length: Error detection
};

/* Data structure to be received from power board */
struct PowerBoardData
{
    uint32_t transponderTimestamp;         // 0x00: Time
    float batteryVoltage;                  // 0x04: Voltage value of battery
    float latitude;                        // 0x08: GPS latitude
    float longitude;                       // 0x0C: GPS longitude
    bool abortCommand;                     // 0x10: Command to abort, default false
    uint32_t statusMsgLength;              // 0x11: Length of the variable data section
    char statusMsg[MAX_STATUS_MSG_LENGTH]; // 0x15: Status message and heartbeat
    uint16_t checksum;                     // Checksum at the end
};
#pragma pack(pop)

class SerialCommunication
{
private:
    HardwareSerial &serialPort;    // Serial for data transmission
    BoardType boardType;           // Type of board this instance represents
    uint8_t *buffer;               // Buffer for receiving data
    size_t bufferSize;             // Current buffer size
    bool headerFound;              // Flag for packet header detection
    size_t bytesRead;              // Counter for bytes read
    unsigned long packetStartTime; // Timing for packet reception

    // Internal helper for checksum calculation
    uint16_t calculateChecksum(const uint8_t *data, size_t length)
    {
        uint16_t sum = 0;
        for (size_t i = 0; i < length; i++)
        {
            sum += data[i];
        }
        return sum;
    }

    // Allocate the receive buffer if needed
    void ensureBufferSize(size_t size)
    {
        if (buffer && bufferSize == size)
            return;

        if (buffer)
            delete[] buffer;
        bufferSize = size;
        buffer = new uint8_t[bufferSize];
        resetReceiveState();
    }

    // Reset packet reception state
    void resetReceiveState()
    {
        headerFound = false;
        bytesRead = 0;
    }

public:
    // Constructor
    SerialCommunication(HardwareSerial &serialPort, BoardType boardType)
        : serialPort(serialPort), boardType(boardType), buffer(nullptr), bufferSize(0),
          headerFound(false), bytesRead(0), packetStartTime(0)
    {
    }

    // Destructor
    ~SerialCommunication()
    {
        if (buffer)
            delete[] buffer;
    }

    // Initialize communication with Serial1 initialization directly
    void begin()
    {
        serialPort.begin(baud);
        Serial.println("Serial Communication initialized");

        if (boardType == BoardType::CONTROL_BOARD)
        {
            Serial.println("Configured as Control Board");
            ensureBufferSize(sizeof(PowerBoardData));
        }
        else
        {
            Serial.println("Configured as Power Board");
            ensureBufferSize(sizeof(ControlBoardData));
        }

        // Add debug info about serialPort configuration
        Serial.printf("Serial1 configured at %d baud\n", baud);

        // More thorough clear buffers operation
        clearBuffers();
        delay(100); // Give some time for things to stabilize
    }

    // Discard any pending data in buffers
    void clearBuffers()
    {
        // Discard any pending data
        while (serialPort.available())
            serialPort.read();

        // Reset packet reception state
        resetReceiveState();

        // Flush any outgoing data
        serialPort.flush();

        // Brief delay to let things settle
        delay(10);
    }

    // Send ControlBoardData (should be called from Control Board)
    bool sendData(ControlBoardData &data)
    {
        // Validate board type
        if (boardType != BoardType::CONTROL_BOARD)
            return false;

        // Calculate checksum
        size_t checksumOffset = sizeof(data) - sizeof(data.checksum);
        data.checksum = calculateChecksum((uint8_t *)&data, checksumOffset);

        // Send packet
        serialPort.write(PACKET_HEADER, sizeof(PACKET_HEADER));
        serialPort.write((uint8_t *)&data, sizeof(data));
        serialPort.flush();

        return true;
    }

    // Send PowerBoardData (should be called from Power Board)
    bool sendData(PowerBoardData &data)
    {
        // Validate board type
        if (boardType != BoardType::POWER_BOARD)
            return false;

        // Calculate checksum
        size_t checksumOffset = sizeof(data) - sizeof(data.checksum);
        data.checksum = calculateChecksum((uint8_t *)&data, checksumOffset);

        // Send packet
        serialPort.write(PACKET_HEADER, sizeof(PACKET_HEADER));
        serialPort.write((uint8_t *)&data, sizeof(data));
        serialPort.flush();

        return true;
    }

    // Receive PowerBoardData (should be called from Control Board)
    bool receiveData(PowerBoardData &data, unsigned long timeout = 1000)
    {
        if (boardType != BoardType::CONTROL_BOARD)
        {
            Serial.println("Error: Only control board should receive PowerBoardData");
            return false;
        }

        ensureBufferSize(sizeof(PowerBoardData));
        return receivePacket((uint8_t *)&data, timeout);
    }

    // Receive ControlBoardData (should be called from Power Board)
    bool receiveData(ControlBoardData &data, unsigned long timeout = 1000)
    {
        if (boardType != BoardType::POWER_BOARD)
        {
            Serial.println("Error: Only power board should receive ControlBoardData");
            return false;
        }

        ensureBufferSize(sizeof(ControlBoardData));
        return receivePacket((uint8_t *)&data, timeout);
    }

    // Core packet reception logic
    bool receivePacket(uint8_t *dataPtr, unsigned long timeout)
    {
        unsigned long startTime = millis();

        while (millis() - startTime < timeout)
        {
            // Look for packet header
            if (!headerFound)
            {
                if (serialPort.available() >= 2)
                {
                    uint8_t h1 = serialPort.read();
                    uint8_t h2 = serialPort.read();

                    if (h1 == PACKET_HEADER[0] && h2 == PACKET_HEADER[1])
                    {
                        headerFound = true;
                        packetStartTime = millis();
                        bytesRead = 0;
                    }
                }
            }
            // Collect packet data after header
            else
            {
                // Read available bytes into buffer
                while (serialPort.available() > 0 && bytesRead < bufferSize)
                {
                    buffer[bytesRead++] = serialPort.read();
                }

                // If we have a complete packet
                if (bytesRead == bufferSize)
                {
                    // Copy buffer to output data
                    memcpy(dataPtr, buffer, bufferSize);

                    // Verify checksum
                    size_t checksumOffset = bufferSize - sizeof(uint16_t);
                    uint16_t receivedChecksum = *reinterpret_cast<uint16_t *>(dataPtr + checksumOffset);
                    uint16_t calculatedChecksum = calculateChecksum(dataPtr, checksumOffset);

                    // Reset for next packet
                    resetReceiveState();

                    // Return true if checksum is valid
                    if (receivedChecksum == calculatedChecksum)
                    {
                        return true;
                    }
                    else
                    {
                        Serial.printf("Invalid checksum: expected 0x%04X, got 0x%04X\n",
                                      calculatedChecksum, receivedChecksum);
                        return false;
                    }
                }

                // Packet timeout check
                if (millis() - packetStartTime > timeout)
                {
                    Serial.printf("Packet timeout: %d/%zu bytes\n", bytesRead, bufferSize);
                    resetReceiveState();
                    return false;
                }
            }
        }

        return false; // Overall timeout
    }

    // Print control board data in a formatted way
    void printControlBoardData(const ControlBoardData &data)
    {
        Serial.println("\n======================= CONTROL BOARD DATA =======================");
        Serial.println("| Field              | Value                                      ");
        Serial.println("|--------------------|--------------------------------------------");
        Serial.printf("| Timestamp          | %u ms\n", data.timestamp);
        Serial.printf("| Pressure           | %u hPa\n", data.pressure);
        Serial.printf("| Altitude           | %u m\n", data.altitude);
        Serial.printf("| Temperature        | %u °C\n", data.temperature);
        Serial.printf("| Accel X            | %.2f m/s²\n", data.accelX);
        Serial.printf("| Accel Y            | %.2f m/s²\n", data.accelY);
        Serial.printf("| Accel Z            | %.2f m/s²\n", data.accelZ);
        Serial.printf("| Angular Velocity X | %.2f deg/s\n", data.angularVelocityX);
        Serial.printf("| Angular Velocity Y | %.2f deg/s\n", data.angularVelocityY);
        Serial.printf("| Angular Velocity Z | %.2f deg/s\n", data.angularVelocityZ);
        Serial.printf("| Yaw                | %.2f deg\n", data.orientationYaw);
        Serial.printf("| Pitch              | %.2f deg\n", data.orientationPitch);
        Serial.printf("| Roll               | %.2f deg\n", data.orientationRoll);
        Serial.printf("| Status Length      | %u bytes\n", data.statusMsgLength);
        Serial.printf("| Status Message     | %s\n", data.statusMsg);

        // Calculate checksum offset (the checksum field itself)
        size_t checksumOffset = sizeof(data) - sizeof(data.checksum);
        uint16_t calculatedChecksum = calculateChecksum((uint8_t *)&data, checksumOffset);
        bool checksumValid = (calculatedChecksum == data.checksum);

        Serial.printf("| Checksum           | 0x%04X (%s)\n",
                      data.checksum, checksumValid ? "Valid" : "Invalid");
        Serial.println("==================================================================");
    }

    // Print power board data in a formatted way
    void printPowerBoardData(const PowerBoardData &data)
    {
        Serial.println("\n======================== POWER BOARD DATA =======================");
        Serial.println("| Field                | Value                                    ");
        Serial.println("|----------------------|------------------------------------------");
        Serial.printf("| Transponder Timestamp | %u ms\n", data.transponderTimestamp);
        Serial.printf("| Battery Voltage       | %.2f V\n", data.batteryVoltage);
        Serial.printf("| Latitude              | %.6f°\n", data.latitude);
        Serial.printf("| Longitude             | %.6f°\n", data.longitude);
        Serial.printf("| Abort Command         | %s\n", data.abortCommand ? "TRUE" : "FALSE");
        Serial.printf("| Status Length         | %u bytes\n", data.statusMsgLength);
        Serial.printf("| Status Message        | %s\n", data.statusMsg);

        // Calculate checksum offset (the checksum field itself)
        size_t checksumOffset = sizeof(data) - sizeof(data.checksum);
        uint16_t calculatedChecksum = calculateChecksum((uint8_t *)&data, checksumOffset);
        bool checksumValid = (calculatedChecksum == data.checksum);

        Serial.printf("| Checksum              | 0x%04X (%s)\n",
                      data.checksum, checksumValid ? "Valid" : "Invalid");
        Serial.println("==================================================================");
    }
};

#endif // SERIAL_COMMUNICATION_H
