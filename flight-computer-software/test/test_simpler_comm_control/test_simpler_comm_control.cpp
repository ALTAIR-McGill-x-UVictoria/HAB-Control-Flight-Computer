#include <Arduino.h>
// Serial communication between two Teensy 4.1 boards
// Pins to use for serial:
// Teensy 1 TX1 → Teensy 2 RX1
// Teensy 1 RX1 → Teensy 2 TX1
// Teensy 1 GND → Teensy 2 GND

#define RxD Serial1
#define TxD Serial1
#define baud 230400

// Define a packet header for reliable packet detection
static const uint8_t PACKET_HEADER[] = {0xAA, 0xBB};

// Status message buffer size
#define MAX_STATUS_MSG_LENGTH 64

#pragma pack(push,1)
/* Data structure to be sent from controller */
struct ControllerData {
    uint32_t timestamp;        // 0x00: Time of data capture
    uint32_t pressure;         // 0x04: Atmospheric pressure
    uint32_t temperature;      // 0x08: Temperature probe reading
    float accelX;              // 0x0C: X-axis acceleration
    float accelY;              // 0x10: Y-axis acceleration
    float accelZ;              // 0x14: Z-axis acceleration
    float velocityX;           // 0x18: X-axis velocity
    float velocityY;           // 0x1C: Y-axis velocity
    float velocityZ;           // 0x20: Z-axis velocity
    uint16_t statusMsgLength;  // 0x24: Length of the variable data section
    char statusMsg[MAX_STATUS_MSG_LENGTH]; // 0x26: Extra data with dynamic length
    uint16_t checksum;         // Checksum at the end
};

/* Data structure to be received from remote device */
struct DeviceData {
    float batteryVoltage;      // 0x00: Voltage value of battery
    float latitude;            // 0x04: GPS latitude
    float longitude;           // 0x08: GPS longitude
};

ControllerData txData;   // Data to send
DeviceData rxData;       // Data to receive
#pragma pack(pop)

// Function to calculate simple checksum
uint16_t calculateChecksum(uint8_t* data, size_t length) {
    uint16_t sum = 0;
    for (size_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum;
}

void ClearSerialBuffers() {
    TxD.flush();
    RxD.clear();
    delay(100);
    TxD.flush();
    RxD.clear();
}

void setup() {
    // Initialize serial interface
    Serial.begin(9600);
    RxD.begin(baud);
    TxD.begin(baud);

    pinMode(LED_BUILTIN, OUTPUT);
    
    // Wait until the serial interfaces are active
    while (!Serial && millis() < 5000);  // Wait for Serial (USB) to become active
    Serial.println("Serial ports active:");
    Serial.println("     Sending on Serial1 Tx, Receiving on Serial1 Rx");
    Serial.println("Controller Board initialized");
    Serial.printf("Size of ControllerData: %d bytes\n", sizeof(ControllerData));

    // Initialize data with default values
    txData.timestamp = 0;
    txData.statusMsgLength = 0;
    strcpy(txData.statusMsg, "System initialized");
    txData.statusMsgLength = strlen(txData.statusMsg);
    
    ClearSerialBuffers();
    delay(1000);    // Allow sync time between Teensys
}

// Timer for periodic sending
elapsedMillis sendTimer;
#define SEND_INTERVAL 5000  // Send data every 5 seconds

// Timeout for receiving response
elapsedMillis receiveTimeout;
#define RECEIVE_TIMEOUT 2000

void loop() {
    // Send data every 5 seconds
    if (sendTimer >= SEND_INTERVAL) {
        sendTimer = 0;
        
        // Update data values (in a real application, you would get these from sensors)
        txData.timestamp = millis();
        txData.pressure = random(950, 1050);      // Simulated pressure data (in hPa)
        txData.temperature = random(15, 35);      // Simulated temperature (in Celsius)
        txData.accelX = random(-100, 100) / 10.0f;  // Simulated acceleration
        txData.accelY = random(-100, 100) / 10.0f;
        txData.accelZ = random(-100, 100) / 10.0f;
        txData.velocityX = random(-50, 50) / 10.0f; // Simulated velocity
        txData.velocityY = random(-50, 50) / 10.0f;
        txData.velocityZ = random(-50, 50) / 10.0f;
        
        // Update status message occasionally
        if (random(5) == 0) {
            sprintf(txData.statusMsg, "Status update at %u ms", txData.timestamp);
            txData.statusMsgLength = strlen(txData.statusMsg);
        }
        
        // Calculate checksum - exclude the checksum field itself
        size_t checksumOffset = sizeof(txData) - sizeof(txData.checksum);
        txData.checksum = calculateChecksum((uint8_t*)&txData, checksumOffset);
        
        // Send the data packet with header
        digitalWriteFast(LED_BUILTIN, HIGH);
        TxD.write(PACKET_HEADER, sizeof(PACKET_HEADER));  // Send header first
        TxD.write((uint8_t*)&txData, sizeof(txData));     // Then send data
        TxD.flush();
        digitalWriteFast(LED_BUILTIN, LOW);
        
        // Print sent data in a nice format
        Serial.println("\n========================== SENT DATA ==========================");
        Serial.println("| Field          | Value                                      |");
        Serial.println("|----------------|-------------------------------------------|");
        Serial.printf("| Timestamp      | %u ms                                  |\n", txData.timestamp);
        Serial.printf("| Pressure       | %u hPa                                 |\n", txData.pressure);
        Serial.printf("| Temperature    | %u °C                                  |\n", txData.temperature);
        Serial.printf("| Acceleration X | %.2f m/s²                                |\n", txData.accelX);
        Serial.printf("| Acceleration Y | %.2f m/s²                                |\n", txData.accelY);
        Serial.printf("| Acceleration Z | %.2f m/s²                                |\n", txData.accelZ);
        Serial.printf("| Velocity X     | %.2f m/s                                 |\n", txData.velocityX);
        Serial.printf("| Velocity Y     | %.2f m/s                                 |\n", txData.velocityY);
        Serial.printf("| Velocity Z     | %.2f m/s                                 |\n", txData.velocityZ);
        Serial.printf("| Status Length  | %u bytes                                |\n", txData.statusMsgLength);
        Serial.printf("| Status Message | %s                     |\n", txData.statusMsg);
        Serial.printf("| Checksum       | 0x%04X                                   |\n", txData.checksum);
        Serial.println("==============================================================");
        
        // Wait for response
        receiveTimeout = 0;
        bool responseReceived = false;
        bool headerFound = false;
        uint8_t responseBuffer[sizeof(DeviceData)];
        size_t bytesRead = 0;
        
        while (receiveTimeout < RECEIVE_TIMEOUT) {
            // First look for the header
            if (!headerFound && RxD.available() >= 2) {
                uint8_t h1 = RxD.read();
                uint8_t h2 = RxD.read();
                
                if (h1 == PACKET_HEADER[0] && h2 == PACKET_HEADER[1]) {
                    headerFound = true;
                    Serial.println("Response header found");
                }
            }
            
            // After header is found, read the response data
            if (headerFound) {
                while (RxD.available() > 0 && bytesRead < sizeof(DeviceData)) {
                    responseBuffer[bytesRead++] = RxD.read();
                }
                
                if (bytesRead == sizeof(DeviceData)) {
                    memcpy(&rxData, responseBuffer, sizeof(DeviceData));
                    responseReceived = true;
                    break;
                }
            }
            
            delay(10);  // Small delay to prevent CPU hogging
        }
        
        if (responseReceived) {
            // Print received data
            Serial.println("\n====================== RECEIVED DATA ======================");
            Serial.println("| Field           | Value                                |");
            Serial.println("|-----------------|--------------------------------------|");
            Serial.printf("| Battery Voltage | %.2f V                               |\n", rxData.batteryVoltage);
            Serial.printf("| Latitude        | %.6f°                          |\n", rxData.latitude);
            Serial.printf("| Longitude       | %.6f°                         |\n", rxData.longitude);
            Serial.println("==============================================================");
        } else {
            Serial.println("Timeout waiting for response data");
            if (headerFound) {
                Serial.printf("Header was found but only received %d of %d bytes\n", 
                             bytesRead, sizeof(DeviceData));
            } else {
                Serial.println("Response header not found");
            }
            ClearSerialBuffers();
        }
    }
}