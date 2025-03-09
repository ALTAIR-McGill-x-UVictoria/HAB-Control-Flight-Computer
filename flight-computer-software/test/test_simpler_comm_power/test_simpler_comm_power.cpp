#include <Arduino.h>
// Serial communication between two Teensy 4.1 boards
// Pins to use for serial:
// Teensy 1 TX1 → Teensy 2 RX1
// Teensy 1 RX1 → Teensy 2 TX1
// Teensy 1 GND → Teensy 2 GND

#define RxD Serial1
#define TxD Serial1
#define baud 230400

// Status message buffer size
#define MAX_STATUS_MSG_LENGTH 64

#pragma pack(push,1)
/* Data structure to be received from controller */
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

/* Data structure to be sent back to controller */
struct DeviceData {
    float batteryVoltage;      // 0x00: Voltage value of battery
    float latitude;            // 0x04: GPS latitude
    float longitude;           // 0x08: GPS longitude
};

ControllerData rxData;   // Data to receive
DeviceData txData;       // Data to send back
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
    // Initialize serial interface
    Serial.begin(9600);
    RxD.begin(baud);
    TxD.begin(baud);

    pinMode(LED_BUILTIN, OUTPUT);
    
    // Wait until the serial interfaces are active
    while (!Serial && millis() < 5000);  // Wait for Serial (USB) to become active
    Serial.println("Serial ports active:");
    Serial.println("     Receiving on Serial1 Rx, Sending on Serial1 Tx");
    Serial.println("Receiver Board initialized");

    ClearSerialBuffers();
}

void loop() {
    // Check if we have enough data to receive a complete packet
    if (RxD.available() >= sizeof(rxData)) {
        // Turn on LED to indicate receiving
        digitalWriteFast(LED_BUILTIN, HIGH);
        
        // Read the data packet
        RxD.readBytes((uint8_t*)&rxData, sizeof(rxData));
        
        // Verify checksum
        size_t checksumOffset = sizeof(rxData) - sizeof(rxData.checksum);
        uint16_t calculatedChecksum = calculateChecksum((uint8_t*)&rxData, checksumOffset);
        bool checksumValid = (calculatedChecksum == rxData.checksum);
        
        // Print received data in a nice format
        Serial.println("\n======================== RECEIVED DATA ========================");
        Serial.println("| Field          | Value                                      |");
        Serial.println("|----------------|-------------------------------------------|");
        Serial.printf("| Timestamp      | %u ms                                  |\n", rxData.timestamp);
        Serial.printf("| Pressure       | %u hPa                                 |\n", rxData.pressure);
        Serial.printf("| Temperature    | %u °C                                  |\n", rxData.temperature);
        Serial.printf("| Acceleration X | %.2f m/s²                                |\n", rxData.accelX);
        Serial.printf("| Acceleration Y | %.2f m/s²                                |\n", rxData.accelY);
        Serial.printf("| Acceleration Z | %.2f m/s²                                |\n", rxData.accelZ);
        Serial.printf("| Velocity X     | %.2f m/s                                 |\n", rxData.velocityX);
        Serial.printf("| Velocity Y     | %.2f m/s                                 |\n", rxData.velocityY);
        Serial.printf("| Velocity Z     | %.2f m/s                                 |\n", rxData.velocityZ);
        Serial.printf("| Status Length  | %u bytes                                |\n", rxData.statusMsgLength);
        Serial.printf("| Status Message | %s                     |\n", rxData.statusMsg);
        Serial.printf("| Checksum       | 0x%04X (%s)                           |\n", 
                     rxData.checksum, checksumValid ? "Valid" : "Invalid");
        Serial.println("==============================================================");
        
        // Prepare response data
        txData.batteryVoltage = getBatteryVoltage();
        txData.latitude = getLatitude();
        txData.longitude = getLongitude();
        
        // Send the response
        TxD.write((uint8_t*)&txData, sizeof(txData));
        TxD.flush();
        
        // Print sent data
        Serial.println("\n========================== SENT DATA ==========================");
        Serial.println("| Field           | Value                                   |");
        Serial.println("|-----------------|------------------------------------------|");
        Serial.printf("| Battery Voltage | %.2f V                                  |\n", txData.batteryVoltage);
        Serial.printf("| Latitude        | %.6f°                             |\n", txData.latitude);
        Serial.printf("| Longitude       | %.6f°                            |\n", txData.longitude);
        Serial.println("==============================================================\n");
        
        // Turn off LED
        digitalWriteFast(LED_BUILTIN, LOW);
    }
}