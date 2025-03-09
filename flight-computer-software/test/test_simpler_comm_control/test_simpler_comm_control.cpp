#include <Arduino.h>
// Serial test code 
// Pins to use for serial:
// Teensy 1 TX1 → Teensy 2 RX1
// Teensy 1 RX1 → Teensy 2 TX1
// Teensy 1 GND → Teensy 2 GND

// Source for test code: https://forum.pjrc.com/index.php?threads/serial-communication-between-two-teensy-4-1.71093/
/************************
     Send and Receive Code
************************/

#define RxD Serial7
#define TxD Serial6

#define baud 230400 //115200

#pragma pack(push,1)

/* Data Type to be sent received */
struct dataType {
    uint32_t dataIdx;
    float vPos, hPos, vCurrent, hCurrent;
};

dataType tx, rx;   // Variables to hold the data for Send/Receive
#pragma pack(pop)

//placeholder for information getting sent
float ivpos = 0.10, ihpos = 0.20, ivcurrent = 0.0, ihcurrent = 0.0;
float avpos = 0.50, ahpos = 0.70, avcurrent = 0.29, ahcurrent = 0.35;

void ClearSerialBuffers() {
    TxD.flush();
    RxD.clear();
    delay(1000);
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
    Serial.println("     Sending on Serial6 Tx, Receiving on Serial 7 Rx");

    tx.dataIdx = 0;
    ClearSerialBuffers();
    delay(1000);    // Just to allow synch time between T4.1s
}

elapsedMillis timeout;
#define waitTime 5000

void loop() {

    tx.vPos = avpos;
    tx.hPos = ahpos;
    tx.vCurrent = avcurrent;
    tx.hCurrent = ahcurrent;
    tx.dataIdx++;

    /*  SendPacket */
    digitalWriteFast(LED_BUILTIN, HIGH);
    TxD.write((uint8_t*)&tx, sizeof(tx));    //Send the data
    TxD.flush();

    digitalWriteFast(LED_BUILTIN, LOW);

    Serial.print("Sent packet ---->  ");
    Serial.print(tx.dataIdx);
    Serial.print(" , ");
    Serial.print(tx.vPos);
    Serial.print(" , ");
    Serial.print(tx.hPos);
    Serial.print(" , ");
    Serial.print(tx.vCurrent);
    Serial.print(" , ");
    Serial.print(tx.hCurrent);
    Serial.println();

    timeout = 0;

    while ((RxD.available() < sizeof(rx)) && (timeout <= waitTime)) {}       // Wait for complete data received
    /*        ReceivePacket */
    if (timeout >= waitTime) {
        Serial.println("Timeout waiting for Rx Data");
        ClearSerialBuffers();
    }
    else {
        rx.dataIdx = 0;
        rx.hCurrent = 0;
        rx.hPos = 0;
        rx.vCurrent = 0;
        rx.vPos = 0;

        RxD.readBytes((uint8_t*)&rx, sizeof(rx));   // Get the data from Serial Buffer
        Serial.print("Read packet ---->  ");
        Serial.print(rx.dataIdx);
        Serial.print(" , ");
        Serial.print(rx.vPos);
        Serial.print(" , ");
        Serial.print(rx.hPos);
        Serial.print(" , ");
        Serial.print(rx.vCurrent);
        Serial.print(" , ");
        Serial.print(rx.hCurrent);
        Serial.println();

        if (rx.dataIdx != tx.dataIdx) {
            Serial.println("Lost Packet");
            ClearSerialBuffers();
        }

    }
}