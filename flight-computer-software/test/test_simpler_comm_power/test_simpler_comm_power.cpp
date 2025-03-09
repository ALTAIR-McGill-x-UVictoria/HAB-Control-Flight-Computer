#include <Arduino.h>

// Source for test code: https://forum.pjrc.com/index.php?threads/serial-communication-between-two-teensy-4-1.71093/
/*****************************
  Receive and Echo Back Code
******************************/
// Teensy 1 TX1 → Teensy 2 RX1
// Teensy 1 RX1 → Teensy 2 TX1
// Teensy 1 GND → Teensy 2 GND

#define RxD Serial7
#define TxD Serial6
#pragma pack(push,1)

/* Data Type to be sent received */
struct dataType {
    uint32_t dataIdx;
    float vPos, hPos, vCurrent, hCurrent;
};

dataType rx;   // Variables to hold the data for Send/Receive
#pragma pack(pop)

#define baud 230400  //115200

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
    Serial.println("     Receiving on Serial7 Rx, Sending on Serial 6 Tx");

    ClearSerialBuffers();

}
void loop() {

    if (RxD.available() >= sizeof(rx)){
 
    /*        ReceivePacket */
        digitalWriteFast(LED_BUILTIN, HIGH);
        rx.dataIdx = 0;
        rx.hCurrent = 0;
        rx.hPos = 0;
        rx.vCurrent = 0;
        rx.vPos = 0;
        RxD.readBytes((uint8_t*)&rx, sizeof(rx));   // Get the data from Serial Buffer
        /*  SendPacket */
        TxD.write((uint8_t*)&rx, sizeof(rx));    //Send the data
        TxD.flush();
        digitalWriteFast(LED_BUILTIN, LOW);

    }
}
