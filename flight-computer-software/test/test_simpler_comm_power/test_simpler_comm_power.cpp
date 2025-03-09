#include <SPI.h>
#include <Arduino.h>
#include "SPI_MSTransfer_T4.h"

// Forced to SPI1
SPI_MSTransfer_T4<&SPI1, 0x1234> mySPI;

// SPI Communication Pins
const int cs = 0;     // Chip Select
const int miso = 1;   // Master In Slave Out
const int mosi = 26;  // Master Out Slave In
const int sck = 27;   // Serial Clock

void myCB(uint16_t *buffer, uint16_t length, AsyncMST info) {
    for ( int i = 0; i < length; i++ ) {
        Serial.print(buffer[i], HEX); Serial.print(" ");
    }
    Serial.print(" --> Length: "); Serial.print(length);
    Serial.print(" --> PacketID: "); Serial.println(info.packetID, HEX);
}

void setup() {
    // Configure SPI pins for slave mode
    pinMode(cs, INPUT);
    pinMode(mosi, INPUT);
    pinMode(miso, OUTPUT);
    pinMode(sck, INPUT);
    // Set pins specific to SPI1
    // SPI1.setCS(cs);
    // SPI1.setMISO(miso);
    // SPI1.setMOSI(mosi);
    // SPI1.setSCK(sck);
    mySPI.begin();
    mySPI.onTransfer(myCB);
}

void loop() {
    mySPI.events();
    static uint32_t t = millis();
    if ( millis() - t > 1000 ) {
        Serial.print("millis: "); Serial.println(millis());
        t = millis();
    }
}