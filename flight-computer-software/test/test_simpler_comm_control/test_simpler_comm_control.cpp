#include <SPI.h>  // include the SPI library:
#include "SPI_communication.h"

void setup() {
    // Configure SP
    // SPI Communication Pins
    // const int cs = 0;     // Chip Select
    // const int miso = 1;   // Master In Slave Out
    // const int mosi = 26;  // Master Out Slave In
    // const int sck = 27;   // Serial ClockI pins
    pinMode(cs, OUTPUT);
    pinMode(mosi, OUTPUT);
    pinMode(miso, INPUT);
    pinMode(sck, OUTPUT);
    // Initialize SPI
    SPI.begin();
}

void loop() {
    // Assert Chip Select
    // Teensy 4.1 is ARM-based, thus uses little-endian byte order
    SPI.beginTransaction(SPISettings(1000000, LSBFIRST, SPI_MODE0));
    digitalWrite(cs, LOW);
    // Creating loop to send numbers in increasing order
    for (int i = 0; i < 100; i++) {
        // Send data
        SPI.transfer(uint8_t(i));
        Serial.println(i);
        delay(500);
    }
    digitalWrite(cs, HIGH);
    SPI.endTransaction();
}