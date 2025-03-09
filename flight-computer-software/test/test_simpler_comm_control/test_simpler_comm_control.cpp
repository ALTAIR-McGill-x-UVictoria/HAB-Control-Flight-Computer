#include <SPI.h>  // include the SPI library:
#include "SPI_communication.h"

void setup() {
    // Configure SPI pins
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
}