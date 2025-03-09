#include <SPI.h>  // include the SPI library:
#include "SPI_communication.h"

void setup() {
    // Configure SPI pins for slave mode
    pinMode(cs, INPUT);
    pinMode(mosi, INPUT);
    pinMode(miso, OUTPUT);
    pinMode(sck, INPUT);
    // Initialize SPI in slave mode
    SPI.begin();
}

void loop() {
    // Assert Chip Select
    if (digitalRead(cs) == LOW) {
        // Reading integer values from master
        uint8_t i = SPI.transfer(0x00);
        // Print the integer values
        Serial.println(i);
        delay(500);
    }
}