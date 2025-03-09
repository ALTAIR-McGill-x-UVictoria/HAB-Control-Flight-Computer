#include <SPI.h>  // include the SPI library:
#include "SPI_communication.h"

void setup() {
    // Configure SPI pins
    pinMode(cs, OUTPUT);
    pinMode(mosi, OUTPUT);
    pinMode(miso, INPUT);
    pinMode(sck, OUTPUT);
    // initialize SPI:
    SPI.begin(); 
}

void loop() {
    // Assert Chip Select
    digitalWrite(cs, LOW);
    // Creating loop to send numbers in increasing order
    for (int i = 0; i < 100; i++) {
        // Send data
        SPI.transfer(uint8_t(i));
        delay(500);
    }
}