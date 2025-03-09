#include <SPI.h>
#include <Arduino.h>
#include "SPI_MSTransfer_MASTER.h"

// SPI Communication Pins
const int cs = 0;     // Chip Select
const int miso = 1;   // Master In Slave Out
const int mosi = 26;  // Master Out Slave In
const int sck = 27;   // Serial Clock
// Master          Slave
// CS0 (Pin 0)	    CS0 (Pin 0)
// MOSI0 (Pin 1)	MISO0 (Pin 26)
// MISO0 (Pin 26)	MOSI0 (Pin 1)
// SCK0 (Pin 27)	SCK0 (Pin 27)
SPI_MSTransfer_MASTER<&SPI1, cs, 0x1234> mySPI1234;

void setup() {
    pinMode(cs, OUTPUT);
    pinMode(mosi, OUTPUT);
    pinMode(miso, INPUT);
    pinMode(sck, OUTPUT);
    // Initialize SPI
    // SPI1.setCS(cs);
    // SPI1.setMISO(miso);
    // SPI1.setMOSI(mosi);
    // SPI1.setSCK(sck);
    // Further initialization
    Serial.begin(115200);
    // SPI.begin();
    SPI1.begin();
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
    mySPI1234.begin();
}

void loop() {
static uint32_t t = millis();
  if ( millis() - t > 1000 ) {
    Serial.println(millis());

    uint16_t buf2[5] = { 0xBEEF, 0xF7, 0xF8, 0xF9, 0xDEAD };
    mySPI1234.transfer16(buf2, 5, random(0x1000, 0x8000));

    mySPI1234.detectSlaves();

    t = millis();
  }
}