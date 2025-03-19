/*
 *  This is a test program for LogQueue, with one thread producing data to enqueue and
 *  another thread consuming that data by dequeueing, printing to the serial monitor,
 *  and writing to the SD card.
 */

#include <Arduino.h>
#include <TeensyThreads.h>
#include <SD.h>
#include <SPI.h>
#include "../../include/LogQueue.h"

// Define maximum message length for string messages
#define MAX_MESSAGE_LENGTH 128

// String message type
struct StringMessage {
    char text[MAX_MESSAGE_LENGTH];
};

// Example sensor data struct to demonstrate the template capabilities
struct SensorData {
    float temperature;
    float humidity;
    float pressure;
    uint32_t timestamp;
};

// Global queue instances
LogQueue<StringMessage> logQueue;
LogQueue<SensorData> sensorQueue;

void producerThread()
{
    int counter = 0;
    while (true)
    {
        // Produce string message
        StringMessage message;
        snprintf(message.text, MAX_MESSAGE_LENGTH, "Log message #%d", counter++);
        if (logQueue.enqueue(message))
        {
            Serial.print("Enqueued: ");
            Serial.println(message.text);
        }
        else
        {
            Serial.println("**String queue full**");
        }        
        // Produce sensor data
        SensorData data = {
            random(0, 400) / 10.0f,
            random(0, 1000) / 10.0f,
            random(9800, 10200) / 10.0f,
            millis()
        };
        if (sensorQueue.enqueue(data)) 
        {
            Serial.println("Enqueued sensor data");
        } 
        else 
        {
            Serial.println("**Sensor queue full**");
        }
        threads.delay(1000);
    }
}

void consumerThread()
{
    while (true)
    {
        // Check for string messages
        StringMessage textMsg;
        if (logQueue.dequeue(textMsg))
        {
            // Print to serial monitor
            Serial.print("Dequeued text: ");
            Serial.println(textMsg.text);
            // Write to SD card
            File dataFile = SD.open("test_log_queue.txt", FILE_WRITE);
            if (dataFile) 
            {
                dataFile.println(textMsg.text);
                dataFile.close();
                Serial.println("String written to SD card.");
            } 
            else 
            {
                Serial.println("Failed to open file for writing.");
            }
        }
        else
        {
            Serial.println("**String queue empty**");
        }
        // Check for sensor data
        SensorData sensorData;
        if (sensorQueue.dequeue(sensorData)) 
        {
            // Print to serial monitor
            Serial.print("Dequeued sensor at ");
            Serial.print(sensorData.timestamp);
            Serial.print("ms: Temp=");
            Serial.print(sensorData.temperature);
            Serial.print("°C, Humidity=");
            Serial.print(sensorData.humidity);
            Serial.print("%, Pressure=");
            Serial.println(sensorData.pressure);
            // Write to SD card
            File dataFile = SD.open("test_log_queue.txt", FILE_WRITE);
            if (dataFile) 
            {
                dataFile.print("Sensor data at ");
                dataFile.print(sensorData.timestamp);
                dataFile.print("ms: Temp=");
                dataFile.print(sensorData.temperature);
                dataFile.print("°C, Humidity=");
                dataFile.print(sensorData.humidity);
                dataFile.print("%, Pressure=");
                dataFile.println(sensorData.pressure);
                dataFile.close();
                Serial.println("Sensor data written to SD card.");
            } 
            else 
            {
                Serial.println("Failed to open file for writing.");
            }
        }
        else
        {
            Serial.println("**Sensor queue empty**");
        }
        threads.delay(1200);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }
    delay(1000); // Give time for the serial monitor to open
    Serial.println("Initializing SD card...");
    while (!SD.begin(BUILTIN_SDCARD))
    {
        delay(100); // Wait for SD card to initialize
        Serial.println("SD card initialization failed!");
    }

    Serial.println("Log Queue Test Starting...");

    threads.addThread(producerThread);
    threads.addThread(consumerThread);
}

void loop()
{
    // Nothing to do here
}
