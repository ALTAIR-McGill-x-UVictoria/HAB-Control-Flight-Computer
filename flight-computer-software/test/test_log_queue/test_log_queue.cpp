/*
 *  This is a test program for LogQueue, with one thread producing data to enqueue and
 *  another thread consuming that data by dequeueing it and printing it.
 */

#include <Arduino.h>
#include "../../include/LogQueue.h"

// Global queue instance
LogQueue logQueue;

// Producer thread function
void producerThread()
{
    int counter = 0;
    while (true)
    {
        char message[MAX_MESSAGE_LENGTH];
        snprintf(message, MAX_MESSAGE_LENGTH, "Log message #%d", counter++);

        if (logQueue.enqueue(message))
        {
            Serial.print("Enqueued: ");
            Serial.println(message);
        }
        else
        {
            Serial.println("**Queue full, could not add message**");
        }

        threads.delay(500);
    }
}

// Consumer thread function
void consumerThread()
{
    char message[MAX_MESSAGE_LENGTH];
    while (true)
    {
        if (logQueue.dequeue(message, MAX_MESSAGE_LENGTH))
        {
            Serial.print("Dequeued: ");
            Serial.println(message);
        }

        threads.delay(700);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }
    Serial.println("Log Queue Test Starting...");

    threads.addThread(producerThread);
    threads.addThread(consumerThread);
}

void loop()
{
    // Nothing to do here
}
