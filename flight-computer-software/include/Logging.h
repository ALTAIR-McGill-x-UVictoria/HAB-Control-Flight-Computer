#include <Arduino.h>
#include "LogQueue.h"
#include "Sensors.h"
#include "SerialCommunication.h"

#ifndef LOGGING_H
#define LOGGING_H

#define MAX_MESSAGE_LENGTH 128 // Maximum message length for string messages

// String message type for sd log queue
struct StringMessage
{
    char text[MAX_MESSAGE_LENGTH];
};

extern LogQueue<ControlBoardData> telemetryTransmitQueue; // Changed to extern declaration
extern LogQueue<StringMessage> sdCardLoggingQueue;        // Changed to extern declaration

extern SerialCommunication serialComm;

// Function declarations instead of definitions
void emitLog(const char *format, ...);
void emitTelemetry(Sensors &sensors, const char *format, ...);
void processLogQueue();
void processTelemetryQueue();

#endif // LOGGING_H