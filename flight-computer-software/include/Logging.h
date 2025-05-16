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

DMAMEM LogQueue<ControlBoardData> telemetryTransmitQueue; // Changed to proper type
DMAMEM LogQueue<StringMessage> sdCardLoggingQueue;

extern SerialCommunication serialComm;

void emitLog(const char *format, ...)
{
    char logMessage[MAX_MESSAGE_LENGTH];
    uint32_t currentTime = millis();

    // Format the prefix with timestamp
    int prefixLen = snprintf(logMessage, MAX_MESSAGE_LENGTH, "[%lu ms] ", currentTime);

    // Format the rest of the message with variable arguments
    va_list args;
    va_start(args, format);
    vsnprintf(logMessage + prefixLen, MAX_MESSAGE_LENGTH - prefixLen, format, args);
    va_end(args);

    // Send to log queue
    StringMessage msg;
    strncpy(msg.text, logMessage, MAX_MESSAGE_LENGTH - 1);
    msg.text[MAX_MESSAGE_LENGTH - 1] = '\0';
    sdCardLoggingQueue.enqueue(msg);
}

void emitTelemetry(Sensors &sensors, const char *format, ...)
{
    // Format the message string with variable arguments
    char message[MAX_MESSAGE_LENGTH];
    va_list args;
    va_start(args, format);
    vsnprintf(message, MAX_MESSAGE_LENGTH - 1, format, args);
    va_end(args);
    message[MAX_MESSAGE_LENGTH - 1] = '\0';

    // Populate txData with telemetry data from sensors class
    float xLinearAcceleration, yLinearAcceleration, zLinearAcceleration;
    sensors.getFusedLinearAcceleration(xLinearAcceleration, yLinearAcceleration, zLinearAcceleration);
    float xAngularVelocity, yAngularVelocity, zAngularVelocity;
    sensors.getFusedAngularVelocity(xAngularVelocity, yAngularVelocity, zAngularVelocity);
    float yawOrientation, pitchOrientation, rollOrientation;
    sensors.getFusedOrientation(yawOrientation, pitchOrientation, rollOrientation);

    // Create the telemetry data structure
    ControlBoardData txData = {
        .timestamp = millis(),
        .pressure = sensors.getPressure(),
        .altitude = sensors.getAltitude(),
        .temperature = sensors.getTemperature(),
        .accelX = xLinearAcceleration,
        .accelY = yLinearAcceleration,
        .accelZ = zLinearAcceleration,
        .angularVelocityX = xAngularVelocity,
        .angularVelocityY = yAngularVelocity,
        .angularVelocityZ = zAngularVelocity,
        .orientationYaw = yawOrientation,
        .orientationPitch = pitchOrientation,
        .orientationRoll = rollOrientation,
        .statusMsgLength = strlen(message),
    };

    // Copy status message
    strncpy(txData.statusMsg, message, sizeof(txData.statusMsg) - 1);
    txData.statusMsg[sizeof(txData.statusMsg) - 1] = '\0';

    // Add telemetry data directly to queue
    telemetryTransmitQueue.enqueue(txData);

    // Log telemetry data to SD card
    emitLog(
        "Enqueuing for TX Telemetry: Time=%u, P=%f, Alt=%f, T=%f, LinAccel=(%f,%f,%f), AngVel=(%f,%f,%f), Orient=(%f,%f,%f), Status=%s",
        txData.timestamp, txData.pressure, txData.altitude, txData.temperature,
        txData.accelX, txData.accelY, txData.accelZ,
        txData.angularVelocityX, txData.angularVelocityY, txData.angularVelocityZ,
        txData.orientationYaw, txData.orientationPitch, txData.orientationRoll,
        txData.statusMsg);
}

void processLogQueue()
{
    File dataFile = SD.open("datalog.txt", FILE_WRITE); // Open the file
    if (!dataFile) {
        emitLog("Failed to open the file on the SD card");
    }

    while (dataFile)
    {
        StringMessage logMessage;
        if (sdCardLoggingQueue.dequeue(logMessage))
        {
            // Dequeuing most recent data to print it on the SD card
            emitLog(logMessage.text);
            size_t count = dataFile.println(logMessage.text);
            if (count != strlen(logMessage.text)) {
                dataFile.close();
                dataFile = NULL;
                emitLog("Failed to emit log to the SD card");
                break;
            }
        }
        threads.yield();
    }
}

void processTelemetryQueue()
{
    // Process telemetry data from the queue
    ControlBoardData txData;
    if (telemetryTransmitQueue.dequeue(txData))
    {
        // Send telemetry data
        if (serialComm.sendData(txData))
        {
            emitLog(
                "TX Telemetry Data Sent: Time=%u, P=%f, Alt=%f, T=%f, LinAccel=(%f,%f,%f), AngVel=(%f,%f,%f), Orient=(%f,%f,%f), Status=%s",
                txData.timestamp, txData.pressure, txData.altitude, txData.temperature,
                txData.accelX, txData.accelY, txData.accelZ,
                txData.angularVelocityX, txData.angularVelocityY, txData.angularVelocityZ,
                txData.orientationYaw, txData.orientationPitch, txData.orientationRoll,
                txData.statusMsg);
        }
    }
}

#endif // LOGGING_H