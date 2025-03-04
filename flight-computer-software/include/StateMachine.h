#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <Arduino.h>
#include <TeensyThreads.h>
#include <EEPROM.h>
#include <SD.h>
#include "Sensors.h"

// Define constants
#define EEPROM_STATE_ADDRESS 0
#define SD_CHIP_SELECT 10
#define TARGET_ALTITUDE 20000.0 // meters

// Enum for tracking current system state
enum SystemState {
    INITIALIZATION,
    FAULT,
    CALIBRATION,
    TELEMETRY_HOLD,
    DATA_COLLECTION_HOLD,
    READY,
    ASCENT,
    STABILIZATION,
    DESCENT,
    TERMINATION
};

// Enum for calibration state subtypes
enum CalibrationSubState {
    CALIBRATION_NORMAL,
    CALIBRATION_WARNING,
    CALIBRATION_FAULT
};

// Enum for sensor status
enum SensorStatus {
    SENSOR_OK,
    SENSOR_WARNING,
    SENSOR_FAULT
};

class StateMachine {
public:
    StateMachine();
    void begin();
    void run();

private:
    // Member variables to store collected data
    float temperature;
    float pressure;
    float altitude;
    float ax, ay, az, accuracy;
    float yaw, pitch, roll, accuracyDegrees;

    // State management
    SystemState currentState;
    SystemState previousState;
    CalibrationSubState calibrationSubState;

    // Sensors and threads
    Sensors sensors;
    int calibrationThreadId;
    int telemetryThreadId;
    int dataCollectionThreadId;

    // Device status
    bool deviceStatus[5]; // Array to track device statuses

    // Internal methods
    void saveStateToEEPROM();
    void loadStateFromEEPROM();

    // Initialization methods
    bool initializeSensors();
    bool initializeBattery();
    bool initializeSD();
    bool initializeTelemetry();

    // Calibration methods
    void runCalibration();
    void handleCalibrationWarning();
    void handleCalibrationFault();

    // State transition methods
    void transitionToState(SystemState newState);

    // Logging methods
    void logSystemEvent(const String& event);
    void writeToSD(const String& data);

    // Flight detection methods
    bool isAscending();
    bool isDescending();
    bool hasReachedTargetAltitude();

    // Thread methods
    static void calibrationThreadMethod(void* arg);
    static void telemetryThreadMethod(void* arg);
    static void dataCollectionThreadMethod(void* arg);

    // Telemetry and data collection
    void processTelemetry();
    void collectData();

    // Utility methods
    void activationBeepSound();
    void faultBeepSound();
    void updateStatusLog();

    // Sensor status checking
    SensorStatus checkSensorStatus(int sensorIndex);
};

#endif // STATE_MACHINE_H