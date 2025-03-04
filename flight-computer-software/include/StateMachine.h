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
    READY,
    ASCENT,
    STABILIZATION,
    DESCENT,
    TERMINATION,
    TELEMETRY_HOLD,
    DATA_COLLECTION_HOLD
};

// Enum for calibration state subtypes
enum CalibrationSubState {
    CALIBRATION_NORMAL,
    CALIBRATION_WARNING,
    CALIBRATION_FAULT
};

class StateMachine {
public:
    StateMachine();
    void begin();
    void run();

private:
    // Sensor data
    float temperature, pressure, altitude;
    float ax, ay, az, accuracy;
    float yaw, pitch, roll, accuracyDegrees;
    
    // State variables
    SystemState currentState;
    CalibrationSubState calibrationSubState;
    
    // Hardware management
    Sensors sensors;
    SensorStatus sensorInitStatus;
    int telemetryThreadId, dataCollectionThreadId;
    
    // Sensor status tracking
    bool imu1Working, imu2Working, imu3Working;
    int workingImuCount;

    // Core functionality
    bool initializeHardware();
    void transitionToState(SystemState newState);
    void handleCurrentState();
    void logSystemEvent(const String& event);
    
    // State handlers
    void handleInitializationState();
    void handleCalibrationState();
    void handleReadyState();
    void handleAscentState();
    void handleStabilizationState();
    void handleDescentState();
    void handleTerminationState();
    void handleFaultState();

    // Calibration handling
    void runCalibration();
    void handleCalibrationWarning();
    void handleCalibrationFault();

    // EEPROM functions
    void loadStateFromEEPROM();
    void saveStateToEEPROM();
    
    // Hardware initialization
    bool initializeSensors();
    bool initializeBattery();
    bool initializeSD();
    bool initializeTelemetry();
    
    // Flight monitoring
    bool hasReachedTargetAltitude();

    // Thread methods
    static void telemetryThreadMethod(void* arg);
    static void dataCollectionThreadMethod(void* arg);
    void processTelemetry();
    void collectData();
    void checkSensorsStatus();
    
    // Utility methods
    void activationBeepSound();
    void faultBeepSound();
    void updateStatusLog();
    void writeToSD(const String& data);
    
    // Sensor status methods
    bool isTemperatureSensorWorking();
    bool isPressureSensorWorking();
    bool areEnoughImusWorking();
    int getInitializedImuCount();

    // Mutex locks for thread safety
    Threads::Mutex sensorDataMutex;    // Protects sensor readings
    Threads::Mutex loggingMutex;       // Protects SD card operations

    // Add thread control flags
    bool threadsShouldRun;  // Flag to control thread execution
    
    // Add thread control methods
    void startThreads();
    void stopThreads();
};

#endif // STATE_MACHINE_H