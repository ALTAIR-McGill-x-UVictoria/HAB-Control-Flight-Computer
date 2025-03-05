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
#define TARGET_ALTITUDE 20000.0
#define FAULT_TIMEOUT 30000
#define STABILIZATION_TIMEOUT 60000

// Main states
enum SystemState {
    INITIALIZATION,
    INITIALIZED,
    FAULT,
    TERMINATION
};

// Initialized sub-states
enum InitializedSubState {
    CALIBRATION,
    READY,
    ASCENT,
    STABILIZATION,
    DESCENT
};

// Calibration sub-states
enum CalibrationSubState {
    TELEMETRY_CHECK,
    BATTERY_CHECK,
    PRESSURE_CHECK,
    TEMPERATURE_CHECK,
    IMU_CHECK
};

// Thread state
enum ThreadState {
    THREAD_RUNNING,
    THREAD_PAUSED,
    THREAD_STOPPED
};

class StateMachine {
public:
    StateMachine();
    void begin();
    void run();
    void setAbortCondition();

private:
    // Sensor data
    float temperature, pressure, altitude;
    float ax, ay, az, accuracy;
    float yaw, pitch, roll, accuracyDegrees;
    
    // State variables
    SystemState currentState;
    InitializedSubState initializedSubState;
    CalibrationSubState calibrationSubState;
    unsigned long stateEntryTime;
    bool abortCondition;
    
    // Hardware management
    Sensors sensors;
    SensorStatus sensorInitStatus;
    int telemetryThreadId, dataCollectionThreadId;
    ThreadState telemetryThreadState, dataCollectionThreadState;
    
    // Sensor tracking
    int workingImuCount;

    // Core functionality
    bool initializeHardware();
    void transitionToState(SystemState newState);
    void transitionToSubState(InitializedSubState newSubState);
    void transitionToCalibrationSubState(CalibrationSubState newCalibState);
    void handleCurrentState();
    void logSystemEvent(const String& event);
    
    // State handlers
    void handleInitializationState();
    void handleInitializedState();
    void handleFaultState();
    void handleTerminationState();
    
    // Sub-state handlers
    void handleCalibrationState();
    void handleReadyState();
    void handleAscentState();
    void handleStabilizationState();
    void handleDescentState();
    
    // Calibration sub-state handlers
    void handleTelemetryCheckState();
    void handleBatteryCheckState();
    void handlePressureCheckState();
    void handleTemperatureCheckState();
    void handleImuCheckState();

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
    bool hasTouchedDown();

    // Thread methods
    static void telemetryThreadMethod(void* arg);
    static void dataCollectionThreadMethod(void* arg);
    void processTelemetry();
    void collectData();
    void checkSensorsStatus();
    void startThreads();
    void pauseThreads();
    void resumeThreads();
    void stopThreads();
    
    // Sensor status methods
    bool isTemperatureSensorWorking();
    bool isPressureSensorWorking();
    bool areEnoughImusWorking();
    int getInitializedImuCount();

    // Thread safety
    Threads::Mutex sensorDataMutex;
    Threads::Mutex loggingMutex;
};

#endif // STATE_MACHINE_H