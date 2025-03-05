#include "StateMachine.h"

StateMachine::StateMachine() : 
    temperature(0), pressure(0), altitude(0),
    currentState(INITIALIZATION),
    initializedSubState(CALIBRATION),
    calibrationSubState(TELEMETRY_CHECK),
    stateEntryTime(0), abortCondition(false),
    telemetryThreadId(-1), dataCollectionThreadId(-1),
    telemetryThreadState(THREAD_STOPPED), dataCollectionThreadState(THREAD_STOPPED),
    workingImuCount(0) {
}

void StateMachine::begin() {
    loadStateFromEEPROM();
    stateEntryTime = millis();
    
    // Start in INITIALIZATION state
    currentState = INITIALIZATION;
    logSystemEvent("Starting system initialization");
    
    // Start data collection thread immediately to get sensor readings
    startThreads();
}

void StateMachine::run() {
    // Check for abort condition in flight states
    if (abortCondition && currentState == INITIALIZED && 
        (initializedSubState == READY || 
         initializedSubState == ASCENT || 
         initializedSubState == STABILIZATION)) {
        logSystemEvent("Abort condition detected");
        transitionToSubState(DESCENT);
        abortCondition = false;
    }
    
    // Handle the current state
    handleCurrentState();
}

void StateMachine::handleCurrentState() {
    switch(currentState) {
        case INITIALIZATION: handleInitializationState(); break;
        case INITIALIZED:    handleInitializedState();    break;
        case FAULT:          handleFaultState();          break;
        case TERMINATION:    handleTerminationState();    break;
    }
}

void StateMachine::handleInitializationState() {
    // Basic hardware initialization
    if (!initializeHardware()) {
        logSystemEvent("Hardware initialization failed");
        transitionToState(FAULT);
    } else {
        logSystemEvent("Hardware initialization successful");
        transitionToState(INITIALIZED);
        transitionToSubState(CALIBRATION);
        transitionToCalibrationSubState(TELEMETRY_CHECK);
    }
}

void StateMachine::handleInitializedState() {
    switch(initializedSubState) {
        case CALIBRATION:    handleCalibrationState();    break;
        case READY:          handleReadyState();          break;
        case ASCENT:         handleAscentState();         break;
        case STABILIZATION:  handleStabilizationState();  break;
        case DESCENT:        handleDescentState();        break;
    }
}

void StateMachine::handleCalibrationState() {
    switch(calibrationSubState) {
        case TELEMETRY_CHECK:    handleTelemetryCheckState();     break;
        case BATTERY_CHECK:      handleBatteryCheckState();       break;
        case PRESSURE_CHECK:     handlePressureCheckState();      break;
        case TEMPERATURE_CHECK:  handleTemperatureCheckState();   break;
        case IMU_CHECK:          handleImuCheckState();           break;
    }
}

// Calibration sub-state handlers
void StateMachine::handleTelemetryCheckState() {
    if (initializeTelemetry()) {
        logSystemEvent("Telemetry check passed");
        transitionToCalibrationSubState(BATTERY_CHECK);
    } else {
        logSystemEvent("Telemetry check failed");
        transitionToState(FAULT);
    }
}

void StateMachine::handleBatteryCheckState() {
    if (initializeBattery()) {
        logSystemEvent("Battery check passed");
        transitionToCalibrationSubState(PRESSURE_CHECK);
    } else {
        logSystemEvent("Battery check failed");
        transitionToState(FAULT);
    }
}

void StateMachine::handlePressureCheckState() {
    if (isPressureSensorWorking()) {
        logSystemEvent("Pressure sensor check passed");
        transitionToCalibrationSubState(TEMPERATURE_CHECK);
    } else {
        logSystemEvent("Pressure sensor check failed");
        transitionToState(FAULT);
    }
}

void StateMachine::handleTemperatureCheckState() {
    if (isTemperatureSensorWorking()) {
        logSystemEvent("Temperature check passed");
    } else {
        logSystemEvent("Temperature check warning");
    }
    transitionToCalibrationSubState(IMU_CHECK);
}

void StateMachine::handleImuCheckState() {
    checkSensorsStatus(); // Update IMU status
    
    if (workingImuCount < 2) {
        logSystemEvent("Not enough working IMUs: " + String(workingImuCount));
        transitionToState(FAULT);
    } else if (workingImuCount == 2) {
        logSystemEvent("Warning: Only 2 IMUs working");
        transitionToSubState(READY);
    } else {
        logSystemEvent("All IMUs working properly");
        transitionToSubState(READY);
    }
}

// Initialized sub-state handlers
void StateMachine::handleReadyState() {
    if (sensors.isAscending()) {
        logSystemEvent("Ascent detected");
        transitionToSubState(ASCENT);
    }
}

void StateMachine::handleAscentState() {
    if (hasReachedTargetAltitude()) {
        logSystemEvent("Target altitude reached");
        transitionToSubState(STABILIZATION);
    }
}

void StateMachine::handleStabilizationState() {
    if (millis() - stateEntryTime > STABILIZATION_TIMEOUT) {
        logSystemEvent("Stabilization timeout");
        transitionToSubState(DESCENT);
    }
}

void StateMachine::handleDescentState() {
    if (hasTouchedDown()) {
        logSystemEvent("Touchdown detected");
        transitionToState(TERMINATION);
    }
}

void StateMachine::handleFaultState() {
    if (millis() - stateEntryTime > FAULT_TIMEOUT) {
        logSystemEvent("Fault timeout reached");
        transitionToState(TERMINATION);
    }
}

void StateMachine::handleTerminationState() {
    logSystemEvent("System terminated");
    stopThreads();
}

// State transition methods
void StateMachine::transitionToState(SystemState newState) {
    logSystemEvent("State: " + String(currentState) + " -> " + String(newState));
    
    currentState = newState;
    stateEntryTime = millis();
    saveStateToEEPROM();
}

void StateMachine::transitionToSubState(InitializedSubState newSubState) {
    if (currentState != INITIALIZED) return;
    
    logSystemEvent("SubState: " + String(initializedSubState) + " -> " + String(newSubState));
    
    initializedSubState = newSubState;
    stateEntryTime = millis();
}

void StateMachine::transitionToCalibrationSubState(CalibrationSubState newCalibState) {
    if (initializedSubState != CALIBRATION) return;
    
    calibrationSubState = newCalibState;
    stateEntryTime = millis();
}

// Hardware initialization
bool StateMachine::initializeHardware() {
    return initializeSensors() && initializeSD();
}

bool StateMachine::initializeSensors() {
    sensorInitStatus = sensors.begin();
    checkSensorsStatus();
    return sensorInitStatus.temperature && 
           sensorInitStatus.pressure && 
           getInitializedImuCount() >= 1;
}

bool StateMachine::initializeBattery() {
    return true; // Placeholder
}

bool StateMachine::initializeSD() {
    if (!SD.begin(SD_CHIP_SELECT)) {
        return false;
    }
    return true;
}

bool StateMachine::initializeTelemetry() {
    return true; // Placeholder
}

// Thread management
void StateMachine::startThreads() {
    if (telemetryThreadId == -1) {
        telemetryThreadId = threads.addThread(telemetryThreadMethod, this);
        telemetryThreadState = THREAD_RUNNING;
    }
    
    if (dataCollectionThreadId == -1) {
        dataCollectionThreadId = threads.addThread(dataCollectionThreadMethod, this);
        dataCollectionThreadState = THREAD_RUNNING;
    }
}

void StateMachine::pauseThreads() {
    if (telemetryThreadState == THREAD_RUNNING) {
        threads.suspend(telemetryThreadId);
        telemetryThreadState = THREAD_PAUSED;
    }
    
    if (dataCollectionThreadState == THREAD_RUNNING) {
        threads.suspend(dataCollectionThreadId);
        dataCollectionThreadState = THREAD_PAUSED;
    }
}

void StateMachine::resumeThreads() {
    if (telemetryThreadState == THREAD_PAUSED) {
        threads.restart(telemetryThreadId);
        telemetryThreadState = THREAD_RUNNING;
    }
    
    if (dataCollectionThreadState == THREAD_PAUSED) {
        threads.restart(dataCollectionThreadId);
        dataCollectionThreadState = THREAD_RUNNING;
    }
}

void StateMachine::stopThreads() {
    if (telemetryThreadId != -1) {
        threads.kill(telemetryThreadId);
        telemetryThreadId = -1;
        telemetryThreadState = THREAD_STOPPED;
    }
    
    if (dataCollectionThreadId != -1) {
        threads.kill(dataCollectionThreadId);
        dataCollectionThreadId = -1;
        dataCollectionThreadState = THREAD_STOPPED;
    }
}

// Thread methods
void StateMachine::telemetryThreadMethod(void* arg) {
    StateMachine* sm = static_cast<StateMachine*>(arg);
    while(true) {
        sm->processTelemetry();
        threads.delay(100);
    }
}

void StateMachine::dataCollectionThreadMethod(void* arg) {
    StateMachine* sm = static_cast<StateMachine*>(arg);
    while(true) {
        sm->collectData();
        threads.delay(100);
    }
}

void StateMachine::processTelemetry() {
    // Send data via telemetry link
    // This would be implemented based on your hardware
}

void StateMachine::collectData() {
    sensorDataMutex.lock();
    
    if (sensorInitStatus.temperature) {
        temperature = sensors.getTemperature();
    }
    
    if (sensorInitStatus.pressure) {
        pressure = sensors.getPressure();
        altitude = sensors.getAltitude();
    }
    
    if (workingImuCount > 0) {
        sensors.getFusedOrientation(yaw, pitch, roll, accuracyDegrees);
        sensors.getFusedLinearAcceleration(ax, ay, az, accuracy);
    }
    
    sensorDataMutex.unlock();
}

// Sensor status methods
void StateMachine::checkSensorsStatus() {
    float tempYaw, tempPitch, tempRoll, tempAccuracy;
    workingImuCount = 0;
    
    if (sensorInitStatus.imu1) {
        sensors.getOrientation(sensors.imu1, tempYaw, tempPitch, tempRoll, tempAccuracy);
        if (tempAccuracy >= 0) workingImuCount++;
    }
    
    if (sensorInitStatus.imu2) {
        sensors.getOrientation(sensors.imu2, tempYaw, tempPitch, tempRoll, tempAccuracy);
        if (tempAccuracy >= 0) workingImuCount++;
    }
    
    if (sensorInitStatus.imu3) {
        sensors.getOrientation(sensors.imu3, tempYaw, tempPitch, tempRoll, tempAccuracy);
        if (tempAccuracy >= 0) workingImuCount++;
    }
}

// Utility methods
bool StateMachine::isTemperatureSensorWorking() {
    if (!sensorInitStatus.temperature) return false;
    return temperature > -100.0 && temperature < 200.0;
}

bool StateMachine::isPressureSensorWorking() {
    if (!sensorInitStatus.pressure) return false;
    return pressure > 0.0 && pressure < 1500.0;
}

bool StateMachine::areEnoughImusWorking() {
    return workingImuCount >= 2;
}

int StateMachine::getInitializedImuCount() {
    return (sensorInitStatus.imu1 ? 1 : 0) + 
           (sensorInitStatus.imu2 ? 1 : 0) + 
           (sensorInitStatus.imu3 ? 1 : 0);
}

// Flight monitoring
bool StateMachine::hasReachedTargetAltitude() {
    return altitude >= TARGET_ALTITUDE;
}

bool StateMachine::hasTouchedDown() {
    return altitude <= 10.0; // 10 meters threshold
}

// EEPROM methods
void StateMachine::loadStateFromEEPROM() {
    EEPROM.get(EEPROM_STATE_ADDRESS, currentState);
    if (currentState > TERMINATION) currentState = INITIALIZATION;
}

void StateMachine::saveStateToEEPROM() {
    EEPROM.put(EEPROM_STATE_ADDRESS, currentState);
}

// Logging
void StateMachine::logSystemEvent(const String& event) {
    loggingMutex.lock();
    Serial.println(event);
    loggingMutex.unlock();
}

void StateMachine::setAbortCondition() {
    abortCondition = true;
}
