#include "StateMachine.h"

StateMachine::StateMachine() : 
    currentState(INITIALIZATION), 
    previousState(INITIALIZATION),
    calibrationSubState(CALIBRATION_NORMAL),
    calibrationThreadId(-1),
    telemetryThreadId(-1),
    dataCollectionThreadId(-1) {
    // Initialize device statuses to false
    for(int i = 0; i < 5; i++) {
        deviceStatus[i] = false;
    }
}

void StateMachine::begin() {
    // Activation beep sound
    activationBeepSound();

    // Load last known state from EEPROM
    loadStateFromEEPROM();

    // Initialize hardware components
    if (!initializeSensors() || 
        !initializeBattery() || 
        !initializeSD() ||
        !initializeTelemetry()) {
        transitionToState(FAULT);
        return;
    }

    // Setup threads for concurrent states
    calibrationThreadId = threads.addThread(calibrationThreadMethod, this);
    telemetryThreadId = threads.addThread(telemetryThreadMethod, this);
    dataCollectionThreadId = threads.addThread(dataCollectionThreadMethod, this);

    // Initial transition to initialization state
    transitionToState(INITIALIZATION);
}

void StateMachine::loadStateFromEEPROM() {
    EEPROM.get(EEPROM_STATE_ADDRESS, currentState);
    if (currentState == 0) {
        currentState = INITIALIZATION; // Default to initialization if no previous state
    }
}

void StateMachine::saveStateToEEPROM() {
    EEPROM.put(EEPROM_STATE_ADDRESS, currentState);
}

bool StateMachine::initializeSensors() {
    // Use Sensors class to initialize
    bool sensorsReady = sensors.begin();
    
    // Update device statuses
    deviceStatus[0] = sensors.checkTemperatureSensorStatus();
    deviceStatus[1] = sensors.checkPressureSensorStatus();
    deviceStatus[2] = sensors.checkIMUSensorStatus();
    
    return sensorsReady;
}

bool StateMachine::initializeBattery() {
    // Battery initialization logic
    // TODO: Implement battery voltage check
    return true;
}

bool StateMachine::initializeSD() {
    // Initialize SD card
    if (!SD.begin(SD_CHIP_SELECT)) {
        logSystemEvent("SD Card Initialization Failed");
        return false;
    }
    return true;
}

bool StateMachine::initializeTelemetry() {
    // Telemetry initialization placeholder
    return true;
}

void StateMachine::run() {
    switch(currentState) {
        case INITIALIZATION:
            // Verify telemetry, sensors, battery
            if (initializeSensors() && 
                initializeBattery() && 
                initializeTelemetry()) {
                transitionToState(CALIBRATION);
            }
            break;
        
        case CALIBRATION:
            runCalibration();
            
            // Transition based on calibration substate
            switch(calibrationSubState) {
                case CALIBRATION_WARNING:
                    handleCalibrationWarning();
                    break;
                case CALIBRATION_FAULT:
                    handleCalibrationFault();
                    break;
                case CALIBRATION_NORMAL:
                    if (sensors.checkTemperatureSensorStatus() && 
                        sensors.checkPressureSensorStatus() && 
                        sensors.checkIMUSensorStatus()) {
                        transitionToState(READY);
                    }
                    break;
            }
            break;
        
        case READY:
            if (isAscending()) {
                transitionToState(ASCENT);
            }
            break;
        
        case ASCENT:
            if (hasReachedTargetAltitude()) {
                transitionToState(STABILIZATION);
            }
            break;
        
        case STABILIZATION:
            // Run control algorithm
            // Output propulsion signals
            if (isDescending()) {
                transitionToState(DESCENT);
            }
            break;
        
        case DESCENT:
            // Descent logic
            // Potential touchdown detection
            transitionToState(TERMINATION);
            break;
        
        case TERMINATION:
            // Stop all motors
            // Release payload
            break;
        
        case FAULT:
            faultBeepSound();
            // Log fault details
            logSystemEvent("System Fault Detected");
            break;
    }

    // Update status log in all states
    updateStatusLog();
}

void StateMachine::runCalibration() {
    float temperature = sensors.getTemperature();
    
    // Calibration checks
    if (temperature < -20 || temperature > 80) {
        calibrationSubState = CALIBRATION_WARNING;
    }
    
    // Additional calibration checks can be added here
}

void StateMachine::handleCalibrationWarning() {
    // Log warning
    logSystemEvent("Calibration Warning: Temperature out of range");
    
    // Continue calibration with caution or potentially transition to fault
}

void StateMachine::handleCalibrationFault() {
    // Log fault
    logSystemEvent("Calibration Fault Detected");
    
    // Transition to fault state
    transitionToState(FAULT);
}

void StateMachine::transitionToState(SystemState newState) {
    previousState = currentState;
    currentState = newState;
    saveStateToEEPROM();
    logSystemEvent("State Transition: " + String(previousState) + " -> " + String(newState));
}

bool StateMachine::isAscending() {
    float currentAltitude = sensors.getAltitude();
    
    bool ascending = (currentAltitude - previousAltitude > 0.1); // 10cm threshold
    previousAltitude = currentAltitude;
    
    return ascending;
}

bool StateMachine::isDescending() {
    float currentAltitude = sensors.getAltitude();
    
    bool descending = (previousAltitude - currentAltitude > 0.1); // 10cm threshold
    previousAltitude = currentAltitude;
    
    return descending;
}

bool StateMachine::hasReachedTargetAltitude() {
    float currentAltitude = sensors.getAltitude();
    
    return currentAltitude >= TARGET_ALTITUDE;
}

void StateMachine::logSystemEvent(const String& event) {
    // Log to SD card
    writeToSD(event);
    
    // TODO: Implement telemetry logging when telemetry class is ready
}

void StateMachine::writeToSD(const String& data) {
    File logFile = SD.open("system_log.txt", FILE_WRITE);
    if (logFile) {
        logFile.println(data);
        logFile.close();
    }
}

void StateMachine::activationBeepSound() {
    // Implement activation beep sound
    // This would typically involve a buzzer or speaker
}

void StateMachine::faultBeepSound() {
    // Implement fault beep sound pattern
    // Different from activation beep to indicate error
}

void StateMachine::updateStatusLog() {
    // Log current system status periodically
    static unsigned long lastLogTime = 0;
    if (millis() - lastLogTime > 1000) { // Log every second
        String statusLog = "State: " + String(currentState) + 
                           ", Sensors: " + 
                           (sensors.checkTemperatureSensorStatus() ? "T" : "t") +
                           (sensors.checkPressureSensorStatus() ? "P" : "p") +
                           (sensors.checkIMUSensorStatus() ? "I" : "i");
        
        logSystemEvent(statusLog);
        lastLogTime = millis();
    }
}

// Thread method placeholders
void StateMachine::calibrationThreadMethod(void* arg) {
    StateMachine* stateMachine = static_cast<StateMachine*>(arg);
    while(true) {
        // Calibration thread logic
        threads.delay(1000);
    }
}

void StateMachine::telemetryThreadMethod(void* arg) {
    StateMachine* stateMachine = static_cast<StateMachine*>(arg);
    while(true) {
        stateMachine->processTelemetry();
        threads.delay(500);
    }
}

void StateMachine::dataCollectionThreadMethod(void* arg) {
    StateMachine* stateMachine = static_cast<StateMachine*>(arg);
    while(true) {
        stateMachine->collectData();
        threads.delay(100);
    }
}

void StateMachine::processTelemetry() {
    // Telemetry processing placeholder
}

void StateMachine::collectData() {
    // Collect sensor data using pass-by-reference methods
    temperature = sensors.getTemperature();
    pressure = sensors.getPressure();
    altitude = sensors.getAltitude();
    sensors.getLinearAcceleration(sensors.imu1, ax, ay, az, accuracy);
    sensors.getOrientation(sensors.imu1, yaw, pitch, roll, accuracyDegrees);
}

SensorStatus StateMachine::checkSensorStatus(int sensorIndex) {
    // Implement sensor status checking logic
    // Return SENSOR_OK, SENSOR_WARNING, or SENSOR_FAULT
    return SENSOR_OK;
}