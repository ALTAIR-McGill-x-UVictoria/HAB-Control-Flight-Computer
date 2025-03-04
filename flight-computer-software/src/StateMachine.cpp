#include "StateMachine.h"

StateMachine::StateMachine() : 
    temperature(0), pressure(0), altitude(0),
    currentState(INITIALIZATION),
    calibrationSubState(CALIBRATION_NORMAL),
    telemetryThreadId(-1), dataCollectionThreadId(-1),
    imu1Working(false), imu2Working(false), imu3Working(false), workingImuCount(0) {
}

void StateMachine::begin() {
    activationBeepSound();
    loadStateFromEEPROM();

    // Initialize hardware - go to FAULT state if unsuccessful
    if (!initializeHardware()) {
        transitionToState(FAULT);
        return;
    }

    // Start data collection and telemetry threads
    telemetryThreadId = threads.addThread(telemetryThreadMethod, this);
    dataCollectionThreadId = threads.addThread(dataCollectionThreadMethod, this);
    transitionToState(INITIALIZATION);
}

bool StateMachine::initializeHardware() {
    return initializeSensors() && initializeBattery() && 
           initializeSD() && initializeTelemetry();
}

void StateMachine::run() {
    handleCurrentState();
    updateStatusLog();
}

void StateMachine::handleCurrentState() {
    switch(currentState) {
        case INITIALIZATION: handleInitializationState(); break;
        case CALIBRATION_TELEMETRY:    handleCalibrationState();    break;

        case READY:          handleReadyState();          break;
        case ASCENT:         handleAscentState();         break;
        case STABILIZATION:  handleStabilizationState();  break;
        case DESCENT:        handleDescentState();        break;
        case TERMINATION:    handleTerminationState();    break;
        case FAULT:          handleFaultState();          break;
        case TELEMETRY_HOLD:
        case DATA_COLLECTION_HOLD:
            // Threads handle these states
            break;
    }
}

// EEPROM methods
void StateMachine::loadStateFromEEPROM() {
    EEPROM.get(EEPROM_STATE_ADDRESS, currentState);
    if (currentState == 0) currentState = INITIALIZATION;
}

void StateMachine::saveStateToEEPROM() {
    EEPROM.put(EEPROM_STATE_ADDRESS, currentState);
}

// Hardware initialization
bool StateMachine::initializeSensors() {
    sensorInitStatus = sensors.begin();
    checkSensorsStatus();
    return sensorInitStatus.temperature && 
           sensorInitStatus.pressure && 
           getInitializedImuCount() >= 2;
}

bool StateMachine::initializeBattery() {
    // TODO: Battery initialization
    return true;
}

bool StateMachine::initializeSD() {
    if (!SD.begin(SD_CHIP_SELECT)) {
        logSystemEvent("SD Card Init Failed");
        return false;
    }
    return true;
}

bool StateMachine::initializeTelemetry() {
    // TODO: Implement telemetry
    return true;
}

// State handlers
void StateMachine::handleInitializationState() {
    if (isTemperatureSensorWorking() && 
        isPressureSensorWorking() && 
        areEnoughImusWorking()) {
        transitionToState(CALIBRATION);
    }
}

void StateMachine::handleCalibrationState() {
    runCalibration();
    
    switch(calibrationSubState) {
        case CALIBRATION_WARNING: handleCalibrationWarning(); break;
        case CALIBRATION_FAULT:   handleCalibrationFault();   break;
        case CALIBRATION_NORMAL:
            if (isTemperatureSensorWorking() && 
                isPressureSensorWorking() && 
                areEnoughImusWorking()) {
                transitionToState(READY);
            }
            break;
    }
}

void StateMachine::handleReadyState() {
    if (sensors.isAscending()) transitionToState(ASCENT);
}

void StateMachine::handleAscentState() {
    if (hasReachedTargetAltitude()) transitionToState(STABILIZATION);
}

void StateMachine::handleStabilizationState() {
    if (sensors.isDescending()) transitionToState(DESCENT);
}

void StateMachine::handleDescentState() {
    static unsigned long descentStartTime = millis();
    if (millis() - descentStartTime > 60000) transitionToState(TERMINATION);
}

void StateMachine::handleTerminationState() {
    logSystemEvent("Mission terminated");
}

void StateMachine::handleFaultState() {
    faultBeepSound();
    logSystemEvent("System in FAULT state");
}

// Calibration methods
void StateMachine::runCalibration() {
    float temperature = sensors.getTemperature();
    if (temperature < -20 || temperature > 80) {
        calibrationSubState = CALIBRATION_WARNING;
    }
}

void StateMachine::handleCalibrationWarning() {
    logSystemEvent("Calibration Warning: Temperature out of range");
}

void StateMachine::handleCalibrationFault() {
    logSystemEvent("Calibration Fault Detected");
    transitionToState(FAULT);
}

// State transition
void StateMachine::transitionToState(SystemState newState) {
    SystemState previousState = currentState;
    currentState = newState;
    saveStateToEEPROM();
    logSystemEvent("State Transition: " + String(previousState) + " -> " + String(newState));
}

bool StateMachine::hasReachedTargetAltitude() {
    return altitude >= TARGET_ALTITUDE;
}

// Logging methods
void StateMachine::logSystemEvent(const String& event) {
    loggingMutex.lock();
    writeToSD(event);
    loggingMutex.unlock();
    // TODO: Log to telemetry
}

void StateMachine::writeToSD(const String& data) {
    File logFile = SD.open("system_log.txt", FILE_WRITE);
    if (logFile) {
        logFile.println(data);
        logFile.close();
    }
}

// Sound effects
void StateMachine::activationBeepSound() {
    // TODO: Implement beep
}

void StateMachine::faultBeepSound() {
    // TODO: Implement fault beep
}

// Status logging - simplified to remove compact status code
void StateMachine::updateStatusLog() {
    static unsigned long lastLogTime = 0;
    if (millis() - lastLogTime > 1000) { // Log every second
        // Create a local copy of the data under mutex protection
        float currentTemp, currentPressure, currentAltitude;
        int currentImuCount;
        
        sensorDataMutex.lock();
        currentTemp = temperature;
        currentPressure = pressure;
        currentAltitude = altitude;
        currentImuCount = workingImuCount;
        sensorDataMutex.unlock();
        
        // Build detailed sensor status string
        String statusMessage = "State: " + String(currentState) + " | ";
        
        // Temperature info
        statusMessage += "Temp: " + String(currentTemp, 1) + "C ";
        statusMessage += isTemperatureSensorWorking() ? "(OK) " : "(FAULT) ";
        
        // Pressure and altitude info
        statusMessage += "Press: " + String(currentPressure, 0) + "hPa ";
        statusMessage += isPressureSensorWorking() ? "(OK) " : "(FAULT) ";
        statusMessage += "Alt: " + String(currentAltitude, 0) + "m | ";
        
        // IMU status
        statusMessage += "IMUs: " + String(currentImuCount) + "/3 ";
        if (currentImuCount == 3) statusMessage += "(OK)";
        else if (currentImuCount == 2) statusMessage += "(WARN)";
        else statusMessage += "(FAULT)";
        
        // Log the status information
        loggingMutex.lock();
        logSystemEvent("Status: " + statusMessage);
        loggingMutex.unlock();
        
        lastLogTime = millis();
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
    // TODO: Implement telemetry processing
}

void StateMachine::collectData() {
    // Lock sensor data mutex before updating values
    sensorDataMutex.lock();
    
    // Read temperature and pressure
    if (sensorInitStatus.temperature) {
        temperature = sensors.getTemperature();
    }
    
    if (sensorInitStatus.pressure) {
        pressure = sensors.getPressure();
        altitude = sensors.getAltitude();
    }
    
    // Get fused IMU data if any IMUs are working
    if (workingImuCount > 0) {
        sensors.getFusedOrientation(yaw, pitch, roll, accuracyDegrees);
        sensors.getFusedLinearAcceleration(ax, ay, az, accuracy);
    } else {
        yaw = pitch = roll = ax = ay = az = 0;
        accuracyDegrees = accuracy = -1;
    }
    
    sensorDataMutex.unlock();
}

// Sensor status methods
void StateMachine::checkSensorsStatus() {
    float tempYaw, tempPitch, tempRoll, tempAccuracy;
    workingImuCount = 0;
    
    // Check IMU statuses
    if (sensorInitStatus.imu1) {
        sensors.getOrientation(sensors.imu1, tempYaw, tempPitch, tempRoll, tempAccuracy);
        imu1Working = (tempAccuracy >= 0);
        if (imu1Working) workingImuCount++;
    }
    
    if (sensorInitStatus.imu2) {
        sensors.getOrientation(sensors.imu2, tempYaw, tempPitch, tempRoll, tempAccuracy);
        imu2Working = (tempAccuracy >= 0);
        if (imu2Working) workingImuCount++;
    }
    
    if (sensorInitStatus.imu3) {
        sensors.getOrientation(sensors.imu3, tempYaw, tempPitch, tempRoll, tempAccuracy);
        imu3Working = (tempAccuracy >= 0);
        if (imu3Working) workingImuCount++;
    }
    
    // Read temperature and pressure
    if (sensorInitStatus.temperature) {
        temperature = sensors.getTemperature();
    }
    
    if (sensorInitStatus.pressure) {
        pressure = sensors.getPressure();
        altitude = sensors.getAltitude();
    }
}

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
