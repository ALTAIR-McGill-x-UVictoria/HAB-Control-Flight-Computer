#include <Arduino.h>
#include <TeensyThreads.h>
#include "StateMachine.h"

// States
enum States {
    INITIALIZATION = 0,
    TELEMETRY_CHECK = 1,
    BATTERY_CHECK = 2,
    SENSOR_CHECK = 3,
    FAULT = 4,
    TERMINATION = 5,
    ACTION = 6,
    // Substates within Action
    READY = 7,
    ASCENT = 8,
    STABILIZATION = 9,
    DESCENT = 10,
    // Concurrent states within Action
    TELEMETRY = 11,
    DATA_COLLECTION = 12
};

// Global state machine
StateMachine<13, 20> stateMachine;

// Function declarations
void initialization_entry();
void initialization_do();
void telemetry_check_entry();
void telemetry_check_do();
void battery_check_entry();
void battery_check_do();
void sensor_check_entry();
void sensor_check_do();
void fault_entry();
void fault_do();
void termination_entry();
void action_entry();
void action_do();
void ready_entry();
void ready_do();
void ascent_do();
void stabilization_do();
void descent_entry();
void telemetry_do();
void data_collection_do();

// Transition condition functions
bool check_initialization_ok();
bool check_telemetry_ok();
bool check_battery_ok();
bool check_sensors_ok();
bool check_initialization_fault();
bool check_telemetry_fault();
bool check_battery_fault();
bool check_sensors_fault();
bool check_ascending();
bool check_target_altitude();
bool check_timeout();
bool check_touchdown();

// Global thread tracking
int telemetryThreadId = -1;
int dataCollectionThreadId = -1;

// Threads for concurrent states within Action
void telemetry_thread() {
    while(true) {
        telemetry_do();
        threads.delay(100);
    }
}

void data_collection_thread() {
    while(true) {
        data_collection_do();
        threads.delay(200);
    }
}

// Start concurrent threads
void start_action_threads() {
    // Stop any existing threads first
    if (telemetryThreadId != -1) {
        threads.kill(telemetryThreadId);
        telemetryThreadId = -1;
    }
    if (dataCollectionThreadId != -1) {
        threads.kill(dataCollectionThreadId);
        dataCollectionThreadId = -1;
    }

    // Start new threads
    telemetryThreadId = threads.addThread(telemetry_thread);
    dataCollectionThreadId = threads.addThread(data_collection_thread);
}

// Stop concurrent threads
void stop_action_threads() {
    if (telemetryThreadId != -1) {
        threads.kill(telemetryThreadId);
        telemetryThreadId = -1;
    }
    if (dataCollectionThreadId != -1) {
        threads.kill(dataCollectionThreadId);
        dataCollectionThreadId = -1;
    }
}

// Implementation of state functions
void initialization_entry() {
    Serial.println("Entering Initialization");
}

void initialization_do() {
    // Initialization tasks
}

void telemetry_check_entry() {
    Serial.println("Entering Telemetry Check");
}

void telemetry_check_do() {
    // Telemetry checks
}

void battery_check_entry() {
    Serial.println("Entering Battery Check");
}

void battery_check_do() {
    // Battery checks
}

void sensor_check_entry() {
    Serial.println("Entering Sensor Check");
}

void sensor_check_do() {
    // Sensor checks
}

void fault_entry() {
    Serial.println("Entering Fault");
    // Stop threads if they're running
    stop_action_threads();
}

void fault_do() {
    // Fault handling
}

void termination_entry() {
    Serial.println("Entering Termination");
    // Stop threads if they're running
    stop_action_threads();
}

void action_entry() {
    Serial.println("Entering Action");
    // Start concurrent threads when entering Action state
    start_action_threads();
}

void action_do() {
    // Manage concurrent states and overall action logic
}

void action_exit() {
    // Stop threads when exiting Action state
    stop_action_threads();
}

void ready_entry() {
    Serial.println("Entering Ready");
}

void ready_do() {
    // Ready state logic
}

void ascent_do() {
    // Ascent logic
}

void stabilization_do() {
    // Stabilization control
}

void descent_entry() {
    Serial.println("Entering Descent");
}

void telemetry_do() {
    // Process telemetry input/output
}

void data_collection_do() {
    // Sensor data collection
}

// Transition conditions
bool check_initialization_ok() {
    return true;
}

bool check_telemetry_ok() {
    return true;
}

bool check_battery_ok() {
    return true;
}

bool check_sensors_ok() {
    return true;
}

bool check_initialization_fault() {
  return !check_initialization_ok();
}

bool check_telemetry_fault() {
  return !check_telemetry_ok();
}

bool check_battery_fault() {
  return !check_battery_ok();
}

bool check_sensors_fault() {
  return !check_sensors_ok();
}

bool check_ascending() {
    return true;
}

bool check_target_altitude() {
    return true;
}

bool check_timeout() {
    return true;
}

bool check_touchdown() {
    return true;
}

void setup() {
    Serial.begin(115200);

    // Add states
    stateMachine.addState(INITIALIZATION, initialization_entry, initialization_do);
    stateMachine.addState(TELEMETRY_CHECK, telemetry_check_entry, telemetry_check_do);
    stateMachine.addState(BATTERY_CHECK, battery_check_entry, battery_check_do);
    stateMachine.addState(SENSOR_CHECK, sensor_check_entry, sensor_check_do);
    stateMachine.addState(FAULT, fault_entry, fault_do);
    stateMachine.addState(TERMINATION, termination_entry);
    
    // Explicit Action state
    stateMachine.addState(ACTION, action_entry, action_do, action_exit);
    
    // Action substates
    stateMachine.addState(READY, ready_entry, ready_do);
    stateMachine.addState(ASCENT, nullptr, ascent_do);
    stateMachine.addState(STABILIZATION, nullptr, stabilization_do);
    stateMachine.addState(DESCENT, descent_entry);
    
    // Concurrent Action states
    stateMachine.addState(TELEMETRY, nullptr, telemetry_do);
    stateMachine.addState(DATA_COLLECTION, nullptr, data_collection_do);

    // Add transitions
    stateMachine.addTransition(INITIALIZATION, TELEMETRY_CHECK, check_initialization_ok);
    stateMachine.addTransition(TELEMETRY_CHECK, BATTERY_CHECK, check_telemetry_ok);
    stateMachine.addTransition(BATTERY_CHECK, SENSOR_CHECK, check_battery_ok);
    stateMachine.addTransition(SENSOR_CHECK, ACTION, check_sensors_ok);
    
    // Fault transitions
    stateMachine.addTransition(INITIALIZATION, FAULT, check_initialization_fault);
    stateMachine.addTransition(TELEMETRY_CHECK, FAULT, check_telemetry_fault);
    stateMachine.addTransition(BATTERY_CHECK, FAULT, check_battery_fault);
    stateMachine.addTransition(SENSOR_CHECK, FAULT, check_sensors_fault);
    
    // Action state internal transitions
    stateMachine.addTransition(READY, ASCENT, check_ascending);
    stateMachine.addTransition(ASCENT, STABILIZATION, check_target_altitude);
    stateMachine.addTransition(STABILIZATION, DESCENT, check_timeout);
    stateMachine.addTransition(DESCENT, TERMINATION, check_touchdown);

    // Set initial state
    stateMachine.setInitialState(INITIALIZATION);
}

void loop() {
    // Update state machine in the main loop
    stateMachine.update();
    
    // Add any additional loop logic if needed
    delay(50);  // Small delay to prevent excessive CPU usage
}