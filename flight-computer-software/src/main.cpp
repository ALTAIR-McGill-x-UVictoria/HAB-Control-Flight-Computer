#include <Arduino.h>
#include <TeensyThreads.h>
#include "StateMachine.h"

enum State {
    INITIALIZATION,
    TELEMETRY_CHECK,
    BATTERY_CHECK,
    SENSOR_CHECK,
    FAULT,
    TERMINATION,
    READY,
    ASCENT,
    STABILIZATION,
    DESCENT
};

bool initialized;           // Initialization status flag
bool aborted;               // Mission abortion status flag
int lastTime;               // Used for timeouts and delays
char telemetry_status;      // Telemetry status flag
char gps_status;            // GPS status flag
char battery_status;        // Battery status flag
char altimeter_status;      // Altimeter status flag
char temperature_status;    // Temperature status flag
char imu_status[3];         // IMUs status flag

StateMachine<10, 15> flight_fsm;
int telemetry_thread_id = -1;
int data_thread_id = -1;

void initialization_entry() {
    // Init data storage
    // Init libs and models
    // Activation beep
}

void telemetry_check_entry() {
    // Init spi telemetry api
    // Verify connection
    // Verify GPS (using api)
    // Print status
}

void battery_check_entry() {
    // Verify battery (using api)
    // Print status
}

void sensor_check_entry() {
    // Init sensors (altimeter, imu, temperature)
    // Verify altimeter sensor
    // Verify temperature sensor
    // Verify imu sensors
    // Print status
}

void fault_entry() {
    // Fault logging (maybe do that in the transition too for specific faults)
    // Stop all motors
    // Stop all sensors
    // Print status
}

void fault_do() {
    // Fault beep
    // Wait for timeout
}

void termination_entry() {
    // Print status
    stop_all_threads();
}

void ready_entry() {
    // Print status
    start_all_threads();
}

void ready_do() {
    // Compute altitude ascended using pressure sensor
}

void ascent_do() {
    // Status log
}

void stabilization_do() {
    // Control algorithm
    // Output to motors
    // Status log
}

void descent_entry() {
    // Stop all motors
    // Release payload
}

void descent_do() {
    // Status log
    // Compute if reached touchdown
}

bool is_initialized() {
    return initialized;
}

bool has_failed() {
    // Print error message
    // Check all states for error
    return false;
}

bool has_telemetry() {
    // Check if telemetry and gps is available
    return telemetry_status > 0 && gps_status > 0;
}

bool has_battery() {
    // Check if battery is available
    return battery_status > 0;
}

void telemetry_thread() {
    while(true) {
        // Read telemetry data
        // Send telemetry data
    }
}

bool has_sensors() {
    // Check if the minimum number of sensors are available
    // Give a warning if non critical sensors are missing
    // char available_imus = imu_status[0] > 0 + imu_status[1] > 0 + imu_status[2] > 0;
    return true;
}

bool is_ascendeding() {
    // Check if the HAB has ascended to the desired altitude
    return false;
}

bool can_stabilize() {
    // Check if the HAB has reached the desired altitude
    // Check if the HAB is ready to stabilize
    return false;
}

bool is_aborted() {
    // Check if the HAB has aborted
    return aborted;
}

bool stabilization_timeout() {
    // Check if the stabilization has timed out
    return false;
}

bool has_landed() {
    // Check if the HAB has landed
    return false;
}

bool has_fault_timeout() {
    // Check if fault has timed out
    return false;
}

void data_collection_thread() {
    while(true) {
        // data_collection_do();
        // threads.delay(200);
    }
}

void stop_all_threads() {
    if (telemetry_thread_id != -1) {
        threads.kill(telemetry_thread_id);
        telemetry_thread_id = -1;
    }
    if (data_thread_id != -1) {
        threads.kill(data_thread_id);
        data_thread_id = -1;
    }
}

void start_all_threads() {
    stop_all_threads();
    telemetry_thread_id = threads.addThread(telemetry_thread);
    data_thread_id = threads.addThread(data_collection_thread);
}

void setup() {
    Serial.begin(115200);

    // Flight state machine
    flight_fsm.addState(INITIALIZATION, initialization_entry, nullptr, nullptr);
    flight_fsm.addState(TELEMETRY_CHECK, telemetry_check_entry, nullptr, nullptr);
    flight_fsm.addState(BATTERY_CHECK, battery_check_entry, nullptr, nullptr);
    flight_fsm.addState(SENSOR_CHECK, sensor_check_entry, nullptr, nullptr);
    flight_fsm.addState(FAULT, fault_entry, fault_do), nullptr;
    flight_fsm.addState(TERMINATION, termination_entry, nullptr);
    flight_fsm.addState(READY, ready_entry, ready_do, nullptr);
    flight_fsm.addState(ASCENT, nullptr, ascent_do, nullptr);
    flight_fsm.addState(STABILIZATION, nullptr, stabilization_do, nullptr);
    flight_fsm.addState(DESCENT, descent_entry, descent_do, nullptr);
    
    flight_fsm.setInitialState(INITIALIZATION);

    // Add transitions
    flight_fsm.addTransition(INITIALIZATION, TELEMETRY_CHECK, is_initialized);
    flight_fsm.addTransition(INITIALIZATION, FAULT, has_failed);
    flight_fsm.addTransition(TELEMETRY_CHECK, BATTERY_CHECK, has_telemetry);
    flight_fsm.addTransition(TELEMETRY_CHECK, FAULT, has_failed);
    flight_fsm.addTransition(BATTERY_CHECK, SENSOR_CHECK, has_battery);
    flight_fsm.addTransition(BATTERY_CHECK, FAULT, has_failed);
    flight_fsm.addTransition(SENSOR_CHECK, READY, has_sensors);
    flight_fsm.addTransition(SENSOR_CHECK, FAULT, has_failed);
    flight_fsm.addTransition(READY, ASCENT, is_ascendeding);
    flight_fsm.addTransition(ASCENT, STABILIZATION, can_stabilize);
    flight_fsm.addTransition(ASCENT, DESCENT, is_aborted);
    flight_fsm.addTransition(STABILIZATION, DESCENT, stabilization_timeout);
    flight_fsm.addTransition(STABILIZATION, DESCENT, is_aborted);
    flight_fsm.addTransition(DESCENT, TERMINATION, has_landed);
    flight_fsm.addTransition(FAULT, TERMINATION, has_fault_timeout);

    lastTime = millis();
}

void loop() {
    flight_fsm.update();
    threads.yield();
}