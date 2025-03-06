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

bool initialization_status;
char telemetry_status;
char battery_status;
char altimeter_status;
char temperature_status;
char imu_status[3];
char gps_status;

StateMachine<10, 20> flight_fsm;
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
    // Stop threads if they're running
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


void telemetry_thread() {
    while(true) {
        // Read telemetry data
        // Send telemetry data
    }
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

bool sensor_check_to_ready() {
    char available_imus = imu_status[0] > 0 + imu_status[1] > 0 + imu_status[2] > 0;
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
    flight_fsm.addState(READY, nullptr, ready_do, nullptr);
    flight_fsm.addState(ASCENT, nullptr, ascent_do, nullptr);
    flight_fsm.addState(STABILIZATION, nullptr, stabilization_do, nullptr);
    flight_fsm.addState(DESCENT, descent_entry, descent_do, nullptr);
    
    flight_fsm.setInitialState(INITIALIZATION);

    // Add transitions
    flight_fsm.addTransition(SENSOR_CHECK, READY, sensor_check_to_ready);
}

void loop() {
    flight_fsm.update();
    threads.yield();
}