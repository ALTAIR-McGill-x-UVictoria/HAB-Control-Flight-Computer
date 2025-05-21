#include <Arduino.h>
#include <TeensyThreads.h>
#include <SPI.h>
#include <SD.h>
#include <ArduTFLite.h>
#include "StateMachine.h"
#include "Sensors.h"  // Include this only once
#include "Logging.h"  // This already includes Sensors.h, so the above is technically redundant
#include "hab_model.h"
#include "Propulsion.h"
#include <core_pins.h> // Required for DMAMEM

// Function declarations

// Entry functions
void initialization_entry();
void termination_entry();
void stabilization_entry();
void ready_entry();
void fault_entry();
void descent_entry();
void telemetry_check_entry();
void battery_check_entry();
void sensor_check_entry();

// Do functions
void ready_do();
void ascent_do();
void stabilization_do();
void descent_do();
bool stabilization_timeout();

// Status checks
bool is_initialized();
bool has_telemetry();
bool has_battery();
bool has_sensors();
bool is_ascending();
bool can_stabilize();
bool is_aborted();
bool has_landed();
bool has_fault_timeout(); //transition btw fault and termination

// State fail exit functions
bool has_failed_initialization();
bool has_failed_telemetry();
bool has_failed_battery_check();
bool has_failed_sensor_check();

