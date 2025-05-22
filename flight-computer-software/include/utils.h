#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <TeensyThreads.h>
#include <SPI.h>
#include <SD.h>
#include <ArduTFLite.h>
#include "StateMachine.h"
#include "Sensors.h"
#include "Logging.h"
#include "hab_model.h"
#include "Propulsion.h"
#include <core_pins.h>

#define MAX_SENSOR_RETRIES 20
#define FAULT_TIMEOUT_DELAY 2000
#define SPEAKER_PIN 33

#define COMM_BAUD_RATE 115200
#define TARGET_STABILIZATION_ALTITUDE 20000
#define TARGET_STABILIZATION_DURATION 30000

// Forward declarations - now just declared, defined in globals.cpp
extern float safeSigmoid(float x);

// Status variables - now all properly declared as extern
extern bool telemetry_status;
extern bool gps_status;
extern bool battery_status;

extern bool initialized;
extern bool aborted;
extern unsigned long last_time;
extern float prev_altitude;
extern float vertical_speed; 

extern Sensors sensors;

// IMU data 
extern float sensor_x_accel, sensor_y_accel, sensor_z_accel;
extern float sensor_vx, sensor_vy, sensor_vz;
extern float sensor_px, sensor_py, sensor_pz;
extern float sensor_x_angular_vel, sensor_y_angular_vel, sensor_z_angular_vel;
extern float sensor_yaw, sensor_pitch, sensor_roll;
extern float gps_lon, gps_lat, gps_alt;
extern float gps_vx, gps_vy, gps_vz;

// Timers
extern unsigned long lastIMU1time, lastIMU2time, lastIMU3time;

// Altimeter data
extern float baro_altitude, baro_pressure, baro_temp;

// Tensor arena
extern const int kTensorArenaSize;
extern uint8_t tensor_arena[];

// Control and communication
extern PowerBoardData receivedData;
extern SerialCommunication serialComm;
extern Propulsion propulsion;

// Model state variables
extern float prevAction0;
extern float prevAction1;
extern float actionDiff0;
extern float actionDiff1;

// Model state tracking
extern bool model_inputs_updated;
extern bool model_outputs_updated;
extern unsigned long last_model_run;

// State controller
extern StateMachine<10, 15> flight_fsm;

enum State
{
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

// Function declarations
void initialization_entry();
void termination_entry();
void stabilization_entry();
void ready_entry();
void fault_entry();
void descent_entry();
void telemetry_check_entry();
void battery_check_entry();
void sensor_check_entry();

void ready_do();
void ascent_do();
void stabilization_do();
void descent_do();
bool stabilization_timeout();

bool is_initialized();
bool has_telemetry();
bool has_battery();
bool has_sensors();
bool is_ascending();
bool can_stabilize();
bool is_aborted();
bool has_landed();
bool has_fault_timeout();

bool has_failed_initialization();
bool has_failed_telemetry();
bool has_failed_battery_check();
bool has_failed_sensor_check();

bool testModelInferenceWithDummyData();

void SDsetup();
void ModelSetup();
void TestModelSetup();
void PropulsionSetup();
void ESCSetup();
void SensorSetup();
void SignalSetupComplete();

#endif // UTILS_H
