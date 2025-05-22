#include "utils.h"
#include "hab_model.h"
#include "Sensors.h"
#include "Propulsion.h"
#include "SerialCommunication.h"
#include <ArduTFLite.h>
#include "StateMachine.h"

// Define constants
const int kTensorArenaSize = 8 * 1024;

// Define the tensor arena in DMAMEM (DMA-accessible memory)
DMAMEM uint8_t tensor_arena[kTensorArenaSize];

// Define the sensor variables
float sensor_x_accel = 0.0f, sensor_y_accel = 0.0f, sensor_z_accel = 0.0f;
float sensor_vx = 0.0f, sensor_vy = 0.0f, sensor_vz = 0.0f;
float sensor_px = 0.0f, sensor_py = 0.0f, sensor_pz = 0.0f;
float sensor_x_angular_vel = 0.0f, sensor_y_angular_vel = 0.0f, sensor_z_angular_vel = 0.0f;
float sensor_yaw = 0.0f, sensor_pitch = 0.0f, sensor_roll = 0.0f;
float gps_lon = 0.0f, gps_lat = 0.0f, gps_alt = 0.0f;
float gps_vx = 0.0f, gps_vy = 0.0f, gps_vz = 0.0f;

float baro_altitude = 0.0f, baro_pressure = 0.0f, baro_temp = 0.0f;

// Define the model state variables
float prevAction0 = 0.0f;
float prevAction1 = 0.0f;
float actionDiff0 = 0.0f;
float actionDiff1 = 0.0f;

// Define the model state tracking
bool model_inputs_updated = false;
bool model_outputs_updated = false;
unsigned long last_model_run = 0;

// Define status variables
bool telemetry_status = false;
bool gps_status = false;
bool battery_status = false;
bool initialized = false;
bool aborted = false;
unsigned long last_time = 0;
float prev_altitude = 0.0f;
float vertical_speed = 0.0f;

// Define the hardware objects
Sensors sensors;
SerialCommunication serialComm(Serial1, BoardType::CONTROL_BOARD);
Propulsion propulsion;

// Power board data from telemetry
PowerBoardData receivedData = {};

// State machine 
StateMachine<10, 15> flight_fsm;

// Define the safeSigmoid function (previously defined in multiple places)
float safeSigmoid(float x) {
    // Constrain input to avoid overflow
    if (x > 10.0f) return 1.0f;
    if (x < -10.0f) return 0.0f;
    
    // Standard sigmoid calculation
    return 1.0f / (1.0f + exp(-x));
}