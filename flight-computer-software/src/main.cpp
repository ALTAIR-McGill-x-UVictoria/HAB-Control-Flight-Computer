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
void stop_all_threads();
void start_all_threads();
bool has_failed_initialization();
bool has_failed_telemetry();
bool has_failed_battery_check();
bool has_failed_sensor_check();
bool testModelInferenceWithDummyData();

// Thread function declarations
void initialization_entry();
void telemetry_check_entry();
void battery_check_entry();
void sensor_check_entry();
void fault_entry();
void termination_entry();
void ready_entry();
void stabilization_entry();
void descent_entry();
void model_thread();
void telemetry_thread();

#define MAX_SENSOR_RETRIES 20       // Maximum number of retries for sensor initialization
#define FAULT_TIMEOUT_DELAY 2000    // Delay for the fault state before triggering the timeout
#define SPEAKER_PIN 33
#define COMM_BAUD_RATE 115200       // Default baud rate set to 115200 as per testing specifications performed
#define TARGET_STABILIZATION_ALTITUDE 20000 // Target altitude for stabilization in meters
#define TARGET_STABILIZATION_DURATION 30000 // Target duration for stabilization in milliseconds

// Define tensor arena for the model
// constexpr int kTensorArenaSize = 12 * 1024; // Same size as in test_model.cpp
constexpr int kTensorArenaSize = 8 * 1024; // Reduced size for RAM2
DMAMEM alignas(32) uint8_t tensor_arena[kTensorArenaSize]; // New: Place in RAM2

DMAMEM Sensors sensors;
DMAMEM PowerBoardData receivedData; // New: Place in RAM2
DMAMEM SerialCommunication serialComm = SerialCommunication(Serial1, BoardType::CONTROL_BOARD); // If it has significant internal buffers
DMAMEM Propulsion propulsion;

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

DMAMEM bool telemetry_status;                      // Telemetry status flag
DMAMEM bool gps_status;                            // GPS status flag
DMAMEM bool battery_status;                        // Battery status flag

DMAMEM bool initialized;                           // Initialization status flag
DMAMEM bool aborted;                               // Mission abortion status flag
DMAMEM unsigned long last_time;                    // Used for timeouts and delays
DMAMEM float prev_altitude;                        // Last altitude for vertical speed calculations
DMAMEM float vertical_speed;                       // Rate of ascent of the HAB

// Model state variables
DMAMEM float prevAction0 = 0.0f;
DMAMEM float prevAction1 = 0.0f;
DMAMEM float actionDiff0 = 0.0f;
DMAMEM float actionDiff1 = 0.0f;

DMAMEM StateMachine<10, 15> flight_fsm; // New: Place in RAM2
int telemetry_thread_id = -1;
int data_thread_id = -1;
int model_thread_id = -1;

DMAMEM Threads::Mutex modelMutex;

// Add these declarations at file scope (with your other globals)
DMAMEM static uint8_t data_thread_stack[8192];
DMAMEM static uint8_t telemetry_thread_stack[8192];
DMAMEM static uint8_t model_thread_stack[12288]; // Larger for the model thread if needed

// Before stabilization_do function, add static global variables
DMAMEM static float sensor_x_accel, sensor_y_accel, sensor_z_accel;
DMAMEM static float sensor_vx, sensor_vy, sensor_vz;
DMAMEM static float sensor_px, sensor_py, sensor_pz;
DMAMEM static float sensor_x_angular_vel, sensor_y_angular_vel, sensor_z_angular_vel;
DMAMEM static float sensor_yaw, sensor_pitch, sensor_roll;

void initialization_entry()
{
    // Init data storage
    emitLog("Initializing SD card... ");
    if (!SD.begin(BUILTIN_SDCARD)) {
        emitLog("Card failed, or not present.");
        initialized = false;
        return;
    }
    emitLog("SD card initialized.");

    // Initialize the RL model
    emitLog("Initializing TensorFlow Lite model...");
    if (!modelInit(hab_model_tflite, tensor_arena, kTensorArenaSize)) {
        emitLog("Model initialization failed!");
        initialized = false;
        return;
    }
    emitLog("Model initialization complete.");
    
    // Initialize propulsion system
    emitLog("Initializing propulsion system...");
    if (!propulsion.init()) {
        emitLog("Propulsion initialization failed!");
        initialized = false;
        return;
    }
    emitLog("Calibrating ESCs...");
    propulsion.calibrate();
    emitLog("Propulsion system ready.");

    // Activation beep
    // tone(uint8_t pin, uint16_t frequency, uint32_t duration)
    tone(SPEAKER_PIN, 700, 500);
    threads.delay(500);
    noTone(SPEAKER_PIN);
    initialized = true;
    emitLog("Initialization completed.");
}

void telemetry_check_entry()
{
    // Init serial telemetry api
    serialComm.begin();
    
    // Verify connection (assuming manually verification through console output on groundstation)
    emitTelemetry(sensors, "Telemetry check...");
    ControlBoardData txData;
    if (telemetryTransmitQueue.dequeue(txData))
    {
        // Send telemetry data
        if (serialComm.sendData(txData))
        {
            emitLog("Telemetry data sent: %s", txData.statusMsg);
        }
    }

    PowerBoardData tempData;
    if (serialComm.receiveData(tempData, 30000))
    {
        receivedData = tempData;
        emitLog("Telemetry data received: %s", receivedData.statusMsg);
    }

    emitTelemetry(sensors, "Telemetry check...");

    if (telemetryTransmitQueue.dequeue(txData))
    {
        // Send telemetry data
        if (serialComm.sendData(txData))
        {
            emitLog("Telemetry data sent: %s", txData.statusMsg);
        }
    }
    
    // TODO: Verify GPS (using api)
    // For now, telemetry_status set to true if this passes
    telemetry_status = true;
    gps_status = true;
    emitLog("Telemetry check completed");
}

void battery_check_entry()
{
    // Verify battery (using api)
    // Print status
    // Status log
    // Actual voltage will be determined later on based on what the acceptable range of values are given by the ECSE Power Team
    battery_status = true;
    emitLog("Battery check completed");
}

void sensor_check_entry()
{
    // Try initialization the sensors (altimeter, imu, temperature)
    int counter = 0;
    while (counter++ < MAX_SENSOR_RETRIES)
    {
        sensors.begin();
        if (sensors.status.imu1 && sensors.status.imu2 && sensors.status.imu3 && sensors.status.pressure && sensors.status.temperature) {
            // Start collecting data from sensors
            sensors.start();
            emitLog("Sensor check completed");
            return;
        } else {
            // Failed, try again
            threads.delay(100);
        }
    }
    emitLog("Sensor check max retries reached");
}

void fault_entry()
{
    // error needs to be shown on serial and sd card
    //  Fault logging (maybe do that in the transition too for specific faults)

    // Stop all motors
    // Stop all sensors
    // Print status
    // Status log
    emitLog("Fault detected, aborting mission");
    
    // Fault beep
    // tone(uint8_t pin, uint16_t frequency, uint32_t duration)
    tone(SPEAKER_PIN, 1500, FAULT_TIMEOUT_DELAY);
    threads.delay(FAULT_TIMEOUT_DELAY);
    noTone(SPEAKER_PIN);
}

void termination_entry()
{
    // Print status
    stop_all_threads();
    // Status log
    emitLog("Termination completed");
}

void ready_entry()
{
    // Print status
    start_all_threads();
    emitLog("Ready state reached");
}

void ready_do() //updates the vertical speed 
{
    // Compute altitude ascended using pressure sensor
    // Status log with data
    if (prev_altitude == 0) prev_altitude = sensors.getAltitude(); // Initialize prev_time if not set
    if (last_time == 0) last_time = millis(); // Initialize last_time if not set

    if (last_time <= millis() - 5000)  // Calculate vertical speed every 5 seconds
    {
        // Calculate vertical speed
        float current_altitude = sensors.getAltitude();
        vertical_speed = (current_altitude - prev_altitude) / (millis() - last_time);
        emitLog("Ready state, Vertical speed: %f", vertical_speed, "current altitude: %f", current_altitude);
        prev_altitude = current_altitude;
        last_time = millis();  
    }
}

void ascent_do()
{
    // Status log
    float currentaltitude = sensors.getAltitude();
    emitLog("Ascent state, current altitude: %f", currentaltitude);
}

void stabilization_entry() {
    Serial.println("DEBUG: stabilization_entry - START");
    emitLog("*** ENTERING DIRECT STABILIZATION MODE ***");

    // Initialize SD card for logging
    Serial.println("DEBUG: stabilization_entry - Initializing SD card...");
    if (!SD.begin(BUILTIN_SDCARD)) {
        emitLog("WARNING: SD card failed, continuing without logging");
    } else {
        emitLog("SD card initialized");
    }
    Serial.println("DEBUG: stabilization_entry - SD card init DONE.");

    // Initialize the RL model
    Serial.println("DEBUG: stabilization_entry - Initializing TFLite model...");
    if (!modelInit(hab_model_tflite, tensor_arena, kTensorArenaSize)) {
        emitLog("ERROR: Model initialization failed!");
        Serial.println("DEBUG: stabilization_entry - TFLite model init FAILED.");
        aborted = true;
        return;
    }
    emitLog("Model initialization complete");
    Serial.println("DEBUG: stabilization_entry - TFLite model init DONE.");

    // Call this from stabilization_entry after model initialization
    if (testModelInferenceWithDummyData()) {
        Serial.println("Model test passed");
    } else {
        Serial.println("Model test failed - may crash in main loop");
    }

    // Initialize propulsion system
    Serial.println("DEBUG: stabilization_entry - Initializing propulsion...");
    if (!propulsion.init()) {
        emitLog("ERROR: Propulsion initialization failed!");
        Serial.println("DEBUG: stabilization_entry - Propulsion init FAILED.");
        aborted = true;
        return;
    }
    emitLog("Calibrating ESCs...");
    Serial.println("DEBUG: stabilization_entry - Calibrating ESCs...");
    propulsion.calibrate();
    emitLog("Propulsion system ready");
    Serial.println("DEBUG: stabilization_entry - Propulsion ready.");

    // Initialize sensors
    Serial.println("DEBUG: stabilization_entry - Initializing sensors...");
    sensors.begin();
    if (!(sensors.status.imu1 || sensors.status.imu2 || sensors.status.imu3)) {
        emitLog("WARNING: No IMUs available, using simulated data");
    } else {
        emitLog("At least one IMU available");
    }
    sensors.start();
    Serial.println("DEBUG: stabilization_entry - Sensors initialized and started.");

    // Reset model state variables
    prevAction0 = 0.0f;
    prevAction1 = 0.0f;
    actionDiff0 = 0.0f;
    actionDiff1 = 0.0f;

    last_time = millis();
    emitLog("Stabilization initialization complete");

    // Start beep pattern to indicate stabilization mode
    tone(SPEAKER_PIN, 900, 200);
    threads.delay(300);
    tone(SPEAKER_PIN, 1200, 200);
    threads.delay(300);
    tone(SPEAKER_PIN, 1500, 200);
    Serial.println("DEBUG: stabilization_entry - END");

}

void stabilization_do()
{
    // Serial.println("DEBUG: stabilization_do - START");
    static unsigned long debugPrintTime = 0;

    if (last_time == 0) {
        last_time = millis(); 
        emitLog("Starting stabilization phase");
        Serial.println("DEBUG: stabilization_do - Initialized last_time");
    }

    // Get sensor readings and normalize them
    sensors.getFusedWorldLinearAcceleration(sensor_x_accel, sensor_y_accel, sensor_z_accel);
    sensors.getRelativeVelocity(sensor_vx, sensor_vy, sensor_vz);
    sensors.getRelativePosition(sensor_px, sensor_py, sensor_pz);
    sensors.getFusedOrientation(sensor_yaw, sensor_pitch, sensor_roll);
    sensors.getFusedWorldAngularVelocity(sensor_x_angular_vel, sensor_y_angular_vel, sensor_z_angular_vel);

    // Before setting model inputs, add this validation
    bool hasInvalidInputs = false;
    // Check each input for NaN or Inf values
    if (isnan(sensor_x_accel) || isinf(sensor_x_accel)) {
        Serial.println("WARNING: Invalid xWorldLinearAcceleration");
        hasInvalidInputs = true;
        sensor_x_accel = 0; // Provide safe value
    }

    // Add similar checks for all other inputs

    // If any inputs are invalid, log it
    if (hasInvalidInputs) {
        Serial.println("WARNING: Some sensor inputs were invalid and replaced with safe values");
    }

    // Normalize data
    sensor_x_accel /= 10.0f;
    sensor_y_accel /= 10.0f;
    sensor_vx /= 10.0f;
    sensor_vy /= 10.0f;
    sensor_px /= 10.0f;
    sensor_py /= 10.0f;

    // Convert degrees to radians and normalize
    sensor_roll = (sensor_roll * M_PI / 180.0f) / 6.28f;
    sensor_pitch = (sensor_pitch * M_PI / 180.0f) / 6.28f;
    sensor_yaw = (sensor_yaw * M_PI / 180.0f) / 6.28f;

    sensor_x_angular_vel = (sensor_x_angular_vel * M_PI / 180.0f) / 6.28f;
    sensor_y_angular_vel = (sensor_y_angular_vel * M_PI / 180.0f) / 6.28f;
    sensor_z_angular_vel = (sensor_z_angular_vel * M_PI / 180.0f) / 6.28f;

    // Set model inputs
    modelSetInput(sensor_x_accel, IDX_X_ACCEL);
    modelSetInput(sensor_y_accel, IDX_Y_ACCEL);
    modelSetInput(sensor_vx, IDX_X_VEL);
    modelSetInput(sensor_vy, IDX_Y_VEL);
    modelSetInput(sensor_px, IDX_X_POS);
    modelSetInput(sensor_py, IDX_Y_POS);
    modelSetInput(sensor_roll, IDX_ROLL);
    modelSetInput(sensor_pitch, IDX_PITCH);
    modelSetInput(sensor_yaw, IDX_YAW);
    modelSetInput(sensor_x_angular_vel, IDX_X_ANG_VEL);
    modelSetInput(sensor_y_angular_vel, IDX_Y_ANG_VEL);
    modelSetInput(sensor_z_angular_vel, IDX_Z_ANG_VEL);
    modelSetInput(prevAction0, IDX_PREV_ACTION_0);
    modelSetInput(prevAction1, IDX_PREV_ACTION_1);
    modelSetInput(actionDiff0, IDX_ACTION_DIFF_0);
    modelSetInput(actionDiff1, IDX_ACTION_DIFF_1);
    // Serial.println("DEBUG: stabilization_do - Model inputs set.");
    // extern char _heap_end; // Stack end
    // extern char *__brkval; // Heap position
    // char stack_dummy = 0;  // Stack pointer reference
    // Serial.printf("MEMORY: Free RAM=%d bytes\n", &stack_dummy - __brkval);
    // Serial.flush();
    // Serial.println("DEBUG: stabilization_do - Model inputs set.");
    // Serial.flush(); // Ensure this message is sent before potential crash

    // // Run inference
    // modelMutex.lock();
    // Serial.println("DEBUG: stabilization_do - Attempting modelRunInference().");
    // Serial.flush(); // Ensure this message is sent
    // if (!modelRunInference()) {
    //     emitLog("Model inference failed!");
    //     Serial.println("DEBUG: stabilization_do - modelRunInference() FAILED.");
    //     aborted = true;
    //     modelMutex.unlock();
    //     return;
    // }
    // modelMutex.unlock();
    // Serial.println("DEBUG: stabilization_do - modelRunInference() SUCCEEDED.");


    // Get model outputs and apply actions
    float action0 = safeSigmoid(modelGetOutput(0));
    float action1 = safeSigmoid(modelGetOutput(1));

    // Apply throttle (scaling to 0-20% as in test_model.cpp)
    float leftThrottle = action0 * 0.20 * 100.0;
    float rightThrottle = action1 * 0.20 * 100.0;
    propulsion.setLeftThrottle(leftThrottle);
    propulsion.setRightThrottle(rightThrottle);

    // Update tracking variables
    actionDiff0 = prevAction0 - action0;
    actionDiff1 = prevAction1 - action1;
    prevAction0 = action0;
    prevAction1 = action1;

    // Enhanced motor control debug output (print every 500ms to avoid flooding terminal)
    if (millis() - debugPrintTime > 500) {
        debugPrintTime = millis();

        // Original orientation values in degrees for better readability
        float roll_deg = sensor_roll * 6.28f * 180.0f / M_PI;
        float pitch_deg = sensor_pitch * 6.28f * 180.0f / M_PI;
        float yaw_deg = sensor_yaw * 6.28f * 180.0f / M_PI;

        // Print debug information to Serial
        Serial.println("=== STABILIZATION DEBUG ===");
        Serial.printf("Time: %lu ms\n", millis());
        Serial.printf("MOTORS: Left=%.2f%%, Right=%.2f%%\n", leftThrottle, rightThrottle);
        Serial.printf("ORIENTATION: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n", 
                     roll_deg, pitch_deg, yaw_deg);
        Serial.printf("ACCEL: X=%.2f, Y=%.2f, Z=%.2f\n", 
                     sensor_x_accel*10.0f, sensor_y_accel*10.0f, sensor_z_accel*10.0f);
        Serial.printf("VELOCITY: X=%.2f, Y=%.2f, Z=%.2f\n", sensor_vx*10.0f, sensor_vy*10.0f, sensor_vz);
        Serial.printf("ANGULAR VELOCITY: X=%.2f, Y=%.2f, Z=%.2f\n", 
                     sensor_x_angular_vel*6.28f*180.0f/M_PI, 
                     sensor_y_angular_vel*6.28f*180.0f/M_PI, 
                     sensor_z_angular_vel*6.28f*180.0f/M_PI);
        Serial.printf("MODEL: Action0=%.4f, Action1=%.4f\n", action0, action1);
        Serial.printf("DIFF: Action0Diff=%.4f, Action1Diff=%.4f\n", actionDiff0, actionDiff1);
        Serial.println("==========================");
    }

    // Log stabilization data
    emitLog("Stabilization: Motors[L:%.2f%%, R:%.2f%%], Orientation[R:%.2f,P:%.2f,Y:%.2f]",
            leftThrottle, rightThrottle,
            sensor_roll * 6.28f * 180.0f / M_PI,
            sensor_pitch * 6.28f * 180.0f / M_PI,
            sensor_yaw * 6.28f * 180.0f / M_PI);
    // Serial.println("DEBUG: stabilization_do - END");
}

void descent_entry()
{
    // Stop all motors
    propulsion.stopMotors();
    
    // Release payload (if applicable)
    // [payload release code would go here]
    
    // Status log
    emitLog("Descent state - motors stopped");
}

void descent_do() 
{
    if (prev_altitude == 0) prev_altitude = sensors.getAltitude(); // Initialize prev_time if not set
    if (last_time == 0) last_time = millis(); // Initialize last_time if not set
    
    if (last_time <= millis() - 5000)  // Calculate vertical speed every 5 seconds
    {
        // Calculate vertical speed
        float current_altitude = sensors.getAltitude();
        vertical_speed = (current_altitude - prev_altitude) / (millis() - last_time);
        emitLog("Descent State, Vertical speed: %f", vertical_speed, "current altitude: %f", current_altitude);
        prev_altitude = current_altitude;
        last_time = millis();  
    }
}

bool is_initialized()
{
    return initialized;
}

bool has_telemetry()
{
    // Check if telemetry and gps is available
    return telemetry_status && gps_status;
}

bool has_battery()
{
    // Check if battery is available
    // Battery voltage will be extracted from the receivedData and checked against specification provided by the ECSE Power Team
    return battery_status;
}

bool has_sensors()
{
    // Check if the minimum number of sensors are available
    // Give a warning if non critical sensors are missing    
    int workingIMUs = 0;  //want a minimum of 2 working IMUs
    if (sensors.status.imu1) workingIMUs++;
    if (sensors.status.imu2) workingIMUs++;
    if (sensors.status.imu3) workingIMUs++;

    if (sensors.status.pressure > 0 && sensors.status.temperature > 0 && workingIMUs >= 2)
    { // all sensors are working
        emitLog("%d IMUs are working, all other sensors are working", workingIMUs);
        return true;
    }
    else if (sensors.status.pressure > 0 && sensors.status.temperature == 0 && workingIMUs >= 2)
    { // temp sensor is the only noncritical sensor
        emitLog("Temperature sensor missing, this is a non-critical sensor. %d IMUs are working and altimeter working", workingIMUs);
        return true;
    }
    return false;
}

bool is_ascending() //check if it has started ascending by checking vertical speed
{
    if (vertical_speed > 0.5)
    {
        emitLog("TRANSITION(Ready->Ascent), Vertical speed: %f", vertical_speed, "current altitude: %f", prev_altitude);
        return true;
    }
    return false;
}

bool can_stabilize()
{
    // Check if the HAB has reached the desired altitude
    // Check if the HAB is ready to stabilize
    if (sensors.getAltitude() >= TARGET_STABILIZATION_ALTITUDE)
    {
        emitLog("reached %dkm altitude, entering stabilization state", TARGET_STABILIZATION_ALTITUDE / 1000);
        return true; // 20km altitude
    }
    return false;
}

bool is_aborted()
{
    // Check if the HAB has aborted
    // If aborted command was extracted, it will return it, else it stays false
    aborted = aborted || has_failed_telemetry() || has_failed_battery_check() || has_failed_sensor_check();
    return aborted;
}

bool stabilization_timeout()
{
    // Check if the stabilization has timed out
    if (millis() - last_time > TARGET_STABILIZATION_DURATION)
    {
        return true;
    }
    return false;
}

bool has_landed()
{
    // Check if the HAB has landed
    float altitude = sensors.getAltitude();
    if (vertical_speed < 0.5 && altitude < 5000)
    {
        // Touchdown
        emitLog("HAB has landed, entering termination state");
        return true;
    }
    return false;
}

bool has_fault_timeout() //transition btw fault and termination
{
    // Check if fault has timed out
    return stabilization_timeout();
}

// Function based on the Datalogging example under SD in Examples
void data_collection_thread()
{
    while (true) {
        processLogQueue();
        threads.yield();
    }
}

void telemetry_thread()
{
    while (true)
    {   
        processTelemetryQueue();
        
        // Receive telemetry data
        PowerBoardData tempData;
        if (serialComm.receiveData(tempData))
        {
            receivedData = tempData;
            emitLog("RX Telemetry Data Received: Time=%u, Voltage=%.2f, Latitude=%.6f, Longitude=%.6f, Abort=%d, Status=%s",
                receivedData.transponderTimestamp,
                receivedData.batteryVoltage,
                receivedData.latitude,
                receivedData.longitude,
                receivedData.abortCommand,
                receivedData.statusMsg);
            
            aborted = aborted || receivedData.abortCommand;
        }
        threads.yield();
    }
}

void model_thread()
{
    Serial.println("DEBUG: model_thread - START");
    while (true)
    {
        modelMutex.lock();
        // Run the model inference
        if (!modelRunInference())
        {
            emitLog("Model inference failed!");
            aborted = true;
            modelMutex.unlock();
            return;
        }
        modelMutex.unlock();
        threads.yield();
    }
}

void stop_all_threads()
{
    if (telemetry_thread_id != -1)
    {
        threads.kill(telemetry_thread_id);
        telemetry_thread_id = -1;
    }
    if (data_thread_id != -1)
    {
        threads.kill(data_thread_id);
        data_thread_id = -1;
    }
}

void start_all_threads()
{
    stop_all_threads();
    // Use 0 instead of NULL for the integer argument
    telemetry_thread_id = threads.addThread(telemetry_thread, 0, sizeof(telemetry_thread_stack), telemetry_thread_stack);
    data_thread_id = threads.addThread(data_collection_thread, 0, sizeof(data_thread_stack), data_thread_stack);
}

bool has_failed_initialization()
{
    return !is_initialized();
}

bool has_failed_telemetry()
{
    return !has_telemetry();
}

bool has_failed_battery_check()
{
    return !has_battery();
}

bool has_failed_sensor_check()
{
    return !has_sensors();
}

void setup()
{
    Serial.begin(COMM_BAUD_RATE);

    // Flight state machine
    flight_fsm.addState(INITIALIZATION, initialization_entry, nullptr, nullptr);
    flight_fsm.addState(TELEMETRY_CHECK, telemetry_check_entry, nullptr, nullptr);
    flight_fsm.addState(BATTERY_CHECK, battery_check_entry, nullptr, nullptr);
    flight_fsm.addState(SENSOR_CHECK, sensor_check_entry, nullptr, nullptr);
    flight_fsm.addState(FAULT, fault_entry, nullptr, nullptr);
    flight_fsm.addState(TERMINATION, termination_entry, nullptr, nullptr);
    flight_fsm.addState(READY, ready_entry, ready_do, nullptr);
    flight_fsm.addState(ASCENT, nullptr, ascent_do, nullptr);
    // flight_fsm.addState(STABILIZATION, nullptr, stabilization_do, nullptr);
    flight_fsm.addState(STABILIZATION, stabilization_entry, stabilization_do, nullptr);

    // Begin threads before starting initial state
    data_thread_id = threads.addThread(data_collection_thread, 0, sizeof(data_thread_stack), data_thread_stack);
    model_thread_id = threads.addThread(model_thread, 0, sizeof(model_thread_stack), model_thread_stack);

    // flight_fsm.setInitialState(INITIALIZATION);
    flight_fsm.setInitialState(STABILIZATION); // Set the initial state to STABILIZATION

    // Add transitions
    // flight_fsm.addTransition(INITIALIZATION, TELEMETRY_CHECK, is_initialized);
    // flight_fsm.addTransition(INITIALIZATION, FAULT, has_failed_initialization);
    // flight_fsm.addTransition(TELEMETRY_CHECK, BATTERY_CHECK, has_telemetry);
    // flight_fsm.addTransition(TELEMETRY_CHECK, FAULT, has_failed_telemetry);
    // flight_fsm.addTransition(BATTERY_CHECK, SENSOR_CHECK, has_battery);
    // flight_fsm.addTransition(BATTERY_CHECK, FAULT, has_failed_battery_check);
    // flight_fsm.addTransition(SENSOR_CHECK, READY, has_sensors);
    // flight_fsm.addTransition(SENSOR_CHECK, FAULT, has_failed_sensor_check);
    // flight_fsm.addTransition(READY, ASCENT, is_ascending);
    // flight_fsm.addTransition(ASCENT, STABILIZATION, can_stabilize);
    // flight_fsm.addTransition(ASCENT, DESCENT, is_aborted);
    // flight_fsm.addTransition(STABILIZATION, DESCENT, stabilization_timeout);
    // flight_fsm.addTransition(STABILIZATION, DESCENT, is_aborted);
    // flight_fsm.addTransition(DESCENT, TERMINATION, has_landed);
    // flight_fsm.addTransition(FAULT, TERMINATION, has_fault_timeout);

    flight_fsm.addTransition(INITIALIZATION, STABILIZATION, is_initialized);
    // Start data collection thread


    Serial.println("Flight computer initialized in STABILIZATION test mode");
    Serial.println("Motor control data will be printed every 500ms");

    last_time = millis();
}

void loop()
{
    flight_fsm.update();
    threads.yield();
}

// Add this function and call it from stabilization_entry to test just the model
bool testModelInferenceWithDummyData() {
    // Set simple values for all inputs
    for (int i = 0; i < 16; i++) {
        modelSetInput(0.1f, i);
    }
    
    Serial.println("TEST: Running inference with dummy data");
    bool result = modelRunInference();
    if (result) {
        Serial.println("TEST: Inference succeeded!");
    } else {
        Serial.println("TEST: Inference failed!");
    }
    return result;
}
