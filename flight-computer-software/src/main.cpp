// #include <Arduino.h>
// #include <TeensyThreads.h>
// #include <SPI.h>
// #include <SD.h>
// #include <ArduTFLite.h>
// #include "StateMachine.h"
// #include "Sensors.h"  // Include this only once
// #include "Logging.h"  // This already includes Sensors.h, so the above is technically redundant
// #include "hab_model.h"
// #include "Propulsion.h"
// #include <core_pins.h> // Required for DMAMEM
#include "utils.h"

// Function declarations

// Thread function declarations
void stop_all_threads();
void start_all_threads();
void model_thread();
void telemetry_thread();

// Debugging
void printCurrentData();

float throttle_percent = 0.20f;

int telemetry_thread_id = -1;
int data_thread_id = -1;
int model_thread_id = -1;

DMAMEM Threads::Mutex modelMutex;

// Add these declarations at file scope (with your other globals)
DMAMEM static uint8_t data_thread_stack[8192];
DMAMEM static uint8_t telemetry_thread_stack[8192];
DMAMEM static uint8_t model_thread_stack[12288]; // Larger for the model thread if needed

// Define the model state variables
static unsigned long debugPrintTime = 0;
static unsigned long lastModelRunTime = 0;
static float leftThrottle = 0.0f;
static float rightThrottle = 0.0f;
float action0 = 0.0f;
float action1 = 0.0f;


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

// Update the model thread to use try_lock
void model_thread()
{
    Serial.println("DEBUG: model_thread - START");
    while (true)
    {
        // Only run inference when new inputs are available
        if (model_inputs_updated) {
            bool mutexAcquired = modelMutex.try_lock();
            if (mutexAcquired) {
                // Run the model inference
                if (!modelRunInference()) {
                    emitLog("Model inference failed!");
                    aborted = true;
                    modelMutex.unlock();
                    return;
                }
                
                // Mark that new outputs are available and inputs have been processed
                model_inputs_updated = false;
                model_outputs_updated = true;
                last_model_run = millis();
                
                modelMutex.unlock();
                
                // Print confirmation every 2 seconds
                static unsigned long last_print = 0;
                if (millis() - last_print > 2000) {
                    Serial.println("MODEL: Inference successfully ran");
                    last_print = millis();
                }
            } else {
                // Debug message if mutex couldn't be acquired
                static unsigned long lastMutexWarnTime = 0;
                if (millis() - lastMutexWarnTime > 5000) {
                    Serial.println("WARNING: Model thread could not acquire mutex");
                    lastMutexWarnTime = millis();
                }
            }
        }
        
        // Yield to other threads with a short delay
        threads.delay(10); // Increased delay to give main loop more time
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

void start_all_threads()
{
    stop_all_threads();
    // Use 0 instead of NULL for the integer argument
    telemetry_thread_id = threads.addThread(telemetry_thread, 0, sizeof(telemetry_thread_stack), telemetry_thread_stack);
    data_thread_id = threads.addThread(data_collection_thread, 0, sizeof(data_thread_stack), data_thread_stack);
}



void setup()
{
    Serial.begin(COMM_BAUD_RATE);
    Serial.println("==== DIRECT RL MODEL & MOTOR CONTROL MODE ====");
    
    // Initialize SD card for logging
    SDsetup();
    
    // Initialize the RL model
    ModelSetup();
    // Test model with dummy data
    TestModelSetup();
    
    // Initialize propulsion system
    PropulsionSetup();
    
    // Calibrate ESCs
    ESCSetup();
    
    // Initialize sensors
    SensorSetup();
    
    
    
    // Start data thread for logging only
    data_thread_id = threads.addThread(data_collection_thread, 0, sizeof(data_thread_stack), data_thread_stack);
    
    // DO NOT start model thread - we'll run inference directly
    // model_thread_id = threads.addThread(model_thread, 0, sizeof(model_thread_stack), model_thread_stack);
    
    
    // Properly initialize the last model run time
    last_model_run = millis();


    SignalSetupComplete();
    Serial.println("System initialized! Starting direct motor control...");
}

// Modify the loop function to ensure proper mutex handling and thread safety
void loop()
{
    
    // Update sensor readings and run model at a controlled rate (10Hz is sufficient)
    if (millis() - lastModelRunTime >= 100) {
        lastModelRunTime = millis();
        
        // Get sensor readings
        sensors.getFusedWorldLinearAcceleration(sensor_x_accel, sensor_y_accel, sensor_z_accel);
        sensors.getRelativeVelocity(sensor_vx, sensor_vy, sensor_vz);
        sensors.getRelativePosition(sensor_px, sensor_py, sensor_pz);
        sensors.getFusedOrientation(sensor_yaw, sensor_pitch, sensor_roll);
        sensors.getFusedWorldAngularVelocity(sensor_x_angular_vel, sensor_y_angular_vel, sensor_z_angular_vel);
        
        // Get GPS data
        gps_lon = sensors.gpsData.longitude;
        gps_lat = sensors.gpsData.latitude;
        gps_alt = sensors.gpsData.altitude;
        gps_vx = sensors.gpsData.vx;
        gps_vy = sensors.gpsData.vy;
        gps_vz = sensors.gpsData.vz;

        // Get altimeter data
        baro_altitude = sensors.getAltitude();
        baro_pressure = sensors.getPressure();

        // Sanitize inputs (keep your existing code)
        if (isnan(sensor_x_accel) || isinf(sensor_x_accel)) sensor_x_accel = 0;
        // ...other sanity checks...
        
        // Normalize data
        sensor_x_accel = (sensor_x_accel / 10.0f);
        sensor_y_accel = (sensor_y_accel / 10.0f);
        sensor_vx = (sensor_vx / 10.0f);
        sensor_vy = (sensor_vy / 10.0f);
        sensor_px = (sensor_px / 10.0f);
        sensor_py = (sensor_py / 10.0f);
        
        // Convert degrees to radians and normalize WITHOUT adding noise
        sensor_roll = (sensor_roll * M_PI / 180.0f) / 6.28f;
        sensor_pitch = (sensor_pitch * M_PI / 180.0f) / 6.28f;
        sensor_yaw = (sensor_yaw * M_PI / 180.0f) / 6.28f;

        // // Add low-pass filtering using the existing FILTER_FACTOR from Sensors.h
        // static float filtered_roll_local = sensor_roll;
        // static float filtered_pitch_local = sensor_pitch;
        // static float filtered_yaw_local = sensor_yaw;

        // // Apply low-pass filter (slowly blend new readings with previous filtered values)
        // filtered_roll_local = filtered_roll_local * (1.0f - FILTER_FACTOR) + sensor_roll * FILTER_FACTOR;
        // filtered_pitch_local = filtered_pitch_local * (1.0f - FILTER_FACTOR) + sensor_pitch * FILTER_FACTOR;
        // filtered_yaw_local = filtered_yaw_local * (1.0f - FILTER_FACTOR) + sensor_yaw * FILTER_FACTOR;

        // // Use the filtered values for the model inputs
        // modelSetInput(filtered_roll_local, IDX_ROLL);
        // modelSetInput(filtered_pitch_local, IDX_PITCH);
        // modelSetInput(filtered_yaw_local, IDX_YAW);

        // // Store the filtered values back
        // sensor_roll = filtered_roll_local;
        // sensor_pitch = filtered_pitch_local;
        // sensor_yaw = filtered_yaw_local;
        
        sensor_x_angular_vel = ((sensor_x_angular_vel * M_PI / 180.0f) / 6.28f);
        sensor_y_angular_vel = ((sensor_y_angular_vel * M_PI / 180.0f) / 6.28f);
        sensor_z_angular_vel = ((sensor_z_angular_vel * M_PI / 180.0f) / 6.28f);
        
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
        
        // Run inference directly without threads
        // Serial.print("Running inference... ");
        if (modelRunInference()) {
            // Serial.println("Success!");
            last_model_run = millis();
            
            // Get model outputs
            action0 = safeSigmoid(modelGetOutput(0));
            action1 = safeSigmoid(modelGetOutput(1));
            
            
            leftThrottle = action0 * throttle_percent * 100.0;
            rightThrottle = action1 * throttle_percent * 100.0;
            propulsion.setLeftThrottle(leftThrottle);
            propulsion.setRightThrottle(rightThrottle);
            
            // Update tracking variables for next iteration
            actionDiff0 = action0 - prevAction0;
            actionDiff1 = action1 - prevAction1;
            prevAction0 = action0;
            prevAction1 = action1;
        } else {
            Serial.println("Failed!");
        }
    }
    
    // Debug output every 500ms
    if (millis() - debugPrintTime > 200) {
        debugPrintTime = millis();
        printCurrentData();
    }
}

void printCurrentData(){
    float roll_deg = sensor_roll * 6.28f * 180.0f / M_PI;
    float pitch_deg = sensor_pitch * 6.28f * 180.0f / M_PI;
    float yaw_deg = sensor_yaw * 6.28f * 180.0f / M_PI;
    float x_ang_vel_deg = sensor_x_angular_vel * 6.28f * 180.0f / M_PI;
    float y_ang_vel_deg = sensor_y_angular_vel * 6.28f * 180.0f / M_PI;
    float z_ang_vel_deg = sensor_z_angular_vel * 6.28f * 180.0f / M_PI;
    
    // GPS data
    

    Serial.println("=== MOTOR CONTROL STATUS ===");
    Serial.printf("Time: %lu ms, Last model run: %lu ms ago\n", 
                    millis(), millis() - last_model_run);
    
    // IMU Data Section
    Serial.println("--- IMU SENSOR DATA ---");
    Serial.printf("ORIENTATION: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n", 
                    roll_deg, pitch_deg, yaw_deg);
    Serial.printf("ANGULAR VEL: X=%.2f°/s, Y=%.2f°/s, Z=%.2f°/s\n", 
                    x_ang_vel_deg, y_ang_vel_deg, z_ang_vel_deg);
    Serial.printf("ACCELERATION: X=%.2fm/s², Y=%.2fm/s², Z=%.2fm/s²\n", 
                    sensor_x_accel*10.0f, sensor_y_accel*10.0f, sensor_z_accel*10.0f);
    Serial.printf("VELOCITY: X=%.2fm/s, Y=%.2fm/s, Z=%.2fm/s\n", 
                    sensor_vx*10.0f, sensor_vy*10.0f, sensor_vz*10.0f);
    Serial.printf("POSITION: X=%.2fm, Y=%.2fm, Z=%.2fm\n", 
                    sensor_px*10.0f, sensor_py*10.0f, sensor_pz*10.0f);
    
    // Altitude comparison
    Serial.printf("GPS altitude: %.2f, Baro altitude: %.2f, Diff: %.2f\n", gps_alt, baro_altitude, gps_alt - baro_altitude);

    // IMU Status (if available)
    Serial.printf("IMU STATUS: IMU1=%s, IMU2=%s, IMU3=%s\n",
                    sensors.status.imu1 ? "OK" : "FAIL",
                    sensors.status.imu2 ? "OK" : "FAIL", 
                    sensors.status.imu3 ? "OK" : "FAIL");

    // Control Output Section
    Serial.println("--- CONTROL OUTPUTS ---");
    Serial.printf("MOTORS: Left=%.2f%%, Right=%.2f%%\n", 
                    leftThrottle, rightThrottle);
    Serial.printf("MODEL: Action0=%.4f, Action1=%.4f\n", prevAction0, prevAction1);
    Serial.printf("DIFF: Action0Diff=%.4f, Action1Diff=%.4f\n", actionDiff0, actionDiff1);
    Serial.println("==========================");
}
