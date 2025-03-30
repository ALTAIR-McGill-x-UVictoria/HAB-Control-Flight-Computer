#include <Arduino.h>
#include <TeensyThreads.h>
#include <SPI.h>
#include <SD.h>
#include "StateMachine.h"
#include "Sensors.h"
#include "Logging.h"

#define MAX_SENSOR_RETRIES 20       // Maximum number of retries for sensor initialization
#define FAULT_TIMEOUT_DELAY 2000    // Delay for the fault state before triggering the timeout
#define SPEAKER_PIN 33
#define COMM_BAUD_RATE 115200       // Default baud rate set to 115200 as per testing specifications performed
#define TARGET_STABILIZATION_ALTITUDE 20000 // Target altitude for stabilization in meters
#define TARGET_STABILIZATION_DURATION 30000 // Target duration for stabilization in milliseconds

Sensors sensors;
PowerBoardData receivedData;
SerialCommunication serialComm = SerialCommunication(Serial1, BoardType::CONTROL_BOARD);

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

bool telemetry_status;                      // Telemetry status flag
bool gps_status;                            // GPS status flag
bool battery_status;                        // Battery status flag

bool initialized;                           // Initialization status flag
bool aborted;                               // Mission abortion status flag
unsigned long last_time;                    // Used for timeouts and delays
float prev_altitude;                        // Last altitude for vertical speed calculations
float vertical_speed;                       // Rate of ascent of the HAB

StateMachine<10, 15> flight_fsm;
int telemetry_thread_id = -1;
int data_thread_id = -1;

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

    // TODO: Init libs and models

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
    ControlBoardData txData;
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

void stabilization_do()
{
    last_time = millis(); // get current time at start of stabilization to count to 30sec
    //TODO:
    // set stability
    // Control algorithm
    // Output to motors
    // Status log
    emitLog("Stabilization state");
}

void descent_entry()
{
    //TODO:
    // Stop all motors
    // Release payload
    // Status log
    emitLog("Descent state");
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
    else if (sensors.status.pressure > 0 && sensors.status.temperature < 0 && workingIMUs >= 2)
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
    telemetry_thread_id = threads.addThread(telemetry_thread);
    data_thread_id = threads.addThread(data_collection_thread);
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
    flight_fsm.addState(STABILIZATION, nullptr, stabilization_do, nullptr);
    flight_fsm.addState(DESCENT, descent_entry, descent_do, nullptr);

    flight_fsm.setInitialState(INITIALIZATION);

    // Add transitions
    flight_fsm.addTransition(INITIALIZATION, TELEMETRY_CHECK, is_initialized);
    flight_fsm.addTransition(INITIALIZATION, FAULT, has_failed_initialization);
    flight_fsm.addTransition(TELEMETRY_CHECK, BATTERY_CHECK, has_telemetry);
    flight_fsm.addTransition(TELEMETRY_CHECK, FAULT, has_failed_telemetry);
    flight_fsm.addTransition(BATTERY_CHECK, SENSOR_CHECK, has_battery);
    flight_fsm.addTransition(BATTERY_CHECK, FAULT, has_failed_battery_check);
    flight_fsm.addTransition(SENSOR_CHECK, READY, has_sensors);
    flight_fsm.addTransition(SENSOR_CHECK, FAULT, has_failed_sensor_check);
    flight_fsm.addTransition(READY, ASCENT, is_ascending);
    flight_fsm.addTransition(ASCENT, STABILIZATION, can_stabilize);
    flight_fsm.addTransition(ASCENT, DESCENT, is_aborted);
    flight_fsm.addTransition(STABILIZATION, DESCENT, stabilization_timeout);
    flight_fsm.addTransition(STABILIZATION, DESCENT, is_aborted);
    flight_fsm.addTransition(DESCENT, TERMINATION, has_landed);
    flight_fsm.addTransition(FAULT, TERMINATION, has_fault_timeout);

    last_time = millis();
}

void loop()
{
    flight_fsm.update();
    threads.yield();
}
