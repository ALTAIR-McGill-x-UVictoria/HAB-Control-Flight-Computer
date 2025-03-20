#include <Arduino.h>
#include <TeensyThreads.h>
#include "StateMachine.h"
#include "Sensors.h"
#include <SPI.h>
#include <SD.h>
#include "Logging.h"

#define MAX_SENSOR_RETRIES 20       // Maximum number of retries for sensor initialization
#define FAULT_TIMEOUT_DELAY 2000    // Delay for the fault state before triggering the timeout
#define SPEAKER_PIN 33

Sensors sensors;
SerialCommunication serialComm = SerialCommunication(Serial1, BoardType::CONTROL_BOARD);
LogQueue<ControlBoardData> telemetryTransmitQueue; // Changed to proper type
LogQueue<StringMessage> sdCardLoggingQueue;

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

bool initialized;                           // Initialization status flag
bool aborted;                               // Mission abortion status flag
unsigned long last_time;                    // Used for timeouts and delays
char telemetry_status;                      // Telemetry status flag
char gps_status;                            // GPS status flag
char battery_status;                        // Battery status flag
float prev_altitude;                        // Last altitude for vertical speed calculations
float vertical_speed;                       // Rate of descent of the HAB

StateMachine<10, 15> flight_fsm;
int telemetry_thread_id = -1;
int data_thread_id = -1;

void initialization_entry()
{
    // Init data storage
    Serial.print("Initializing SD card... ");
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("Card failed, or not present.");
        initialized = false;
        return;
    }
    Serial.println("card initialized.");

    // TODO: Init libs and models

    // Activation beep
    // tone(uint8_t pin, uint16_t frequency, uint32_t duration)
    tone(SPEAKER_PIN, 700, 500);
    threads.delay(500);
    noTone(SPEAKER_PIN);

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
        rxData = tempData;
        emitLog("Telemetry data received: %s", rxData.statusMsg);
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

    // Status log
    emitLog("Telemetry check completed");
}

void battery_check_entry()
{
    // Verify battery (using api)
    // Print status
    // Status log
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
            break;
        } else {
            // Failed, try again
            threads.delay(100);
        }
    }
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
    aborted = true;
    
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
}

void ready_do()
{
    // Compute altitude ascended using pressure sensor
    // Status log with data
    emitLog("Ready state reached");
}

void ascent_do()
{
    // Status log

    
    emitLog("Ascent state reached");
}

void stabilization_do()
{
    last_time = millis(); // get current time at start of stabilization to count to 30sec

    // set stability

    // Control algorithm
    // Output to motors
    // Status log
    emitLog("Stabilization state reached");
}

void descent_entry()
{
    // Stop all motors
    // Release payload
    // Status log
    emitLog("Descent state reached");
}

void descent_do()
{
    // Check if the HAB has landed
    //calculate the rate of descent
    // Status log with data

    float current_altitude = sensors.getAltitude();
    if (last_time <= millis() - 5000)  // Calculate vertical speed every 5 seconds
    {
        // Calculate vertical speed
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

bool has_failed()
{
    // Print error message
    // Check all states for error
    // Main failure points: telemetry, gps, battery, altimeter, temperature,

    if (!initialized) { // If the initializ
        return;

        // Initialization state failed    }
    
    if (telemetry_status < 0)
    {
        Serial.println("Telemetry failed");
        return true;
    }
    if (gps_status < 0)
    {
        Serial.println("GPS failed");
        return true;
    }
    if (battery_status < 0)
    {
        Serial.println("Battery failed");
        return true;
    }
    if (!sensors.status.imu1)
    {
        Serial.println("IMU 1 failed");
        return true;
    }
    if (!sensors.status.imu2)
    {
        Serial.println("IMU 2 failed");
        return true;
    }
    if (!sensors.status.imu3)
    {
        Serial.println("IMU 3 failed");
        return true;
    }
    if (!sensors.status.pressure)
    {
        Serial.println("Altimeter failed");
        return true;
    }
    if (!sensors.status.temperature)
    {
        //TODO: logging (not printing): Serial.println("Temperature sensor failed");
        return true; //TODO: change all these returns to avoid early return in this case, we want to log all errors
    }
    // TODO: IMU check more elaborate due to having to check if at least 2 sensors are operational
    return false;
}
}

bool has_telemetry()
{
    // Check if telemetry and gps is available
    return telemetry_status > 0 && gps_status > 0;
}

bool has_battery()
{
    // Check if battery is available
    return battery_status > 0;
}

void telemetry_thread()
{
    while (true)
    {
        // Receive telemetry data
        PowerBoardData tempData;
        if (serialComm.receiveData(tempData))
        {
            // Process received data
            rxData = tempData;
        }
        
        // Dequeue telemetry data and send it directly
        ControlBoardData txData;
        if (telemetryTransmitQueue.dequeue(txData))
        {
            // Send telemetry data
            if (serialComm.sendData(txData))
            {
                Serial.println("Telemetry data sent successfully.");
            }
        }
        
        // In case the queue is empty
        threads.yield();
    }
}

bool has_sensors()
{
    // Check if the minimum number of sensors are available
    // Give a warning if non critical sensors are missing
    // char available_imus = imu_status[0] > 0 + imu_status[1] > 0 + imu_status[2] > 0;

    if (sensors.status.pressure > 0 && sensors.status.temperature > 0 && sensors.status.imu1 > 0 && sensors.status.imu2 > 0 && sensors.status.imu3 > 0)
    {
        emitLog("All sensors are working");
        return true;
    }
    else if (sensors.status.pressure > 0 && sensors.status.temperature < 0 && sensors.status.imu1 > 0 && sensors.status.imu2 > 0 && sensors.status.imu3 > 0)
    { // temp sensor is the only noncritical sensor
        // give a warning if temp sensor is missing
        emitLog("Temperature sensor is missing (non-critical sensor). All other sensors are working");
        return true;
    }
    else
        emitLog("Critical sensors are missing");
        return false;
}

bool is_ascending()
{
    // Check if the HAB has started ascending
    float current_altitude = sensors.getAltitude();
    if (last_time <= millis() - 5000)  // Calculate vertical speed every 5 seconds
    {
        // Calculate vertical speed
        vertical_speed = (current_altitude - prev_altitude) / (millis() - last_time);
        emitLog("TRANSITION(Ready->Ascent), Vertical speed: %f", vertical_speed, "current altitude: %f", current_altitude);
        prev_altitude = current_altitude;
        last_time = millis();  
    }
    if (vertical_speed > 0.5)
    {
        return true;
    } 
}

bool can_stabilize()
{
    // Check if the HAB has reached the desired altitude
    // Check if the HAB is ready to stabilize
    if (sensors.getAltitude() >= 20000)
    {
        return true; // 20km altitude
    }
    return false;
}

bool is_aborted()
{
    // Check if the HAB has aborted
    return aborted;
}

bool stabilization_timeout()
{
    // Check if the stabilization has timed out
    int timeout = 30000; // 30 seconds

    if (millis() - last_time > timeout)
    {
        return true;
    }
    return false;
}

bool has_landed()
{
    // Check if the HAB has landed
    float altitude = sensors.getAltitude();
    if (vertical_speed < 0.5 && altitude < 1000)
    {
        // Touchdown
        // TODO
        return true;
    }
    return false;
}

bool has_fault_timeout()
{
    // Check if fault has timed out
    return false;
}

// Function based on the Datalogging example under SD in Examples
void data_collection_thread()
{
    File dataFile;
    while (true) {
        dataFile = SD.open("datalog.txt", FILE_WRITE); // Open the status file
        if (!dataFile) {
            emitLog("Failed to open the file on the SD card");
        }

        while (dataFile)
        {
            StringMessage logMessage;
            if (sdCardLoggingQueue.dequeue(&logMessage))
            {
                // Dequeuing most recent data to print it on the SD card
                emitLog(logMessage.text);
                size_t count = dataFile.println(logMessage.text);
                if (count != strlen(logMessage.text)) {
                    dataFile.close();
                    dataFile = NULL;
                    emitLog("Failed to emit log to the SD card");
                    break;
                }
            }
            threads.yield();
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

void setup()
{
    Serial.begin(115200);

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
    flight_fsm.addTransition(INITIALIZATION, FAULT, has_failed); // TODO: split up the has_failed function to check for each state
    flight_fsm.addTransition(TELEMETRY_CHECK, BATTERY_CHECK, has_telemetry);
    flight_fsm.addTransition(TELEMETRY_CHECK, FAULT, has_failed);
    flight_fsm.addTransition(BATTERY_CHECK, SENSOR_CHECK, has_battery);
    flight_fsm.addTransition(BATTERY_CHECK, FAULT, has_failed);
    flight_fsm.addTransition(SENSOR_CHECK, READY, has_sensors);
    flight_fsm.addTransition(SENSOR_CHECK, FAULT, has_failed);
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