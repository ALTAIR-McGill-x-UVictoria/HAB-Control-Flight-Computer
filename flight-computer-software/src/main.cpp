#include <Arduino.h>
#include <TeensyThreads.h>
#include "StateMachine.h"
#include "Sensors.h"
#include "SerialCommunication.h"
#include "LogQueue.h"
#include <SD.h>
#include <SPI.h>

const int chipSelect = 10;  // SD card chip select pin

Sensors sensors;
SerialCommunication serialComm = SerialCommunication(Serial1, BoardType::CONTROL_BOARD);
LogQueue telemetryTransmitQueue = LogQueue();

// Data structures (Data to send and receive)
PowerBoardData rxData;

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

bool initialized;               // Initialization status flag
bool aborted;                   // Mission abortion status flag
int lastTime;                   // Used for timeouts and delays
char telemetry_status;          // Telemetry status flag
char gps_status;                // GPS status flag
char battery_status;            // Battery status flag
char altimeter_status;          // Altimeter status flag
char temperature_status;        // Temperature status flag
char imu_status[3];             // IMUs status flag
float altitude;                 // Altitude of the HAB
float initialAltitude;          // Initial altitude of the HAB
float stabilizationStartTime;   // Time to stabilize the HAB
char status_message[MAX_STATUS_MSG_LENGTH];       // Status message

StateMachine<10, 15> flight_fsm;
int telemetry_thread_id = -1;
int data_thread_id = -1;

void initialization_entry() {
    // Init data storage
    Serial.begin(9600);
    while (!Serial) {
    ; // wait for serial port to connect.
    }

    Serial.print("Initializing SD card...");
    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
        while (1) {
        // No SD card, so don't do anything more - stay stuck here
        }
    }
    Serial.println("card initialized.");
    
    
    
    // Init libs and models
    //TODO
    
    
    // Activation beep- PWM tone on pin33 of teensy
    tone(33, 700, 500); //tone(uint8_t pin, uint16_t frequency, uint32_t duration)
    delay(500);
    noTone(33); //stop playing a note on pin 33
}

void telemetry_check_entry() {
    // Init serial telemetry api
    serialComm.begin();
    // Verify connection
    int retry_counter = 0;
    while(retry_counter < 5) {
        // Status printing within the verifyConnection function
        if(serialComm.verifyConnection()) {
            telemetry_status = 1;
            break;
        } else {
            telemetry_status = 0;
            retry_counter++;
        }
    }
    // TODO: Verify GPS (using api)
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
    Serial.begin(115200);
    delay(100);
    SensorStatus status = {false, false, false, false, false};
    while (true)
    {
    status = sensors.begin(status);
    if (!status.imu1)
      Serial.println("Failed to initialize IMU 1.");
    if (!status.imu2)
      Serial.println("Failed to initialize IMU 2.");
    if (!status.imu3)
      Serial.println("Failed to initialize IMU 3.");
    if (!status.pressure)
      Serial.println("Failed to initialize Altimeter.");
    if (!status.temperature)
      Serial.println("Failed to initialize Temperature Probe.");
    if (status.imu1 && status.imu2 && status.imu3 && status.pressure && status.temperature)
    {
      Serial.println("Successfully initialized all sensors.");
      
      break;
    }
    else
    {
      Serial.println("Retrying sensor initialization.");
      delay(100);
    }
  }
  sensors.enableReports(20);
  sensors.startDataCollection();
  initialAltitude = sensors.getAltitude();
}

void fault_entry() {
    //error needs to be shown on serial and sd card
    // Fault logging (maybe do that in the transition too for specific faults)


    // Stop all motors
    // Stop all sensors
    // Print status
}

void fault_do() {
    // Fault beep
    tone(33, 1500, 500); //tone(uint8_t pin, uint16_t frequency, uint32_t duration)
    delay(500);
    noTone(33); //stop playing a note on pin 33
    
    // Wait for timeout
    int timeout = 2000; //CHANGE WHAT WE WANT TIMEOUT TO BE
    while (millis() - lastTime < timeout) {
        // Wait
    }
    // Abort mission
    aborted = true;


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
    altitude = sensors.getAltitude();
}

void ascent_do() {
    // Status log
}

void stabilization_do() {
    stabilizationStartTime = millis(); //get current time at start of stabilization
    
    //set stability
   

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
    //TODO

    // Compute if reached touchdown

    altitude = sensors.getAltitude();
    if (altitude <= initialAltitude + 10) {
        // Touchdown
        //TODO
    }
}

bool is_initialized() {
    return initialized;
}

bool has_failed() {
    // Print error message
    // Check all states for error
    // Main failure points: telemetry, gps, battery, altimeter, temperature,
    if (telemetry_status < 0) {
        Serial.println("Telemetry failed");
        return true;
    }
    if (gps_status < 0) {
        Serial.println("GPS failed");
        return true;
    }
    if (battery_status < 0) {
        Serial.println("Battery failed");
        return true;
    }
    if (altimeter_status < 0) {
        Serial.println("Altimeter failed");
        return true;
    }
    if (temperature_status < 0) {
        Serial.println("Temperature failed");
        return true;
    }

    // IMU check more elaborate due to having to check if at least 2 sensors are operational
    // TODO
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

void populate_telemetry_data() {
    // Populate txData with telemetry data from sensors class
    // To be called at the end of the states that perform logging as well
    float xLinearAcceleration, yLinearAcceleration, zLinearAcceleration;
    sensors.getFusedLinearAcceleration(xLinearAcceleration, yLinearAcceleration, zLinearAcceleration);
    float xAngularVelocity, yAngularVelocity, zAngularVelocity;
    sensors.getFusedAngularVelocity(xAngularVelocity, yAngularVelocity, zAngularVelocity);
    float yawOrientation, pitchOrientation, rollOrientation;
    sensors.getFusedOrientation(yawOrientation, pitchOrientation, rollOrientation);
    ControlBoardData txData;
    txData = {
        .timestamp = millis(),
        .pressure = sensors.getPressure(),
        .altitude = sensors.getAltitude(),
        .temperature = sensors.getTemperature(),
        .accelX = xLinearAcceleration,
        .accelY = yLinearAcceleration,
        .accelZ = zLinearAcceleration,
        .angularVelocityX = xAngularVelocity,
        .angularVelocityY = yAngularVelocity,
        .angularVelocityZ = zAngularVelocity,
        .orientationYaw = yawOrientation,
        .orientationPitch = pitchOrientation,
        .orientationRoll = rollOrientation,
        .statusMsgLength = strlen(status_message),
    };
    strcpy(rxData.statusMsg, status_message); 
    // Add telemetry data to queue for transmission by converting it to a char array
    char telemetryData[sizeof(ControlBoardData)];
    memcpy(telemetryData, &txData, sizeof(ControlBoardData));
    telemetryTransmitQueue.enqueue(telemetryData);
}

void telemetry_thread() {
    while(true) {
        // Receive telemetry data
        PowerBoardData tempData;
        if (serialComm.receiveData(tempData)) {
            // Process received data
            rxData = tempData;
        }
        // dequeue data from the queue, convert it to the struct, and send it
        char telemetryData[sizeof(ControlBoardData)];
        if (telemetryTransmitQueue.dequeue(telemetryData, sizeof(telemetryData))) {
            ControlBoardData txData;
            memcpy(&txData, telemetryData, sizeof(ControlBoardData));
            // Send telemetry data
            if (serialComm.sendData(txData)) {
                Serial.println("Telemetry data sent successfully.");
            }
        }
        // In case the queue is empty
        threads.yield();
    }
}

bool has_sensors() {
    // Check if the minimum number of sensors are available
    // Give a warning if non critical sensors are missing
    // char available_imus = imu_status[0] > 0 + imu_status[1] > 0 + imu_status[2] > 0;
    
    if (altimeter_status > 0 && temperature_status > 0 && imu_status[0] > 0 && imu_status[1] > 0 && imu_status[2] > 0) {
        return true;
    } else if (altimeter_status > 0 && temperature_status < 0 && imu_status[0] > 0 && imu_status[1] > 0 && imu_status[2] > 0){ //temp sensor is the only noncritical sensor
        //give a warning if temp sensor is missing
        Serial.println("Temperature sensor is missing. This is a non-critical sensor. All other sensors are working");
        return true;
    } else return false;
}

bool is_ascendeding() {
    // Check if the HAB has ascended to the desired altitude
    altitude = sensors.getAltitude();
    if (altitude >= 20000) {
        return true; // 20km altitude
    }
    return false;
}

bool can_stabilize() {
    // Check if the HAB has reached the desired altitude
    // Check if the HAB is ready to stabilize
    if (altitude >= 20000) {
        return true; // 20km altitude
    }

    return false;
}

bool is_aborted() {
    // Check if the HAB has aborted
    return aborted;
}

bool stabilization_timeout() {
    // Check if the stabilization has timed out
    int timeout = 30000; // 30 seconds
    
    if (millis() - stabilizationStartTime > timeout) {
        return true;
    }
    return false;
}

bool has_landed() {
    // Check if the HAB has landed
    altitude = sensors.getAltitude();
    if (altitude <= initialAltitude + 10) {
        // Touchdown
        //TODO
    }
    return false;
}

bool has_fault_timeout() {
    // Check if fault has timed out
    return false;
}

void data_collection_thread() { //this is from the Datalogging example under SD in Examples
    
  // make a string for assembling the data to log:
  String dataString = "";

  
  //LogQueue.Dequeue; //call function here to get data from queue

  // open the file.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening datalog.txt");
  }
  delay(100); // run at a reasonable not-too-fast speed
  
  
  
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