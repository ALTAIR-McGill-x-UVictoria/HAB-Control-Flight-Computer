/*
 * Test script for Propulsion.h on Teensy 4.1
 * This program allows control of ESCs/motors through serial commands
 */

#include "../../include/Propulsion.h"

// Create a Propulsion object
Propulsion propulsion;

// Command buffer
String inputCommand = "";
bool commandComplete = false;
bool systemInitialized = false;

// Function declarations
void processCommand(String command);

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Wait for serial port to connect or timeout after 3 seconds
    }

    Serial.println("\n=== Propulsion System Test ===");
    Serial.println("Initializing propulsion system...");
    
    // Initialize propulsion system
    if (propulsion.init()) {
        systemInitialized = true;
        Serial.println("ESCs ready");
    } else {
        Serial.println("ERROR: Failed to initialize propulsion system!");
        Serial.println("Please check connections and restart.");
    }
    
    // Only show commands if system initialized successfully
    if (systemInitialized) {
        Serial.println("\nAvailable commands:");
        Serial.println("  a[0-100] - Set throttle for all motors (percentage)");
        Serial.println("  l[0-100] - Set left motor throttle (percentage)");
        Serial.println("  r[0-100] - Set right motor throttle (percentage)");
        Serial.println("  s - Stop all motors");
        Serial.println("  d - Deinitialize propulsion system");
        Serial.println("  i - Reinitialize propulsion system");
        Serial.println("Enter command:");
    }
}

void loop() {
    // If system failed to initialize, don't process commands
    if (!systemInitialized) {
        delay(1000);
        return;
    }

    // Read serial input
    while (Serial.available() > 0) {
        char inChar = (char)Serial.read();
        
        if (inChar == '\n' || inChar == '\r') {
            if (inputCommand.length() > 0) {
                commandComplete = true;
            }
        } else {
            inputCommand += inChar;
        }
    }
    
    // Process command if complete
    if (commandComplete) {
        processCommand(inputCommand);
        inputCommand = "";
        commandComplete = false;
        Serial.println("\nEnter command:");
    }
}

void processCommand(String command) {
    command.trim();
    
    // Handle empty commands
    if (command.length() == 0) {
        return;
    }
    
    char cmd = command.charAt(0);
    
    switch (cmd) {
        case 'a': {
            float value = command.substring(1).toFloat();
            Serial.print("Setting both motors to ");
            Serial.print(value);
            Serial.println("%");
            propulsion.setThrottle(value);
            break;
        }
        
        case 'l': {
            float value = command.substring(1).toFloat();
            Serial.print("Setting left motor to ");
            Serial.print(value);
            Serial.println("%");
            propulsion.setLeftThrottle(value);
            break;
        }
        
        case 'r': {
            float value = command.substring(1).toFloat();
            Serial.print("Setting right motor to ");
            Serial.print(value);
            Serial.println("%");
            propulsion.setRightThrottle(value);
            break;
        }
        
        case 's':
            Serial.println("Stopping all motors");
            propulsion.stopMotors();
            break;
            
        case 'd':
            Serial.println("Deinitializing propulsion system...");
            propulsion.stopMotors();
            propulsion.deinit();
            systemInitialized = false;
            Serial.println("Propulsion system deinitialized. Restart to use again.");
            break;

        case 'i':
            Serial.println("Reinitializing propulsion system...");
            if (propulsion.init()) {
                systemInitialized = true;
                Serial.println("ESCs ready");
            } else {
                Serial.println("ERROR: Failed to reinitialize propulsion system!");
                Serial.println("Please check connections and try again.");
            }
            break;
            
        default:
            Serial.println("Unknown command");
            break;
    }
}