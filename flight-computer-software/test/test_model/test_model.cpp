#include <ArduTFLite.h>
#include "TeensyThreads.h"
#include "hab_model.h"
#include "../../include/Sensors.h"
#include "../../include/Propulsion.h"
// #include <SD.h>

#define SPEAKER_PIN 33

// The Tensor Arena memory area is used by TensorFlow Lite to store input, output and intermediate tensors
// It must be defined as a global array of byte (or u_int8 which is the same type on Arduino) 
// The Tensor Arena size must be defined by trials and errors. We use here a quite large value.
// The alignas(16) directive is used to ensure that the array is aligned on a 16-byte boundary,
// this is important for performance and to prevent some issues on ARM microcontroller architectures.
constexpr int kTensorArenaSize = 12 * 1024;
alignas(16) uint8_t tensor_arena[kTensorArenaSize];
Sensors sensors;
Propulsion propulsion;
// File dataFile = NULL;

float xWorldLinearAcceleration, yWorldLinearAcceleration, zWorldLinearAcceleration;
float vx, vy, vz;
float xWorldAngularVelocity, yWorldAngularVelocity, zWorldAngularVelocity;
float yawOrientation, pitchOrientation, rollOrientation;
float prevAction0, prevAction1;
float actionDiff0, actionDiff1;
float px, py, pz;
long start_time = 0;
const long duration = 60000;

float safeSigmoid(float x) {
    if (x < -10.0f) x = -10.0f;
    else if (x > 10.0f) x = 10.0f;
    return 1.0f / (1.0f + exp(-x));
}

void stopMotors() {
    propulsion.stopMotors();
    propulsion.deinit();

    // if (dataFile) {
    //     dataFile.close();
    // }

    // Stop motor tone
    tone(SPEAKER_PIN, 700, 1000);
    threads.delay(1000);
    noTone(SPEAKER_PIN);
}

void setup() {
  Serial.begin(9600);
  while(!Serial);

  while (true) {
    sensors.begin();
    if (sensors.status.imu1 && sensors.status.imu2 && sensors.status.imu3) {
        Serial.println("Successfully initialized all sensors.");
        break;
    } else {
        Serial.println("Retrying sensor initialization.");
        delay(100);
    }
  }

      // Initialize propulsion system
      if (propulsion.init()) {
        Serial.println("ESCs ready");
    } else {
        Serial.println("ERROR: Failed to initialize propulsion system!");
        Serial.println("Please check connections and restart.");
        while(true);
    }
    // Calibrate ESCs
    propulsion.calibrate();
    Serial.println("ESCs calibrated");

  Serial.println("Initializing TensorFlow Lite Micro Interpreter...");
  if (!modelInit(hab_model_tflite, tensor_arena, kTensorArenaSize)){
    Serial.println("Model initialization failed!");
    while(true);
  }
  Serial.println("Model initialization done.");

//   dataFile = SD.open("test_model.txt", FILE_WRITE);
//   if (dataFile) {
//     Serial.println("SD initialized.");
//   } else {
//     Serial.println("Failed to open file in SD card for writing.");
//     while(true);
//   }

  sensors.start();
  prevAction0 = 0;
  prevAction1 = 0;
  actionDiff0 = 0;
  actionDiff1 = 0;

  // Start motors tone
  tone(SPEAKER_PIN, 900, 1000);
  threads.delay(1000);
  noTone(SPEAKER_PIN);
  threads.delay(10000);

  start_time = millis();
  Serial.println("Starting mission...");
}

void loop() {
    Serial.printf("time: %ld", millis());
    // dataFile.printf("time: %ld", millis());
    sensors.getFusedWorldLinearAcceleration(xWorldLinearAcceleration, yWorldLinearAcceleration, zWorldLinearAcceleration);
    // dataFile.printf(", xAcc: %.2f, yAcc: %.2f", xWorldLinearAcceleration, yWorldLinearAcceleration);
    Serial.printf(", xAcc: %.2f, yAcc: %.2f", xWorldLinearAcceleration, yWorldLinearAcceleration);
    xWorldLinearAcceleration /= 10.0f;
    yWorldLinearAcceleration /= 10.0f;

    sensors.getRelativeVelocity(vx, vy, vz);
    
    Serial.printf(", vx: %.2f, vy: %.2f", vx, vy);
    // dataFile.printf(", vx: %.2f, vy: %.2f", vx, vy);
    vx /= 10.0f;
    vy /= 10.0f;

    sensors.getRelativePosition(px, py, pz);
    // dataFile.printf(", px: %.2f, py: %.2f", px, py);
    Serial.printf(", px: %.2f, py: %.2f", px, py);
    px /= 10.0f;
    py /= 10.0f;

    sensors.getFusedOrientation(yawOrientation, pitchOrientation, rollOrientation);
    rollOrientation *= 3.14 / 180.0;
    pitchOrientation *= 3.14 / 180.0;
    yawOrientation *= 3.14 / 180.0;
    // dataFile.printf(", roll: %.2f, pitch: %.2f, yaw: %.2f", rollOrientation, pitchOrientation, yawOrientation);
    Serial.printf(", roll: %.2f, pitch: %.2f, yaw: %.2f", rollOrientation, pitchOrientation, yawOrientation);
    rollOrientation /= 6.28f;
    pitchOrientation /= 6.28f;
    yawOrientation /= 6.28f;

    sensors.getFusedWorldAngularVelocity(xWorldAngularVelocity, yWorldAngularVelocity, zWorldAngularVelocity);
    xWorldAngularVelocity *= 3.14 / 180.0;
    yWorldAngularVelocity *= 3.14 / 180.0;
    zWorldAngularVelocity *= 3.14 / 180.0;
    // dataFile.printf(", xAngVel: %.2f, yAngVel: %.2f, zAngVel: %.2f", xWorldAngularVelocity, yWorldAngularVelocity, zWorldAngularVelocity);
    Serial.printf(", xAngVel: %.2f, yAngVel: %.2f, zAngVel: %.2f", xWorldAngularVelocity, yWorldAngularVelocity, zWorldAngularVelocity);
    xWorldAngularVelocity /= 6.28f;
    yWorldAngularVelocity /= 6.28f;
    zWorldAngularVelocity /= 6.28f;

    modelSetInput(xWorldLinearAcceleration, 0);
    modelSetInput(yWorldLinearAcceleration, 1);
    modelSetInput(vx, 2);
    modelSetInput(vy, 3);
    modelSetInput(px, 4);
    modelSetInput(py, 5);
    modelSetInput(rollOrientation, 6);
    modelSetInput(pitchOrientation, 7);
    modelSetInput(yawOrientation, 8);
    modelSetInput(xWorldAngularVelocity, 9);
    modelSetInput(yWorldAngularVelocity, 10);
    modelSetInput(zWorldAngularVelocity, 11);
    modelSetInput(prevAction0, 12);
    modelSetInput(prevAction1, 13);
    modelSetInput(actionDiff0, 14);
    modelSetInput(actionDiff1, 15);

    if(!modelRunInference()){
        Serial.println("RunInference Failed!");
        stopMotors();
        while(true);
    }

    float action0 = safeSigmoid(modelGetOutput(0));
    float action1 = safeSigmoid(modelGetOutput(1));
    propulsion.setLeftThrottle(action0 * 0.20 * 100.0);
    propulsion.setRightThrottle(action1 * 0.20 * 100.0);

    Serial.printf(
        ", prevA0: %.2f, prevA1: %.2f, diffA0: %.2f, diffA1: %.2f, motor0: %.2f, motor1: %.2f\n",
        prevAction0, prevAction1, actionDiff0, actionDiff1,
        action0, action1
    );
    // Serial.printf(", %.2f, %.2f\n", action0, action1);
    actionDiff0 = prevAction0 - action0;
    actionDiff1 = prevAction1 - action1;
    prevAction0 = action0;
    prevAction1 = action1;

    if (millis() - start_time >= duration) {
        Serial.println("Mission timeout");
        stopMotors();
        while(true);
    }

    threads.delay(5);
}
