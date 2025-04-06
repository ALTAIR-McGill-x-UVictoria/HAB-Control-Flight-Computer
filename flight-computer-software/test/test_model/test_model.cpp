#include <ArduTFLite.h>
#include "TeensyThreads.h"
#include "hab_model.h"
#include "../../include/Sensors.h"

// The Tensor Arena memory area is used by TensorFlow Lite to store input, output and intermediate tensors
// It must be defined as a global array of byte (or u_int8 which is the same type on Arduino) 
// The Tensor Arena size must be defined by trials and errors. We use here a quite large value.
// The alignas(16) directive is used to ensure that the array is aligned on a 16-byte boundary,
// this is important for performance and to prevent some issues on ARM microcontroller architectures.
constexpr int kTensorArenaSize = 20 * 1024;
alignas(16) uint8_t tensor_arena[kTensorArenaSize];
Sensors sensors;
float xWorldLinearAcceleration, yWorldLinearAcceleration, zWorldLinearAcceleration;
float vx, vy, vz;
float xWorldAngularVelocity, yWorldAngularVelocity, zWorldAngularVelocity;
float yawOrientation, pitchOrientation, rollOrientation;
float prevAction0, prevAction1;
float actionDiff0, actionDiff1;
float px, py, pz;

float safeSigmoid(float x) {
    if (x < -10.0f) x = -10.0f;
    else if (x > 10.0f) x = 10.0f;
    return 1.0f / (1.0f + exp(-x));
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

  Serial.println("Initializing TensorFlow Lite Micro Interpreter...");
  if (!modelInit(hab_model_tflite, tensor_arena, kTensorArenaSize)){
    Serial.println("Model initialization failed!");
    while(true);
  }
  Serial.println("Model initialization done.");

  sensors.start();
  prevAction0 = 0;
  prevAction1 = 0;
  actionDiff0 = 0;
  actionDiff1 = 0;
}

void loop() {
    sensors.getFusedWorldLinearAcceleration(xWorldLinearAcceleration, yWorldLinearAcceleration, zWorldLinearAcceleration);
    xWorldLinearAcceleration /= 10.0f;
    yWorldLinearAcceleration /= 10.0f;

    sensors.getRelativeVelocity(vx, vy, vz);
    vx /= 10.0f;
    vy /= 10.0f;

    sensors.getRelativePosition(px, py, pz);
    px /= 10.0f;
    py /= 10.0f;

    sensors.getFusedOrientation(yawOrientation, pitchOrientation, rollOrientation);
    rollOrientation /= 6.28f;
    pitchOrientation /= 6.28f;
    yawOrientation /= 6.28f;

    sensors.getFusedWorldAngularVelocity(xWorldAngularVelocity, yWorldAngularVelocity, zWorldAngularVelocity);
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

    // Serial.printf(
    //     "xAcc: %.2f, yAcc: %.2f, vx: %.2f, vy: %.2f, px: %.2f, py: %.2f, roll: %.2f, pitch: %.2f, yaw: %.2f, "
    //     "xAngVel: %.2f, yAngVel: %.2f, zAngVel: %.2f, prevA0: %.2f, prevA1: %.2f, diffA0: %.2f, diffA1: %.2f\n",
    //     xWorldLinearAcceleration, yWorldLinearAcceleration, vx, vy, px, py,
    //     rollOrientation, pitchOrientation, yawOrientation,
    //     xWorldAngularVelocity, yWorldAngularVelocity, zWorldAngularVelocity,
    //     prevAction0, prevAction1, actionDiff0, actionDiff1
    // );

    if(!modelRunInference()){
        Serial.println("RunInference Failed!");
        while(true);
    }

    float action0 = safeSigmoid(modelGetOutput(0));
    float action1 = safeSigmoid(modelGetOutput(1));
    Serial.printf("%.2f, %.2f\n", action0, action1);
    actionDiff0 = prevAction0 - action0;
    actionDiff1 = prevAction1 - action1;
    prevAction0 = action0;
    prevAction1 = action1;
}
