#include "ArduTFLite.h"

DMAMEM tflite::MicroMutableOpResolver<8> tflOpsResolver;
DMAMEM const tflite::Model* tflModel = nullptr;
DMAMEM tflite::MicroInterpreter* tflInterpreter = nullptr;
DMAMEM TfLiteTensor* tflInputTensor = nullptr;
DMAMEM TfLiteTensor* tflOutputTensor = nullptr;

bool modelInit(const unsigned char* model, byte* tensorArena, int tensorArenaSize){
  tflModel = tflite::GetModel(model);
  if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema version mismatch!");
    return false;
  }
  
  // Use operator-specific registration functions instead of AddBuiltin
  tflOpsResolver.AddFullyConnected();
  tflOpsResolver.AddSoftmax();
  tflOpsResolver.AddRelu();
  tflOpsResolver.AddMul();
  tflOpsResolver.AddAdd();
  tflOpsResolver.AddTanh();
  tflOpsResolver.AddLogistic();
  tflOpsResolver.AddQuantize();
  
  tflInterpreter = new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena, tensorArenaSize);
  tflInterpreter->AllocateTensors();
  tflInputTensor = tflInterpreter->input(0);
  tflOutputTensor = tflInterpreter->output(0);
  return true;
}

bool modelSetInput(float inputValue, int index){
    if (tflInputTensor == nullptr || index >= tflInputTensor->bytes / sizeof(float)) {
        Serial.println("Input tensor index out of range!");
        return false;
    }
    tflInputTensor->data.f[index] = inputValue;

    return true;
}

bool modelRunInference(){
    // Add more error checking and debugging
    if (tflInterpreter == nullptr) {
        Serial.println("ERROR: Interpreter is null");
        return false;
    }
    
    // Serial.println("DEBUG: Starting inference...");
    TfLiteStatus invokeStatus = tflInterpreter->Invoke();
    if (invokeStatus != kTfLiteOk) {
        Serial.printf("ERROR: Inference failed with status code %d\n", invokeStatus);
        return false;
    }
    // Serial.println("DEBUG: Inference completed successfully");
    return true;
}

float modelGetOutput(int index) {
    if (tflOutputTensor == nullptr || index >= tflOutputTensor->bytes / sizeof(float)) {
        Serial.println("Output tensor index out of range!");
        return -1;
    }

    return tflOutputTensor->data.f[index];
}