#include <utils.h>

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