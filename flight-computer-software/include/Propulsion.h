#ifndef PROPULSION_H
#define PROPULSION_H

#include <Arduino.h>
#include <Teensy_PWM.h>  // Include Teensy_PWM library

class Propulsion {
public:
    // Constructor and destructor
    Propulsion(int esc1Pin = 4, int esc2Pin = 5)
        : ESC1_PIN(esc1Pin), ESC2_PIN(esc2Pin), esc1_pwm(nullptr), esc2_pwm(nullptr) {
        // Constructor initializes pin numbers and nulls the PWM pointers
    }
    
    ~Propulsion() {
        // Ensure resources are freed
        deinit();
    }
    
    // Main control methods
    bool init() {
        // Initialize Teensy_PWM objects with 0% duty cycle
        esc1_pwm = new Teensy_PWM(ESC1_PIN, ESC_FREQUENCY, 0.0f);
        esc2_pwm = new Teensy_PWM(ESC2_PIN, ESC_FREQUENCY, 0.0f);
        
        // Check if initialization was successful
        if (!esc1_pwm || !esc2_pwm || 
            !esc1_pwm->isPWMEnabled() || !esc2_pwm->isPWMEnabled()) {
            return false;
        }
        
        return true;
    }
    
    void deinit() {
        if (esc1_pwm) {
            delete esc1_pwm;
            esc1_pwm = nullptr;
        }
        if (esc2_pwm) {
            delete esc2_pwm;
            esc2_pwm = nullptr;
        }
    }
    
    void stopMotors() {
        if (esc1_pwm && esc2_pwm) {
            esc1_pwm->setPWM(ESC1_PIN, ESC_FREQUENCY, MIN_DUTY_CYCLE);
            esc2_pwm->setPWM(ESC2_PIN, ESC_FREQUENCY, MIN_DUTY_CYCLE);
        }
    }
    
    void calibrateESCs() {
        Serial.println("Starting ESC calibration sequence...");
        
        // Step 1: Set throttle to maximum position
        esc1_pwm->setPWM(ESC1_PIN, ESC_FREQUENCY, MAX_DUTY_CYCLE);
        esc2_pwm->setPWM(ESC2_PIN, ESC_FREQUENCY, MAX_DUTY_CYCLE);
        Serial.println("Throttle set to maximum");
        
        // Step 2: Connect battery while holding maximum throttle
        Serial.println("Please connect the battery now");
        Serial.println("Wait for the ESCs to emit a confirmation tone sequence");
        delay(5000);  // Wait for user to connect battery and hear the tones
        
        // Step 3: After confirmation tones, set throttle to minimum
        esc1_pwm->setPWM(ESC1_PIN, ESC_FREQUENCY, MIN_DUTY_CYCLE);
        esc2_pwm->setPWM(ESC2_PIN, ESC_FREQUENCY, MIN_DUTY_CYCLE);
        Serial.println("Throttle set to minimum");
        
        // Step 4: Wait for confirmation beeps
        delay(2000);
        Serial.println("Calibration complete! ESCs should be ready");
        
        // Ensure motors are stopped after calibration
        stopMotors();
    }
    
    // Arm ESCs
    void armESCs() {
        // Set to minimum throttle and wait for ESCs to initialize
        stopMotors();
        delay(1000);
    }
    
    // Throttle control methods
    void setThrottle(int percent) {
        percent = constrain(percent, 0, 100);
        int pwmValue = map(percent, 0, 100, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
        float dutyCycle = microsecondsToDutyCycle(pwmValue);
        
        if (esc1_pwm && esc2_pwm) {
            esc1_pwm->setPWM(ESC1_PIN, ESC_FREQUENCY, dutyCycle);
            esc2_pwm->setPWM(ESC2_PIN, ESC_FREQUENCY, dutyCycle);
        }
    }
    
    void setLeftThrottle(int percent) {
        percent = constrain(percent, 0, 100);
        int pwmValue = map(percent, 0, 100, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
        float dutyCycle = microsecondsToDutyCycle(pwmValue);
        
        if (esc1_pwm) {
            esc1_pwm->setPWM(ESC1_PIN, ESC_FREQUENCY, dutyCycle);
        }
    }
    
    void setRightThrottle(int percent) {
        percent = constrain(percent, 0, 100);
        int pwmValue = map(percent, 0, 100, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
        float dutyCycle = microsecondsToDutyCycle(pwmValue);
        
        if (esc2_pwm) {
            esc2_pwm->setPWM(ESC2_PIN, ESC_FREQUENCY, dutyCycle);
        }
    }

private:
    // ESC pins
    const int ESC1_PIN;
    const int ESC2_PIN;
    
    // ESC pulse width values (microseconds)
    static const int MIN_PULSE_WIDTH = 1000;  // Zero throttle
    static const int MAX_PULSE_WIDTH = 2000;  // Full throttle
    
    // PWM frequency in Hz
    static const int ESC_FREQUENCY = 400;
    
    // PWM duty cycle values for ESC control
    // For 400Hz signal, period is 2500μs, so we map:
    // 1000μs pulse = 40% duty cycle, 2000μs pulse = 80% duty cycle
    static constexpr float MIN_DUTY_CYCLE = 40.0f;  // Corresponds to 1000μs pulse
    static constexpr float MAX_DUTY_CYCLE = 80.0f;  // Corresponds to 2000μs pulse
    
    // PWM objects for the ESCs
    Teensy_PWM* esc1_pwm;
    Teensy_PWM* esc2_pwm;
    
    // Helper function to convert microseconds to duty cycle
    float microsecondsToDutyCycle(int microseconds) const {
        return map(microseconds, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
    }
};

#endif // PROPULSION_H
