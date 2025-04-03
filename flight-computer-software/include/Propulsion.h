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
    
    // Throttle control methods
    void setThrottle(float throttlePercent) {
        float dutyCycle = computeDutyCycle(throttlePercent);
        if (esc1_pwm && esc2_pwm) {
            esc1_pwm->setPWM(ESC1_PIN, ESC_FREQUENCY, dutyCycle);
            esc2_pwm->setPWM(ESC2_PIN, ESC_FREQUENCY, dutyCycle);
        }
    }
    
    void setLeftThrottle(float throttlePercent) {
        float dutyCycle = computeDutyCycle(throttlePercent);
        if (esc1_pwm) {
            esc1_pwm->setPWM(ESC1_PIN, ESC_FREQUENCY, dutyCycle);
        }
    }
    
    void setRightThrottle(float throttlePercent) {
        float dutyCycle = computeDutyCycle(throttlePercent);
        if (esc2_pwm) {
            esc2_pwm->setPWM(ESC2_PIN, ESC_FREQUENCY, dutyCycle);
        }
    }

private:
    // ESC pins
    const int ESC1_PIN;
    const int ESC2_PIN;
    
    // ESC frequency in Hz
    static const int ESC_FREQUENCY = 400;
    // ESC period in microseconds
    static constexpr float ESC_PERIOD = 1000000.0f / ESC_FREQUENCY;

    // ESC pulse width values in microseconds
    static const int MIN_PULSE_WIDTH = 1000;  // Zero throttle
    static const int MAX_PULSE_WIDTH = 2000;  // Full throttle
    
    // At 400Hz, period is 2500μs (1/400 = 0.0025s = 2500μs)
    // 1000/2500 = 40% duty cycle (zero throttle)
    // 2000/2500 = 80% duty cycle (full throttle)
    // Duty cycle values for ESC throttle percentage
    static constexpr float MIN_DUTY_CYCLE = (MIN_PULSE_WIDTH / ESC_PERIOD) * 100.0f;
    static constexpr float MAX_DUTY_CYCLE = (MAX_PULSE_WIDTH / ESC_PERIOD) * 100.0f;
    
    // PWM objects for the ESCs
    Teensy_PWM* esc1_pwm;
    Teensy_PWM* esc2_pwm;

    // Helper function to map throttle percentage to duty cycle
    float computeDutyCycle(float throttlePercent) const {
        return (throttlePercent * (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) / 100.0f) + MIN_DUTY_CYCLE;
    }
};

#endif // PROPULSION_H