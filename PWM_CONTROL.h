#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

#include <Arduino.h>

class PWMControl {
public:
    // Constructor
    PWMControl(int pin);

    // Method to initialize the PWM control
    void begin(unsigned long interval, float minDutyCycle = 0, float maxDutyCycle = 100, bool equateMinToZero = false, bool equateMaxToHundred = true);

    // Method to update the duty cycle (0 to 100)
    void updateDutyCycle(float dutyCycle);

    // Method to update the duty cycle with custom dead-man switch timeout
    void updateDutyCycle(float dutyCycle, unsigned long deadManSwitch);

    // Method to trigger PWM control process
    void trigger();

    // Method to stop PWM output
    void stop();

private:
    int _pin;
    unsigned long _interval;
    float _minDutyCycle;
    float _maxDutyCycle;
    bool _equateMinToZero;
    bool _equateMaxToHundred;
    float _dutyCycle;
    unsigned long _lastUpdateTime;
    unsigned long _lastCycleTime;
    unsigned long _deadManSwitchTimeout;
    bool _isDeadManSwitchActive;

    void handlePWM();
};

#endif // PWM_CONTROL_H
