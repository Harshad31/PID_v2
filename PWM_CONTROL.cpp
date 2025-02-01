#include "PWM_CONTROL.h"
#include <Arduino.h>

PWMControl::PWMControl(int pin) {
    _pin = pin;
    _dutyCycle = 0;
    _lastUpdateTime = 0;
    _lastCycleTime = 0;
    _deadManSwitchTimeout = 0;
    _isDeadManSwitchActive = false;
}

void PWMControl::begin(unsigned long interval, float minDutyCycle, float maxDutyCycle, bool equateMinToZero, bool equateMaxToHundred) {
    _interval = interval;
    _minDutyCycle = minDutyCycle;
    _maxDutyCycle = maxDutyCycle;
    _equateMinToZero = equateMinToZero;
    _equateMaxToHundred = equateMaxToHundred;
    pinMode(_pin, OUTPUT);
}

void PWMControl::updateDutyCycle(float dutyCycle) {
    updateDutyCycle(dutyCycle, 0);
}

void PWMControl::updateDutyCycle(float dutyCycle, unsigned long deadManSwitch) {
    if (dutyCycle < _minDutyCycle) {
        dutyCycle = _equateMinToZero ? 0 : _minDutyCycle;
    } else if (dutyCycle > _maxDutyCycle) {
        dutyCycle = _equateMaxToHundred ? 100 : _maxDutyCycle;
    }
    
    _dutyCycle = dutyCycle;
    _deadManSwitchTimeout = deadManSwitch;
    _lastUpdateTime = millis();
    _isDeadManSwitchActive = false;
}

void PWMControl::trigger() {
    if (millis() - _lastUpdateTime > _deadManSwitchTimeout) {
        stop();
        _isDeadManSwitchActive = true;
    }

    handlePWM();
}

void PWMControl::stop() {
    digitalWrite(_pin, LOW);  // Set pin LOW (OFF)
}

void PWMControl::handlePWM() {
    unsigned long currentTime = millis();
    if (_isDeadManSwitchActive) return;

    if (currentTime - _lastCycleTime >= _interval) {
        int onTime = (_dutyCycle / 100.0) * _interval;
        int offTime = _interval - onTime;

        if (_dutyCycle > 0) {
            digitalWrite(_pin, HIGH);  // Turn pin ON
            delay(onTime);
            digitalWrite(_pin, LOW);   // Turn pin OFF
            delay(offTime);
        } else {
            digitalWrite(_pin, LOW);   // Make sure the pin is OFF
        }

        _lastCycleTime = millis();
    }
}
