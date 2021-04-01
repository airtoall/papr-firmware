#pragma once
#include "Hardware.h"

/********************************************************************
 * Press detector for buttons
 ********************************************************************/

class PressDetector {
private:
    unsigned int _pin;
    unsigned long _requiredMillis;
    void (*_callback)(const int);
    int _currentState;
    unsigned long _pressMillis;
    bool _callbackCalled;

public:
    PressDetector(int pin, unsigned long requiredMillis, void(*callback)(const int))
        : _pin(pin), _requiredMillis(requiredMillis), _callback(callback),
        _currentState(BUTTON_RELEASED), _pressMillis(0), _callbackCalled(true) {}

    void update()
    {
        int state = digitalRead(_pin);

        if (state == BUTTON_PUSHED) {
            if (_currentState == BUTTON_PUSHED) {
                // The button is already pressed. See if the button has been pressed long enough.
                if (!_callbackCalled && (millis() - _pressMillis > _requiredMillis)) {
                    _callback(state);
                    _callbackCalled = true;
                }
            } else {
                // The button has just been pushed. Record the start time of this press.
                _pressMillis = millis();
                _callbackCalled = false;
            }
        }
        _currentState = state;
    }

    int state()
    {
        return _currentState;
    }
};