// An object that can detect button presses, and call 
// a specified function when the button is pressed and released.
// We do "debouncing", which means that a press is detected
// only if the button is held for a specified period. 
#pragma once
#include "Hardware.h"

class PressDetector {
private:
    uint16_t _pin;
    uint32_t _requiredMillis;
    void (*_callback)();
    void (*_releaseCallback)();
    int _currentState;
    uint32_t _pressMillis;
    bool _callbackCalled;

public:
    // parameters are:
    // pin:             which Arduino pin number the button is attached to
    // requiredMillis:  a press will be detected only if the button is held for at least this long
    // callback:        the function to call when the button is pressed
    // releaseCallback: (optional) the function to call when the button is released
    PressDetector(int pin, uint32_t requiredMillis, void(*callback)(), void(*releaseCallback)() = 0)
        : _pin(pin), _requiredMillis(requiredMillis), _callback(callback), _releaseCallback(releaseCallback),
        _currentState(BUTTON_RELEASED), _pressMillis(0UL), _callbackCalled(true) {}

    void update()
    {
        int state = Hardware::instance.digitalRead(_pin);

        if (state == BUTTON_PUSHED) {
            if (_currentState == BUTTON_PUSHED) {
                // The button is already pressed. See if the button has been pressed long enough.
                if (!_callbackCalled && (Hardware::instance.millis() - _pressMillis > _requiredMillis)) {
                    _callback();
                    _callbackCalled = true;
                }
            } else {
                // The button has just been pushed. Record the start time of this press.
                _pressMillis = Hardware::instance.millis();
                _callbackCalled = false;
            }
        } else {
            if (_currentState == BUTTON_PUSHED) {
                // The button has just been released.
                if (_releaseCallback) {
                    _releaseCallback();
                }
            }
        }
        _currentState = state;
    }

    int state()
    {
        return _currentState;
    }
};
