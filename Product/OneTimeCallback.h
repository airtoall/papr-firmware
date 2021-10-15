// A minimal timer utility, that simply gives the ability to
// call a function at a specied future time. There are lots of
// Timer libraries out there that can do much more - this one does less.
#pragma once
#include <stdint.h>

extern uint32_t getMillis();

class OneTimeCallback {
public:
    OneTimeCallback(void (*callback)()) : _intervalMillis(0UL), _callback(callback) {}

    // schedules a callback to occur at the specified time interval from now
    void start(uint32_t intervalMillis) {
        _intervalMillis = intervalMillis;
        _intervalStartMillis = getMillis();
    }

    // call this from loop()
    void update() {
        if (_intervalMillis && ((getMillis() - _intervalStartMillis) > _intervalMillis)) {
            _intervalMillis = 0UL;
            (*_callback)();
        }
    }

    void cancel() {
        _intervalMillis = 0UL;
    }

private:
    uint32_t _intervalMillis;
    uint32_t _intervalStartMillis;
    void (*_callback)();
};
