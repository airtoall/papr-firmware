// A minimal timer utility, that simply gives the ability to
// call a function repeatedly at a specified interval. There are lots of
// timer libraries out there that can do much more - this one does less.
#pragma once

extern uint32_t getMillis();

class PeriodicCallback {
public:
    PeriodicCallback(void (*callback)()) : _callback(callback), _active(false) {}

    void start(uint32_t intervalMillis) {
        _active = true;
        _intervalMillis = intervalMillis;
        _intervalStartMillis = getMillis();
    }

    void update() {
        if (_active) {
            uint32_t now = getMillis();
            if ((now - _intervalStartMillis) > _intervalMillis) {
                _intervalStartMillis = now;
                (*_callback)();
            }
        }
    }

    void stop() {
        _active = false;
    }

    bool isActive() {
        return _active;
    }

private:
    uint32_t _intervalMillis;
    uint32_t _intervalStartMillis;
    bool _active;
    void (*_callback)();
};
