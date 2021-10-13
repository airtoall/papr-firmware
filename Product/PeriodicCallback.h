// A minimal timer utility, that simply gives the ability to
// call a function repeatedly at a specified interval. There are lots of
// timer libraries out there that can do much more - this one does less.
#pragma once

extern unsigned long getMillis();

class PeriodicCallback {
public:
    PeriodicCallback(void (*callback)()) : _callback(callback), _active(false) {}

    void start(unsigned long intervalMillis) {
        _active = true;
        _intervalMillis = intervalMillis;
        _intervalStartMillis = getMillis();
    }

    void update() {
        if (_active) {
            unsigned long now = getMillis();
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
    unsigned long _intervalMillis;
    unsigned long _intervalStartMillis;
    bool _active;
    void (*_callback)();
};
