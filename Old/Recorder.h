#pragma once

// For testing and debugging: this code captures data from various hardware and software sources,
// and periodically writes that data to the serial port. 

// Call this function as often as possible.
void updateRecorder(unsigned int fanRPM, int currentDutyCycle, bool isCharging, long long picoCoulombs);

// Call this on startup, or when waking up from a nap.
void resetRecorder();
