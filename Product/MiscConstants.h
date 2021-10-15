// MiscConstants.h
#pragma once
#include <stdint.h>

// A string that identifies this product's version and build.
extern const char* PRODUCT_ID;

const int numFanSpeeds = 3;

// Human-readable names for the fan speeds.
extern const char* FAN_SPEED_NAMES[];

// The duty cycle for each fan speed. Indexed by FanSpeed.
extern const int fanDutyCycles[];

// The expected RPM for each fan speed. Indexed by FanSpeed.
extern const uint16_t expectedFanRPM[];

const int32_t BUZZER_FREQUENCY = 2500; // in Hz

const int BUZZER_DUTYCYCLE = 50; // in percent
