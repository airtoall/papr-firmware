// This interface gives access to the programmable PWM generator on pin PB2 of the ATMega328P MCU
#pragma once
#include <stdint.h>

// Start generating a PWM signal on pin PB2, with the given frequency and duty cycle.
void startPB2PWM(int32_t frequencyHz, int dutyCyclePercent);

// Stop generating a PWM signal on pin PB2
void stopPB2PWM();

