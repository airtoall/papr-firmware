#pragma once

// Start generating a PWM signal on pin PD3, with the given frequency and duty cycle.
void startPD3PWM(long frequencyHz, int dutyCyclePercent);

// Stop generating a PWM signal on pin PD3
void stopPD3PWM();

