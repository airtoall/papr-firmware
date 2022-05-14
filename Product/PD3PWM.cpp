#include "PD3PWM.h"
#include "Arduino.h"

// PD3PWM uses the 328p's built-in Timer #2 to generate a signal with the correct frequency and duty cycle. To do this
// we have to set specific values into various 328p registers.
//  
// This code is roughly based on http://www.righto.com/2009/07/secrets-of-arduino-pwm.html
// We use the method "Varying the timer top limit: phase-correct PWM", because this gives the
// most accurate control over frequency. Note: this web page describes how to do it with Timer 2 and pin 11,
// and we're using pin 3, which is very similar.
//
// For complete details about the timer and its registers, see the "Timer 2" chapter of the ATMega328p data sheet.

void startPD3PWM(long frequencyHz, int dutyCyclePercent)
{
    const int pinNumber = 3; // 3 = FAN_PWM_PIN = PD3 = OC2B
    const int prescale = 1;
    pinMode(pinNumber, OUTPUT);
    TCCR2A = _BV(COM2B1) // Clear OC2B on Compare Match when up-counting, set OC2B on Compare Match when down-counting.
           | _BV(WGM20); // PWM, Phase Correct, OCRA, TOP, BOTTOM 
    TCCR2B = _BV(WGM22)
           | _BV(CS20);  // no prescale
    OCR2A = F_CPU / prescale / (2 * frequencyHz);
    OCR2B = dutyCyclePercent * OCR2A / 100;
}

void stopPD3PWM() {
    TCCR2A = 0;
    TCCR2B = 0;
    OCR2A = 0;
    OCR2B = 0;
}

