#include "PB2PWM.h"
#include "Arduino.h"

// There are several bad ways to generate a PWM signal on pin PB2:
// 1. use analogWrite(). I tried this and it simply doesn't work. I'm not sure why. Also, it doesn't give
//    control over the frequency.
// 2. use tone(). This works, but the documentation says that it will interfere with pins 3 and 11, which is not acceptable.
// 3. write code that wakes up periodically and toggles the pin between High and Low. This works, but does not
//    give much control over frequency and duty cycle.
//
// Fortunately, there is one good way to do it:
// 4. Use the 328p's built-in Timer #1 to generate a signal with the correct frequency and duty cycle. To do this
//    we have to set specific values into various 328p registers.
//  
// This code is roughly based on http://www.righto.com/2009/07/secrets-of-arduino-pwm.html
// We use the method "Varying the timer top limit: phase-correct PWM", because this gives the
// most accurate control over frequency. Note: this web page describes how to do it with Timer 2 and pin 11, 
// but we are using Timer 1 and pin 10, which works the same way.
//
// For complete details about the timer and its registers, see the "Timer 1" chapter of the ATMega328p data sheet.

void startPB2PWM(int32_t frequencyHz, int dutyCyclePercent)
{
    const int pinNumber = 10; // 10 = BUZZER_PIN = PB2 = OC1B
    const int prescale = 1;
    pinMode(pinNumber, OUTPUT);
    TCCR1A = _BV(COM1B1)  // Clear OC1B on Compare Match when up-counting, set OC1B on Compare Match when down-counting.
            | _BV(WGM10)  // PWM, Phase Correct, OCR1A, TOP, BOTTOM 
            | _BV(WGM11);
    TCCR1B = _BV(WGM13)
            | _BV(CS10); // no prescale
    OCR1A = F_CPU / prescale / (2 * frequencyHz);
    OCR1B = dutyCyclePercent * OCR1A / 100;

}

void stopPB2PWM() {
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = 0;
    OCR1B = 0;
}

