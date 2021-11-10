/*
 * Debugging.cpp
 */
#include "Debugging.h"
//#include "PAPRHwDefs.h"
#include "Hardware.h"

// Import a few things from Main.
extern void allLEDsOff();
extern Hardware& hw;
extern const byte LEDpins[];
extern const int numLEDs;

// Flash all the LEDs, one at a time, starting with firstIndex, ending at lastIndex.
void exerciseEachLED(int firstIndex, int lastIndex, int duration)
{
    const int increment = firstIndex < lastIndex ? 1 : -1;
    int i = firstIndex;
    do {
        hw.digitalWrite(LEDpins[i], LED_ON);
        hw.delay(duration);
        hw.digitalWrite(LEDpins[i], LED_OFF);
        if (i == lastIndex) break;
        i += increment;
    } while (1);
}

void writeHexDigitToLights(int hexDigit)
{
    allLEDsOff();

    // Turn on the leftmost light, to show that we're displaying a number
    hw.digitalWrite(LEDpins[0], LED_ON);

    // Turn on the lights corresponding to the bits of hexDigit
    hw.digitalWrite(LEDpins[3], (hexDigit & 8) ? LED_ON : LED_OFF);
    hw.digitalWrite(LEDpins[4], (hexDigit & 4) ? LED_ON : LED_OFF);
    hw.digitalWrite(LEDpins[5], (hexDigit & 2) ? LED_ON : LED_OFF);
    hw.digitalWrite(LEDpins[6], (hexDigit & 1) ? LED_ON : LED_OFF);

    hw.delay(1500);
    allLEDsOff();
}

// Write a 16-bit number to the LEDs, in hex.
void writeNumberToLights(uint16_t number)
{
    exerciseEachLED(0, numLEDs - 1, 50);

    writeHexDigitToLights((number >> 12) & 0xf);
    hw.delay(500);

    writeHexDigitToLights((number >> 8) & 0xf);
    hw.delay(500);

    writeHexDigitToLights((number >> 4) & 0xf);
    hw.delay(500);

    writeHexDigitToLights((number) & 0xf);
    hw.delay(500);

    exerciseEachLED(numLEDs - 1, 0, 50);
}
