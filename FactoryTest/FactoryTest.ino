/*
*   FactoryTest.ino
*
* This is the main program for the v2 PAPR board Factory Test app.
*
*/

#include "PAPRHwDefs.h"
#include "libraries/ButtonDebounce/src/ButtonDebounce.h"
#include "libraries/FanController/FanController.h"

const int DELAY_100ms = 100;
const int DELAY_200ms = 200;
const int DELAY_300ms = 300;
const int DELAY_500ms = 500;
const int DELAY_1sec = 1000;
const int DELAY_2sec = 2000;
const int DELAY_3sec = 3000;

// ----------------------- Button data -----------------------

// The ButtonDebounce object polls the pin, and calls a callback when the pin value changes. There is one ButtonDebounce object per button.
ButtonDebounce buttonFanUp(FAN_UP_PIN, DELAY_100ms);
ButtonDebounce buttonFanDown(FAN_DOWN_PIN, DELAY_100ms);

// ----------------------- Fan data -----------------------

// How many milliseconds should there be between readings of the fan speed. A smaller value will update
// more often, while a higher value will give more accurate and smooth readings.
const int FAN_SPEED_READING_INTERVAL = 1000;

FanController fanController(FAN_RPM_PIN, FAN_SPEED_READING_INTERVAL, FAN_PWM_PIN);

// The medium duty cycle is arbitrarily set at 50%.
// A duty cycle of 32 results in an RPM of 9560, halfway between min and max RPM.
// However, what we really want (I think) is for the medium duty cycle to produce an AIRFLOW halfway between min and max,
// which could be determined empirically using a fully assembled PAPR unit.
const byte FAN_DUTYCYCLE_MINIMUM = 0;
const byte FAN_DUTYCYCLE_MEDIUM = 50;
const byte FAN_DUTYCYCLE_MAXIMUM = 100;

// The expected fan speeds were empirically determined using a fan with completely unobstructed airflow.
const unsigned int MINIMUM_EXPECTED_FAN_SPEED = 3123;
const unsigned int MEDIUM_EXPECTED_FAN_SPEED = 12033; 
const unsigned int MAXIMUM_EXPECTED_FAN_SPEED = 15994;

// ----------------------- LED data -----------------------

// A list of all the LEDs, from left to right as they appear on the board.
byte LEDpins[] = {
    BATTERY_LED_1_PIN,
    BATTERY_LED_2_PIN,
    BATTERY_LED_3_PIN,
    Error_LED_PIN,
    MODE_LED_1_PIN,
    MODE_LED_2_PIN,
    MODE_LED_3_PIN
};
const int numLEDs = sizeof(LEDpins) / sizeof(byte);

/********************************************************************
 * LEDs
 ********************************************************************/

// Turn off all LEDs
void allLEDsOff() {
    for (int i = 0; i < numLEDs; i += 1) {
        digitalWrite(LEDpins[i], LED_OFF);
    }
}

// Turn a single LED on, then off again.
void flashLED(byte pin, unsigned long duration) {
    digitalWrite(pin, LED_ON);
    delay(duration);
    digitalWrite(pin, LED_OFF);
}

// Turn all the LEDs on, then off again.
void flashLEDs(unsigned long duration) {
    // Turn on all LEDs
    for (int i = 0; i < numLEDs; i += 1) {
        digitalWrite(LEDpins[i], LED_ON);
    }

    delay(duration);

    // Turn off all LEDs
    for (int i = 0; i < numLEDs; i += 1) {
        digitalWrite(LEDpins[i], LED_OFF);
    }

    delay(duration);
}

// Flash all the LEDs a few times
void exerciseAllLEDs() {
    delay(DELAY_300ms);
    for (int k = 0; k < 3; k += 1) {
        flashLEDs(DELAY_300ms);
    }
    delay(DELAY_300ms);
}

// Flash all the LEDs, one at a time, starting with firstIndex, ending at lastIndex.
void exerciseEachLED(int firstIndex, int lastIndex, int duration) {
    int increment = firstIndex < lastIndex ? 1 : -1;
    int i = firstIndex;
    do {
        digitalWrite(LEDpins[i], LED_ON);
        delay(duration);
        digitalWrite(LEDpins[i], LED_OFF);
        if (i == lastIndex) break;
        i += increment;
    } while (1);
}

// Give a sign to the user that an exercise has started 
void startExercise() {
    for (int i = 3; i >= 0; i -= 1) {
        digitalWrite(LEDpins[i], LED_ON);
        digitalWrite(LEDpins[6 - i], LED_ON);
        delay(DELAY_100ms);
        digitalWrite(LEDpins[i], LED_OFF);
        digitalWrite(LEDpins[6 - i], LED_OFF);
    }
}

// Give a sign to the user that an exercise has ended 
void endExercise() {
    for (int i = 0; i <= 3; i += 1) {
        digitalWrite(LEDpins[i], LED_ON);
        digitalWrite(LEDpins[6 - i], LED_ON);
        delay(DELAY_100ms);
        digitalWrite(LEDpins[i], LED_OFF);
        digitalWrite(LEDpins[6 - i], LED_OFF);
    }
}

/********************************************************************
 * Temporary debug code, because we don't yet have a usable serial port or debugger.
 ********************************************************************/

void writeHexDigitToLights(int hexDigit) {
    allLEDsOff();

    // Turn on the leftmost light, to show that we're displaying a number
    digitalWrite(LEDpins[0], LED_ON);

    // Turn on the lights corresponding to the bits of hexDigit
    digitalWrite(LEDpins[3], (hexDigit & 8) ? LED_ON : LED_OFF);
    digitalWrite(LEDpins[4], (hexDigit & 4) ? LED_ON : LED_OFF);
    digitalWrite(LEDpins[5], (hexDigit & 2) ? LED_ON : LED_OFF);
    digitalWrite(LEDpins[6], (hexDigit & 1) ? LED_ON : LED_OFF);

    delay(DELAY_2sec);
    allLEDsOff();
}

// Write a 16-bit number to the LEDs, in hex. It looks like this ...
//    all LEDs quickly flash, left to right
//    high-order hex digit, in binary
//    next hex digit, in binary
//    next hex digit, in binary
//    low-order hex digit, in binary
//    all LEDs quickly flash, right to left
void writeNumberToLights(uint16_t number) {
    exerciseEachLED(0, numLEDs - 1, 50);

    writeHexDigitToLights((number >> 12) & 0xf);
    delay(DELAY_1sec);

    writeHexDigitToLights((number >> 8) & 0xf);
    delay(DELAY_1sec);

    writeHexDigitToLights((number >> 4) & 0xf);
    delay(DELAY_1sec);

    writeHexDigitToLights((number) & 0xf);
    delay(DELAY_1sec);

    exerciseEachLED(numLEDs - 1, 0, 50);
}

/********************************************************************
 * Fan
 ********************************************************************/

void exerciseFan(byte dutyCycle, unsigned int expectedSpeed) {
    // Set the fan speed
    fanController.setDutyCycle(dutyCycle);

    // Wait a few seconds for the fan speed to stabilize
    unsigned long startTime = millis();
    while (millis() - startTime < DELAY_3sec) {
        fanController.getSpeed(); // The fan controller speed function works better if we call it often.
    }

    // Check the fan speed
    unsigned int speed = fanController.getSpeed();
    fanController.setDutyCycle(FAN_DUTYCYCLE_MINIMUM); // we don't need the fan running any more

    // If the speed is too low or too high, show an error indication
    if ((speed < (0.8 * expectedSpeed)) || (speed > (1.2 * expectedSpeed))) {
        writeNumberToLights(speed);
        exerciseAllLEDs();
        exerciseAllLEDs();
    }
}

void exerciseFanMaximum() {
    exerciseFan(FAN_DUTYCYCLE_MAXIMUM, MAXIMUM_EXPECTED_FAN_SPEED);
}

void exerciseFanMedium() {
    exerciseFan(FAN_DUTYCYCLE_MEDIUM, MEDIUM_EXPECTED_FAN_SPEED);
}

void exerciseFanMinimum() {
    exerciseFan(FAN_DUTYCYCLE_MINIMUM, MINIMUM_EXPECTED_FAN_SPEED);
}

/********************************************************************
 * Other Devices
 ********************************************************************/

void exerciseBuzzer() {
    for (int i = 0; i < 5; i += 1) {
        analogWrite(BUZZER_PIN, BUZZER_ON);
        delay(DELAY_500ms);

        analogWrite(BUZZER_PIN, BUZZER_OFF);
        delay(DELAY_200ms);
    }
}

void exerciseBatteryVoltage() {
    // For the next 10 seconds we will display the battery voltage on the LEDs.
    // Empty battery = 1 LEDs. Full battery = 7 LEDs.
    // As you change the input voltage, the LEDs will update accordingly.
    unsigned long startTime = millis();
    while (millis() - startTime < 10000) {
        unsigned int fullness = readBatteryFullness();

        // Calculate how many of the 7 LEDs we should show. 
        uint16_t howManyLEDs = ((fullness * 6) / 100) + 1;

        // Display the calculated number of LEDs.
        allLEDsOff();
        for (int i = 0; i < howManyLEDs; i += 1) {
            digitalWrite(LEDpins[i], LED_ON);
        }
    }
}

/********************************************************************
 * Main program that drives the sequence of exercises.
 ********************************************************************/

void (*exercises[])() = {
        exerciseAllLEDs,
        exerciseBuzzer,
        exerciseFanMaximum,
        exerciseFanMedium,
        exerciseFanMinimum,
        exerciseBatteryVoltage
};
const int numberOfExercises = sizeof(exercises) / sizeof(void (*)());

int currentExercise;

// Do the next exercise. If we've done them all, then start over at the first.
void doNextExercise() {
    currentExercise = (currentExercise + 1) % numberOfExercises;

    startExercise();
    (*exercises[currentExercise])();
    endExercise();
}

// Handler for Fan Button Down
// This button runs the fan-button-down exercise.
void onButtonDownChange(const int state) {
    if (state == BUTTON_RELEASED) {
        startExercise();
        exerciseEachLED(0, numLEDs - 1, DELAY_500ms);
        endExercise();
    }
}

// Handler for Fan Button Up
// This button advances to the next exercise.
void onButtonUpChange(const int state) {
    if (state == BUTTON_RELEASED) {
        // advance to the next exercise
        doNextExercise();
    }
}

void onMonitorActive()
{
    // The monitor input becomnes active when the user pushes the POWER OFF button.

    // Turn on all LEDs
    for (int i = 0; i < numLEDs; i += 1) {
        digitalWrite(LEDpins[i], LED_ON);
    }

    // Turn on the buzzer
    analogWrite(BUZZER_PIN, BUZZER_ON);

    // Leave the lights and buzzer on. We will lose power in just a moment.
}

void setup() {
    // Initialize the hardware
    configurePins();
    initializeDevices();

    // Initialize the software
    buttonFanUp.setCallback(onButtonUpChange);
    buttonFanDown.setCallback(onButtonDownChange);
    fanController.begin();

    // Set fan to default speed
    fanController.setDutyCycle(FAN_DUTYCYCLE_MINIMUM);

    // Do our startup exercise
    currentExercise = -1;
    doNextExercise();
}

void loop() {
    // Run various polling functions and dispatch any events that have occured.
    // All the functionality of the app takes place in event handlers.
    buttonFanUp.update();
    buttonFanDown.update();
    fanController.getSpeed(); // The fan controller speed function works better if we call it often.
    
    if (digitalRead(Monitor_PIN) == LOW) {
        onMonitorActive();
    }
}