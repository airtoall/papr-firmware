/* ProductionTest.cpp
* 
* This file contains a set of tests that exercise our printed circuit board. 
* The tests are described in detail in the document "Production Test.pdf", 
* located in https://github.com/airtoall/papr-firmware/tree/master/Product/Docs
* 
* Every time the MCU is reset or powered up, it listens on the serial port for a few seconds.
* If there is a terminal connected to the serial port and the the user types "T"
* during that interval, we enter "test mode", in which the function
* productionTestSetup() gets called once, and then productionTestLoop() gets called
* repeatedly. The loop function listens for the user to type in a one-character command, and 
* then performs the command. The user observes the output on the serial port, along with
* the LEDs, the buzzer, the fan, the power supply, etc, and decides whether
* the board is working properly. When the user is finished running
* tests, they can power down the board, or use the "r" command to reset the firmware.
* 
* Commands typically run forever, and use pause() to periodically
* check if the user has hit a key to stop the command. While a command is
* running, it gives periodic feedback on the terminal and/or LEDs, to give the
* user a warm feeling that something is happening.
* 
* A note on the "F()" macro: this macro tells the compiler to leave the string
* in flash memory, and not copy it to RAM before use as it would usually do.
* Using F() reduces the amount of RAM needed by this firmware. If we don't 
* use F(), then we actually run out of RAM (with no warning!) and the firmware
* starts behaving erratically.
*/
#include "ProductionTest.h"
#include "PB2PWM.h"
#include "MySerial.h"
#include "Hardware.h"
#include "FanController.h"
#include "MiscConstants.h"

/********************************************************************
 * Misc definitions
 ********************************************************************/

#define hw Hardware::instance

int32_t getMilliAmps() {
	// Do several consecutive readings, so that the low-pass filter in
	// readMicroAmps() will smooth out any jitter in the readings.
	for (int j = 0; j < 500; j += 1) hw.readMicroAmps();

	return (int32_t)(hw.readMicroAmps() / -1000LL);
}

/********************************************************************
 * Command handling
 ********************************************************************/

bool isLineEnd(int c) {
	return (c == 10) || (c == 13);
}

// Reads a one-character command from the serial port. We skip over and ignore any
// line-end characters, and also any characters on a line after the command character.
int getNextCommand() {
	// Loop until we find a command
	while (true) {
		int c;

		// Skip over line-ends
		while (c = Serial.peek(), isLineEnd(c)) {
			Serial.read();
		}
		if (c != -1) {
			// Grab the command character
			int result = Serial.read();

			// Some terminal apps send each character as soon as it is typed. Other
			// terminal apps save up typed characters and sends them when the user hits "enter".
			// In case we are talking to the latter, we will delay for a moment, and
			// then discard any other characters on the same line as the command.
			hw.delay(100L);
			while (Serial.peek() != -1) {
				Serial.read();
			}

			return result;
		}
	}
}

// When a command is in progress, you should call this function to periodically check 
// if the user has hit any character, which tells us to stop the command.
bool doPause(int waitTimeMillis) {
	while (waitTimeMillis > 0) {
		if (Serial.peek() != -1) {
			hw.delay(100L);
			while (Serial.peek() != -1) Serial.read();
			return false;
		}
		hw.delay(50L);
		waitTimeMillis -= 50;
	}
	return true;
}

#define pause(millis) if (!doPause(millis)) goto done;

/********************************************************************
 * LED Test
 ********************************************************************/

// Set a single LED to a given state
void setLED(const int pin, int onOff) {
    hw.digitalWrite(pin, onOff);
}

// Set a list of LEDs to a given state.
void setLEDs(const byte* pinList, int count, int onOff)
{
    for (int i = 0; i < count; i += 1) {
        setLED(pinList[i], onOff);
    }
}

void testLEDs() {
	serialPrintln(F("LED Test"));
	while (true) {
		for (int i = 0; i < numLEDs; i += 1) {
			setLED(LEDpins[i], LED_ON);
			serialPrint(F("."));
			pause(1000);
			setLED(LEDpins[i], LED_OFF);
		}
		setLEDs(LEDpins, numLEDs, LED_ON);
		serialPrint(F("."));
		pause(2000);
		setLEDs(LEDpins, numLEDs, LED_OFF);
		serialPrint(F("."));
		pause(1000);
	}

done:
	setLEDs(LEDpins, numLEDs, LED_OFF);
	serialPrintln(F("\r\nLED Test ended"));
}

/********************************************************************
 * Fan Test
 ********************************************************************/

void testFan() {
	serialPrintln(F("Fan Test"));
	
	FanController fanController(FAN_RPM_PIN, 100L, FAN_PWM_PIN);
	fanController.begin();
	fanController.getRPM();

	while (true) {
		hw.digitalWrite(FAN_ENABLE_PIN, FAN_ON);
		for (int speed = 0; speed < numFanSpeeds; speed += 1) {
			fanController.setDutyCycle(fanDutyCycles[speed]);
			serialPrintf("%s speed, duty cycle %d", FAN_SPEED_NAMES[speed], fanDutyCycles[speed]);
			for (int i = 0; i < 5; i += 1) {
				pause(1000);
				uint16_t rpm = fanController.getRPM();
				serialPrintf("   RPM %u, %ld milliAmps", rpm, getMilliAmps());
			}
		}
		hw.digitalWrite(FAN_ENABLE_PIN, FAN_OFF);
		for (int i = 0; i < 8; i += 1) {
			serialPrint(F("."));
			pause(1000);
		}
		serialPrintln(F(""));
	}

done:
	fanController.end();
	hw.digitalWrite(FAN_ENABLE_PIN, FAN_OFF);
	
	serialPrintln(F("\r\nFan Test ended"));
}

/********************************************************************
 * Speaker Test
 ********************************************************************/

void testSpeaker() {
	serialPrintln(F("Speaker Test"));
	startPB2PWM(BUZZER_FREQUENCY, BUZZER_DUTYCYCLE);
	for (int i = 0; i < 12; i += 1) {
		for (int j = 0; j < 5; j += 1) {
			serialPrint(F("."));
			pause(1000);
		}
		serialPrintf("\r\n%ld milliAmps", getMilliAmps());
	}

done:
	stopPB2PWM();
	serialPrintln(F("\r\nSpeaker Test ended"));
}

/********************************************************************
 * Button Test
 ********************************************************************/

void testButtons() {
	serialPrintln(F("Button Test"));

	setLEDs(LEDpins, numLEDs, LED_OFF);
	while (true) {
		serialPrint(F("."));
		for (int i = 0; i < 20; i += 1) {
			setLED(LEDpins[0], (hw.digitalRead(POWER_OFF_PIN) == BUTTON_PUSHED) ? LED_ON : LED_OFF);
			setLED(LEDpins[2], (hw.digitalRead(POWER_ON_PIN) == BUTTON_PUSHED) ? LED_ON : LED_OFF);
			setLED(LEDpins[4], (hw.digitalRead(FAN_DOWN_PIN) == BUTTON_PUSHED) ? LED_ON : LED_OFF);
			setLED(LEDpins[6], (hw.digitalRead(FAN_UP_PIN) == BUTTON_PUSHED) ? LED_ON : LED_OFF);
			pause(50);
		}
	}

done:
	setLEDs(LEDpins, numLEDs, LED_OFF);
	serialPrintln(F("\r\nButton Test ended"));
}

/********************************************************************
 * Voltage Reference Test
 ********************************************************************/

void testVoltageReference() {
	serialPrintln(F("Voltage Reference Test"));
	while (true) {
		// Here's a note from Brent Bolton: the printout from this test
		// is the current sensor reference voltage, which is calculated as follows:
		//      ADC reference voltage * PC1_count / 2**ADC_BITS
		// The ADC reference voltage is usually selectable in the ADC setup, but typically
		// defaults to the power supply voltage, which is 5000 milliVolts in our case. All
		// that translates to a nominal value of 511 for PC1 with a 10 - bit ADC.
		//
		// The valid range for PC1 should be the range within which it's possible
		// to calculate worst case currents in either direction, though we probably
		// want to specify it more tightly than that to allow for variations over
		// the life of the unit.
		int32_t referenceReading = hw.analogRead(REFERENCE_VOLTAGE_PIN);
		serialPrintf("%ld milliVolts", (5000L * referenceReading) / 1024L);
		pause(1000);
	}

done:
	serialPrintln(F("\r\nVoltage Reference Test ended"));
}

/********************************************************************
 * Battery Voltage Test
 ********************************************************************/

void testBatteryVoltage() {
	serialPrintln(F("Battery Voltage Test"));
	while (true) {
		serialPrintf("%ld milliVolts", (int32_t)(hw.readMicroVolts() / 1000LL));
		pause(1000);
	}

done:
	serialPrintln(F("\r\nBattery Voltage Test ended"));
}

/********************************************************************
 * Charger Detect Test
 ********************************************************************/

void testChargerDetect() {
	serialPrintln(F("Charger Detect Test"));

	// The serial port input pin is the same pin as the Charger Connected indicator. 
	// We must not use the serial port while this test is in progress.

	// Give the user a few seconds to disconnect the serial cable.
	serialPrintln(F("Please disconnect the serial cable."));
	serialPrintln(F("The test will begin when the orange LED starts flickering."));
	serialPrintln(F("When you are finished testing, please re-connect the serial cable."));
	for (int j = 0; j < numLEDs; j += 1) {
		setLED(LEDpins[j], LED_ON);
		hw.delay(500L);
		setLED(LEDpins[j], LED_OFF);
		hw.delay(500L);
	}

	// Temporarily disable the serial port and enable the charger connected indication.
	Serial.end(); 
	hw.pinMode(CHARGER_CONNECTED_PIN, INPUT_PULLUP);

	setLEDs(LEDpins, numLEDs, LED_OFF);
	bool toggle = false;
	// We can't print dots right now, so we will flash LED number 3 as the warm feeling indicator for this test.
	// Since the serial port is disabled, we cannot end the test when the user hits a key. 
	// Instead, we will end the test when the connected signal has not been received for 4 seconds (i.e. 80 * 50 milliseconds).
	for (int i = 0; i < 80; i += 1) {
		toggle = !toggle;
		setLED(LEDpins[3], toggle ? LED_ON : LED_OFF);
		hw.delay(50L);
		bool connected = hw.digitalRead(CHARGER_CONNECTED_PIN) == CHARGER_CONNECTED;
		setLED(LEDpins[0], connected ? LED_ON : LED_OFF);
		setLED(LEDpins[numLEDs - 1], connected ? LED_ON : LED_OFF);
		if (connected) {
			i = 0;
		}
	}
	setLEDs(LEDpins, numLEDs, LED_OFF);

	// Test is finished. Give the user a few seconds to reconnect the serial cable.
	for (int j = numLEDs - 1; j >= 0; j -= 1) {
		setLED(LEDpins[j], LED_ON);
		hw.delay(500L);
		setLED(LEDpins[j], LED_OFF);
		hw.delay(500L);
	}

	serialBegin(true);
	serialPrintln(F("\r\nCharger Detect Test ended"));
}

/********************************************************************
 * Current Sensor Test
 ********************************************************************/

void testCurrentSensor() {
	serialPrintln(F("Current Sensor Test"));

	while (true) {
		pause(1000);
		serialPrintf("%ld milliAmps", getMilliAmps());
	}

done:
	serialPrintln(F("\r\nCurrent Sensor Test ended"));
}

/********************************************************************
 * Setup and loop
 ********************************************************************/

void productionTestSetup() {
	hw.digitalWrite(FAN_ENABLE_PIN, FAN_OFF);

	serialPrintln(F("\r\n\n<<< Test Mode >>>"));
	serialPrintln(F("Commands are:"));
	serialPrintln(F("1 - LED test"));
	serialPrintln(F("2 - Fan test"));
	serialPrintln(F("3 - Speaker test"));
	serialPrintln(F("4 - Button test"));
	serialPrintln(F("5 - Voltage Reference test"));
	serialPrintln(F("6 - Battery Voltage test"));
	serialPrintln(F("7 - Charger Detect test"));
	serialPrintln(F("8 - Current Sensor test"));
	serialPrintln(F("r - Reset"));
}

void productionTestLoop() {
	serialPrintln(F("\r\nEnter Test Command:"));
	int command = getNextCommand();
	serialPrintln(F(""));
	switch(command) {
		case '1': testLEDs(); break;
		case '2': testFan(); break;
		case '3': testSpeaker(); break;
		case '4': testButtons(); break;
		case '5': testVoltageReference(); break;
		case '6': testBatteryVoltage(); break;
		case '7': testChargerDetect(); break;
		case '8': testCurrentSensor(); break;
		case 'r': case 'R': hw.reset(); break;
		default: serialPrintf("Unknown command '%c'", command); break;
	}
}
