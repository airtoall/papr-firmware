// ProductionTest.cpp

#include "PB2PWM.h"
#include "MySerial.h"
#include "Hardware.h"
#include "FanController.h"
// #include "EEProm.h"
// #include <EEPROM.h>
#include "MiscConstants.h"


/********************************************************************
 * Misc definitions
 ********************************************************************/

#define hw Hardware::instance

// https://www.avrfreaks.net/forum/tut-c-using-eeprom-memory-avr-gcc?name=PNphpBB2&file=viewtopic&t=38417
// https://cse537-2011.blogspot.com/2011/02/accessing-internal-eeprom-on-atmega328p.html

void serialPrint(const char* const string) {
	Serial.print(string);
	Serial.flush();
}

//bool shouldEnterTestMode() {
//	testMode = false;
//
//	//uint8_t runMode = eeprom_read_byte(RUN_MODE_ADDR);
//	uint8_t runMode = EEPROM.read(RUN_MODE_ADDR);
//	if (runMode != NORMAL_MODE_FLAG) {
//		testMode = true;
//	} 
//
//	/*else {
//    	hw.pinMode(POWER_OFF_PIN, INPUT_PULLUP);
//		hw.pinMode(POWER_ON_PIN, INPUT_PULLUP);
//		hw.pinMode(FAN_DOWN_PIN, INPUT_PULLUP);
//		hw.pinMode(FAN_UP_PIN, INPUT_PULLUP);
//
//		testMode = hw.digitalRead(POWER_OFF_PIN) == BUTTON_PUSHED
//			      && hw.digitalRead(POWER_ON_PIN)  == BUTTON_PUSHED
//			      && hw.digitalRead(FAN_DOWN_PIN)  == BUTTON_RELEASED
//			      && hw.digitalRead(FAN_UP_PIN)    == BUTTON_RELEASED;
//	}*/
//	return testMode;
//}
//
//bool inTestMode() {
//	return testMode;
//}

long getMilliAmps() {
	for (int j = 0; j < 20; j += 1) hw.readMicroAmps();
	return (long)(hw.readMicroAmps() / -1000LL);
}

/********************************************************************
 * Command handling
 ********************************************************************/

bool isLineEnd(int c) {
	return (c == 10) || (c == 13);
}

int getNextCommand() {
	while (true) {
		int c;
		while (c = Serial.peek(), isLineEnd(c)) {
			Serial.read();
		}
		if (c != -1) {
			int result = Serial.read();
			hw.delay(100);
			while (Serial.peek() != -1) {
				Serial.read();
			}

			return result;
		}
	}
}

bool doPause(int waitTimeMillis) {
	while (waitTimeMillis > 0) {
		if (Serial.peek() != -1) {
			hw.delay(100);
			while (Serial.peek() != -1) Serial.read();
			return false;
		}
		hw.delay(50);
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
	serialPrintf("LED Test");
	while (true) {
		for (int i = 0; i < numLEDs; i += 1) {
			setLED(LEDpins[i], LED_ON);
			serialPrint(".");
			pause(1000);
			setLED(LEDpins[i], LED_OFF);
		}
		setLEDs(LEDpins, numLEDs, LED_ON);
		serialPrint(".");
		pause(2000);
		setLEDs(LEDpins, numLEDs, LED_OFF);
		serialPrint(".");
		pause(1000);
	}

done:
	setLEDs(LEDpins, numLEDs, LED_OFF);
	serialPrintf("\r\nLED Test ended");
}

/********************************************************************
 * Fan Test
 ********************************************************************/

void testFan() {
	serialPrintf("Fan Test");

	FanController fanController(FAN_RPM_PIN, 100, FAN_PWM_PIN);
	fanController.begin();
	fanController.getRPM();

	while (true) {
		hw.digitalWrite(FAN_ENABLE_PIN, FAN_ON);
		for (int speed = 0; speed < numFanSpeeds; speed += 1) {
			fanController.setDutyCycle(fanDutyCycles[speed]);
			serialPrintf("%s speed, duty cycle %d", FAN_SPEED_NAMES[speed], fanDutyCycles[speed]);
			for (int i = 0; i < 5; i += 1) {
				pause(1000);
				serialPrintf("   RPM %u, %ld milliAmps", fanController.getRPM(), getMilliAmps());
			}
		}
		hw.digitalWrite(FAN_ENABLE_PIN, FAN_OFF);
		for (int i = 0; i < 8; i += 1) {
			serialPrint(".");
			pause(1000);
		}
		serialPrintf("");
	}

done:
	//fanController.end();
	hw.digitalWrite(FAN_ENABLE_PIN, FAN_OFF);
	serialPrintf("\r\nFan Test ended");
}

/********************************************************************
 * Speaker Test
 ********************************************************************/

void testSpeaker() {
	serialPrintf("Speaker Test");
	startPB2PWM(BUZZER_FREQUENCY, BUZZER_DUTYCYCLE);
	for (int i = 0; i < 12; i += 1) {
		for (int j = 0; j < 5; j += 1) {
			serialPrint(".");
			pause(1000);
		}
		serialPrintf("\r\n%ld milliAmps", getMilliAmps());
	}

done:
	stopPB2PWM();
	serialPrintf("\r\nSpeaker Test ended");
}

/********************************************************************
 * Button Test
 ********************************************************************/

void testButtons() {
	serialPrintf("Button Test");

	setLEDs(LEDpins, numLEDs, LED_OFF);
	while (true) {
		serialPrint(".");
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
	serialPrintf("\r\nButton Test ended");
}

/********************************************************************
 * Voltage Reference Test
 ********************************************************************/


void testVoltageReference() {
	serialPrintf("Voltage Reference Test");
	while (true) {
		// Here's a note from Brent Bolton: this value is calculated as follows:
		//      ADC reference voltage * PC1_count / 2**ADC_BITS
		// The ADC reference voltage is usually selectable in the ADC setup, but typically
		// defaults to the power supply voltage, which is 5000 milliVolts in our case. All
		// that translates to a nominal value of 511 for PC1 with a 10 - bit ADC.
		//
		// The valid range for PC1 should be the range within which it's possible
		// to calculate worst case currents in either direction, though we probably
		// want to specify it more tightly than that to allow for variations over
		// the life of the unit.
		long referenceReading = hw.analogRead(REFERENCE_VOLTAGE_PIN);
		serialPrintf("%ld milliVolts", (5000L * referenceReading) / 1024L);
		pause(1000);
	}

done:
	serialPrintf("\r\nVoltage Reference Test ended");
}

/********************************************************************
 * Battery Voltage Test
 ********************************************************************/

void testBatteryVoltage() {
	serialPrintf("Battery Voltage Test");
	while (true) {
		serialPrintf("%ld milliVolts", (long)(hw.readMicroVolts() / 1000LL));
		pause(1000);
	}

done:
	serialPrintf("\r\nBattery Voltage Test ended");
}

/********************************************************************
 * Charger Detect Test
 ********************************************************************/

void testChargerDetect() {
	serialPrintf("Charger Detect Test");

	Serial.end();
	//UCSR0B = UCSR0B & ~(1 << RXCIE0); // disable RX Complete Interrupt Enable
	//UCSR0B = UCSR0B & ~(1 << RXEN0); // disable USART Receiver.
	hw.pinMode(CHARGER_CONNECTED_PIN, INPUT_PULLUP);

	setLEDs(LEDpins, numLEDs, LED_OFF);
	bool toggle = false;
	for (int i = 0; i < 200; i += 1) {
		bool connected = hw.digitalRead(CHARGER_CONNECTED_PIN) == CHARGER_CONNECTED;
		if (connected) {
			serialPrintf("connected, cc pin %d", hw.digitalRead(CHARGER_CONNECTED_PIN));
		}
		setLED(LEDpins[0], connected ? LED_ON : LED_OFF);
		setLED(LEDpins[numLEDs - 1], connected ? LED_ON : LED_OFF);
		toggle = !toggle;
		setLED(LEDpins[3], toggle ? LED_ON : LED_OFF);
		hw.delay(50);
		if (connected) {
			i = 0;
		}
	}
	setLEDs(LEDpins, numLEDs, LED_OFF);

	serialInit(true);
	serialPrintf("\r\nCharger Detect Test ended");
}

/********************************************************************
 * Current Sensor Test
 ********************************************************************/

void testCurrentSensor() {
	serialPrintf("Current Sensor Test");

	while (true) {
		pause(1000);
		serialPrintf("%ld milliAmps", getMilliAmps());
	}

done:
	serialPrintf("\r\nCurrent Sensor Test ended");
}

/********************************************************************
 * 
 ********************************************************************/

// void exitProductionTest() {
// 	serialPrintf("\r\Production Test marked 'PASSED'. Test Mode is terminated, future boots will be Normal Mode.");
// 
// 	EEPROM.write(RUN_MODE_ADDR, NORMAL_MODE_FLAG);
// 	//eeprom_write_byte(RUN_MODE_ADDR, NORMAL_MODE_FLAG);
// }

/********************************************************************
 * Setup and loop
 ********************************************************************/

/*
void productionTestSetup() {
	hw.watchdogStartup();
	hw.setPowerMode(fullPowerMode);

	hw.pinMode(CHARGER_CONNECTED_PIN, INPUT_PULLUP);
	hw.pinMode(CHARGING_LED_PIN, OUTPUT);
	hw.pinMode(BATTERY_LED_LOW_PIN, OUTPUT);
	hw.pinMode(FAN_UP_PIN, INPUT_PULLUP);
	hw.pinMode(FAN_HIGH_LED_PIN, OUTPUT);
	DDRB |= (1 << DDB6); // pinMode(FAN_ENABLE_PIN, OUTPUT); // we can't use pinMode because it doesn't support pin PB6

	hw.digitalWrite(FAN_ENABLE_PIN, FAN_OFF);
}

void productionTestLoop() {
	int count = 0;
	bool toggle = false;
	while (true) {
		if (++count == 10000) {
			hw.digitalWrite(BATTERY_LED_LOW_PIN, toggle ? LED_ON : LED_OFF);
			count = 0;
			toggle = !toggle;
		}

		bool connected = hw.digitalRead(CHARGER_CONNECTED_PIN) == CHARGER_CONNECTED;
		hw.digitalWrite(CHARGING_LED_PIN, connected ? LED_ON : LED_OFF);

		bool buttonPushed = hw.digitalRead(FAN_UP_PIN) == BUTTON_PUSHED;
		hw.digitalWrite(FAN_HIGH_LED_PIN, buttonPushed ? LED_ON : LED_OFF);
	}
}
*/

void productionTestSetup() {
	// hw.setup();
	// hw.digitalWrite(FAN_ENABLE_PIN, FAN_OFF);
	// delay(1000UL);
	// serialInit(true);
	hw.digitalWrite(FAN_ENABLE_PIN, FAN_OFF);

	serialPrintf("\r\n\n<<< Test Mode >>>");
	serialPrintf("Commands are:");
	serialPrintf("1 - LED test");
	serialPrintf("2 - Fan test");
	serialPrintf("3 - Speaker test");
	serialPrintf("4 - Button test");
	serialPrintf("5 - Voltage Reference test");
	serialPrintf("6 - Battery Voltage test");
	serialPrintf("7 - Charger Detect test");
	serialPrintf("8 - Current Sensor test");
}

void productionTestLoop() {
	serialPrintf("\r\nEnter Test Command:");
	int command = getNextCommand();
	serialPrintf("");
	serialPrintf("command %c", command);
	switch(command) {
		case '1': testLEDs(); break;
		//case '2': testFan(); break;
		case '3': testSpeaker(); break;
		case '4': testButtons(); break;
		//case '5': testVoltageReference(); break;
		//case '6': testBatteryVoltage(); break;
		//case '7': testChargerDetect(); break;
		//case '8': testCurrentSensor(); break;
		default: serialPrintf("Unknown command '%c'", command); break;
	}
}
