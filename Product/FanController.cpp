#include "FanController.h"
#include "Arduino.h"
#include "Hardware.h"

#define hw Hardware::instance

FanController::FanController(byte sensorPin, uint32_t sensorThreshold, byte pwmPin)
{
	_sensorPin = sensorPin;
	_sensorThreshold = sensorThreshold;
	_pwmPin = pwmPin;
	hw.pinMode(pwmPin, OUTPUT);
}

void FanController::begin()
{
	hw.digitalWrite(_sensorPin, HIGH);
	setDutyCycle(100);
	_attachInterrupt();
	_lastMillis = hw.millis();
	_lastReading = 0;
}

void FanController::end()
{
	_detachInterrupt();
}

uint16_t FanController::getRPM() {
	uint32_t elapsed = hw.millis() - _lastMillis;
	if (elapsed > _sensorThreshold)
	{
		noInterrupts();
		float correctionFactor = 1000.0 / elapsed; // float is quite slow on our little MCU, but it's OK because we don't do it very much.
		_lastReading = correctionFactor * _halfRevs / 2 * 60;
		_halfRevs = 0;
		_lastMillis = hw.millis();
		interrupts();
	}
	return _lastReading;
}

void FanController::setDutyCycle(byte dutyCycle) {
	hw.analogWrite(_pwmPin, 2.55 * min((int)dutyCycle, 100));
}

void FanController::_attachInterrupt() {
	hw.setFanRPMInterruptCallback(this);
}

void FanController::_detachInterrupt() {
	hw.setFanRPMInterruptCallback(0);
}

// This function gets called each time there is a pin change interrupt on the RPM sensor pin
void FanController::callback() {
	if (hw.digitalRead(_sensorPin) == LOW) {
		_halfRevs++;
	}
}
