/*
 * A class that knows how to control the fan. This code is based on Giorgio Aresu's Arduino FanController library.
*/
#pragma once
#include "Hardware.h"

class FanController : public InterruptCallback
{
public:
	FanController(byte sensorPin, uint32_t sensorThreshold, byte pwmPin = 0);
	void begin();
	void end();
	uint16_t getRPM();
	void setDutyCycle(byte dutyCycle);
	
private:
	void _attachInterrupt();
	void _detachInterrupt();
	byte _sensorPin;
	uint32_t _sensorThreshold;
	byte _pwmPin;
	uint16_t _lastReading;
	volatile uint16_t _halfRevs;
	uint32_t _lastMillis;

public:
	virtual void callback();
};
