/*
 * A class that knows how to control the fan. This code is based on Giorgio Aresu's Arduino FanController library.
*/
#pragma once
#include "Hardware.h"

class FanController : public InterruptCallback
{
public:
	FanController(byte sensorPin, unsigned long sensorThreshold, byte pwmPin = 0);
	void begin();
	void end();
	unsigned int getRPM();
	void setDutyCycle(byte dutyCycle);
	
private:
	void _attachInterrupt();
	void _detachInterrupt();
	byte _sensorPin;
	unsigned long _sensorThreshold;
	byte _pwmPin;
	unsigned int _lastReading;
	volatile unsigned int _halfRevs;
	unsigned long _lastMillis;

public:
	virtual void callback();
};
