/*
 * Debugging.h
 * 
 * Temporary debug code for writing information to the LEDs.
 * This code is no longer used, because we now are able to write
 * to the serial port. This code probably doesn't compile any more
 * but maybe someday it will be useful, so I'm keeping it.
 * 
 */
#pragma once
#include "stdint.h"

 // Write a 16-bit number to the LEDs, in hex. It looks like this ...
 //    all LEDs quickly flash, left to right
 //    high-order hex digit, in binary
 //    next hex digit, in binary
 //    next hex digit, in binary
 //    low-order hex digit, in binary
 //    all LEDs quickly flash, right to left
void writeNumberToLights(uint16_t number);
