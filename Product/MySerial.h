// This interface gives access to the serial port
#pragma once
#include "WString.h"

void serialBegin(bool inputAllowed);
void serialEnd();
bool serialActive();
void serialPrintf(const char* __fmt, ...);
void serialPrint(const __FlashStringHelper* string);
void serialPrintln(const __FlashStringHelper* string);
void serialPrintToBuffer(char* buffer, size_t bufferSize, const char* __fmt, ...);
char* renderLongLong(int64_t num);