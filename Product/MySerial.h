#pragma once
#include "WString.h"

void serialBegin(bool inputAllowed);
void serialEnd();
bool serialActive();
void serialPrintf(const char* __fmt, ...);
void serialPrint(const __FlashStringHelper* string);
void serialPrintln(const __FlashStringHelper* string);
char* renderLongLong(long long num);