#pragma once
#include "WString.h"

#define SERIAL_ENABLED
// It's probably safe to #define SERIAL_ENABLED in the product build,
// but it's unlikely to be used, and maybe could cause trouble somehow,
// so I think it's better to #undef it in the product.

#ifdef SERIAL_ENABLED
void serialInit(bool inputAllowed);
void serialPrintf(const char* __fmt, ...);
void serialPrint(const __FlashStringHelper* string);
void serialPrintln(const __FlashStringHelper* string);
char* renderLongLong(long long num);
#else
#define serialInit()
#define serialPrintf(...)
#define renderLongLong(...)
#define serialPrint(...)
#define serialPrintln(...)
#endif