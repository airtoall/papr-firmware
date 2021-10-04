#pragma once

#define SERIAL_ENABLED
// It's probably safe to #define SERIAL_ENABLED in the product build,
// but it's unlikely to be used, and maybe could cause trouble somehow,
// so I think it's better to #undef it in the product.

#ifdef SERIAL_ENABLED
void serialInit();
void serialPrintf(const char* __fmt, ...);
char* renderLongLong(long long num);
#else
#define serialInit()
#define serialPrintf(...)
#define renderLongLong(...)
#endif