#include "MySerial.h"

#ifdef SERIAL_ENABLED
//#include "stdarg.h"
//#include "stdio.h"
#include "Arduino.h"
#include "Hardware.h"

static char buffer1[30];
static char buffer2[30];
static char buffer3[30];
static char buffer4[30];
static char* buffers[] = { buffer1, buffer2, buffer3, buffer4 };
static int nextBuffer = 0;
static const int numBuffers = 4;

void serialPrintf(const char* __fmt, ...) {
	va_list args;
	char buffer[300];
	va_start(args, __fmt);
	vsnprintf(buffer, sizeof(buffer), __fmt, args);
	va_end(args);
	Serial.println(buffer);
	Serial.flush(); // TEMP DON'T HAVE THIS IN PRODUCT
	nextBuffer = 0;
}

void serialInit() {
	Serial.begin(57600);

	// Make sure the serial software doesn't try to use pin 0 to receive data,
	// because that pin is being used as a digital input.
	UCSR0B = UCSR0B & ~(1 << RXCIE0); // disable RX Complete Interrupt Enable
	UCSR0B = UCSR0B & ~(1 << RXEN0); // disable USART Receiver.
	pinMode(CHARGER_CONNECTED_PIN, INPUT_PULLUP);
}

//char* renderDouble(double number, char* pBuffer)
//{
//	if (!pBuffer) {
//		const int size = 50;
//		static char buffer[size];
//		pBuffer = buffer;
//	}
//	bool negative;
//	if (negative = number < 0) number = -number;
//	long integerPart = (long)number;
//	double fraction = number - (double)integerPart;
//	char buff[15];
//	sprintf(buff, "%ld", (long)(fraction * 1000000.0) + 1000000);
//	sprintf(pBuffer, "%s%ld.%s", negative ? "-" : "", integerPart, &buff[1]);
//	return pBuffer;
//}

char* renderLongLong(long long num) {
	// doesn't work for LLONG_MAX + 1, a.k.a. -LLONG_MAX - 1

	if (num == 0) {
		static char* zero = "0";
		return zero;
	}

	if (nextBuffer >= numBuffers) {
		return "NO BUFFER";
	}
	char* pBuffer = buffers[nextBuffer++];

	bool negative = false;
	if (num < 0) {
		num = -num;
		negative = true;
	}

	char rev[128];
	char* p = rev + 1;
	int digitCount = 0;
	while (num > 0) {
		if (digitCount && (digitCount % 3 == 0)) *p++ = ',';
		*p++ = '0' + (num % 10);
		num /= 10;
		digitCount += 1;
	}

	int i = 0;
	if (negative) {
		pBuffer[i++] = '-';
	}

	/*Print the number which is now in reverse*/
	p--;
	while (p > rev) {
		pBuffer[i++] = *p--;
	}

	pBuffer[i++] = 0;
	return pBuffer;
}

/* code to test renderLongLong

    long long a = 922337203685475807LL;
    long long b = 382178998712349833LL;
    long long c = a + b; // should be 1304516202397825640
    serialPrintf("1: %s %s %s", renderLongLong(a), renderLongLong(b), renderLongLong(c));
    c += 100;
    serialPrintf("2: %s", renderLongLong(c));

    a = -a;
    b = -b;
    c = a + b; // should be -1304516202397825640
    serialPrintf("3: %s %s %s", renderLongLong(a), renderLongLong(b), renderLongLong(c));

    a = LLONG_MAX;
    b = -a;
    c = a + b;
    long long d = a;
    serialPrintf("4: %s %s %s %s", renderLongLong(a), renderLongLong(b), renderLongLong(c), renderLongLong(d));
*/
#else
#include "Arduino.h"
void serialInit() { }
void serialPrintf(const char* __fmt, ...) {}
char* renderLongLong(long long num) { return 0; }
#endif
