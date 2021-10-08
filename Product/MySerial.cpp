#include "MySerial.h"
#include "stdarg.h"
#include "stdio.h"
#include "Hardware.h"

static char buffer1[30];
static char buffer2[30];
static char buffer3[30];
static char buffer4[30];
static char* buffers[] = { buffer1, buffer2, buffer3, buffer4 };
static int nextBuffer = 0;
static const int numBuffers = 4;
static bool _serialActive = false;

void serialPrintf(const char* __fmt, ...) {
	if (!_serialActive) return;
	va_list args;
	char buffer[300];
	va_start(args, __fmt);
	vsnprintf(buffer, sizeof(buffer), __fmt, args);
	va_end(args);
	Serial.println(buffer);
	Serial.flush();
	nextBuffer = 0;
}

void serialPrint(const __FlashStringHelper* string) {
	if (!_serialActive) return;
	Serial.print(string);
	Serial.flush();
}

void serialPrintln(const __FlashStringHelper* string) {
	if (!_serialActive) return;
	Serial.println(string);
	Serial.flush();
}

void serialBegin(bool inputAllowed) {
	Serial.begin(57600);

	if (!inputAllowed) {
		// Make sure the serial software doesn't try to use pin 0 to receive data,
		// because that pin is being used as a digital input.
		UCSR0B = UCSR0B & ~(1 << RXCIE0); // disable RX Complete Interrupt Enable
		UCSR0B = UCSR0B & ~(1 << RXEN0); // disable USART Receiver.
		pinMode(CHARGER_CONNECTED_PIN, INPUT_PULLUP);
	}

	_serialActive = true;
}

void serialEnd() {
	_serialActive = false;
	Serial.end();
}

bool serialActive() {
	return _serialActive;
}
char* renderLongLong(long long num) {
	// doesn't work for LLONG_MAX + 1, a.k.a. -LLONG_MAX - 1

	if (_serialActive) return "";

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
