#include "FanController.h"
#include "PressDetector.h"
#include "MySerial.h"
#include "Hardware.h"
#include "Recorder.h"
#include "Battery.h"
#include <limits.h>

#define hw Hardware::instance

// This app exercises all the pins defined in Hardware.h

///////////////////////////////////////////////////////////////////////
//
// Data
//
///////////////////////////////////////////////////////////////////////

Battery battery;
FanController fanController(FAN_RPM_PIN, 1000, FAN_PWM_PIN);
int currentDutyCycle;
bool toneOn = false;

void onUpButton();
void onDownButton();
void onOnButton();
void onOffButton();

PressDetector offButton(POWER_OFF_PIN, 100, onOffButton);
PressDetector onButton(POWER_ON_PIN, 100, onOnButton);
PressDetector downButton(FAN_DOWN_PIN, 100, onDownButton);
PressDetector upButton(FAN_UP_PIN, 100, onUpButton);

int increment = 10;
bool skipUpRelease = false;
bool skipDownRelease = false;

///////////////////////////////////////////////////////////////////////
//
// Code
//
///////////////////////////////////////////////////////////////////////

void allLEDs(int state)
{
    hw.digitalWrite(BATTERY_LED_LOW_PIN, state);
    hw.digitalWrite(BATTERY_LED_MED_PIN, state);
    hw.digitalWrite(BATTERY_LED_HIGH_PIN, state);
    hw.digitalWrite(CHARGING_LED_PIN, state);
    hw.digitalWrite(FAN_LOW_LED_PIN, state);
    hw.digitalWrite(FAN_MED_LED_PIN, state);
    hw.digitalWrite(FAN_HIGH_LED_PIN, state);
}

void flashLEDs(int interval, int count)
{
    while (count-- > 0) {
        allLEDs(LED_ON);
        hw.delay(interval);
        allLEDs(LED_OFF);
        hw.delay(interval);
    }
}

void setFanDutyCycle(int dutyCycle)
{
    if (dutyCycle < -10) dutyCycle = -10;
    if (dutyCycle > 100) dutyCycle = 100;
    currentDutyCycle = dutyCycle;
    if (currentDutyCycle >= 0) {
        hw.digitalWrite(FAN_ENABLE_PIN, FAN_ON);
        fanController.setDutyCycle(dutyCycle);
    } else {
        hw.digitalWrite(FAN_ENABLE_PIN, FAN_OFF);
    }
    resetRecorder();
    serialPrintf("Duty cycle %d\r\n\r\n", dutyCycle);
}

void onUpButton()
{
    setFanDutyCycle(currentDutyCycle + increment);
}

void onDownButton()
{
    setFanDutyCycle(currentDutyCycle - increment);
}

void onOffButton() {
    toneOn = !toneOn;
    if (toneOn) {
        hw.analogWrite(BUZZER_PIN, 128);
    }
    else {
        hw.analogWrite(BUZZER_PIN, 0);
    }
    serialPrintf("Sound is now %s", toneOn ? "on" : "off");
}

void onOnButton() {
    increment = (increment == 1) ? 10 : 1;
    serialPrintf("Increment is now %d", increment);
}

void initializeSerial() {
    Serial.begin(57600);
    hw.delay(10);
    Serial.println("\n\nPAPR Calibrator for Rev 3.1A board");
    Serial.println("Off button: toggle sound");
    Serial.println("On button: toggle increment");
    Serial.println("Down button: decrease fan speed");
    Serial.println("Up button: increase fan speed\n");
}

class PowerOnButtonInterruptCallback : public InterruptCallback {
public:
    virtual void callback() {
        serialPrintf("PowerOnButtonInterruptCallback, button is now %s", 
            (hw.digitalRead(POWER_ON_PIN) == BUTTON_PUSHED) ? "pushed" : "released");
    }
};

PowerOnButtonInterruptCallback powerOnButtonInterruptCallback;

//unsigned long loopCount;
//unsigned long startMillis;

void setup()
{
    hw.setup();
    initializeSerial();
    fanController.begin();
    setFanDutyCycle(-10);
    hw.digitalWrite(FAN_HIGH_LED_PIN, LED_ON);
    hw.digitalWrite(BATTERY_LED_LOW_PIN, LED_ON);
    hw.setPowerOnButtonInterruptCallback(&powerOnButtonInterruptCallback);
    //loopCount = 0;
    //startMillis = hw.millis();

    // test the long long datatype
    //#define LLONG_MAX 9223372036854775807LL
    //long long a = LLONG_MAX;
    //long long b = LONG_MAX;
    //long long c = LONG_MAX / 100;
    //serialPrintf("long long test: %s %s %s %s", renderLongLong(a), renderLongLong(0), renderLongLong(a / 1000000LL), renderLongLong(a / a));
    //serialPrintf("long long test: %s %s %s %s", renderLongLong(-a), renderLongLong(b * 1000000LL), renderLongLong(b * c), renderLongLong(b * c / 1000));
}

//unsigned long lastExtraInfoMillis = 0;

//unsigned long lastHeartBeatToggleMilliSecs = 0;
//bool heartBeatToggle = false;

void loop()
{
    //unsigned long nowMilliSecs = hw.millis();
    //if (nowMilliSecs - lastHeartBeatToggleMilliSecs > 2000) {
    //    lastHeartBeatToggleMilliSecs = nowMilliSecs;
    //    heartBeatToggle = !heartBeatToggle;
    //    hw.digitalWrite(CHARGING_LED_PIN, heartBeatToggle ? LED_ON : LED_OFF);
    //}
    offButton.update();
    onButton.update();
    downButton.update();
    upButton.update();
    fanController.getRPM();
    //unsigned long nowMilliSecs = hw.millis();
    //bool extraInfo = false;
    //if (nowMilliSecs - lastExtraInfoMillis > 2000) {
    //    lastExtraInfoMillis = nowMilliSecs;
    //    extraInfo = true;
    //}
    battery.update();
    updateRecorder(fanController.getRPM(), currentDutyCycle, battery.isCharging(), battery.getPicoCoulombs());

    //if (!battery.isCharging()) {
    //    serialPrintf("------- not charging"); // happens quite often when there is no battery connected
    //}

    //loopCount += 1;
    //if (hw.millis() - startMillis >= 10000) {
    //    serialPrintf("%ld loops in %d seconds", loopCount, 10);
    //    loopCount = 0;
    //    startMillis = hw.millis();
    //}
}