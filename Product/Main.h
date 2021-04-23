/*
 * Main.h
 *
 * This defines the main program of the PAPR product firmware. This file is intended to be invoked either by
 * by the Arduino runtime (product environment) or by the unit test runtime (unit test environment).
 */
#pragma once
#include "Timer.h"
#include "Battery.h"
#ifdef UNITTEST
#include "UnitTest/MyButtonDebounce.h"
#include "UnitTest/MyFanController.h"
#else
#include "PressDetector.h"
#include "FC.h"
#endif

class PAPRMainTest;

// The different kinds of alert we can present to the user.
enum Alert { alertNone, alertBatteryLow, alertFanRPM };

// The user can choose any of these speeds
enum FanSpeed { fanLow, fanMedium, fanHigh };

// We can be either on or off, and either charging or not charging.
enum PAPRState { stateOff, stateOn, stateOffCharging, stateOnCharging };

class Main : public InterruptCallback {
public:
    Main();

    // Arduino-style main loop
    void setup();
    void loop();

    // The PressDetector object polls a pin, and calls a callback when the pin value changes. There is one PressDetector object per button.
    PressDetector buttonFanUp;
    PressDetector buttonFanDown;
    PressDetector buttonPowerOff;
    PressDetector buttonPowerOn;

    // The object that controls and monitors the fan.
    FanController fanController;

private:
    // Internal functions
    void allLEDsOff();
    void allLEDsOn();
    void setLEDs(const int* pinList, int onOff);
    void flashAllLEDs(int millis, int count);
    void onToggleAlert();
    void enterAlertState(Alert alert);
    void setFanSpeed(FanSpeed speed);
    void checkForFanAlert();
    void checkForBatteryAlert();
    void onPowerOffPress();
    void onPowerOnPress();
    void enterState(PAPRState newState);
    void nap();
    void doAllUpdates();
    void updateFanLEDs();
    void updateBatteryLEDs();
    void cancelAlert();
    bool doPowerOffWarning();

    // Event handler glue code
    static void staticToggleAlert();
    static void staticFanDownPress(const int);
    static void staticFanUpPress(const int);
    static void staticPowerOffPress(const int);
    static void staticPowerOnPress(const int);

    /********************************************************************
     * Fan data
     ********************************************************************/

     // The current fan speed selected by the user.
    FanSpeed currentFanSpeed;

    // After we change the fan speed, we stop checking the RPMs for a few seconds, to let the speed stabilize.
    unsigned long dontCheckFanSpeedUntil;

    /********************************************************************
     * Alert data
     ********************************************************************/

     // Data used when we are in the alert state.
    Alert currentAlert;
    const int* currentAlertLEDs = nullptr;
    const int* currentAlertMillis = nullptr;
    bool alertToggle;

    // The timer that pulses the lights and buzzer during an alert.
    Timer alertTimer;

    /********************************************************************
     * Etc.
     ********************************************************************/
    PAPRState paprState;
    Battery battery;

public:
    // Glue
    //unsigned long millis()
    static Main* instance;
    virtual void callback();
};
