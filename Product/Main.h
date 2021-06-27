/*
 * Main.h
 *
 * This defines the main program of the PAPR product firmware. This file is intended to be invoked either by
 * by the Arduino runtime (product environment) or by the unit test runtime (unit test environment).
 */
#pragma once
#include "OneTimeCallback.h"
#include "Battery.h"
#include "PeriodicCallback.h"
#ifdef UNITTEST
#include "UnitTest/MyButtonDebounce.h"
#include "UnitTest/MyFanController.h"
#else
#include "PressDetector.h"
#include "FanController.h"
#endif

class PAPRMainTest;

// The different kinds of alert we can present to the user.
enum Alert { alertNone, alertBatteryLow, alertFanRPM };

// The user can choose any of these speeds
enum FanSpeed { fanLow, fanMedium, fanHigh };

// PAPRState is the states that the user perceives: the power is either on or off,
// and the charger is either connected or disconnected. In reality, the MCU and the PCB
// starts running when a battery is first connected and runs continuously until the battery is 
// removed or is exhausted. If the user never drains the battery and keeps the battery charged as needed,
// this software will keep running for months at a time. When we are in stateOff or stateOffCharging,
// the user thinks the device is turned off, but really we just turn off the fan and most/all of the lights, 
// and then monitor the charging (if stateOffCharging) or take a low-power nap (if stateOff).
enum PAPRState { stateOff, stateOn, stateOffCharging, stateOnCharging };

class Main : public InterruptCallback {
public:
    Main();

    // Arduino-style main loop
    void setup();
    void loop();

    // The PressDetector object polls a pin, and calls a callback when the pin value changes.
    // There is one PressDetector object per button.
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
    void setLED(const int pin, int onOff);
    void setLEDs(const int* pinList, int onOff);
    void flashAllLEDs(int millis, int count);
    void onToggleAlert();
    void onChargeReminder();
    void onStatusReport();
    void onBeepTimer();
    void raiseAlert(Alert alert);
    void setFanSpeed(FanSpeed speed);
    void checkForFanAlert();
    void checkForBatteryAlert();
    void onPowerOffPress();
    void onPowerOnPress();
    void onFanDownPress();
    void onFanUpPress();
    void enterState(PAPRState newState);
    void nap();
    void doAllUpdates();
    void updateFanLEDs();
    void updateBatteryLEDs();
    void cancelAlert();
    bool doPowerOffWarning();
    int getBatteryPercentFull();
    void setBuzzer(int onOff);
    const char* currentAlertName() { return (currentAlert == alertNone) ? "no" : ((currentAlert == alertBatteryLow) ? "batt" : "fan"); }
    
    /********************************************************************
     * Fan data
     ********************************************************************/

     // The current fan speed selected by the user.
    FanSpeed currentFanSpeed;

    // After we change the fan speed, we stop checking the RPMs for a few seconds, to let the speed stabilize.
    unsigned long lastFanSpeedChangeMilliSeconds;
    bool fanSpeedRecentlyChanged;

    /********************************************************************
     * Alert data
     ********************************************************************/

     // Data used when an alert is active.
    Alert currentAlert;
    const int* currentAlertLEDs = nullptr;
    const int* currentAlertMillis = nullptr;
    bool alertToggle;

    // The timer that pulses the lights and buzzer during an alert.
    OneTimeCallback alertTimer;

    // The timer that does the reminder beeps when the battery gets below 15%.
    PeriodicCallback chargeReminder;
    OneTimeCallback beepTimer;

    /********************************************************************
     * Etc.
     ********************************************************************/

    // What state are we currently in?
    PAPRState paprState;

    // This object keeps track of the battery state-of-charge.
    Battery battery;

    // Data for the periodic status reports that we send to the serial port. 
    // For testing and debugging use.
    int ledState[numLEDs];          // the current state of the LEDs
    int buzzerState;                // the current state of the buzzer
    PeriodicCallback statusReport;  // a timer that periodically triggers a status report

public:
    // Glue
    static Main* instance;   // the one and only instance of Main.
    virtual void callback(); // handler for Power On button pin-change interrupts
};
