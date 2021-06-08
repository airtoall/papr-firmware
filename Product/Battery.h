#pragma once

class Battery {
public:
    bool isCharging();
    void update();
    void wakeUp();
    long long getPicoCoulombs() { return picoCoulombs; }
    void notifySystemActive(bool active) { systemActive = active; }
    void DEBUG_incrementPicoCoulombs(long long increment);
    void initializeCoulombCount();
    Battery();

private:
    void updateBatteryVoltage();
    void updateBatteryTimers();
    static long long estimatePicoCoulombsFromVoltage(long long microVolts);

    long long picoCoulombs; // How much charge is in the battery right now.
    long long microVolts;   // The voltage right now.
    unsigned long lastCoulombsUpdateMicroSecs;
    unsigned long chargeStartMilliSecs;       // millisecond timestamp of when the battery charger started up
    unsigned long lastVoltageChangeMilliSecs; // millisecond timestamp of when the battery voltage last changed
    bool prevIsCharging;
    long long prevMicroVolts;
    bool systemActive;
    bool maybeChargingFinished;
    unsigned long maybeChargingFinishedMilliSecs;
};