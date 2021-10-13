#pragma once
/*
 * Battery.h
 *
 * The Battery class encapsulates all the code that manages the battery. This class provides
 * 2 key pieces of information: (1) how full is the battery? (2) is the charger attached? 
 */

 // The charger can be in various states.
enum ChargerStatus {
    chargerNotConnected,    // the charger is not connected
    chargerCharging,        // the charger is connected, and the battery is charging
    chargerFull,            // the charger is connected, and the battery is not charging because it's full
    chargerError            // the charger is connected, and the battery is not charging because of an error
                            // (the error might be that the charger is not connected to power, or maybe there is an equipment malfunction).
};

class Battery {
public:
    // The constructor. You only need one instance of this class.        
    Battery();

    // Is the charger currently connected?
    // CAUTION: this only means that the charger is connnected to our PCB. It does NOT
    // mean that the charger is plugged into the wall and receiving power.
    bool isChargerConnected();

    // What is the status of the charger right now?
    ChargerStatus getChargerStatus();

    // How much charge is currently in the battery? 
    //
    // You should not assume that this is accurate to within a picoCoulomb. The accuracy depends on
    // the accuracy of the current sensor hardware and the MCU clock, so it's probably only
    // within a few percent. The reason we use the "long long" data type is to minimize
    // cumulative rounding errors in the coulomb counting algorithm. We use "long long" instead of
    // "double" because our C++ compiler doesn't fully support double.
    long long getPicoCoulombs() { return picoCoulombs; }

    // You must call this function periodically, ideally every few milliseconds. Exception: we don't
    // expect you to call it when the system is sleeping (and therefore consuming neglible power).
    void update();

    // You must call this function when the system wakes up from sleeping.
    void wakeUp();

    // For testing and debugging: change the current coulomb count, to fool the system into
    // thinking the battery is more (or less) charged that it really is.
    void DEBUG_incrementPicoCoulombs(long long increment);

    // When the system is starting up, call this function.
    void initializeCoulombCount();

private:
    // Update the timer information that we use to determine when the battery is fully charged.
    void updateBatteryTimers();

    // When the system first starts up, we have no idea how much charge is in the battery.
    // This function estimates the charge based on the voltage reading. This is a rough
    // estimate based on measurements from a "typical" battery. It is not very reliable
    // because of the nature of Li-ion batteries, but it's better than nothing. Anyway,
    // we only rely on the estimate until the first time the battery becomes fully charged,
    // at which time we know how much charge it has.
    static long long estimatePicoCoulombsFromVoltage(long long microVolts);

    // Return the voltage that the battery will have when it is fully charged.
    static long long getFullyChargedMicrovolts();

    // Update the saved value of the battery's fully-charged voltage.
    static void updateFullyChargedMicrovolts(long long microVolts);

    long long picoCoulombs; // How much charge is in the battery right now.
    long long microVolts;   // The voltage right now.
    unsigned long lastCoulombsUpdateMicroSecs;// microsecond timestamp of when we last sampled the current flow
    unsigned long chargeStartMilliSecs;       // millisecond timestamp of when the battery charger started up
    unsigned long lastVoltageChangeMilliSecs; // millisecond timestamp of when the battery voltage last changed
    bool prevIsChargerConnected;      // what was "isChargerConnected" the last time we checked
    long long prevMicroVolts; // what was "microVolts" the last time we checked
    bool maybeChargingFinished;  // If true, then we suspect that the battery is now fully charged
    unsigned long maybeChargingFinishedMilliSecs;  // millisecond timestamp of when we first suspected charge done
};