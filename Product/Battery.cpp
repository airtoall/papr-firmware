#include "Battery.h"
#include "Hardware.h"
#include "MySerial.h"

#define hw Hardware::instance

/* The main purpose of the Battery class is to keep track of how much charge is currently in the battery.
 * We do this using the "Coulomb Counting" technique, which is discussed widely on the internet.
 * The algorithm is simple: measure the current flowing into or out of the battery, and measure how much time
 * the current has been flowing. From this, we can deduce the amount of charge that has gone into,
 * or out of the battery. Over time, the accuracy decreases due to cumulative roundoff error, as well
 * as inaccuracies in the time and current measurements. We minimize this loss of accuracy by detecting
 * each time the battery becomes fully charged, because at that point we know exactly how much charge
 * is in the battery. 
 *
 * Our charger uses the "constant-current constant-voltage" method to charge the battery. When the charger
 * is first connected to the battery, the charger provides a voltage that is slightly higher than the 
 * battery's, which causes current to start flowing into the battery. The charger keeps increasing the 
 * voltage until the current reaches the allowed maximum (for our system, this appears to be around 2.6 amperes).
 * As the battery fills with charge, the charger keeps changing the voltage as needed to keep the current constant.
 * When it reaches a certain maximum voltage, the charger stops increasing the voltage. The current continues
 * to flow, but gradually reduces as the battery reaches full charge.
 * At that point, the current drops to 0 and we're done.
 *
 * You can see measurements of voltage and charge flow in the following documents:
 * https://docs.google.com/spreadsheets/d/14-mchRN22HC6OSyAcN329NEcRRjF2_VMbKz3yHDDEoI
 * https://docs.google.com/spreadsheets/d/1fPnn2ukakk8MpyGW_KrOW2ediHh8FU6yWr4Kfq2UNJs
 * 
 * The PCB provides information about battery and charger on 4 input pins:
 * BATTERY_VOLTAGE_PIN - a 10-bit ADC that gives voltage. If no charger is connected, this is 
 *                       the battery's voltage. If a charger is connected, then the charger and battery are
 *                       effectively wired together so this is the voltage of both.
 * CHARGE_CURRENT_PIN - a 10-bit ADC that gives the charge flowing into or out of the battery. This value
 *                      must be combined with the REFERENCE_VOLTAGE_PIN reading to get a proper result. 
 *                      When no charger is connected, charge will be flowing out of the battery. When a
 *                      a charger is connected, charge will typically be flowing into the battery, however
 *                      if the battery is full and the system is consuming a lot of power, then the battery
 *                      and the charger will share the load and so you might see that charge is flowing out.
 * REFERENCE_VOLTAGE_PIN - a 10-bit ADC. Readings from this ADC must be combined with CHARGE CURRENT readings.
 * CHARGER_CONNECTED_PIN - a 1-bit digital input that says whether or not a charger is connected.
 */

// Some parameters used by the coulomb counting algorithm.
const unsigned long BATTERY_VOLTAGE_UPDATE_INTERVAL_MILLISECS = 500UL;
const unsigned long CHARGER_WINDDOWN_TIME_MILLIS = 1UL * 60UL * 1000UL; // 1 minute in milliseconds
const long long CHARGE_MICRO_AMPS_WHEN_FULL = 200000LL; // 0.2 Amps
const long long BATTERY_MICRO_VOLTS_CHANGED_THRESHOLD = 100000LL; // 0.1 volts

// Whenever we wake up from sleeping, we have to re-inititialize all the data used for coulomb counting.
// We don't coulomb count when the system is sleeping, because the amount of current flow is 
// negligible during sleep, and because you can't run code when you're sleeping!
void Battery::wakeUp() {
    lastCoulombsUpdateMicroSecs = hw.micros();
    microVolts = 20000000LL;
    chargeStartMilliSecs = hw.millis();
    lastVoltageChangeMilliSecs = hw.millis();
    prevIsCharging = false;
    prevMicroVolts = 0;
    maybeChargingFinished = false;
}

// Given a battery voltage, estimate how much charge is in the battery. This little ad-hoc
// formula is roughly based on measurements I made of a brand-new battery, minus 2000 coulombs
// to be conservative. You can see the raw data in the "Time vs. Battery/Charger Voltage"
// and "Time vs. Battery Charge" charts of the document at
// https://docs.google.com/spreadsheets/d/14-mchRN22HC6OSyAcN329NEcRRjF2_VMbKz3yHDDEoI
//
// Caveat: the relationship between voltage and state-of-charge in Li-ion batteries
// is very unreliable. The best way to determine the true state-of-charge is to fully charge the battery.
long long Battery::estimatePicoCoulombsFromVoltage(long long microVolts) {    
    const long milliVolts = ((long)microVolts) / 1000L;
    long coulombs;

    if (milliVolts >= 20000L) {
        coulombs = 3000L + ((milliVolts - 20000L) * 4);
    } else {
        coulombs = (milliVolts - 16500L);
    }

    return ((long long)coulombs) * 1000000000000LL;
}

// Function to initialize the coulomb counter. We can't do this in the Battery constructor,
// because the constructor runs before the hardware is fully initialized.
void Battery::initializeCoulombCount() {
    picoCoulombs = constrain(estimatePicoCoulombsFromVoltage(hw.readMicroVolts()), 0LL, BATTERY_CAPACITY_PICO_COULOMBS);
}

Battery::Battery()
{
    wakeUp();
}

// The Charger Connected pin indicates whether the charger is connected. It DOES NOT tell you if the 
// battery is charging - for that you have to look at the current sensor to see if charge is flowing
// into or out of the battery.
bool Battery::isCharging()
{
    return hw.digitalRead(CHARGER_CONNECTED_PIN) == CHARGER_CONNECTED;
}

// The coloumb counting algorithm (in Battery::update()) needs to know how long ago the voltage changed,
// and how long ago the battery started charging. This function keeps track of that information in
// the variables "chargeStartMilliSecs" and "lastVoltageChangeMilliSecs".
void Battery::updateBatteryTimers()
{
    bool isChargingNow = isCharging();
    if (isChargingNow && !prevIsCharging) {
        // we have just started charging
        chargeStartMilliSecs = hw.millis();
        //serialPrintf("Start charging at %ld", chargeStartMilliSecs);
    }
    prevIsCharging = isChargingNow;

    // Update "microVolts" which is just a smoothed version of hw.readMicroVolts(). 
    // We do a low pass filter to smooth out random variations in the readings.
    // This is probably not necessary because the readings are very stable, 
    // and because we already have a margin of slop.
    const long long lowPassFilterN = 100LL;
    microVolts = ((microVolts * lowPassFilterN) + hw.readMicroVolts()) / (lowPassFilterN + 1LL);

    if (abs(microVolts - prevMicroVolts) >= BATTERY_MICRO_VOLTS_CHANGED_THRESHOLD) {
        // voltage has changed since last time we checked
        lastVoltageChangeMilliSecs = hw.millis();
        prevMicroVolts = microVolts;
    }
}

// This function implements the Coulomb Counting algorithm.
void Battery::update()
{
    if (hw.getPowerMode() != fullPowerMode) {
        // this should never happen!
        return;
    }

    updateBatteryTimers();

    // TaKe a sample of the current. Read the clock and the current as close as possible to the same moment.
    unsigned long nowMicroSecs = hw.micros();
    long long chargeFlowMicroAmps = hw.readMicroAmps();
    // Note: there is a lot of random variation in charge flow readings (maybe 5-10%).
    // This is not a problem because the data will get smoothed as we accumulate picoCoulombs
    // in many small increments.

    // Calculate the time interval between this sample and the previous, then use that to calculate how much charge
    // has flowed into/outof the battery since the last sample. We will assume that the current remained constant
    // between the current and previous samples.
    unsigned long deltaMicroSecs = nowMicroSecs - lastCoulombsUpdateMicroSecs;
    lastCoulombsUpdateMicroSecs = nowMicroSecs;
    long long deltaPicoCoulombs = chargeFlowMicroAmps * deltaMicroSecs;

    // update our counter of the battery charge. Don't let the number get out of range.
    picoCoulombs = picoCoulombs + deltaPicoCoulombs;
    picoCoulombs = constrain(picoCoulombs, 0LL, BATTERY_CAPACITY_PICO_COULOMBS);
 
    // if the battery is charging, and has now reached the maximum charge,
    // we will set the battery coulomb counter to 100% of the battery capacity.
    // We know we've reached the maximum charge when:
    // 1. the charger is attached, AND
    // 2. we've been charging for at least a few minutes. AND
    // 3. the battery voltage hasn't changed for at least a few minutes
    // 4. the current flow rate is quite low
    unsigned long nowMillis = hw.millis();
    if ((picoCoulombs != BATTERY_CAPACITY_PICO_COULOMBS) &&
        isCharging() &&                                                              // ...the charger is attached, AND
        ((nowMillis - chargeStartMilliSecs) > CHARGER_WINDDOWN_TIME_MILLIS) &&       // ...we've been charging for a few minutes, AND
        ((nowMillis - lastVoltageChangeMilliSecs) > CHARGER_WINDDOWN_TIME_MILLIS) && // ...the battery voltage hasn't changed for a few minutes, AND
        (chargeFlowMicroAmps < CHARGE_MICRO_AMPS_WHEN_FULL))                         // ...the current flow rate is quite low
    {
        // Our 4-point check has passed. So we are probably finished charging. However,
        // because of random fluctuations in the current measurements, we might not be
        // *completely* finished. Let's see if we stay in the "charge finished" state
        // for a few seconds.
        if (maybeChargingFinished) {
            if (nowMillis - maybeChargingFinishedMilliSecs > 5000UL) {
                // We've been in "charge finished" state for long enough. It's now safe
                // to assume that the battery is fully charged.
                picoCoulombs = BATTERY_CAPACITY_PICO_COULOMBS;
                maybeChargingFinished = false;
            }
        } else {
            maybeChargingFinished = true;
            maybeChargingFinishedMilliSecs = nowMillis;
        }
    } else {
        maybeChargingFinished = false;
    }
}

void Battery::DEBUG_incrementPicoCoulombs(long long increment)
{
    picoCoulombs += increment;
    picoCoulombs = constrain(picoCoulombs, 0LL, BATTERY_CAPACITY_PICO_COULOMBS);
}

