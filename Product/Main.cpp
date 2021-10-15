/*
 * Main.cpp
 *
 * The main program of the PAPR product firmware. Here are some coding guidelines
 * that will help minimize coding errors...
 * 
 * KEEP THE MAIN LOOP RUNNING AT ALL TIMES. DO NOT USE DELAY().
 * Be careful - this firmware keeps running even when the user does "Power Off".
 * This code may need to run correctly for months at a time. Don't break this code!
 * All code must work when millis() and micros() wrap around
 * 
 * Once a battery is connected to the PCB, the system runs continuously forever. The 
 * only time we actually shut down is if the user completely drains the battery, or the
 * battery gets disconnected.
 * 
 * Time intervals - be aware that millis() and micros() are unsigned 32 bit numbers that
 * count up to 0xFFFFFFFF and then wrap around to 0. To correctly handle this wrap around,
 * always check for interval expiry by doing "if (hw.millis() - startTime > intervalDuration)".
 * All millisecond/microsecond intervals should be of type "uint32_t".
 * All millisecond/microsecond literals should have the UL suffix, e.g. "uint32_t foo = 123UL;"
 * To avoid confusion, variables that represent times or time intervals should be named "fooMillis" or "fooMicros".
 * 
 * Units - this code deals with many different kinds of data: milliseconds, amperes, volts, etc. 
 * Make sure that all variables specify the precise units, e.g. "fooMilliVolts", "fooAmperes", "fooPicoCoulombs"
 * 
 * Data types - this code uses specific data types for different kinds of data, e.g. "uint32_t" for times,
 * "int64_t" for charges in picoCoulombs, etc. Make sure to consistently use the correct data type for each variable,
 * and make sure your literals have the correct suffix, e.g. "123LL" for int64_t, "123UL" for uint32_t.
 * If you don't do this, the compiler will sometimes generate code that doesn't work the way you expect! Beware!
 */
#include "Main.h"
#include <LowPower.h>
#include "MySerial.h"
#include "Hardware.h"
#include "MiscConstants.h"

#define SERIAL_DEBUG // define this symbol to enable debug output to the serial port

 // The Hardware object gives access to all the microcontroller hardware such as pins and timers. Please always use this object,
 // and never access any hardware or Arduino APIs directly. This gives us the option of using a fake hardware object for unit testing.
#define hw Hardware::instance

// indexed by PAPRState
const char* const STATE_NAMES[] = { "Off", "On", "Off Charging", "On Charging" };

// indexed by ChargerStatus
const char* const CHARGER_STATUS_NAMES[] = { "No", "Charging", "Full", "Error" };

/********************************************************************
 * Fan constants
 ********************************************************************/

// How many milliseconds should there be between readings of the fan speed. A smaller value will update
// more often, while a higher value will give more accurate and smooth readings.
const uint32_t FAN_SPEED_READING_INTERVAL = 1000UL;

/* Here are measured values for fan RPM for the San Ace 9GA0412P3K011
   %    MIN     MAX     AVG

   0,    7461,   7480,    7479
  10,    9431,   9481,    9456
  20,   11264,  11284,   11274
  30,   12908,  12947,   12928
  40,   14580,  14626,   14603
  50,   16047,  16177,   16112
  60,   17682,  17743,   17743
  70,   19092,  19150,   19121
  80,   20408,  20488,   20448
  90,   21510,  21556,   21533
 100,   22215,  22327,   22271 
*/

// How much tolerance do we give when checking for correct fan RPM. We allow +/- 15%.
// We use float here, which is quite slow on our little MCU, but it's OK because we don't do it very much.
const float LOWEST_FAN_OK_RPM = 0.85;
const float HIGHEST_FAN_OK_RPM = 1.15;

// The fan speed when we startup.
const FanSpeed DEFAULT_FAN_SPEED = fanLow;

// When we change the fan speed, allow at least this many milliseconds before checking the speed.
// This gives the fan enough time to stabilize at the new speed.
const uint32_t FAN_STABILIZE_MILLIS = 6000UL;

/********************************************************************
 * Button constants
 ********************************************************************/

// The user must push a button for at least this many milliseconds.
const uint32_t BUTTON_DEBOUNCE_MILLIS = 1000UL;

// The power off button needs a very short debounce interval,
// so it can do a little song and dance before taking effect.
const uint32_t POWER_OFF_BUTTON_DEBOUNCE_MILLIS = 50UL;

// The power off button only takes effect if the user holds it pressed for at least this long.
const uint32_t POWER_OFF_BUTTON_HOLD_MILLIS = 1000UL;

/********************************************************************
 * Alert constants
 ********************************************************************/

// Which LEDs to flash for each type of alert.
const int batteryLowLEDs[] = { BATTERY_LED_LOW_PIN, CHARGING_LED_PIN , -1 };
const int fanRPMLEDs[] = { FAN_LOW_LED_PIN, FAN_MED_LED_PIN, FAN_HIGH_LED_PIN, -1 };
const int* const alertLEDs[] = { 0, batteryLowLEDs, fanRPMLEDs }; // Indexed by enum Alert.

// What are the on & off durations for the pulsed lights and buzzer for each type of alert. 
const uint32_t batteryAlertMillis[] = { 1000UL, 1000UL };
const uint32_t fanAlertMillis[] = { 200UL, 200UL };
const uint32_t* const alertMillis[] = { 0, batteryAlertMillis, fanAlertMillis }; // Indexed by enum Alert.

// A "low battery" alarm is in effect whenever the battery level is at or below the "urgent" amount.
// If a charger is connected then the red LED flashes until the level is above the urgent amount,
// but the buzzer doesn't sound.
// This percentage is supposed to occur when the battery has 30 minutes of charge left. To give a 
// generous margin, it's actually about an hour.
const int URGENT_BATTERY_PERCENT = 8;

/********************************************************************
 * LED
 ********************************************************************/

// Set a single LED to a given state
void Main::setLED(const int pin, int onOff) {
    hw.digitalWrite(pin, onOff);

    // keep track of the LED's state in ledState.
    for (int i = 0; i < numLEDs; i++) {
        if (pin == LEDpins[i]) {
            ledState[i] = onOff;
            return;
        }
    }
}

// TEMP - FOR DEBUGGING
void flashLED(const int pin, const uint16_t millis) {
    hw.digitalWrite(pin, LED_ON);
    if (millis) hw.delay(millis);
    hw.digitalWrite(pin, LED_OFF);
}

// Turn off all LEDs
void Main::allLEDsOff()
{
    for (int i = 0; i < numLEDs; i += 1) {
        setLED(LEDpins[i], LED_OFF);
    }
}

// Turn on all LEDs
void Main::allLEDsOn()
{
    for (int i = 0; i < numLEDs; i += 1) {
        setLED(LEDpins[i], LED_ON);
    }
}

// Set a list of LEDs to a given state.
void Main::setLEDs(const int* pinList, int onOff)
{
    for (int i = 0; pinList[i] != -1; i += 1) {
        setLED(pinList[i], onOff);
    }
}

// Flash all the LEDS for a specified duration and number of flashes.
void Main::flashAllLEDs(uint32_t millis, int count)
{
    while (count--) {
        allLEDsOn();
        hw.delay(millis);
        allLEDsOff();
        hw.delay(millis);
    }
}


/********************************************************************
 * Alert
 ********************************************************************/

// This function pulses the lights and buzzer during an alert.
void Main::onToggleAlert()
{
    alertToggle = !alertToggle;
    setLEDs(currentAlertLEDs, alertToggle ? LED_ON : LED_OFF);
    setBuzzer(alertToggle ? BUZZER_ON : BUZZER_OFF);
    alertTimer.start(currentAlertMillis[alertToggle ? 0 : 1]);
}

// Enter the "alert" state. In this state we pulse the lights and buzzer to 
// alert the user to a problem. Once we are in this state, the only
// way out is for the user to turn the power off.
void Main::raiseAlert(Alert alert)
{
    currentAlert = alert;
    serialPrintf("Begin %s Alert", currentAlertName());
    currentAlertLEDs = alertLEDs[alert];
    currentAlertMillis = alertMillis[alert];
    alertToggle = false;
    onToggleAlert();
}

// Turn off any active alert.
void Main::cancelAlert()
{
    currentAlert = alertNone;
    alertTimer.cancel();
    setBuzzer(false);
    allLEDsOff();
    chargerLEDStatus = chargerNotConnected;
}

// Turn the buzzer on or off.
void Main::setBuzzer(int onOff) {
    hw.setBuzzer(onOff, BUZZER_FREQUENCY, BUZZER_DUTYCYCLE);
    buzzerState = onOff;
}

/********************************************************************
 * Fan
 ********************************************************************/

// Update the fan indicator LEDs to correspond to the current fan setting.
void Main::updateFanLEDs()
{
    setLED(FAN_LOW_LED_PIN, LED_ON);
    setLED(FAN_MED_LED_PIN, currentFanSpeed > fanLow ? LED_ON : LED_OFF);
    setLED(FAN_HIGH_LED_PIN, currentFanSpeed == fanHigh ? LED_ON : LED_OFF);
}

// Set the fan to the indicated speed, and update the fan indicator LEDs.
void Main::setFanSpeed(FanSpeed speed)
{
    fanController.setDutyCycle(fanDutyCycles[speed]);
    currentFanSpeed = speed;
    updateFanLEDs();
    serialPrintf("Set Fan Speed %d", speed);

    // disable fan RPM monitor for a few seconds, until the new fan speed stabilizes
    lastFanSpeedChangeMilliSeconds = hw.millis();
    fanSpeedRecentlyChanged = true;
}

// Call this periodically to check that the fan RPM is within the expected range for the current FanSpeed.
void Main::checkForFanAlert() {
    const uint16_t fanRPM = fanController.getRPM(); 
    // Note: we call getRPM() even if we're not going to use the result, because getRPM() works better if you call it often.

    // If fan RPM checking is temporarily disabled, then do nothing.
    if (fanSpeedRecentlyChanged) {
        if (hw.millis() - lastFanSpeedChangeMilliSeconds < FAN_STABILIZE_MILLIS) {
            return;
        }
        fanSpeedRecentlyChanged = false;
    }

    // If the RPM is too low or too high compared to the expected value, raise an alert.
    const uint16_t expectedRPM = expectedFanRPM[currentFanSpeed];
    if ((fanRPM < (LOWEST_FAN_OK_RPM * expectedRPM)) || (fanRPM > (HIGHEST_FAN_OK_RPM * expectedRPM))) {
        raiseAlert(alertFanRPM);
    }
}

/********************************************************************
 * Battery
 ********************************************************************/

int Main::getBatteryPercentFull() {
    return (int)((battery.getPicoCoulombs() - BATTERY_MIN_CHARGE_PICO_COULOMBS) / ((BATTERY_CAPACITY_PICO_COULOMBS - BATTERY_MIN_CHARGE_PICO_COULOMBS) / 100LL));
}

void Main::updateChargerLED() {
    ChargerStatus status = battery.getChargerStatus();
    if (status == chargerLEDStatus) return;

    chargerLEDFlasher.stop();

    serialPrintf("charger status %d", (int)status);

    switch (status) {
    case chargerNotConnected:
        setLED(CHARGING_LED_PIN, LED_OFF);
        break;
    case chargerCharging:
        chargerLEDToggle = false;
        onChargerLED();
        chargerLEDFlasher.start(2000L);
        break;
    case chargerFull:
        setLED(CHARGING_LED_PIN, LED_ON);
        break;
    case chargerError:
        chargerLEDToggle = false;
        onChargerLED();
        chargerLEDFlasher.start(100L);
        break;
    }

    chargerLEDStatus = status;
}

// Call this periodically to update the battery and charging LEDs.
void Main::updateBatteryLEDs() {
    int percentFull = getBatteryPercentFull();

    // Decide if the red LED should be on or not.
    //bool redLED = (percentFull < 40);
    //if (percentFull <= URGENT_BATTERY_PERCENT && !battery.isChargerConnected()) {
    //    // The battery level is really low. Flash the LED.
    //    bool ledToggle = (hw.millis() / 1000UL) & 1;
    //    redLED = redLED && ledToggle;
    //}

    // Turn on/off the battery LEDs as required
    setLED(BATTERY_LED_LOW_PIN, (percentFull < 40) ? LED_ON : LED_OFF); // red
    setLED(BATTERY_LED_MED_PIN, ((percentFull > 15) && (percentFull < 97)) ? LED_ON : LED_OFF); // yellow
    setLED(BATTERY_LED_HIGH_PIN, (percentFull > 70) ? LED_ON : LED_OFF); // green

    // Turn on/off the charging indicator LED as required
    updateChargerLED();
    
    // Maybe turn the charge reminder on or off.
    // The "charge reminder" is the periodic beep that occurs when the battery is below 15%
    // to remind the user to recharge the unit as soon as possible.
    if (!battery.isChargerConnected() && percentFull <= 15 && currentAlert != alertBatteryLow) {
        if (!chargeReminder.isActive()) {
            onChargeReminder();
            chargeReminder.start(15000UL);
        }
    } else {
        chargeReminder.stop();
    }
}

// Call this periodically to decide if a battery alert should be started or terminated.
void Main::checkForBatteryAlert()
{
    if (currentAlert == alertBatteryLow) {
        if (battery.isChargerConnected() || getBatteryPercentFull() > URGENT_BATTERY_PERCENT) {
            cancelAlert();
        }
    } else if (getBatteryPercentFull() <= URGENT_BATTERY_PERCENT && !battery.isChargerConnected()) {
        chargeReminder.stop();
        chargerLEDFlasher.stop();
        raiseAlert(alertBatteryLow);
    }
}

// This is the callback function for charger indicator LED.
void Main::onChargerLED() {
    chargerLEDToggle = !chargerLEDToggle;
    setLED(CHARGING_LED_PIN, chargerLEDToggle ? LED_ON : LED_OFF);

}

// This is the callback function for the charger reminder. When it's active, this function gets called every minute or so.
// We turn on the buzzer and the charging LED, then set a timer for when to turn buzzer and LED off.
void Main::onChargeReminder() {
    setBuzzer(BUZZER_ON);
    setLED(CHARGING_LED_PIN, LED_ON);
    chargeReminderTimer.start(500UL);
}

// This is the callback function for chargeReminderTimer. This function gets called to turn off the chargeReminder buzzer and LED. 
void Main::onChargeReminderTimer() {
    setBuzzer(BUZZER_OFF);
    setLED(CHARGING_LED_PIN, LED_OFF);
}

/********************************************************************
 * states and modes
 ********************************************************************/

// Go into a new state.
void Main::enterState(PAPRState newState)
{
    serialPrintf("\r\nenter state %s", STATE_NAMES[newState]);
    onStatusReport();

    paprState = newState;
    switch (newState) {
        case stateOn:
        case stateOnCharging:
            hw.digitalWrite(FAN_ENABLE_PIN, FAN_ON);
            setFanSpeed(currentFanSpeed);
            setBuzzer(BUZZER_OFF);
            if (currentAlert != alertFanRPM) {
                updateFanLEDs();
            }
            if (currentAlert != alertBatteryLow) {
                updateBatteryLEDs();
            }
            break;

        case stateOff:
        case stateOffCharging:
            pinMode(BUZZER_PIN, INPUT); // tri-state the output pin, so the buzzer receives no signal and consumes no power.
            hw.digitalWrite(FAN_ENABLE_PIN, FAN_OFF);
            currentFanSpeed = DEFAULT_FAN_SPEED;
            cancelAlert();
            allLEDsOff();
            chargerLEDStatus = chargerNotConnected;
            break;
    }
    onStatusReport();
}

// Set the PCB to its low power state, and put the MCU into its lowest power sleep mode.
// This function will return only when the user presses the Power On button,
// or until the charger is connected. While we are napping, the system uses a negligible amount
// of power, perhaps 1-3% of a full battery charge every month.
//
// Be careful inside this function, it's the only place where we mess around with
// power, speed, watchdog, and sleeping. If you break this code it will mess up
// a lot of things! 
// 
// When this function returns, the board MUST be in full power mode,
// and the watchdog timer MUST be enabled. 
void Main::nap()
{
    hw.wdt_disable();
    hw.setPowerMode(lowPowerMode);
    while (true) {
        LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);

        if (battery.isChargerConnected()) {
            hw.setPowerMode(fullPowerMode);
            enterState(stateOffCharging);
            hw.wdt_enable(WDTO_8S);
            return;
        }

        uint32_t wakeupTimeMillis = hw.millis();
        while (hw.digitalRead(POWER_ON_PIN) == BUTTON_PUSHED) {
            if (hw.millis() - wakeupTimeMillis > 125UL) { // we're at 1/8 speed, so this is really 1000 ms (8 * 125)
                hw.setPowerMode(fullPowerMode);
                enterState(stateOn);
                while (hw.digitalRead(POWER_ON_PIN) == BUTTON_PUSHED) {}
                hw.wdt_enable(WDTO_8S);
                return;
            }
        }
    }
}

/********************************************************************
 * UI event handlers
 ********************************************************************/

// when the user presses Power Off, we want to give the user an audible and visible signal
// in case they didn't mean to do it. If the user holds the button long enough we return true,
// meaning that the user really wants to do it.
bool Main::doPowerOffWarning()
{
    // Turn on all LEDs, and the buzzer
    allLEDsOn();
    setBuzzer(BUZZER_ON);

    // If the user holds the button for long enough, we will return true,
    // which tells the caller to go ahead and enter the off state. 
    uint32_t startMillis = hw.millis();
    while (hw.digitalRead(POWER_OFF_PIN) == BUTTON_PUSHED) {
        if (hw.millis() - startMillis > POWER_OFF_BUTTON_HOLD_MILLIS) {
            allLEDsOff();
            setBuzzer(BUZZER_OFF);
            return true;
        }
    }

    // The user did not hold the button long enough. Restore the UI
    // and tell the caller not to enter the off state.
    allLEDsOff();
    setBuzzer(BUZZER_OFF);
    if (currentAlert != alertFanRPM) {
        updateFanLEDs();
    }
    if (currentAlert != alertBatteryLow) {
        updateBatteryLEDs();
    }
    return false;
}

// This function gets called when the user presses the Power On button
void Main::onPowerOnPress()
{
    switch (paprState) {
        case stateOn:
        case stateOnCharging:
            // do nothing
            break;

        case stateOff: // should never happen
        case stateOffCharging:
            enterState(stateOnCharging);
            break;
    }
}

// This function gets called when the user presses the Power Off button
void Main::onPowerOffPress()
{
    switch (paprState) {
        case stateOn:
            if (doPowerOffWarning()) {
                enterState(stateOff);
            }
            break;

        case stateOff:
        case stateOffCharging:
            // these should never happen
            break;

        case stateOnCharging:
            if (doPowerOffWarning()) {
                enterState(stateOffCharging);
            }
            break;
    }
}

// This function gets called when the user presses the Fan Down button
void Main::onFanDownPress()
{
    /* TEMP for testing/debugging: decrease the current battery level by a few percent. */
    //if (digitalRead(POWER_ON_PIN) == BUTTON_PUSHED) {
    //    battery.DEBUG_incrementPicoCoulombs(-1500000000000000LL);
    //    serialPrintf("Charge is %d%", getBatteryPercentFull());
    //    return;
    //}

    setFanSpeed((currentFanSpeed == fanHigh) ? fanMedium : fanLow);
}

// This function gets called when the user presses the Fan Up button
void Main::onFanUpPress()
{
    /* TEMP for testing/debugging: increase the current battery level by a few percent. */
    //if (digitalRead(POWER_ON_PIN) == BUTTON_PUSHED) {
    //    battery.DEBUG_incrementPicoCoulombs(1500000000000000LL);
    //    serialPrintf("Charge is %d%", getBatteryPercentFull());
    //    return;
    //}

    setFanSpeed((instance->currentFanSpeed == fanLow) ? fanMedium : fanHigh);
}

// This function is an interrupt handler that gets called whenever the user presses the Power On button.
void Main::callback()
{
    if (hw.digitalRead(POWER_ON_PIN) == BUTTON_PUSHED && hw.digitalRead(FAN_UP_PIN) == BUTTON_PUSHED && hw.digitalRead(FAN_DOWN_PIN) == BUTTON_PUSHED) {
        // it's a user reset
        hw.reset();
        // TEMP cause a watchdog timeout
        //while (true) {
        //    setLED(ERROR_LED_PIN, LED_ON);
        //    setLED(ERROR_LED_PIN, LED_OFF);
        //}
    }
}


/********************************************************************
 * Startup and run
 ********************************************************************/

Main::Main() :
    // In the following code we are using the c++ lambda expressions as glue to our event handler functions.
    buttonFanUp(FAN_UP_PIN, BUTTON_DEBOUNCE_MILLIS,
        []() { instance->onFanUpPress(); }),
    buttonFanDown(FAN_DOWN_PIN, BUTTON_DEBOUNCE_MILLIS,
        []() { instance->onFanDownPress(); }),
    buttonPowerOff(POWER_OFF_PIN, POWER_OFF_BUTTON_DEBOUNCE_MILLIS, 
        []() { instance->onPowerOffPress(); }),
    buttonPowerOn(POWER_ON_PIN, BUTTON_DEBOUNCE_MILLIS, 
        []() { instance->onPowerOnPress(); }),
    alertTimer(
        []() { instance->onToggleAlert(); }),
    chargeReminderTimer(
        []() { instance->onChargeReminderTimer(); }),
    chargeReminder(
        []() { instance->onChargeReminder(); }),
    statusReport(
        []() { instance->onStatusReport(); }),
    chargerLEDFlasher(
        []() { instance->onChargerLED(); }),
    fanController(FAN_RPM_PIN, FAN_SPEED_READING_INTERVAL, FAN_PWM_PIN),
    currentFanSpeed(fanLow),
    fanSpeedRecentlyChanged(false),
    buzzerState(BUZZER_OFF),
    chargerLEDStatus(chargerNotConnected),
    currentAlert(alertNone)
{
    instance = this;
    for (int i = 0; i < numLEDs; i += 1) { ledState[i] = LED_OFF; }
}

bool shouldEnterTestMode() {
    for (int i = 0; i < 30; i += 1) {
        int input = Serial.peek();
        if (input == 't' || input == 'T') {
            while(Serial.peek() != -1) Serial.read();
            return true;
        }
        delay(100L);
    }
    return false;
}

// This function gets called when the MCU is powered up. Returns true iff we should go into test mode.
bool Main::setup()
{
    // Make sure watchdog is off. Remember what kind of reset just happened. Setup the hardware.
    int resetFlags = hw.watchdogStartup();
    hw.setup();
    hw.setBuzzer(false, 0, 0);
    flashAllLEDs(50UL, 3); // tell the user we are alive

    // Initialize the serial port with input enabled, and check to see if the user wants to enter Test Mode.
    serialBegin(true);
    serialPrintf("%s %s %s (flags %d)\r\nType 't' to enter test mode", PRODUCT_ID, __DATE__, __TIME__, resetFlags);
    if (shouldEnterTestMode()) {
        return true;
    }
    // The user doesn't want test mode.
    serialPrintln(F("\r\n\n<<< Normal Mode >>>"));
    serialEnd();
    
    #ifdef SERIAL_DEBUG
    serialBegin(false);
    serialPrintln(F("\r\n\n<<< DEBUG ON >>>"));
    #endif

    // Decide what state we should be in.
    PAPRState initialState;
    if (resetFlags & (1 << WDRF)) {
        // Watchdog timer expired. Tell the user that something unusual happened.
        flashAllLEDs(100UL, 10);
        initialState = battery.isChargerConnected() ? stateOnCharging : stateOn;
    } else if (resetFlags == 0) {
        // Manual reset. Tell the user that something unusual happened.
        flashAllLEDs(300UL, 5);
        initialState = battery.isChargerConnected() ? stateOnCharging : stateOn;
    } else {
        // It's a simple power-on. This will happen when:
        // - somebody in the factory just connected the battery to the PCB; or
        // - the battery had been fully drained (and therefore not delivering any power), and the user just plugged in the charger.
        initialState = battery.isChargerConnected() ? stateOffCharging : stateOff;
    }

    // Initialize the fan
    fanController.begin();
    setFanSpeed(DEFAULT_FAN_SPEED);

    // Enable the watchdog timer. (Note: Don't make the timeout value too small - we need to give the IDE a chance to
    // call the bootloader in case something dumb happens during development and the WDT
    // resets the MCU too quickly. Once the code is solid, you could make it shorter.)
    wdt_enable(WDTO_8S);

    // Enable pin-change interrupts for the Power On button, and register a callback to handle those interrupts.
    // The interrupt serves 2 distinct purposes: (1) to get this callback called, and (2) to wake us up if we're napping.
    hw.setPowerOnButtonInterruptCallback(this);

    // and we're done!
    battery.initializeCoulombCount();
    enterState(initialState);
    statusReport.start(5000UL);
    return false;
}

// Call the update() function of everybody who wants to do something each time through the loop() function.
void Main::doAllUpdates()
{
    battery.update();
    if (currentAlert == alertNone) {
        checkForFanAlert();
    }
    if (currentAlert != alertFanRPM) {
        updateFanLEDs();
    }
    checkForBatteryAlert();
    if (currentAlert != alertBatteryLow) {
        updateBatteryLEDs();
    }
    buttonFanUp.update();
    buttonFanDown.update();
    buttonPowerOff.update();
    alertTimer.update();
    chargeReminder.update();
    chargeReminderTimer.update();
    chargerLEDFlasher.update();
    statusReport.update();
}

// This is our main function, which gets called over and over again, forever.
void Main::loop()
{
    hw.wdt_reset_();

    switch (paprState) {
        case stateOn:
            doAllUpdates();
            if (battery.isChargerConnected()) {
                enterState(stateOnCharging); 
            }
            break;

        case stateOnCharging:
            doAllUpdates();
            if (!battery.isChargerConnected()) {
                enterState(stateOn);
            }
            break;

        case stateOff:
            // We are not charging so there are no LEDs to update.
            // We have nothing to do except take a nap. Our nap will end
            // when the state is no longer stateOff.
            nap();
            battery.wakeUp();
            break;

        case stateOffCharging:
            // Only do the work that is necessary when power is off and we're charging
            // - update the battery status and battery LEDs
            // - see if the charger has been unplugged
            // - see if the Power On button was pressed
            battery.update();
            updateBatteryLEDs();
            if (!battery.isChargerConnected()) {
                enterState(stateOff);
            }
            buttonPowerOn.update();
            statusReport.update();
            chargerLEDFlasher.update();
            break;
    }
}

// Write a one-line summary of the status of everything. For use in testing and debugging.
void Main::onStatusReport() {
    if (!serialActive()) return;

    serialPrintf("Fan,%s,Buzzer,%s,Alert,%s,Reminder,%s,Charger,%s,LEDs,%s,%s,%s,%s,%s,%s,%s,milliVolts,%ld,milliAmps,%ld,Coulombs,%ld,charge,%d%%,millis,%lu,micros,%lu",
        (currentFanSpeed == fanLow) ? "lo" : ((currentFanSpeed == fanMedium) ? "med" : "hi"),
        (buzzerState == BUZZER_ON) ? "on" : "off",
        currentAlertName(),
        chargeReminder.isActive() ? "yes" : "no",
        CHARGER_STATUS_NAMES[battery.getChargerStatus()],
        (ledState[0] == LED_ON) ? "red" : "---",
        (ledState[1] == LED_ON) ? "yellow" : "---",
        (ledState[2] == LED_ON) ? "green" : "---",
        (ledState[3] == LED_ON) ? "amber" : "---",
        (ledState[4] == LED_ON) ? "blue" : "---",
        (ledState[5] == LED_ON) ? "blue" : "---",
        (ledState[6] == LED_ON) ? "blue" : "---",
        (int32_t)(hw.readMicroVolts() / 1000LL),
        (int32_t)(hw.readMicroAmps() / 1000LL),
        (int32_t)(battery.getPicoCoulombs() / 1000000000000LL),
        getBatteryPercentFull(),
        hw.millis(),hw.micros());
}

Main* Main::instance;
