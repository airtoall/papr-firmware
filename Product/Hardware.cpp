/*
 * Hardware.cpp
 */
#include "Hardware.h"
#include "PB2PWM.h"
#include <avr/interrupt.h>
#include "eeprom.h"
#include "MySerial.h"

Hardware::Hardware() :powerOnButtonInterruptCallback(0), fanRPMInterruptCallback(0), microAmps(0LL) { }

Hardware Hardware::instance;

/********************************************************************
 * PAPR-specific functions
 ********************************************************************/

// Configure all the microcontroller IO pins that this app uses.
void Hardware::configurePins()
{
    pinMode(FAN_UP_PIN, INPUT_PULLUP);
    pinMode(FAN_DOWN_PIN, INPUT_PULLUP);
    pinMode(POWER_OFF_PIN, INPUT_PULLUP);
    pinMode(POWER_ON_PIN, INPUT_PULLUP);
    pinMode(FAN_PWM_PIN, OUTPUT);
    pinMode(FAN_RPM_PIN, INPUT);
    DDRB |= (1 << DDB6); // pinMode(FAN_ENABLE_PIN, OUTPUT); // we can't use pinMode because it doesn't support pin PB6
    pinMode(BATTERY_VOLTAGE_PIN, INPUT);
    pinMode(CHARGE_CURRENT_PIN, INPUT);
    pinMode(REFERENCE_VOLTAGE_PIN, INPUT);
    pinMode(BOARD_POWER_PIN, OUTPUT);
    pinMode(CHARGER_CONNECTED_PIN, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BATTERY_LED_LOW_PIN, OUTPUT);
    DDRB |= (1 << DDB7); // pinMode(BATTERY_LED_MED_PIN, OUTPUT); // we can't use pinMode because it doesn't support pin PB7
    pinMode(BATTERY_LED_HIGH_PIN, OUTPUT);
    pinMode(CHARGING_LED_PIN, OUTPUT);
    pinMode(FAN_LOW_LED_PIN, OUTPUT);
    pinMode(FAN_MED_LED_PIN, OUTPUT);
    pinMode(FAN_HIGH_LED_PIN, OUTPUT);
}

// Set all devices to an initial state
void Hardware::initializeDevices()
{
    // Fan on at lowest speed
    digitalWrite(FAN_ENABLE_PIN, FAN_ON);
    analogWrite(FAN_PWM_PIN, 0); // set the fan duty cycle to 0.

    // All LEDs off
    digitalWrite(BATTERY_LED_LOW_PIN, LED_OFF);
    digitalWrite(BATTERY_LED_MED_PIN, LED_OFF);
    digitalWrite(BATTERY_LED_HIGH_PIN, LED_OFF);
    digitalWrite(CHARGING_LED_PIN, LED_OFF);
    digitalWrite(FAN_LOW_LED_PIN, LED_OFF);
    digitalWrite(FAN_MED_LED_PIN, LED_OFF);
    digitalWrite(FAN_HIGH_LED_PIN, LED_OFF);

    // Buzzer off
    analogWrite(BUZZER_PIN, BUZZER_OFF);
}

// Here is our version of Arduino's digitalWrite() function. Certain pins
// are not handled by arduino, so we do it ourself.
void Hardware::digitalWrite(uint8_t pin, uint8_t val)
{ 
    switch (pin) {
    case FAN_ENABLE_PIN:
        // We have to access the port register directly, because digitalWrite doesn't support pin PB6.
        if (val == HIGH) {
            PORTB |= (1 << PB6);         // digitalWrite(FAN_ENABLE_PIN, HIGH);
        }
        else {
            PORTB = PORTB & ~(1 << PB6); // digitalWrite(FAN_ENABLE_PIN, LOW);
        }
        break;
    case BATTERY_LED_MED_PIN:
        // We have to access the port register directly, because digitalWrite doesn't support pin PB7.
        if (val == HIGH) {
            PORTB |= (1 << PB7);         // digitalWrite(BATTERY_LED_MED_PIN, HIGH);
        }
        else {
            PORTB = PORTB & ~(1 << PB7); // digitalWrite(BATTERY_LED_MED_PIN, LOW);
        }
        break;
    default:
        ::digitalWrite(pin, val);
    }
}

int Hardware::watchdogStartup(void)
{
    int result = MCUSR;
    MCUSR = 0;
    wdt_disable();
    return result;
}

// prescalerSelect is 0..8, giving division factor of 1..256
void Hardware::setClockPrescaler(int prescalerSelect)
{
    noInterrupts();
    CLKPR = (1 << CLKPCE);
    CLKPR = prescalerSelect;
    interrupts();
}

int64_t Hardware::readMicroVolts() {
    return ((int64_t)analogRead(BATTERY_VOLTAGE_PIN) * NANO_VOLTS_PER_VOLTAGE_UNIT) / 1000LL;
}

int64_t Hardware::readMicroAmps() {
    int32_t currentReading = analogRead(CHARGE_CURRENT_PIN);
    int32_t referenceReading = analogRead(REFERENCE_VOLTAGE_PIN);
    int64_t readingMicroAmps = (((int64_t)(referenceReading - currentReading)) * NANO_AMPS_PER_CHARGE_FLOW_UNIT) / 1000LL;

    const int64_t lowPassFilterN = 500LL;
    microAmps = ((microAmps * lowPassFilterN) + readingMicroAmps) / (lowPassFilterN + 1LL);
    return microAmps;
}

void Hardware::reset()
{
    // "onReset" is a pointer to the RESET interrupt handler at address 0. Call it.
    void(*onReset) (void) = 0;
    onReset();
}

void Hardware::handleInterrupt() {
    if (powerOnButtonInterruptCallback) {
        // A callback has been registered for the Power On Button interrupt. 
        // Only call it if the button state has changed.
        uint16_t newPowerOnButtonState = digitalRead(POWER_ON_PIN);
        if (newPowerOnButtonState != powerOnButtonState) {
            powerOnButtonState = newPowerOnButtonState;
            powerOnButtonInterruptCallback->callback();
        }
    }

    if (fanRPMInterruptCallback) {
        // A callback has been registered for the Fan RPM interrupt. 
        // Only call it if the signal state has changed.
        uint16_t newFanRPMState = digitalRead(FAN_RPM_PIN);
        if (newFanRPMState != fanRPMState) {
            fanRPMState = newFanRPMState;
            fanRPMInterruptCallback->callback();
        }
    }
}

// The hardware interrupt vector points to this code.
ISR(PCINT2_vect)
{
    Hardware::instance.handleInterrupt();
}

void Hardware::updateInterruptHandling() {
    // Here is where we set up handling for Pin Change interrupts
    // that correspond to Power On button presses and Fan RPM signals. 
    // By default, PCMSK2 and PCICR are both 0, so we won't receive any Pin Change interrupts.
    powerOnButtonState = digitalRead(POWER_ON_PIN);
    fanRPMState = digitalRead(FAN_RPM_PIN);

    if (powerOnButtonInterruptCallback) {
        PCMSK2 |=   1 << PCINT23;  // set PCINT23 = 1 to enable PCINT on pin PD7
    } else {
        PCMSK2 &= ~(1 << PCINT23); // set PCINT23 = 0 to disable PCINT on pin PD7
    }

    if (fanRPMInterruptCallback) {
        PCMSK2 |=   1 << PCINT21;  // set PCINT21 = 1 to enable PCINT on pin PD5
    } else {
        PCMSK2 &= ~(1 << PCINT21); // set PCINT21 = 0 to disable PCINT on pin PD5
    }

    if (powerOnButtonInterruptCallback || fanRPMInterruptCallback) {
        PCICR |=   1 << PCIE2;     // set PCIE2 = 1 to enable PC interrupts
    } else {
        PCICR &= ~(1 << PCIE2);    // set PCIE2 = 0 to disable PC interrupts
    }
}

void Hardware::setPowerOnButtonInterruptCallback(InterruptCallback* callback)
{
    powerOnButtonInterruptCallback = callback;
    updateInterruptHandling();
}

void Hardware::setFanRPMInterruptCallback(InterruptCallback* callback)
{
    fanRPMInterruptCallback = callback;
    updateInterruptHandling();
}

void Hardware::setPowerMode(PowerMode mode)
{
    if (mode == fullPowerMode) {
        // Set the PCB to Full Power mode.
        digitalWrite(BOARD_POWER_PIN, BOARD_POWER_ON);

        // Wait for things to settle down
        delay(10UL);

        // Set the clock prescaler to give the max speed.
        setClockPrescaler(0);

        // We are now running at full power, full speed.
    } else {
        // Full speed doesn't work in low power mode, so drop the MCU clock speed to 1 MHz (8 MHz internal oscillator divided by 2**3). 
        setClockPrescaler(3);

        // Now we can enter low power mode,
        digitalWrite(BOARD_POWER_PIN, BOARD_POWER_OFF);

        // Wait for the PCB to fully switch to low power mode (Note: at 1/8 speed, 30 ms is really 240 ms).
        delay(30UL);

        // We are now running at low power, low speed.
    }

    powerMode = mode;
}

void Hardware::setup() {
    // If the power has just come on, then the PCB is in Low Power mode, and the MCU
    // is running at 1 MHz (because the CKDIV8 fuse bit is programmed). Switch to full speed.
    setPowerMode(fullPowerMode);

    // Initialize the hardware
    configurePins();
    initializeDevices();
}

void Hardware::setBuzzer(int onOff, int32_t frequencyHz, int dutyCyclePercent) {
    if (onOff) {
        startPB2PWM(frequencyHz, dutyCyclePercent);
    } else {
        stopPB2PWM();
    }
}

void Hardware::eepromUpdateInt64(uint16_t eeprom_address, int64_t value) {
    int64_t currentValue = eepromReadInt64(eeprom_address);
    if (currentValue != value) {
        eeprom_write_block(&value, (void*)eeprom_address, sizeof(value));
        #ifdef SERIAL_DEBUG
        serialPrintf("write eeprom %s %s", renderLongLong(currentValue), renderLongLong(value));
        #endif
    }
}

int64_t Hardware::eepromReadInt64(uint16_t eeprom_address) {
    int64_t result;
    eeprom_read_block(&result, (void*)eeprom_address, sizeof(result));
    return result;
}



// This global function is used in a couple of places that don't have access to "Hardware.h" 
uint32_t getMillis()
{
    return Hardware::instance.millis();
}

