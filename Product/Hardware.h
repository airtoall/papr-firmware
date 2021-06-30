 // Hardware.h
 // 
 // This file defines all the hardware pins, and a bunch of hardware-specific functions for the Air One PAPR, PCB revision 3. 
 // This includes generic access to IO pins and timing, as well as definitions and functions specific to the PAPR- and/or the MCU,
 // such as configuring pins, initializing the hardware, and using the watchdog timer.
 // 
 // In the future, we intend to provide 2 different implementations of this API
 // - the product version, which runs on a PAPR board with MCU. This is the code you are now reading.
 // - the unit test version, which runs on a PC and pretends to be a PAPR and MCU. This doesn't yet exist.

#pragma once
#ifdef UNITTEST
#include "ArduinoDefs.h"
#else
#include "Arduino.h"
#include <avr/wdt.h> 
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// PCB Power Modes
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

// The PCB can run in either High Power mode or Low Power mode.
// - High Power Mode means that the PCB and MCU are fully powered 
// - Low Power Mode means that the MCU receives reduced voltage (approx 2.5 instead of 5)
//   and the rest of the PCB receives no power. In this mode, the MCU cannot run at full speed,
//   so we use a reduced clock speed: 1 MHz instead of 8 MHz. At this reduced speed,
//   the delay() and millis() functions are 8x slower, and there are probably other things
//   that don't work right. So, to minimize problems, we only use reduced speed mode in a couple of specific 
//   places: when the power first comes on, and when we nap().
//
// The low power mode is intended for use when the PAPR is in the Power Off state. Im this state,
// you can use the MCU to check for button presses or charger activity, while consuming a negligible 
// amount of battery power.
enum PowerMode { lowPowerMode, fullPowerMode };

// On startup, the PCB initializes itself to the low power mode. Also, CLKDIV8 in the MCU's low fuse byte
// initializes the MCU clock divider to 2**3, which results in MCU clock of 1 MHz.


////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Pin definitions and values.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

// Buttons
const int FAN_UP_PIN = 4;             // PD4   input   Digital: LOW = pushed, HIGH = released, requires pullup
const int FAN_DOWN_PIN = 9;           // PB1   input   Digital: LOW = pushed, HIGH = released, requires pullup
const int POWER_OFF_PIN = 8;          // PB0   input   Digital: LOW = pushed, HIGH = released, requires pullup
const int POWER_ON_PIN = 7;           // PD7   input   Digital: LOW = pushed, HIGH = released, requires pullup
const int BUTTON_PUSHED = LOW;        
const int BUTTON_RELEASED = HIGH;     
                                      
// Fan                                
const int FAN_PWM_PIN = 3;            // PD3   output  PWM generated by MCU's built-in PWM capabillity
const int FAN_RPM_PIN = 5;            // PD5   input   Digital: square wave coming from fan, firmware gets interrupt on each edge
const int FAN_ENABLE_PIN = 98;        // PB6   output  Digital: LOW = fan off, HIGH = fan on. Not supported by Arduino APIs.
const int FAN_ON = HIGH;
const int FAN_OFF = LOW;

// Power
const int BATTERY_VOLTAGE_PIN = A7;   // ADC7  input   10-bit ADC: convert to volts using NANO_VOLTS_PER_VOLTAGE_UNIT
const int CHARGE_CURRENT_PIN = A6;    // ADC6  input   10-bit ADC: convert to amps using MICRO_AMPS_PER_CHARGE_FLOW_UNIT
const int REFERENCE_VOLTAGE_PIN = A1; // PC1   input   10-bit ADC
const int CHARGER_CONNECTED_PIN = 0;  // PD0   input   Digital: LOW = connected, HIGH = not connected. Requires pullup. Warning this is the same as SERIAL_RX_PIN.
const int CHARGER_CONNECTED = LOW;
const int BOARD_POWER_PIN = 6;        // PD6   output  Digital: LOW = power off, HIGH = power on
const int BOARD_POWER_ON = HIGH;      
const int BOARD_POWER_OFF = LOW;      

// Sound                              
const int BUZZER_PIN = 10;            // PB2   output  PWM generated by MCU's built-in PWM capabillity
const int BUZZER_ON = 128;            
const int BUZZER_OFF = 0;             

// LEDs                             
const int BATTERY_LED_LOW_PIN = A0;   // PC0   output  Digital: LOW = on, HIGH = off
const int BATTERY_LED_MED_PIN = 99;   // PB7   output  Digital: LOW = on, HIGH = off. Not supported by Arduino APIs.
const int BATTERY_LED_HIGH_PIN = A2;  // PC2   output  Digital: LOW = on, HIGH = off
const int CHARGING_LED_PIN = A3;      // PC3   output  Digital: LOW = on, HIGH = off
const int FAN_LOW_LED_PIN = A4;       // PC4   output  Digital: LOW = on, HIGH = off
const int FAN_MED_LED_PIN = A5;       // PC5   output  Digital: LOW = on, HIGH = off
const int FAN_HIGH_LED_PIN = 2;       // PD2   output  Digital: LOW = on, HIGH = off
const int LED_ON = LOW;               
const int LED_OFF = HIGH;

 // A list of all the LEDs, from left to right.
const byte LEDpins[] = {
    BATTERY_LED_LOW_PIN,
    BATTERY_LED_MED_PIN,
    BATTERY_LED_HIGH_PIN,
    CHARGING_LED_PIN,
    FAN_LOW_LED_PIN,
    FAN_MED_LED_PIN,
    FAN_HIGH_LED_PIN
};
const int numLEDs = sizeof(LEDpins) / sizeof(byte);

// Serial port - these definitions are assumed by the Arduino "Serial" object.                    
const int SERIAL_RX_PIN = 0;          // PD0   input   Cannot be used by Serial, because it's also CHARGER_CONNECTED_PIN.
const int SERIAL_TX_PIN = 1;          // PD1   output  Can be used by Serial, usually only on development machines

// FYI, here are some MCU pins that are used by the PCB, but we don't access from the firmware.
// SPI PORT: MOSI = PB3, MISO = PB4, SCLK = PB5, RESET = PC6
// Power: VCC, AVCC, GND

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Misc board-specific constants
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

// "long long" has 18-19 decimal digits of precision. Watch out for overflow!
const long long NANO_AMPS_PER_CHARGE_FLOW_UNIT = 6516781LL;
const long long NANO_VOLTS_PER_VOLTAGE_UNIT = 29325513LL;             // 0 to 1023 corresponds to 0 to 30 volts
const long long BATTERY_CAPACITY_PICO_COULOMBS = 25200000000000000LL; // 25,200 coulombs // TODO fudge factor? probably 0.8
const long long BATTERY_MIN_CHARGE_PICO_COULOMBS = 2100000000000000LL; // 2,100 coulombs the minimum charge level // TODO fudge factor? probably 0.8

/*
Here is a note from Brent Bolton about how AMPS_PER_CHARGE_FLOW_UNIT and VOLTS_PER_VOLTAGE_UNIT are determined:

VOLTAGE:

The SW_BATT voltage is externally divided by 6 before being sent to
ADC7. So a range of 0 to 30 volts on SW_BATT will be scaled down to 0 to
5 V at the pin.  Because of the precision of the selected divider
resistors, the accuracy of this stage is +/- 2%.

That will nominally be mapped to 0 to 1023 by the ADC, however that will
actually vary by +/- 2% as that's the regulation accuracy of the 5V
power supply used as the ADC reference voltage. So the conversion
factor is correct in the nominal case and the expected accuracy is +/- 4%.

CURRENT: 

The current sense uses a 15 milliOhm sense resistor. The voltage across
the sensor  (which is current x 0.015) is multiplied by 50 and added to
the PC1 voltage for discharge currents or subtracted for charge
currents. That works out to 750 millivolts of ADC6 - PC1 difference per
amp or, equivalently, 1.333 milliamps per millivolt of difference. So
the calculation is:

Current_in_mA = ((ADC6 - PC1) * 5000 * 4 /(1023 * 3); // Signed integer calculation

where the 5000 is millivolts full scale and the 4/3 is the millivolt to
milliamp conversion factor. It follows that the maximum resolvable
current is going to be +/- 3333 mA if PC1 is exactly centered at 2.5
volts. If it's not centered, which it isn't, then the maximum is going
to be truncated in one direction. The maximum charge current is around
2700mA so make sure that can be resolved. If not, I can tweak the
circuit so it is. Max discharge current is less than 1000mA, so that's
no problem.

The final equation therefore is
Current_in_mA = (ADC6 - PC1) * 6.516780710329097
*/

// The MCU's fuse bytes should be set as follows
//   low fuse byte 0x72
//   high fuse byte 0xDA
//   extended fuse byte 0xFF

// If you have code that wants to receive interrupts, your code must be a subclass of InterruptCallback.
class InterruptCallback {
public:
    virtual void callback() = 0;
};

// This singleton class provides hardware-specific functions.
class Hardware {
public:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Basic Hardware access - here are covers for various Arduino API functions. 
    // DO NOT CALL INTO THE ARDUINO API DIRECTLY. ALWAYS USE THESE COVERS.
    // This is because some of these functions have custom implementations that are different
    // than Arduino's, and also because the unit test environment has completely different implementations.
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline void pinMode(uint8_t pin, uint8_t mode) { ::pinMode(pin, mode); }
    void digitalWrite(uint8_t pin, uint8_t val);
    inline int digitalRead(uint8_t pin) { return ::digitalRead(pin); }
    inline int analogRead(uint8_t pin) { return ::analogRead(pin); }
    inline void analogWrite(uint8_t pin, int val) { ::analogWrite(pin, val); }
    inline unsigned long millis(void) { return ::millis(); }
    inline unsigned long micros(void) { return ::micros(); }
    inline void delay(unsigned long ms) { ::delay(ms); }
    inline void wdt_enable(const uint8_t value) { ::wdt_enable(value); }
    inline void wdt_disable() { ::wdt_disable(); }
    inline void wdt_reset_() { wdt_reset(); } // wdt_reset is a macro so we can't use "::"

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Functions for controlling PAPR and MCU hardware
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Cause a complete reset. This is equivalent to disconnecting and reconnecting power. All memory is lost.
    void reset();

    // The very first thing the firmware must do when there's a reset is call watchdogStartup(),
    // to make sure the watchdog timer is off. The return value is the MCUSR register,
    // which tells you why the reset occurred.
    int watchdogStartup(void);

    // Register a callback for an interrupt when the Power On Button is pushed.
    void setPowerOnButtonInterruptCallback(InterruptCallback*);

    // Register a callback for an interrupt whenever the fan RPM input toggles.
    void setFanRPMInterruptCallback(InterruptCallback*);

    // Call this function right after calling watchdogStartup()
    void setup();

    // Set/Get the current power mode.
    void setPowerMode(PowerMode mode);
    PowerMode getPowerMode() { return powerMode; }

    // This function handles various interrupts
    void handleInterrupt();

    // Read the battery/charger voltage. Result is microvolts in the range 0 to 30,000,000
    long long readMicroVolts();

    // Read the battery current in microamperes in the range -6,000,000 to +6,000,000.
    // The value is positive when charging, negative when discharging.
    long long readMicroAmps();

    // There can only be one instance of this object.
    static Hardware instance;

private:
    // Data for interrupt handling
    unsigned int powerOnButtonState;
    unsigned int fanRPMState;
    InterruptCallback* powerOnButtonInterruptCallback;
    InterruptCallback* fanRPMInterruptCallback;
    void updateInterruptHandling();
 
    PowerMode powerMode; // which mode are we currently in?
    long long microAmps; // we use this to help smooth battery current readings.

    // initialization
    Hardware();
    void configurePins();
    void initializeDevices();
    void setClockPrescaler(int prescalerSelect);
};