#include "MySerial.h"
#include "Hardware.h"
#include <limits.h>
#include "FanController.h"

#define hw Hardware::instance

class Sampler {
public:
    long highest;
    long lowest;
    long sampleCount;

    void reset()
    {
        highest = LONG_MIN;
        lowest = LONG_MAX;
        accumulator = 0;
        sampleCount = 0;
    }

    void sample(long value)
    {
        if (value < lowest) lowest = value;
        if (value > highest) highest = value;
        accumulator += value;
        sampleCount += 1;
    }

    long average()
    {
        return sampleCount > 0 ? (accumulator / sampleCount) : 0;
    }

private:
    long long accumulator;
};

unsigned long samplePeriodBeginMillis;
bool skipReport;
Sampler voltage;
Sampler rpm;
Sampler current;
Sampler rawRPM;
Sampler referenceVoltage;
Sampler computedMicroAmps;
Sampler computedMicroVolts;

const float lowPassFilterN = 500.0;

void beginSamplePeriod()
{
    samplePeriodBeginMillis = hw.millis();
    skipReport = false;
    voltage.reset();
    rpm.reset();
    current.reset();
    rawRPM.reset();
    referenceVoltage.reset();
    computedMicroAmps.reset();
    computedMicroVolts.reset();
}

void updateRecorder(unsigned int fanRPM, int currentDutyCycle, bool isCharging, long long picoCoulombs)
{
    if (hw.millis() - samplePeriodBeginMillis > 5000UL) {
        if (!skipReport) {
            long long batteryMilliVolts = voltage.average() * NANO_VOLTS_PER_VOLTAGE_UNIT / 1000000;
            int buzzerDutyCycle = (OCR1BH << 8) | OCR1BL;
            bool toneOn = buzzerDutyCycle != 0;
            char buffer1[50];
            char buffer2[50];
            char buffer3[50];
            serialPrintf("Duty %d  VOLT %ld|%ld|%ld  comp %s\
  CURR %ld|%ld|%ld  REF %ld|%ld|%ld  uAMPS %ld|%ld|%ld  uVOLTS %ld|%ld|%ld  CHARGE %s  PICOCOUL %s  RPM %ld|%ld|%ld  TONE %s  SAMPLES %lu",
                currentDutyCycle,
                voltage.lowest, voltage.average(), voltage.highest, renderLongLong(batteryMilliVolts),
                current.lowest, current.average(), current.highest,
                referenceVoltage.lowest, referenceVoltage.average(), referenceVoltage.highest,
                computedMicroAmps.lowest, computedMicroAmps.average(), computedMicroAmps.highest,
                computedMicroVolts.lowest, computedMicroVolts.average(), computedMicroVolts.highest,
                isCharging ? "yes" : "no", renderLongLong(picoCoulombs),
                rpm.lowest, rpm.average(), rpm.highest,
                toneOn ? "on" : "off", rpm.sampleCount);
            delay(500); // to allow the serial port to finish
        }

        beginSamplePeriod();
    }

    unsigned int batteryLevel = hw.analogRead(BATTERY_VOLTAGE_PIN);

    rpm.sample(fanRPM);
    rawRPM.sample(hw.digitalRead(FAN_RPM_PIN));
    voltage.sample(batteryLevel);
    referenceVoltage.sample(hw.analogRead(REFERENCE_VOLTAGE_PIN));
    current.sample(hw.analogRead(CHARGE_CURRENT_PIN));
    computedMicroAmps.sample((long)(Hardware::instance.readMicroAmps()));
    computedMicroVolts.sample((long)(Hardware::instance.readMicroVolts()));
}

void resetRecorder() {
    beginSamplePeriod();
    skipReport = true;
}