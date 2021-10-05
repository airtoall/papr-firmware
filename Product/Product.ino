/*
 * Product.ino
 * 
 * This file acts as the glue between the Arduino runtime and the main program in Main.cpp.
 * Please don't add any code to this file. Instead, add code to Main.cpp.
 * We do it this way so that we can run the Main program either in the actual product,
 * or from a unit test environment. (Note: as of July 2021 there is no unit test setup,
 * but in theory we can/should do it sometime).
 */
#include "Main.h"
#include "ProductionTest.h"

Main paprMain;
bool inTestMode;

void setup() {
    inTestMode = paprMain.setup();
    if (inTestMode) {
        productionTestSetup();
    }
}

void loop() {
    if (inTestMode) {
        productionTestLoop();
    } else {
        paprMain.loop();
    }
}
