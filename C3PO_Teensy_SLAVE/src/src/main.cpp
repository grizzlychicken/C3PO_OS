#include <Arduino.h>
#include <src/SerialFunctions.hpp>
#include <src/MotionControl.hpp>
#include <src/Definitions.h>
#include <src/Events.hpp>
#include <src/SBUSFunctions.hpp>
#include "BottangoCore.h"
#include "BasicCommands.h"
// #include <RCtoBottangoSwitch.h>

// #define TEENSY_SLAVE1

void setup()
{

    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("Booting C-3PO on Slave");
    SBUSsetup();

    SetupSerial();
    SetupMotion();
    // BottangoCore::bottangoSetup();

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH); // Turn on LED to indicate setup done.
    delay (25);


}

void loop()
{
    SBUSloop();
    SerialLoop();
    CheckRCBottangoMode();
    ProcessEventsLoop();
    MotionLoop();
    // loopSwitches();
    // CheckBottangoMode();
    // toggleControlMode();
}
