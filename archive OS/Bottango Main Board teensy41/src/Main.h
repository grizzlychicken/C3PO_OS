// !!! DRIVER VERSION: 0.7.0p2 !!!
// !!! Api Version: 7 !!!
#include <Arduino.h>
#include <src/SerialFunctions.hpp>
#include <src/SBUSFunctions.hpp>
#include <src/Audio.hpp>
//#include <LEDFunctions.hpp>
#include <src/Events.hpp>
#include <src/MotionControl.hpp>
#include "src/Definitions.h"
#include "src/BottangoCore.h"
#include "src/BasicCommands.h"



void setup()
{

    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("Booting C-3PO");
    SBUSsetup();

    SetupSerial();

    AudioSetup();
    // SetupLEDs();
    SetupMotion();
    BottangoCore::bottangoSetup();

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH); // Turn on LED to indicate setup done.
    delay (25);
    PlayWavA(4055);

}

void loop()
{
    SBUSloop();
    SerialLoop();
    AudioLoop();
    // LEDLoop();
    MotionLoop();
    ProcessEventsLoop();
    loopSwitches();
    CheckBottangoMode();
    toggleControlMode();
    BottangoCore::bottangoLoop();
}
