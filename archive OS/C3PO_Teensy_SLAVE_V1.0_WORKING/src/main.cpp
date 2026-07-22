#include <Arduino.h>
#include <SerialFunctions.hpp>
#include <MotionControl.hpp>
#include <Definitions.h>
#include <Events.hpp>
#include <SBUSFunctions.hpp>
#include <RCtoBottangoSwitch.h>

#define TEENSY_SLAVE1

void setup()
{

    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("Booting C-3PO on Slave");
    SBUSsetup();

    SetupSerial();
    SetupMotion();

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH); // Turn on LED to indicate setup done.
    delay (25);


}

void loop()
{
    SBUSloop();
    SerialLoop();
    MotionLoop();
    ProcessEventsLoop();
    loopSwitches();
    CheckBottangoMode();
    toggleControlMode();
}
