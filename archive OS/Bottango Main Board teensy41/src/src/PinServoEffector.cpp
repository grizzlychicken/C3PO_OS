#include "PinServoEffector.h"
#include "Log.h"

PinServoEffector::PinServoEffector(byte pin, short minPWM, short maxPWM, int maxPWMSec, short startSignal) : LoopDrivenEffector(minPWM, maxPWM, maxPWMSec, startSignal)
{
    this->pin = pin;

    servo.attach(pin);
#ifdef ESP32
    servo.setTimerWidth(16);
#endif
    servo.writeMicroseconds(startSignal);

    Callbacks::onEffectorRegistered(this);
}

void PinServoEffector::driveOnLoop()
{
    bool didChange = false;
    if (currentSignal != targetSignal)
    {
        servo.writeMicroseconds(targetSignal);
        currentSignal = targetSignal;
        didChange = true;
    }
    LoopDrivenEffector::driveOnLoop();
    AbstractEffector::callbackOnDriveComplete(currentSignal, didChange);
}

void PinServoEffector::getIdentifier(char *outArray, short arraySize)
{
    snprintf(outArray, arraySize, "%d", (int)pin);
}

void PinServoEffector::destroy()
{
    servo.detach();
    AbstractEffector::destroy();
}