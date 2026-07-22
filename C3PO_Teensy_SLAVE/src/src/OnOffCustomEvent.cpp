#include "src/OnOffCustomEvent.h"
#include "src/Log.h"
#include "src/OnOffCurve.h"
#include "src/Time.h"

OnOffCustomEvent::OnOffCustomEvent(char *identifier, bool startOn, byte pin) : AbstractEffector(0, 1), pin(pin)
{
    strcpy(myIdentifier, identifier);

    if (pin != 255)
    {
        pinMode(pin, OUTPUT);
    }

    Callbacks::onEffectorRegistered(this);
}

void OnOffCustomEvent::updateOnLoop()
{
    unsigned long currentTime = Time::getCurrentTimeInMs();
    OnOffCurve *targetCurve = NULL;

    for (int i = 0; i < MAX_NUM_CURVES; ++i)
    {
        OnOffCurve *curve = (OnOffCurve *)curves[i];
        if (curve == NULL)
        {
            continue;
        }

        if (curve->startTimeInMs <= currentTime)
        {
            if (targetCurve == NULL || curve->startTimeInMs > targetCurve->startTimeInMs)
            {
                targetCurve = curve;
            }
        }
    }

    // If no curves were in progress, go to the final known state
    if (targetCurve != NULL)
    {
        targetOn = targetCurve->on;
    }
}

void OnOffCustomEvent::driveOnLoop()
{
    if (currentOn != targetOn)
    {
        // callback here
        currentOn = targetOn;
        Callbacks::onOnOffCustomEventOnOffChanged(this, currentOn != 0);
        if (pin != 255)
        {
            digitalWrite(pin, currentOn);
        }
        AbstractEffector::driveOnLoop();
        AbstractEffector::callbackOnDriveComplete(currentOn, true);
    }
    else
    {
        AbstractEffector::driveOnLoop();
        AbstractEffector::callbackOnDriveComplete(currentOn, false);
    }
}

void OnOffCustomEvent::getIdentifier(char *outArray, short arraySize)
{
    strcpy(outArray, myIdentifier);
}