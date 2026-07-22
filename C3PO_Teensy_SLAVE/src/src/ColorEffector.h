#ifndef ColorEffector_h
#define ColorEffector_h

#include "src/AbstractEffector.h"
#include "Arduino.h"
#include "src/Time.h"
#include "src/ColorCurve.h"

class ColorEffector : public AbstractEffector
{
public:
    ColorEffector(byte startingRed, byte startingGreen, byte startingBlue);

    virtual void updateOnLoop() override;
    virtual void driveOnLoop() override;

protected:
    Color currentColor;
    Color targetColor;
};

#endif