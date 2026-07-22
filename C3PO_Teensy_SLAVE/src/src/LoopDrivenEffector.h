#ifndef LoopDrivenEffector_h
#define LoopDrivenEffector_h

#include "src/AbstractEffector.h"
#include "BottangoArduinoConfig.h"
#include "Arduino.h"
#include "src/Time.h"
#include "src/FloatBezierCurve.h"
#include "src/FixedBezierCurve.h"

class LoopDrivenEffector : public AbstractEffector
{
public:
    LoopDrivenEffector(int minSignal, int maxSignal, int maxSignalPerSec, int startingSignal);
    virtual void updateSignalBounds(int minSignal, int maxSignal, int signalSpeed) override;
    virtual void updateOnLoop() override;
    virtual void driveOnLoop() override;

protected:
    unsigned long minMicrosPerSignal = 0;

    int targetSignal = 0;
    int currentSignal = 0;

    unsigned long lastUpdateTimeInUS = 0;

    int speedLimitSingal(int newTarget, unsigned long nowInUS);
};

#endif