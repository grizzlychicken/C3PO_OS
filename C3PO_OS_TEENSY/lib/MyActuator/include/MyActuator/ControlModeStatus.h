#ifndef CONTROLMODESTATUS_H
#define CONTROLMODESTATUS_H

#include <stddef.h>
#include <stdint.h>

namespace MyActuator {
    struct ControlModeStatus {
        uint8_t temp = 0;
        int16_t torque = 0;
        Velocity velocity;
        Angle angle;
    };
};

#endif /* CONTROLMODESTATUS_H */
