#ifndef ACCELERATION_FUNCTION_INDEX_H
#define ACCELERATION_FUNCTION_INDEX_H

#include <stdint.h>

namespace MyActuator {
    enum class AccelerationFunctionIndex : uint8_t {
        PositionAcceleration = 0x0,
        PositionDeceleration = 0x01,
        SpeedAcceleration = 0x02,
        SpeedDeceleration = 0x03
    };
};

#endif /* ACCELERATION_FUNCTION_INDEX_H */
