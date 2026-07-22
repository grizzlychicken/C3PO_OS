#ifndef OPERATINGMODE_H
#define OPERATINGMODE_H

#include <stdint.h>

namespace MyActuator {
    enum class OperatingMode : uint8_t {
        Current = 0x01,
        Velocity = 0x02,
        Position = 0x03
    };
};

#endif /* OPERATINGMODE_H */
