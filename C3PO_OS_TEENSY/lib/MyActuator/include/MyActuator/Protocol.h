#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

namespace MyActuator {
    enum class Protocol : uint8_t {
        V2 = 2,
        V3 = 3
    };
};

#endif