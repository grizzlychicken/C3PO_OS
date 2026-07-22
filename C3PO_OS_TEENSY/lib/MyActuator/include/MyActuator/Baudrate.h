#ifndef Baudrate_H
#define Baudrate_H

#include <stdint.h>

namespace MyActuator {
    enum class Baudrate : uint8_t {
        Baudrate_115_200 = 0x0,
        Baudrate_500_000 = 0x01,
        Baudrate_1_000_000 = 0x02,
        Baudrate_1_500_000 = 0x03,
        Baudrate_2_000_000 = 0x04,
    };
}

#endif /* Baudrate_H */
