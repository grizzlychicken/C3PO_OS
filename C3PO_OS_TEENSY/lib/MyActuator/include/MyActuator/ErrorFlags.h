#ifndef ERRORFLAGS_H
#define ERRORFLAGS_H

#include <stddef.h>
#include <stdint.h>

namespace MyActuator {
    struct ErrorFlags {
        enum class Flags : uint16_t {
            MOTOR_STALLED = 0x0002,
            LOW_PRESSURE = 0x0004,
            OVER_VOLTAGE = 0x0008,
            OVER_CURRENT = 0x0010,
            POWER_OVERRUN = 0x0040,
            SPEEDING = 0x0100,
            OVERHEATING = 0x1000,
            ENCODER_CALIBRATION_ERROR = 0x2000
        };

        private:
        public:
        uint16_t value;
        ErrorFlags(uint16_t value = 0);

        bool operator[](Flags flag) const;
    };
};

#endif /* ERRORFLAGS_H */
