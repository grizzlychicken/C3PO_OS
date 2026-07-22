#ifndef PACKETPARAMETER_H
#define PACKETPARAMETER_H

#include "AccelerationFunctionIndex.h"
#include <MyActuator/Baudrate.h>
#include <MyActuator/OperatingMode.h>
#include <MyActuator/PIDParameters.h>
#include <stddef.h>
#include <stdint.h>

namespace MyActuator {

    struct PacketPadding {
        uint8_t length;

        PacketPadding(uint8_t length)
            : length(length)
        {
        }
    };

    class PacketParameter {
        public:
        static size_t encode(OperatingMode value, uint8_t * buffer);
        static size_t encode(AccelerationFunctionIndex value, uint8_t * buffer);
        static size_t encode(Baudrate value, uint8_t * buffer);
        static size_t encode(bool value, uint8_t * buffer);
        static size_t encode(uint8_t value, uint8_t * buffer);
        static size_t encode(int8_t value, uint8_t * buffer);
        static size_t encode(int16_t value, uint8_t * buffer);
        static size_t encode(uint16_t value, uint8_t * buffer);
        static size_t encode(uint32_t value, uint8_t * buffer);
        static size_t encode(int32_t value, uint8_t * buffer);
        static size_t encode(uint64_t value, uint8_t * buffer);
        static size_t encode(int64_t value, uint8_t * buffer);
        static size_t encode(float value, uint8_t * buffer);
        static size_t encode(PacketPadding value, uint8_t * buffer);
        static size_t encode(PIDParameters::Gains value, uint8_t * buffer);

        static size_t decode(OperatingMode & value, uint8_t * buffer);
        static size_t decode(AccelerationFunctionIndex & value, uint8_t * buffer);
        static size_t decode(Baudrate & value, uint8_t * buffer);
        static size_t decode(bool & value, uint8_t * buffer);
        static size_t decode(uint8_t & value, uint8_t * buffer);
        static size_t decode(int8_t & value, uint8_t * buffer);
        static size_t decode(int16_t & value, uint8_t * buffer);
        static size_t decode(uint16_t & value, uint8_t * buffer);
        static size_t decode(uint32_t & value, uint8_t * buffer);
        static size_t decode(int32_t & value, uint8_t * buffer);
        static size_t decode(uint64_t & value, uint8_t * buffer);
        static size_t decode(int64_t & value, uint8_t * buffer);
        static size_t decode(float & value, uint8_t * buffer);
        static size_t decode(PacketPadding value, uint8_t * buffer);
        static size_t decode(PIDParameters::Gains & value, uint8_t * buffer);
    };
};

#endif /* PACKETPARAMETER_H */
