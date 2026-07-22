#include "PacketParameter.h"
#include <MyActuator/Baudrate.h>

using namespace MyActuator;

size_t PacketParameter::encode(OperatingMode value, uint8_t * buffer)
{
    buffer[0] = (uint8_t)(value);
    return 1;
}

size_t PacketParameter::encode(PacketPadding value, uint8_t * buffer)
{
    for (uint8_t i = 0; i < value.length; i++) {
        buffer[i] = 0x0;
    }
    return value.length;
}

size_t PacketParameter::encode(AccelerationFunctionIndex value, uint8_t * buffer)
{
    buffer[0] = (uint8_t)(value);
    return 1;
}

size_t PacketParameter::encode(Baudrate value, uint8_t * buffer)
{
    buffer[0] = (uint8_t)(value);
    return 1;
}

size_t PacketParameter::encode(bool value, uint8_t * buffer)
{
    buffer[0] = (uint8_t)(value ? 0x01 : 0x00);
    return 1;
}

size_t PacketParameter::encode(uint8_t value, uint8_t * buffer)
{
    buffer[0] = (uint8_t)(value >> 0);
    return 1;
}

size_t PacketParameter::encode(int8_t value, uint8_t * buffer)
{
    buffer[0] = (uint8_t)(value >> 0);
    return 1;
}

size_t PacketParameter::encode(int16_t value, uint8_t * buffer)
{
    buffer[0] = (uint8_t)(value >> 0);
    buffer[1] = (uint8_t)(value >> 8);
    return 2;
}

size_t PacketParameter::encode(uint16_t value, uint8_t * buffer)
{
    buffer[0] = (uint8_t)(value >> 0);
    buffer[1] = (uint8_t)(value >> 8);
    return 2;
}

size_t PacketParameter::encode(uint32_t value, uint8_t * buffer)
{
    buffer[0] = (uint8_t)(value >> 0);
    buffer[1] = (uint8_t)(value >> 8);
    buffer[2] = (uint8_t)(value >> 16);
    buffer[3] = (uint8_t)(value >> 24);
    return 4;
}

size_t PacketParameter::encode(int32_t value, uint8_t * buffer)
{
    buffer[0] = (uint8_t)(value >> 0);
    buffer[1] = (uint8_t)(value >> 8);
    buffer[2] = (uint8_t)(value >> 16);
    buffer[3] = (uint8_t)(value >> 24);
    return 4;
}

size_t PacketParameter::encode(uint64_t value, uint8_t * buffer)
{
    buffer[0] = (uint8_t)(value >> 0);
    buffer[1] = (uint8_t)(value >> 8);
    buffer[2] = (uint8_t)(value >> 16);
    buffer[3] = (uint8_t)(value >> 24);
    buffer[4] = (uint8_t)(value >> 32);
    buffer[5] = (uint8_t)(value >> 40);
    buffer[6] = (uint8_t)(value >> 48);
    buffer[7] = (uint8_t)(value >> 56);
    return 8;
}

size_t PacketParameter::encode(int64_t value, uint8_t * buffer)
{
    buffer[0] = (uint8_t)(value >> 0);
    buffer[1] = (uint8_t)(value >> 8);
    buffer[2] = (uint8_t)(value >> 16);
    buffer[3] = (uint8_t)(value >> 24);
    buffer[4] = (uint8_t)(value >> 32);
    buffer[5] = (uint8_t)(value >> 40);
    buffer[6] = (uint8_t)(value >> 48);
    buffer[7] = (uint8_t)(value >> 56);
    return 8;
}

size_t PacketParameter::encode(float value, uint8_t * buffer)
{
    union float_bytes {
        float value;
        uint8_t bytes[4];
    };

    float_bytes converter;
    converter.value = value;

    buffer[0] = converter.bytes[0];
    buffer[1] = converter.bytes[1];
    buffer[2] = converter.bytes[2];
    buffer[3] = converter.bytes[3];

    return 4;
}

size_t PacketParameter::encode(PIDParameters::Gains value, uint8_t * buffer)
{
    size_t result = 0;
    result += encode(value.kP, buffer + result);
    result += encode(value.kI, buffer + result);
    return result;
}

size_t PacketParameter::decode(OperatingMode & value, uint8_t * buffer)
{
    value = (OperatingMode)buffer[0];
    return 1;
}

size_t PacketParameter::decode(PacketPadding value, uint8_t * buffer)
{
    return value.length;
}

size_t PacketParameter::decode(AccelerationFunctionIndex & value, uint8_t * buffer)
{
    value = (AccelerationFunctionIndex)buffer[0];
    return 1;
}

size_t PacketParameter::decode(Baudrate & value, uint8_t * buffer)
{
    value = (Baudrate)buffer[0];
    return 1;
}

size_t PacketParameter::decode(bool & value, uint8_t * buffer)
{
    value = (buffer[0] == 0x01);
    return 1;
}

size_t PacketParameter::decode(uint8_t & value, uint8_t * buffer)
{
    value = (uint8_t)buffer[0];
    return 1;
}

size_t PacketParameter::decode(int8_t & value, uint8_t * buffer)
{
    value = (int8_t)buffer[0];
    return 1;
}

size_t PacketParameter::decode(int16_t & value, uint8_t * buffer)
{
    value = ((int16_t)buffer[0] << 0) + ((int16_t)buffer[1] << 8);
    return 2;
}

size_t PacketParameter::decode(uint16_t & value, uint8_t * buffer)
{
    value = ((uint16_t)buffer[0] << 0) + ((uint16_t)buffer[1] << 8);
    return 2;
}

size_t PacketParameter::decode(uint32_t & value, uint8_t * buffer)
{
    value = ((uint32_t)buffer[0] << 0) + ((uint32_t)buffer[1] << 8) + ((uint32_t)buffer[2] << 16) + ((uint32_t)buffer[3] << 24);
    return 4;
}

size_t PacketParameter::decode(int32_t & value, uint8_t * buffer)
{
    value = ((int32_t)buffer[0] << 0) + ((int32_t)buffer[1] << 8) + ((int32_t)buffer[2] << 16) + ((int32_t)buffer[3] << 24);
    return 4;
}

size_t PacketParameter::decode(uint64_t & value, uint8_t * buffer)
{
    value = ((uint64_t)buffer[0] << 0) + ((uint64_t)buffer[1] << 8) + ((uint64_t)buffer[2] << 16) + ((uint64_t)buffer[3] << 24) + ((uint64_t)buffer[4] << 32) + ((uint64_t)buffer[5] << 40) + ((uint64_t)buffer[6] << 48) + ((uint64_t)buffer[7] << 56);
    return 8;
}

size_t PacketParameter::decode(int64_t & value, uint8_t * buffer)
{
    value = ((int64_t)buffer[0] << 0) + ((int64_t)buffer[1] << 8) + ((int64_t)buffer[2] << 16) + ((int64_t)buffer[3] << 24) + ((int64_t)buffer[4] << 32) + ((int64_t)buffer[5] << 40) + ((int64_t)buffer[6] << 48) + ((int64_t)buffer[7] << 56);
    return 8;
}

size_t PacketParameter::decode(float & value, uint8_t * buffer)
{
    union float_bytes {
        float value;
        uint8_t bytes[4];
    };

    float_bytes converter;
    converter.bytes[0] = buffer[0];
    converter.bytes[1] = buffer[1];
    converter.bytes[2] = buffer[2];
    converter.bytes[3] = buffer[3];
    value = converter.value;
    return 4;
}

size_t PacketParameter::decode(PIDParameters::Gains & value, uint8_t * buffer)
{
    size_t result = 0;
    result += decode(value.kP, buffer + result);
    result += decode(value.kI, buffer + result);
    return result;
}