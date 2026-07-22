#include <MyActuator/ErrorFlags.h>
using namespace MyActuator;

ErrorFlags::ErrorFlags(uint16_t value)
{
    this->value = value;
}

bool ErrorFlags::operator[](Flags flag) const
{
    return (value & static_cast<uint16_t>(flag)) != 0;
}