#include "crc_functions.h"


uint16_t MyActuator::crc_sum_v3_begin()
{
    return 0xFFFF;
}

void MyActuator::crc_sum_v3_append(uint16_t & result, uint8_t & byte)
{
    result ^= (uint16_t)byte;
    for (int i = 8; i != 0; i--) {
        if ((result & 0x0001) != 0) {
            result >>= 1;
            result ^= 0xA001;
        } else {
            result >>= 1;
        }
    }
}

void MyActuator::crc_sum_v3_append(uint16_t & result)
{
}

void MyActuator::crc_sum_v3_append(uint16_t & result, uint8_t * buffer, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++) {
        crc_sum_v3_append(result, buffer[i]);
    }
}