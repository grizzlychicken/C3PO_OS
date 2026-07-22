#ifndef CRC_FUNCTIONS_H
#define CRC_FUNCTIONS_H

#include <stdint.h>

namespace MyActuator {

    void crc_sum_v3_append(uint16_t & result);
    void crc_sum_v3_append(uint16_t & result, uint8_t & byte);
    void crc_sum_v3_append(uint16_t & result, uint8_t * buffer, uint8_t length);

    uint16_t crc_sum_v3_begin();

    template <typename T0, typename... T1>
    uint16_t crc_sum_v3_begin(T0 base, T1... others)
    {
        uint16_t result = crc_sum_v3_begin();
        crc_sum_v3_append(result, base, others...);
        return result;
    }
};

#endif /* CRC_FUNCTIONS_H */
