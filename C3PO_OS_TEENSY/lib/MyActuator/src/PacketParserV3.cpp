#include "PacketParserV3.h"
#include "Packet.h"
#include "crc_functions.h"
#include <stddef.h>
#include <stdint.h>

using namespace MyActuator;

bool PacketParserV3::apply(Packet & packet)
{
    packet.prepare(Protocol::V3, motor_id, parameter_buffer[0]);
    // skip command byte
    for (uint8_t i = 1; i < data_length; i++) {
        packet.addParameters(parameter_buffer[i]);
    }
    return true;
}

bool PacketParserV3::consume(uint8_t byte)
{
    switch (state) {
        case PacketParserStateV3::Head: {
            if (byte == 0x3E) {
                state = PacketParserStateV3::ID;
                computed_crc = crc_sum_v3_begin(byte);
            }
        } break;
        case PacketParserStateV3::ID: {
            motor_id = byte;
            state = PacketParserStateV3::DataLength;
            crc_sum_v3_append(computed_crc, byte);
        } break;
        case PacketParserStateV3::DataLength: {
            data_length = byte;
            parameter_buffer_tail = parameter_buffer;
            state = PacketParserStateV3::Data;
            crc_sum_v3_append(computed_crc, byte);
        } break;
        case PacketParserStateV3::Data: {
            *parameter_buffer_tail = byte;
            crc_sum_v3_append(computed_crc, byte);
            if ((parameter_buffer_tail - parameter_buffer) == data_length - 1) {
                state = PacketParserStateV3::CRC_L;
            } else {
                parameter_buffer_tail += 1;
            }
        } break;
        case PacketParserStateV3::CRC_L: {
            crc_value = byte;
            state = PacketParserStateV3::CRC_H;
        } break;
        case PacketParserStateV3::CRC_H: {
            crc_value += (uint16_t)(byte << 8);
            state = PacketParserStateV3::Head;
            bool result = (crc_value == computed_crc);
            return result;
        } break;
    }
    return false;
}

PacketParserV3::PacketParserV3()
{
}

PacketParserV3::~PacketParserV3()
{
}