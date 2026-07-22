#include "PacketParserV2.h"
#include "Packet.h"
#include "crc_functions.h"
#include <stddef.h>
#include <stdint.h>

using namespace MyActuator;

bool PacketParserV2::apply(Packet & packet)
{
    packet.prepare(Protocol::V2, motor_id, command_id);
    for (uint8_t i = 0; i < data_length; i++) {
        packet.addParameters(parameter_buffer[i]);
    }
    return true;
}

bool PacketParserV2::consume(uint8_t byte)
{
    switch (state) {
        case PacketParserStateV2::Head: {
            if (byte == 0x3E) {
                state = PacketParserStateV2::Command;
            }
        } break;
        case PacketParserStateV2::Command: {
            command_id = byte;
            state = PacketParserStateV2::ID;
        } break;
        case PacketParserStateV2::ID: {
            motor_id = byte;
            state = PacketParserStateV2::DataLength;
        } break;
        case PacketParserStateV2::DataLength: {
            if (byte > 60) {
                // Data length is longer than the spec allows
                state = PacketParserStateV2::Head;
                return false;
            }
            data_length = byte;
            parameter_buffer_tail = parameter_buffer;
            state = PacketParserStateV2::HeaderChecksum;
        } break;
        case PacketParserStateV2::HeaderChecksum: {
            header_checksum_value = byte;
            if (data_length == 0) {
                state = PacketParserStateV2::Head;
                return isViable();
            }
            state = PacketParserStateV2::Data;
        } break;
        case PacketParserStateV2::Data: {
            *parameter_buffer_tail = byte;
            if ((parameter_buffer_tail - parameter_buffer) == data_length - 1) {
                state = PacketParserStateV2::DataChecksum;
            } else {
                parameter_buffer_tail += 1;
            }
        } break;
        case PacketParserStateV2::DataChecksum: {
            state = PacketParserStateV2::Head;
            data_checksum_value = byte;
            return isViable();
        } break;
    }
    return false;
}

bool PacketParserV2::isViable()
{
    uint8_t header_checksum = 0;
    header_checksum += 0x3E;
    header_checksum += motor_id;
    header_checksum += command_id;
    header_checksum += data_length;

    if (header_checksum != header_checksum_value) {
        return false;
    }

    if (data_length == 0) {
        // No data to check
        return true;
    }

    uint8_t data_checksum = 0;
    for (uint8_t i = 0; i < data_length; i++) {
        data_checksum += parameter_buffer[i];
    }

    bool result = (data_checksum == data_checksum_value);
    return result;
}

PacketParserV2::PacketParserV2()
{
}

PacketParserV2::~PacketParserV2()
{
}