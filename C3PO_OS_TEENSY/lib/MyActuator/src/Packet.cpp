
#include "Packet.h"
#include "PacketParameter.h"
#include "crc_functions.h"

using namespace MyActuator;

Packet::Packet()
{
}

Packet::Packet(Actuator & actuator, uint8_t command, bool utilityPacket)
{
    this->prepare(actuator, command, utilityPacket);
}

Packet::~Packet()
{
}

void Packet::prepareStandardPacketV3(uint8_t device_id, uint8_t command)
{
    isUtilityPacket = false;
    packet_data = nullptr;
    packet_length = 0;
    motor_id = device_id;
    command_id = command;

    protocol_version = Protocol::V3;

    params_head_ptr = data;

    *(params_head_ptr++) = 0x3E;
    *(params_head_ptr++) = motor_id;

    params_length_ptr = params_head_ptr;
    *(params_head_ptr++) = 0;

    params_tail_ptr = params_head_ptr;

    crc_ptr = params_head_ptr + 8;

    // Command byte is always the first parameter in V3 protocol
    addParameters(command);

    // extraction should begin after the command byte
    extraction_head_ptr = params_tail_ptr;

    updateCRC();
}

void Packet::prepareUtilityPacketV3(uint8_t device_id, uint8_t command)
{
    isUtilityPacket = true;
    packet_data = nullptr;
    packet_length = 0;
    motor_id = device_id;
    command_id = command;

    protocol_version = Protocol::V3;

    data[0] = 0x5A;
    data[1] = 0xA5;
    data[2] = command_id;
    data[3] = 0;

    params_length_ptr = &data[3];
    params_head_ptr = &data[4];
    params_tail_ptr = params_head_ptr;
    extraction_head_ptr = params_head_ptr;
    crc_ptr = params_head_ptr;

    updateCRC();
}

void Packet::prepareStandardPacketV2(uint8_t device_id, uint8_t command)
{
    isUtilityPacket = false;
    packet_data = nullptr;
    packet_length = 0;
    command_id = command;
    motor_id = device_id;

    protocol_version = Protocol::V2;

    params_head_ptr = data;

    data[0] = 0x3E;
    data[1] = command;
    data[2] = device_id;
    data[3] = 0;
    data[4] = 0;

    params_length_ptr = &data[3];
    header_checksum_ptr = &data[4];
    params_head_ptr = &data[5];
    params_tail_ptr = params_head_ptr;
    extraction_head_ptr = params_head_ptr;
    crc_ptr = params_head_ptr;
    updateCRC();
}

void Packet::finalizeV3()
{
    if (isUtilityPacket) {
        *params_length_ptr = (uint8_t)(params_tail_ptr - params_head_ptr);
    } else {
        *params_length_ptr = 8;
    }

    uint8_t packet_len = (uint8_t)(crc_ptr - data);

    uint16_t crc_total = crc_sum_v3_begin();
    for (uint8_t pos = 0; pos < packet_len; pos++) {
        crc_sum_v3_append(crc_total, data[pos]);
    }

    crc_ptr[0] = (uint8_t)(crc_total);
    crc_ptr[1] = (uint8_t)(crc_total >> 8);

    packet_length = packet_len + 2;
    packet_data = data;
}

void Packet::finalizeV2()
{
    uint8_t data_length = (uint8_t)(params_tail_ptr - params_head_ptr);
    *params_length_ptr = data_length;

    // Header checksum
    data[4] = data[0] + data[1] + data[2] + data[3];

    uint8_t data_checksum_length = 0;
    if (data_length != 0) {
        uint8_t data_checksum = 0;
        for (uint8_t offset = 0; offset < data_length; offset++) {
            data_checksum += params_head_ptr[offset];
        }
        crc_ptr[0] = data_checksum;
        data_checksum_length = 1;
    }

    packet_length = (params_tail_ptr - data) + data_checksum_length;
    packet_data = data;
}

void Packet::prepare(Actuator & actuator, uint8_t command, bool utilityPacket)
{
    prepare(actuator.getProtocol(), actuator.getID(), command, utilityPacket);
}


void Packet::prepare(Protocol protocol, uint8_t device_id, uint8_t command, bool utilityPacket)
{
    switch (protocol) {
        case Protocol::V2:
            prepareStandardPacketV2(device_id, command);
            break;
        case Protocol::V3:
            if (utilityPacket) {
                prepareUtilityPacketV3(device_id, command);
            } else {
                prepareStandardPacketV3(device_id, command);
            }
            break;
    }
}


void Packet::updateCRC()
{
    switch (protocol_version) {
        case Protocol::V2:
            finalizeV2();
            break;
        case Protocol::V3:
            finalizeV3();
            break;
    }
}
