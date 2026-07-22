#ifndef PACKET_H
#define PACKET_H

#include "PacketParameter.h"
#include <MyActuator/Actuator.h>
#include <MyActuator/Protocol.h>
#include <stddef.h>
#include <stdint.h>

namespace MyActuator {

    class Packet {
        private:
        uint8_t data[64] = {};

        uint8_t * params_length_ptr = nullptr;
        uint8_t * params_tail_ptr = nullptr;
        uint8_t * params_head_ptr = nullptr;
        uint8_t * extraction_head_ptr = nullptr;
        uint8_t * crc_ptr = nullptr;

        uint8_t * header_checksum_ptr = nullptr; // V2 only
        bool isUtilityPacket = false;

        void prepareStandardPacketV3(uint8_t device_id, uint8_t command);
        void prepareUtilityPacketV3(uint8_t device_id, uint8_t command);
        void prepareStandardPacketV2(uint8_t device_id, uint8_t command);
        void finalizeV3();
        void finalizeV2();

        void updateCRC();

        void addParameters()
        {
            updateCRC();
        }

        void extract_impl(uint8_t * const ptr)
        {
            updateCRC();
        }

        template <typename T0, typename... T1>
        void extract_impl(uint8_t * const input_ptr, T0 & t0, T1 &... rest)
        {
            uint8_t * ptr = input_ptr;
            if (input_ptr >= params_tail_ptr) {
                // Prevent out of bounds access
                return;
            }
            ptr += PacketParameter::decode(t0, ptr);
            extract_impl(ptr, rest...);
        }

        public:
        uint8_t packet_length = 0;
        uint8_t * packet_data = nullptr;
        uint8_t motor_id = 0;
        uint8_t command_id = 0;
        Protocol protocol_version = Protocol::V3;

        void prepare(Actuator & actuator, uint8_t command, bool utilityPacket = false);
        void prepare(Protocol protocol, uint8_t device_id, uint8_t command, bool utilityPacket = false);

        template <typename T0, typename... T1>
        void addParameters(T0 const & first, T1 const &... rest)
        {
            size_t length = PacketParameter::encode(first, params_tail_ptr);
            params_tail_ptr += length;
            if (protocol_version != Protocol::V3 || isUtilityPacket) {
                crc_ptr += length;
            }
            addParameters(rest...);
        }

        template <typename T0, typename... T1>
        void extract(T0 & t0, T1 &... rest)
        {
            uint8_t * ptr = extraction_head_ptr;
            extract_impl(ptr, t0, rest...);
        }

        template <typename T0, typename... T1, typename... T2>
        void extract(T0 & t0, T1 &... before, PacketPadding static_value, T2 &... rest)
        {
            uint8_t * ptr = extraction_head_ptr;
            extract_impl(ptr, t0, before..., static_value, rest...);
        }

        template <typename... T1>
        void extract(PacketPadding t0, T1 &... rest)
        {
            uint8_t * ptr = extraction_head_ptr;
            extract_impl(ptr, t0, rest...);
        }

        Packet();
        Packet(Actuator & actuator, uint8_t command, bool utilityPacket = false);
        ~Packet();
    };
};
#endif // PACKET_H
