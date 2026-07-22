#ifndef PACKETPARSERV2_H
#define PACKETPARSERV2_H

#include <stdint.h>

enum class PacketParserStateV2 : uint8_t {
    Head,
    Command,
    ID,
    DataLength,
    HeaderChecksum,
    Data,
    DataChecksum,
};

namespace MyActuator {
    class Packet;
    class PacketParserV2 {
        private:
        uint8_t motor_id = 0;
        uint8_t command_id = 0;
        uint8_t data_length = 0;
        uint8_t header_checksum_value = 0;
        uint8_t parameter_buffer[64];
        uint8_t data_checksum_value = 0;
        uint8_t * parameter_buffer_tail = nullptr;
        PacketParserStateV2 state = PacketParserStateV2::Head;

        bool isViable();

        public:
        bool consume(uint8_t byte);
        bool apply(Packet & packet);

        PacketParserV2();
        ~PacketParserV2();
    };
};

#endif /* PACKETPARSERV2_H */
