#ifndef PACKETPARSERV3_H
#define PACKETPARSERV3_H

#include <stdint.h>

enum class PacketParserStateV3 : uint8_t {
    Head,
    ID,
    DataLength,
    Data,
    CRC_L,
    CRC_H
};

namespace MyActuator {
    class Packet;
    class PacketParserV3 {
        private:
        uint8_t motor_id = 0;
        uint8_t data_length = 0;
        uint16_t crc_value = 0;
        uint16_t computed_crc = 0;
        uint8_t parameter_buffer[32];
        uint8_t * parameter_buffer_tail = nullptr;
        PacketParserStateV3 state = PacketParserStateV3::Head;

        public:
        bool consume(uint8_t byte);
        bool apply(Packet & packet);

        PacketParserV3();
        ~PacketParserV3();
    };
};

#endif /* PACKETPARSERV3_H */
