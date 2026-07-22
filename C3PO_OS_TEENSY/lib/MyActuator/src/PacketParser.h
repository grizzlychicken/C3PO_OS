#ifndef PACKETPARSER_H
#define PACKETPARSER_H

#include "Packet.h"
#include "PacketParser.h"
#include "PacketParserV2.h"
#include "PacketParserV3.h"
#include <MyActuator/Protocol.h>
#include <stddef.h>
#include <stdint.h>

namespace MyActuator {
    class PacketParser {
        private:
        Protocol protocol_version = Protocol::V3;
        PacketParserV3 parserV3;
        PacketParserV2 parserV2;

        public:
        PacketParser();
        ~PacketParser();
        void prepare(Protocol protocol);
        bool consume(uint8_t byte);
        bool apply(Packet & packet);
    };
};
#endif /* PACKETPARSER_H */
