#include "PacketParser.h"
#include <stddef.h>
#include <stdint.h>

using namespace MyActuator;

void PacketParser::prepare(Protocol protocol)
{
    protocol_version = protocol;
}

bool PacketParser::consume(uint8_t byte)
{
    switch (protocol_version) {
        case Protocol::V3:
            return parserV3.consume(byte);
        case Protocol::V2:
            return parserV2.consume(byte);
        default:
            return false;
    }
}

bool PacketParser::apply(Packet & packet)
{
    switch (protocol_version) {
        case Protocol::V3:
            return parserV3.apply(packet);
        case Protocol::V2:
            return parserV2.apply(packet);
        default:
            return false;
    }
}

PacketParser::PacketParser()
{
}

PacketParser::~PacketParser()
{
}
