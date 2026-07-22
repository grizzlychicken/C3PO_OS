#include "Packet.h"
#include <MyActuator/TestSupport/MockSerial.h>

using namespace MyActuator;
using namespace TestSupport;

void MockSerial::begin(uint32_t baud)
{
    setPortOpen(true);
}

void MockSerial::end(void)
{
    setPortOpen(false);
}

size_t MockSerial::available(void)
{
    return inputBuffer.size();
}

int16_t MockSerial::read()
{
    if (inputBuffer.isEmpty()) {
        return -1;
    } else {
        return inputBuffer.shift();
    }
}

size_t MockSerial::write(uint8_t c)
{
    outputBuffer.push(c);
    return 1;
}

size_t MockSerial::write(uint8_t * buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        outputBuffer.push(buf[i]);
    }
    return len;
}

uint32_t MockSerial::getBaud() const
{
    return 115200;
}

void MockSerial::prepareActuatorResponse(Packet & packet)
{
    inputBuffer.push(packet.packet_data, packet.packet_length);
}