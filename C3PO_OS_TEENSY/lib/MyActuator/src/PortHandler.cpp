
#include <MyActuator/PortHandler.h>

using namespace MyActuator;

PortHandler::PortHandler()
    : is_port_open(false)
{
}

bool PortHandler::isPortOpen()
{
    return is_port_open;
}

void PortHandler::setPortOpen(bool state)
{
    is_port_open = state;
}