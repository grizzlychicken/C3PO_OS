#include "ActuatorPrivate.h"
#include <MyActuator/Actuator.h>

using namespace MyActuator;

Actuator::Actuator(uint8_t id, ActuatorModel model, Protocol protocol)
{
    this->id = id;
    this->model = model;
    this->protocol = protocol;
    this->impl = new ActuatorPrivate(model);
}

Actuator::~Actuator()
{
    delete this->impl;
}

uint8_t Actuator::getID()
{
    return this->id;
}

ActuatorModel Actuator::getModel()
{
    return this->model;
}

Protocol Actuator::getProtocol()
{
    return this->protocol;
}

ActuatorPrivate * Actuator::getImpl()
{
    return this->impl;
}