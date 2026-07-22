#ifndef INCLUDE_MYACTUATOR_ACTUATOR
#define INCLUDE_MYACTUATOR_ACTUATOR

#include "ActuatorModel.h"
#include "Protocol.h"
#include <stddef.h>
#include <stdint.h>

namespace MyActuator {
    struct ActuatorPrivate;
    class Actuator {
        private:
        Protocol protocol;
        ActuatorModel model;
        uint8_t id;
        ActuatorPrivate * impl = nullptr;

        public:
        Protocol getProtocol();
        ActuatorModel getModel();
        uint8_t getID();
        ActuatorPrivate * getImpl();

        Actuator(uint8_t id, ActuatorModel model, Protocol protocol);
        ~Actuator();
    };
};

#endif // INCLUDE_MYACTUATOR_ACTUATOR
