#ifndef ACTUATORMODEL_H
#define ACTUATORMODEL_H

#include <stddef.h>
#include <stdint.h>

namespace MyActuator {
    enum class ActuatorModel : uint16_t {
        X4,
        RMD_X8
    };
};
#endif /* ACTUATORMODEL_H */
