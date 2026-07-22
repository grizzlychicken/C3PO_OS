#ifndef PIDPARAMETERS_H
#define PIDPARAMETERS_H

#include <stddef.h>
#include <stdint.h>

namespace MyActuator {

    struct PIDParameters {
        struct Gains {
            uint8_t kP;
            uint8_t kI;

            inline bool operator==(const MyActuator::PIDParameters::Gains & rhs)
            {
                return kP == rhs.kP && kI == rhs.kI;
            }
        };
        Gains current;
        Gains velocity;
        Gains position;

        inline bool operator==(const MyActuator::PIDParameters & rhs)
        {
            return current == rhs.current && velocity == rhs.velocity && position == rhs.position;
        }
    };
};

#endif /* PIDPARAMETERS_H */
