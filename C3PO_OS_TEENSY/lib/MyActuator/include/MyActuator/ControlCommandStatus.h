#ifndef TORQUECONTROLSTATUS_H
#define TORQUECONTROLSTATUS_H

#include "Angle.h"
#include "Velocity.h"
#include <stddef.h>
#include <stdint.h>

namespace MyActuator {

    struct SingleTurnControlCommandStatus {
        int8_t temp;              // 1ºC / LSB
        int16_t torque;           // 0.01A/LSB
        Velocity speed;
        uint16_t encoderPosition; // range determined by number of bits in encoder

        inline bool operator==(const SingleTurnControlCommandStatus & rhs)
        {
            return temp == rhs.temp && torque == rhs.torque && speed == rhs.speed && encoderPosition == rhs.encoderPosition;
        }
    };

    struct TorqueControlCommandStatus {
        int8_t temp;    // 1ºC / LSB
        int16_t torque; // 0.01A/LSB
        Velocity speed;
        Angle position;

        inline bool operator==(const TorqueControlCommandStatus & rhs)
        {
            return temp == rhs.temp && torque == rhs.torque && speed == rhs.speed && position == rhs.position;
        }
    };

    typedef TorqueControlCommandStatus VelocityControlCommandStatus;
    typedef TorqueControlCommandStatus AbsolutePositionControlCommandStatus;
};

#endif /* TORQUECONTROLSTATUS_H */
