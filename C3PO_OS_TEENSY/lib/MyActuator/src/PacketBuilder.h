#ifndef PACKETBUILDER_H
#define PACKETBUILDER_H

#include "AccelerationFunctionIndex.h"
#include "Packet.h"
#include <MyActuator/Actuator.h>
#include <MyActuator/Angle.h>
#include <MyActuator/Baudrate.h>
#include <MyActuator/Protocol.h>
#include <MyActuator/Velocity.h>
#include <stdint.h>

namespace MyActuator {
    class PacketBuilder {
        public:
        static void prepareMotionControl(Packet & packet, Actuator & actuator, Angle position, float speed, float torque, float kP, float kD);
        static void prepareReadPIDParameters(Packet & packet, Actuator & actuator);
        static void prepareWriteVolatilePIDParameters(Packet & packet, Actuator & actuator, const PIDParameters & parameters);
        static void prepareWriteNonVolatilePIDParameters(Packet & packet, Actuator & actuator, const PIDParameters & parameters);
        static void prepareReadAccelerationParameters(Packet & packet, Actuator & actuator);
        static void prepareWriteAccelerationParameters(Packet & packet, Actuator & actuator, AccelerationFunctionIndex function, uint32_t acceleration);
        static void prepareReadMultiTurnEncoderPosition(Packet & packet, Actuator & actuator);
        static void prepareReadMultiTurnEncoderHomePosition(Packet & packet, Actuator & actuator);
        static void prepareReadMultiTurnEncoderZeroOffset(Packet & packet, Actuator & actuator);
        static void prepareWriteMultiTurnEncoderZeroOffset(Packet & packet, Actuator & actuator, int32_t offset);
        static void prepareWriteCurrentMultiTurnPositionAsHome(Packet & packet, Actuator & actuator);
        static void prepareReadSingleTurnEncoderPosition(Packet & packet, Actuator & actuator);
        static void prepareReadMultiTurnAbsoluteAngle(Packet & packet, Actuator & actuator);
        static void prepareReadSingleTurnAngle(Packet & packet, Actuator & actuator);
        static void prepareReadMotorStatusOneAndErrorFlag(Packet & packet, Actuator & actuator);
        static void prepareReadMotorStatusTwo(Packet & packet, Actuator & actuator);
        static void prepareReadMotorStatusThree(Packet & packet, Actuator & actuator);
        static void prepareMotorShutdown(Packet & packet, Actuator & actuator);
        static void prepareMotorStop(Packet & packet, Actuator & actuator);
        static void prepareTorqueControl(Packet & packet, Actuator & actuator, int16_t torque);
        static void prepareVelocityControl(Packet & packet, Actuator & actuator, Velocity velocity);
        static void prepareAbsolutePositionControl(Packet & packet, Actuator & actuator, Angle position, Velocity max_speed);
        static void prepareSingleTurnPositionControl(Packet & packet, Actuator & actuator, bool clockwise, Angle position, Velocity max_speed);
        static void prepareIncrementalPositionControl(Packet & packet, Actuator & actuator, Angle angle, Velocity max_speed);
        static void prepareReadOperatingMode(Packet & packet, Actuator & actuator);
        static void prepareReadPowerConsumption(Packet & packet, Actuator & actuator);
        static void prepareResetCommand(Packet & packet, Actuator & actuator);
        static void prepareBrakeReleaseCommand(Packet & packet, Actuator & actuator);
        static void prepareBrakeLockCommand(Packet & packet, Actuator & actuator);
        static void prepareReadUptimeCommand(Packet & packet, Actuator & actuator);
        static void prepareReadFirmwareVersionDate(Packet & packet, Actuator & actuator);
        static void prepareWriteWatchdogParameters(Packet & packet, Actuator & actuator, uint32_t duration_ms);
        static void prepareWriteBaudrateParameter(Packet & packet, Actuator & actuator, Baudrate baudrate);
        static void prepareReadMotorModel(Packet & packet, Actuator & actuator);
        static void prepareReadActuatorIDs(Packet & packet);
    };
};
#endif // PACKETBUILDER_H
