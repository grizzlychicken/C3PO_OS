
#include "PacketBuilder.h"
#include "Packet.h"
#include <stdint.h>

using namespace MyActuator;

void PacketBuilder::prepareMotionControl(Packet & packet, Actuator & actuator, Angle angle, float speed, float torque, float kP, float kD)
{
    packet.prepare(actuator, 0xC3, true);
    packet.addParameters((float)angle.toRadians(),
                         (float)speed,
                         (float)torque,
                         (float)kP,
                         (float)kD);
}

void PacketBuilder::prepareReadPIDParameters(Packet & packet, Actuator & actuator)
{
    packet.prepare(actuator, 0x30);
}

void PacketBuilder::prepareWriteVolatilePIDParameters(Packet & packet, Actuator & actuator, const PIDParameters & parameters)
{
    packet.prepare(actuator, 0x31);
    if (actuator.getProtocol() == Protocol::V2) {
        packet.addParameters(parameters.position, parameters.velocity, parameters.current);
    } else {
        packet.addParameters(PacketPadding(1), parameters.current, parameters.velocity, parameters.position);
    }
}

void PacketBuilder::prepareWriteNonVolatilePIDParameters(Packet & packet, Actuator & actuator, const PIDParameters & parameters)
{
    packet.prepare(actuator, 0x32);
    if (actuator.getProtocol() == Protocol::V2) {
        packet.addParameters(parameters.position, parameters.velocity, parameters.current);
    } else {
        packet.addParameters(PacketPadding(1), parameters.current, parameters.velocity, parameters.position);
    }
}

void PacketBuilder::prepareReadAccelerationParameters(Packet & packet, Actuator & actuator)
{
    switch (actuator.getProtocol()) {
        case Protocol::V2:
            packet.prepare(actuator, 0x33);
            break;
        case Protocol::V3:
            packet.prepare(actuator, 0x42);
            break;
    }
}

void PacketBuilder::prepareWriteAccelerationParameters(Packet & packet, Actuator & actuator, AccelerationFunctionIndex function, uint32_t acceleration)
{
    switch (actuator.getProtocol()) {
        case Protocol::V2:
            packet.prepare(actuator, 0x34);
            packet.addParameters((uint32_t)acceleration);
            break;
        case Protocol::V3: {
            packet.prepare(actuator, 0x43);
            packet.addParameters(
                (AccelerationFunctionIndex)function,
                PacketPadding(2),
                (uint32_t)acceleration);
        } break;
    }
}

void PacketBuilder::prepareReadMultiTurnEncoderPosition(Packet & packet, Actuator & actuator)
{
    uint8_t command = 0x0;
    switch (actuator.getProtocol()) {
        case Protocol::V2:
            command = 0x90;
            break;
        case Protocol::V3:
            command = 0x60;
            break;
        default:
            // Unsupported
            return;
    }
    packet.prepare(actuator, command);
}

void PacketBuilder::prepareReadMultiTurnEncoderHomePosition(Packet & packet, Actuator & actuator)
{
    uint8_t command = 0x0;
    switch (actuator.getProtocol()) {
        case Protocol::V2:
            command = 0x90;
            break;
        case Protocol::V3:
            command = 0x61;
            break;
        default:
            // Unsupported
            return;
    }
    packet.prepare(actuator, command);
}

void PacketBuilder::prepareReadMultiTurnEncoderZeroOffset(Packet & packet, Actuator & actuator)
{
    uint8_t command = 0x0;
    switch (actuator.getProtocol()) {
        case Protocol::V2:
            command = 0x90;
            break;
        case Protocol::V3:
            command = 0x62;
            break;
        default:
            // Unsupported
            return;
    }
    packet.prepare(actuator, command);
}

void PacketBuilder::prepareWriteMultiTurnEncoderZeroOffset(Packet & packet, Actuator & actuator, int32_t offset)
{
    switch (actuator.getProtocol()) {
        case Protocol::V2: {
            if (offset < 0 || offset > UINT16_MAX) {
                // Unsupported value
                return;
            }
            packet.prepare(actuator, 0x91);
            packet.addParameters((uint16_t)offset);
        } break;
        case Protocol::V3: {
            packet.prepare(actuator, 0x63);
            packet.addParameters(
                PacketPadding(3),
                (int32_t)offset);
        } break;
    }
}

void PacketBuilder::prepareWriteCurrentMultiTurnPositionAsHome(Packet & packet, Actuator & actuator)
{
    switch (actuator.getProtocol()) {
        case Protocol::V2: {
            packet.prepare(actuator, 0x19);
        } break;
        case Protocol::V3: {
            packet.prepare(actuator, 0x64);
        } break;
    }
}

void PacketBuilder::prepareReadSingleTurnEncoderPosition(Packet & packet, Actuator & actuator)
{
    packet.prepare(actuator, 0x90);
}

void PacketBuilder::prepareReadMultiTurnAbsoluteAngle(Packet & packet, Actuator & actuator)
{
    packet.prepare(actuator, 0x92);
}

void PacketBuilder::prepareReadSingleTurnAngle(Packet & packet, Actuator & actuator)
{
    packet.prepare(actuator, 0x94);
}

void PacketBuilder::prepareReadMotorStatusOneAndErrorFlag(Packet & packet, Actuator & actuator)
{
    packet.prepare(actuator, 0x9A);
}

void PacketBuilder::prepareReadMotorStatusTwo(Packet & packet, Actuator & actuator)
{
    packet.prepare(actuator, 0x9C);
}

void PacketBuilder::prepareReadMotorStatusThree(Packet & packet, Actuator & actuator)
{
    packet.prepare(actuator, 0x9D);
}

void PacketBuilder::prepareMotorShutdown(Packet & packet, Actuator & actuator)
{
    packet.prepare(actuator, 0x80);
}

void PacketBuilder::prepareMotorStop(Packet & packet, Actuator & actuator)
{
    packet.prepare(actuator, 0x81);
}

void PacketBuilder::prepareTorqueControl(Packet & packet, Actuator & actuator, int16_t torque)
{
    packet.prepare(actuator, 0xA1);
    packet.addParameters(
        PacketPadding(3),
        (int16_t)torque,
        PacketPadding(2));
}

void PacketBuilder::prepareVelocityControl(Packet & packet, Actuator & actuator, Velocity velocity)
{
    float velocity_lsb = velocity.degreesPerSecond() * 100.0f; // convert to LSB units

    switch (actuator.getProtocol()) {
        case Protocol::V2: {
            packet.prepare(actuator, 0xA2);
            packet.addParameters((int32_t)velocity_lsb);
        } break;
        case Protocol::V3: {
            packet.prepare(actuator, 0xA2);
            packet.addParameters(PacketPadding(3), (int32_t)velocity_lsb);
        } break;
    }
}

void PacketBuilder::prepareAbsolutePositionControl(Packet & packet, Actuator & actuator, Angle angle, Velocity max_speed)
{
    float position_lsb = angle.toDegrees() * 100.0f;                   // convert to LSB units
    float velocity_lsb = fabsf(max_speed.degreesPerSecond()) * 100.0f; // convert to LSB units

    switch (actuator.getProtocol()) {
        case Protocol::V2: {
            packet.prepare(actuator, 0xA4);
            packet.addParameters(
                (int64_t)position_lsb,
                (uint32_t)velocity_lsb);
        } break;
        case Protocol::V3: {
            packet.prepare(actuator, 0xA4);
            packet.addParameters(
                PacketPadding(1),
                (uint16_t)velocity_lsb,
                (int32_t)position_lsb);
        } break;
    }
}

void PacketBuilder::prepareSingleTurnPositionControl(Packet & packet, Actuator & actuator, bool clockwise, Angle angle, Velocity max_speed)
{
    float degrees = angle.toDegrees();

    switch (actuator.getProtocol()) {
        case Protocol::V2:
            degrees = fminf(fmaxf(degrees, 0.0f), 359.99f); // Clamp to single rotation
            break;
        case Protocol::V3:
            degrees = fminf(fmaxf(degrees, -359.99f), 359.99f); // Clamp to single rotation
            break;
    }

    float position_lsb = degrees * 100.0f;                             // convert to LSB units
    float velocity_lsb = fabsf(max_speed.degreesPerSecond()) * 100.0f; // convert to LSB units

    switch (actuator.getProtocol()) {
        case Protocol::V2: {
            packet.prepare(actuator, 0xA6);
            packet.addParameters(
                (uint8_t)(clockwise ? 0x0 : 0x01),
                (uint16_t)position_lsb,
                PacketPadding(1),
                (uint32_t)velocity_lsb);
        } break;
        case Protocol::V3: {
            packet.prepare(actuator, 0xA6);
            packet.addParameters(
                (uint8_t)(clockwise ? 0x0 : 0x01),
                (uint16_t)velocity_lsb,
                (uint16_t)position_lsb);
        } break;
    }
}

void PacketBuilder::prepareIncrementalPositionControl(Packet & packet, Actuator & actuator, Angle angle, Velocity max_speed)
{
    switch (actuator.getProtocol()) {
        case Protocol::V2: {
            float position_lsb = angle.toDegrees() * 100.0f;                   // convert to LSB units
            float velocity_lsb = fabsf(max_speed.degreesPerSecond()) * 100.0f; // convert to LSB units
            packet.prepare(actuator, 0xA8);
            packet.addParameters(
                (int32_t)position_lsb,
                (uint32_t)velocity_lsb);
        } break;
        case Protocol::V3: {
            float position_lsb = angle.toDegrees() * 100.0f;          // convert to LSB units
            float velocity_lsb = fabsf(max_speed.degreesPerSecond()); // convert to LSB units
            packet.prepare(actuator, 0xA8);
            packet.addParameters(
                PacketPadding(1),
                (uint16_t)velocity_lsb,
                (int32_t)position_lsb);
        } break;
    }
}

void PacketBuilder::prepareReadOperatingMode(Packet & packet, Actuator & actuator)
{
    packet.prepare(actuator, 0x70);
}

void PacketBuilder::prepareReadPowerConsumption(Packet & packet, Actuator & actuator)
{
    packet.prepare(actuator, 0x71);
}

void PacketBuilder::prepareResetCommand(Packet & packet, Actuator & actuator)
{
    packet.prepare(actuator, 0x76);
}

void PacketBuilder::prepareBrakeReleaseCommand(Packet & packet, Actuator & actuator)
{
    packet.prepare(actuator, 0x77);
}

void PacketBuilder::prepareBrakeLockCommand(Packet & packet, Actuator & actuator)
{
    packet.prepare(actuator, 0x78);
}

void PacketBuilder::prepareReadUptimeCommand(Packet & packet, Actuator & actuator)
{
    packet.prepare(actuator, 0xB1);
}

void PacketBuilder::prepareReadFirmwareVersionDate(Packet & packet, Actuator & actuator)
{
    packet.prepare(actuator, 0xB2);
}

void PacketBuilder::prepareWriteWatchdogParameters(Packet & packet, Actuator & actuator, uint32_t duration_ms)
{
    packet.prepare(actuator, 0xB3);
    packet.addParameters(
        PacketPadding(3),
        (uint32_t)duration_ms);
}

void PacketBuilder::prepareWriteBaudrateParameter(Packet & packet, Actuator & actuator, Baudrate baudrate)
{
    packet.prepare(actuator, 0xB4);
    packet.addParameters(
        PacketPadding(6),
        (uint8_t)baudrate);
}

void PacketBuilder::prepareReadMotorModel(Packet & packet, Actuator & actuator)
{
    switch (actuator.getProtocol()) {
        case Protocol::V2:
            packet.prepare(actuator, 0x12);
            break;
        case Protocol::V3:
            packet.prepare(actuator, 0xB5);
            break;
    }
}

void PacketBuilder::prepareReadActuatorIDs(Packet & packet)
{
    packet.prepare(Protocol::V3, 0xCD, 0x79);
    packet.addParameters(
        PacketPadding(1),
        (uint8_t)0x1); // read only version of this command
}
