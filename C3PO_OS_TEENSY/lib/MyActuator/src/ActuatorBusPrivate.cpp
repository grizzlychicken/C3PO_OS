#include "ActuatorBusPrivate.h"
#include "ActuatorPrivate.h"
#include "HardwareSerialPortHandler.h"
#include "Packet.h"
#include "PacketBuilder.h"
#include "PacketParser.h"
#include "PacketParserV3.h"
#include "Timeout.h"
#include <MyActuator/Actuator.h>
#include <MyActuator/PortHandler.h>
#include <MyActuator/Velocity.h>
#include <stddef.h>
#include <stdint.h>

using namespace MyActuator;

ActuatorBusPrivate::ActuatorBusPrivate(PortHandler & port)
{
    this->port_handler = &port;
}

#if defined(ESP32)
ActuatorBusPrivate::ActuatorBusPrivate(HardwareSerial & port, const int8_t rxPin, const int8_t txPin, const int8_t dir_pin)
    : port_handler(new HardwareSerialPortHandler(port, rxPin, txPin, dir_pin))
{
}
#elif defined(ARDUINO)
ActuatorBusPrivate::ActuatorBusPrivate(HardwareSerial & port, const int8_t dir_pin)
    : port_handler(new HardwareSerialPortHandler(port, dir_pin))
{
}
#endif

ActuatorBusPrivate::~ActuatorBusPrivate()
{
}

bool ActuatorBusPrivate::begin(int baudrate) //Baudrate baudrate)
{
    //switch (baudrate) {
       // case Baudrate::Baudrate_115_200:
            port_handler->begin(baudrate); //115200);
       //     break;
       // case Baudrate::Baudrate_500_000:
        //    port_handler->begin(500000);
        //    break;
        //case Baudrate::Baudrate_1_000_000:
        //    port_handler->begin(1000000);
        //    break;
       // case Baudrate::Baudrate_1_500_000:
       //     port_handler->begin(1500000);
        //    break;
        //case Baudrate::Baudrate_2_000_000:
        //    port_handler->begin(2000000);
        //    break;
   // }
    return true;
}

bool ActuatorBusPrivate::send(Packet & outgoing, Packet & incoming, PacketParser & parser, Timeout & timeout)
{
    port_handler->write(outgoing.packet_data, outgoing.packet_length);
    parser.prepare(outgoing.protocol_version);

    while (!timeout.isExpired()) {
        if (port_handler->available()) {
            if (parser.consume((uint8_t)port_handler->read())) {
                parser.apply(incoming);
                bool expected_packet = true;
                expected_packet &= (incoming.motor_id == outgoing.motor_id);
                expected_packet &= (incoming.command_id == outgoing.command_id);
                return expected_packet;
            }
        }
    }
    return false;
}

bool ActuatorBusPrivate::add(Actuator & actuator, uint16_t timeout_ms)
{
    Timeout timeout = timeout_ms;

    PacketBuilder::prepareReadMotorModel(sending_packet, actuator);
    bool success = send(sending_packet, receiving_packet, parser, timeout);
    if (success) {
        if (registered_device_count == UINT8_MAX) {
            return false;
        }
        actuator.getImpl()->isRegistered = true;
        registered_device_count++;
    }
    return success;
}

bool ActuatorBusPrivate::setCompliantPosition(Actuator & actuator, Angle angle, float speed, float torque, float kP, float kD, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }

    // Can only work with a single device on the bus
    if (registered_device_count > 1) {
        return false;
    }

    // Must be a V3 device
    if (actuator.getProtocol() != Protocol::V3) {
        return false;
    }

    Timeout timeout = timeout_ms;
    PacketBuilder::prepareMotionControl(sending_packet, actuator, angle, speed, torque, kP, kD);
    send(sending_packet, receiving_packet, parser, timeout);
    return true; // Have not explored what actual return packets look like for this command
}

bool ActuatorBusPrivate::getPIDParameters(Actuator & actuator, PIDParameters & parameters, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareReadPIDParameters(sending_packet, actuator);
    bool success = send(sending_packet, receiving_packet, parser, timeout);
    if (success) {
        if (actuator.getProtocol() == Protocol::V2) {
            receiving_packet.extract(parameters.position, parameters.velocity, parameters.current);
        } else {
            receiving_packet.extract(PacketPadding(1), parameters.current, parameters.velocity, parameters.position);
        }
    }
    return success;
}

bool ActuatorBusPrivate::setPIDParameters(Actuator & actuator, const PIDParameters & parameters, bool save, uint16_t timeout_ms)
{
    if (save) {
        return writeNonVolatilePIDParameters(actuator, parameters, timeout_ms);
    } else {
        return writeVolatilePIDParameters(actuator, parameters, timeout_ms);
    }
}

bool ActuatorBusPrivate::writeVolatilePIDParameters(Actuator & actuator, const PIDParameters & parameters, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareWriteVolatilePIDParameters(sending_packet, actuator, parameters);
    bool success = send(sending_packet, receiving_packet, parser, timeout);
    if (success) {
        PIDParameters response_parameters;
        if (actuator.getProtocol() == Protocol::V2) {
            receiving_packet.extract(response_parameters.position, response_parameters.velocity, response_parameters.current);
        } else {
            receiving_packet.extract(PacketPadding(1), response_parameters.current, response_parameters.velocity, response_parameters.position);
        }
        success &= (response_parameters == parameters);
    }
    return success;
}

bool ActuatorBusPrivate::writeNonVolatilePIDParameters(Actuator & actuator, const PIDParameters & parameters, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareWriteNonVolatilePIDParameters(sending_packet, actuator, parameters);
    bool success = send(sending_packet, receiving_packet, parser, timeout);
    if (success) {
        PIDParameters response_parameters;
        if (actuator.getProtocol() == Protocol::V2) {
            receiving_packet.extract(response_parameters.position, response_parameters.velocity, response_parameters.current);
        } else {
            receiving_packet.extract(PacketPadding(1), response_parameters.current, response_parameters.velocity, response_parameters.position);
        }
        success &= (response_parameters == parameters);
    }
    return success;
}

bool ActuatorBusPrivate::getAcceleration(Actuator & actuator, int32_t & acceleration, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareReadAccelerationParameters(sending_packet, actuator);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result) {
        if (actuator.getProtocol() == Protocol::V2) {
            receiving_packet.extract(acceleration);
        } else {
            receiving_packet.extract(PacketPadding(3), acceleration);
        }
    }
    return result;
}

bool ActuatorBusPrivate::setMaxAcceleration(Actuator & actuator, uint32_t acceleration, OperatingMode controlMode, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }

    AccelerationFunctionIndex function_index;
    switch (controlMode) {
        case OperatingMode::Position: {
            function_index = AccelerationFunctionIndex::PositionAcceleration;
        } break;
        case OperatingMode::Velocity: {
            function_index = AccelerationFunctionIndex::SpeedAcceleration;
            if (actuator.getProtocol() == Protocol::V2) {
                return false; // Not supported
            }
        } break;
        case OperatingMode::Current:
            return false; // Not supported
    }

    Timeout timeout = timeout_ms;
    PacketBuilder::prepareWriteAccelerationParameters(sending_packet, actuator, function_index, acceleration);
    bool success = send(sending_packet, receiving_packet, parser, timeout);
    if (success) {
        switch (actuator.getProtocol()) {
            case Protocol::V2: {
                uint32_t response_acceleration;
                receiving_packet.extract(response_acceleration);
                success &= (response_acceleration == acceleration);
            } break;
            case Protocol::V3: {
                uint32_t response_acceleration;
                AccelerationFunctionIndex response_function_index;
                receiving_packet.extract(response_function_index, PacketPadding(2), response_acceleration);
                success &= (response_function_index == function_index);
                success &= (response_acceleration == acceleration);
            } break;
        }
    }
    return success;
}

bool ActuatorBusPrivate::setMaxDeceleration(Actuator & actuator, uint32_t deceleration, OperatingMode controlMode, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }

    AccelerationFunctionIndex function_index;
    switch (controlMode) {
        case OperatingMode::Position: {
            function_index = AccelerationFunctionIndex::PositionDeceleration;
        } break;
        case OperatingMode::Velocity: {
            function_index = AccelerationFunctionIndex::SpeedDeceleration;
            if (actuator.getProtocol() == Protocol::V2) {
                return false; // Not supported
            }
        } break;
        case OperatingMode::Current:
            return false; // Not supported
    }

    Timeout timeout = timeout_ms;
    PacketBuilder::prepareWriteAccelerationParameters(sending_packet, actuator, function_index, deceleration);
    bool success = send(sending_packet, receiving_packet, parser, timeout);
    if (success) {
        switch (actuator.getProtocol()) {
            case Protocol::V2: {
                uint32_t response_deceleration;
                receiving_packet.extract(response_deceleration);
                success &= (response_deceleration == deceleration);
            } break;
            case Protocol::V3: {
                uint32_t response_deceleration;
                AccelerationFunctionIndex response_function_index;
                receiving_packet.extract(response_function_index, PacketPadding(2), response_deceleration);
                success &= (response_function_index == function_index);
                success &= (response_deceleration == deceleration);
            } break;
        }
    }
    return success;
}

#if ENCODER_INTERACTION_ENABLED
bool ActuatorBusPrivate::getMultiTurnEncoderPosition(Actuator & actuator, int32_t & position, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareReadMultiTurnEncoderPosition(sending_packet, actuator);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result) {
        if (actuator.getProtocol() == Protocol::V2) {
            uint16_t result_position;
            receiving_packet.extract(result_position);
            position = result_position;
        } else {
            receiving_packet.extract(PacketPadding(3), position);
        }
    }
    return result;
}
#endif

#if ENCODER_INTERACTION_ENABLED
bool ActuatorBusPrivate::getMultiTurnEncoderHomePosition(Actuator & actuator, int32_t & position, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareReadMultiTurnEncoderHomePosition(sending_packet, actuator);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result) {
        if (actuator.getProtocol() == Protocol::V2) {
            uint16_t result_position;
            receiving_packet.extract(PacketPadding(2), result_position);
            position = result_position;
        } else {
            receiving_packet.extract(PacketPadding(3), position);
        }
    }
    return result;
}
#endif

#if ENCODER_INTERACTION_ENABLED
bool ActuatorBusPrivate::getMultiTurnEncoderZeroOffset(Actuator & actuator, int32_t & offset, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareReadMultiTurnEncoderZeroOffset(sending_packet, actuator);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result) {
        if (actuator.getProtocol() == Protocol::V2) {
            uint16_t result_offset;
            receiving_packet.extract(PacketPadding(4), result_offset);
            offset = result_offset;
        } else {
            receiving_packet.extract(PacketPadding(3), offset);
        }
    }
    return result;
}
#endif

#if ENCODER_INTERACTION_ENABLED
bool ActuatorBusPrivate::setMultiTurnEncoderZeroOffset(Actuator & actuator, int32_t offset, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareWriteMultiTurnEncoderZeroOffset(sending_packet, actuator, offset);
    bool success = send(sending_packet, receiving_packet, parser, timeout);
    if (success) {
        switch (actuator.getProtocol()) {
            case Protocol::V2: {
                uint16_t response_offset;
                receiving_packet.extract(response_offset);
                success &= (response_offset == offset);
            } break;
            case Protocol::V3: {
                int32_t response_offset;
                receiving_packet.extract(PacketPadding(3), response_offset);
                success &= (response_offset == offset);
            } break;
        }
    }
    return success;
}
#endif

#if ENCODER_INTERACTION_ENABLED
bool ActuatorBusPrivate::setCurrentMultiTurnPositionAsHome(Actuator & actuator, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareWriteCurrentMultiTurnPositionAsHome(sending_packet, actuator);
    return send(sending_packet, receiving_packet, parser, timeout);
}
#endif

#if ENCODER_INTERACTION_ENABLED
bool ActuatorBusPrivate::getSingleTurnEncoderPosition(Actuator & actuator, int16_t & position, int16_t & original_position, int16_t & zero_bias, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareReadSingleTurnEncoderPosition(sending_packet, actuator);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result) {
        switch (actuator.getProtocol()) {
            case Protocol::V2: {
                receiving_packet.extract(position, original_position, zero_bias);
            } break;
            case Protocol::V3: {
                receiving_packet.extract(PacketPadding(1), position, original_position, zero_bias);
            } break;
        }
    }
    return result;
}
#endif

bool ActuatorBusPrivate::getMultiTurnAbsoluteAngle(Actuator & actuator, Angle & angle, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareReadMultiTurnAbsoluteAngle(sending_packet, actuator);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result) {
        switch (actuator.getProtocol()) {
            case Protocol::V2: {
                int64_t result_angle;
                receiving_packet.extract(result_angle);
                angle = Angle::fromDegrees(result_angle * 0.01); // convert from LSB units
            } break;
            case Protocol::V3: {
                int32_t result_angle;
                receiving_packet.extract(PacketPadding(3), result_angle);
                angle = Angle::fromDegrees(result_angle * 0.01); // convert from LSB units
            } break;
        }
    }
    return result;
}

bool ActuatorBusPrivate::getSingleTurnAngle(Actuator & actuator, Angle & angle, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareReadSingleTurnAngle(sending_packet, actuator);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result) {
        switch (actuator.getProtocol()) {
            case Protocol::V2: {
                uint16_t result_angle;
                receiving_packet.extract(result_angle);
                angle = Angle::fromDegrees(result_angle * 0.01); // convert from LSB units
            } break;
            case Protocol::V3: {
                int16_t result_angle;
                receiving_packet.extract(PacketPadding(5), result_angle);
                angle = Angle::fromDegrees(result_angle * 0.01); // convert from LSB units
            } break;
        }
    }
    return result;
}

bool ActuatorBusPrivate::getVoltage(Actuator & actuator, float & voltage, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareReadMotorStatusOneAndErrorFlag(sending_packet, actuator);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result) {
        switch (actuator.getProtocol()) {
            case Protocol::V2: {
                uint16_t returned_voltage;
                receiving_packet.extract(PacketPadding(2), returned_voltage);
                voltage = returned_voltage * 0.1f;
            } break;
            case Protocol::V3:
                uint16_t returned_voltage;
                receiving_packet.extract(PacketPadding(3), returned_voltage);
                voltage = returned_voltage * 0.1f;
                break;
        }
    }
    return result;
}

bool ActuatorBusPrivate::getBrakeStatus(Actuator & actuator, bool & status, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }

    if (actuator.getProtocol() == Protocol::V2) {
        return false; // Not supported
    }

    Timeout timeout = timeout_ms;
    PacketBuilder::prepareReadMotorStatusOneAndErrorFlag(sending_packet, actuator);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result) {
        switch (actuator.getProtocol()) {
            case Protocol::V2: {
                // unsupported
            } break;
            case Protocol::V3:
                receiving_packet.extract(PacketPadding(2), status);
                break;
        }
    }
    return result;
}

bool ActuatorBusPrivate::getErrorFlags(Actuator & actuator, ErrorFlags & error_flags, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareReadMotorStatusOneAndErrorFlag(sending_packet, actuator);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result) {
        uint16_t returned_error_flags;
        receiving_packet.extract(PacketPadding(4), returned_error_flags);
        error_flags = ErrorFlags(returned_error_flags);
    }
    return result;
}

bool ActuatorBusPrivate::getControlModeStatus(Actuator & actuator, ControlModeStatus & status, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    uint8_t control_mode;
    Protocol protocol = actuator.getProtocol();
    uint8_t device_id = actuator.getID();
    PacketBuilder::prepareReadMotorStatusTwo(sending_packet, actuator);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result) {
        switch (protocol) {
            case Protocol::V2: {
                int16_t received_speed;
                uint16_t received_position;
                receiving_packet.extract(status.temp, status.torque, received_speed, received_position);
                status.velocity = Velocity::degreesPerSecond(received_speed);
                status.angle = Angle::fromDegrees(received_position);
            } break;
            case Protocol::V3: {
                int16_t received_speed;
                int16_t received_position;
                receiving_packet.extract(status.temp, status.torque, received_speed, received_position);
                status.velocity = Velocity::degreesPerSecond(received_speed);
                status.angle = Angle::fromDegrees(received_position);
            } break;
        }
    }
    return result;
}

bool ActuatorBusPrivate::getPhaseCurrentStatus(Actuator & actuator, PhaseCurrentStatus & status, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareReadMotorStatusThree(sending_packet, actuator);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result) {
        switch (actuator.getProtocol()) {
            case Protocol::V2: {
                receiving_packet.extract(status.temp, status.phase_a_current, status.phase_b_current, status.phase_c_current);
            } break;
            case Protocol::V3: {
                receiving_packet.extract(status.temp, status.phase_a_current, status.phase_b_current, status.phase_c_current);
            } break;
        }
    }
    return result;
}

bool ActuatorBusPrivate::shutdown(Actuator & actuator, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareMotorShutdown(sending_packet, actuator);
    return send(sending_packet, receiving_packet, parser, timeout);
}

bool ActuatorBusPrivate::stop(Actuator & actuator, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareMotorStop(sending_packet, actuator);
    return send(sending_packet, receiving_packet, parser, timeout);
}

bool ActuatorBusPrivate::setTorque(Actuator & actuator, int16_t torque, TorqueControlCommandStatus * status, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareTorqueControl(sending_packet, actuator, torque);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result && status != nullptr) {
        int16_t received_speed;
        int16_t received_position;
        receiving_packet.extract(status->temp, status->torque, received_speed, received_position);
        status->speed = Velocity::degreesPerSecond(received_speed); // 1 dps/LSB
        status->position = Angle::fromDegrees(received_position);   // 1 degree/LSB
    }
    return result;
}

bool ActuatorBusPrivate::setVelocity(Actuator & actuator, Velocity speed, VelocityControlCommandStatus * status, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareVelocityControl(sending_packet, actuator, speed);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result && status != nullptr) {
        int16_t received_speed;
        int16_t received_position;
        receiving_packet.extract(status->temp, status->torque, received_speed, received_position);
        status->speed = Velocity::degreesPerSecond(received_speed); // 1 dps/LSB
        status->position = Angle::fromDegrees(received_position);   // 1 degree/LSB
    }
    return result;
}

bool ActuatorBusPrivate::setMultiTurnPosition(Actuator & actuator, Angle position, Velocity max_speed, AbsolutePositionControlCommandStatus * status, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareAbsolutePositionControl(sending_packet, actuator, position, max_speed);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result && status != nullptr) {
        int16_t received_speed;
        int16_t received_position;
        receiving_packet.extract(status->temp, status->torque, received_speed, received_position);
        status->speed = Velocity::degreesPerSecond(received_speed); // 1 dps/LSB
        status->position = Angle::fromDegrees(received_position);   // 1 degree/LSB
    }
    return result;
}

bool ActuatorBusPrivate::setSingleTurnPosition(Actuator & actuator, bool clockwise, Angle position, Velocity max_speed, SingleTurnControlCommandStatus * status, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareSingleTurnPositionControl(sending_packet, actuator, clockwise, position, max_speed);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result && status != nullptr) {
        int16_t received_speed;
        receiving_packet.extract(status->temp, status->torque, received_speed, status->encoderPosition);
        status->speed = Velocity::degreesPerSecond(received_speed); // 1 dps/LSB
    }
    return result;
}

bool ActuatorBusPrivate::setIncrementalPosition(Actuator & actuator, Angle angle, Velocity max_speed, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareIncrementalPositionControl(sending_packet, actuator, angle, max_speed);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    return result;
}

bool ActuatorBusPrivate::getOperatingMode(Actuator & actuator, OperatingMode & mode, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }

    if (actuator.getProtocol() == Protocol::V2) {
        return false; // Not supported
    }

    Timeout timeout = timeout_ms;
    PacketBuilder::prepareReadOperatingMode(sending_packet, actuator);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result) {
        receiving_packet.extract(PacketPadding(6), mode);
    }
    return result;
}

bool ActuatorBusPrivate::getPowerConsumption(Actuator & actuator, uint16_t & power_consumption, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }

    if (actuator.getProtocol() == Protocol::V2) {
        return false; // Not supported
    }

    Timeout timeout = timeout_ms;
    PacketBuilder::prepareReadPowerConsumption(sending_packet, actuator);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result) {
        receiving_packet.extract(PacketPadding(5), power_consumption);
    }
    return result;
}

bool ActuatorBusPrivate::reset(Actuator & actuator, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }
    if (actuator.getProtocol() == Protocol::V2) {
        return false;    // Not supported
    }
    Timeout timeout = 0; // motor does not reply to this command
    PacketBuilder::prepareResetCommand(sending_packet, actuator);
    send(sending_packet, receiving_packet, parser, timeout);
    return true;
}

bool ActuatorBusPrivate::releaseBrake(Actuator & actuator, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }

    if (actuator.getProtocol() == Protocol::V2) {
        return false; // Not supported
    }

    Timeout timeout = timeout_ms;
    PacketBuilder::prepareBrakeReleaseCommand(sending_packet, actuator);
    return send(sending_packet, receiving_packet, parser, timeout);
}

bool ActuatorBusPrivate::lockBrake(Actuator & actuator, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }

    if (actuator.getProtocol() == Protocol::V2) {
        return false; // Not supported
    }

    Timeout timeout = timeout_ms;
    PacketBuilder::prepareBrakeLockCommand(sending_packet, actuator);
    return send(sending_packet, receiving_packet, parser, timeout);
}

bool ActuatorBusPrivate::getUptime(Actuator & actuator, uint32_t & uptime, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }

    if (actuator.getProtocol() == Protocol::V2) {
        return false; // Not supported
    }

    Timeout timeout = timeout_ms;
    PacketBuilder::prepareReadUptimeCommand(sending_packet, actuator);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result) {
        receiving_packet.extract(PacketPadding(3), uptime);
    }
    return result;
}

bool ActuatorBusPrivate::getFirmwareVersionDate(Actuator & actuator, uint32_t & date, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }

    if (actuator.getProtocol() == Protocol::V2) {
        return false; // Not supported
    }

    Timeout timeout = timeout_ms;
    PacketBuilder::prepareReadFirmwareVersionDate(sending_packet, actuator);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result) {
        receiving_packet.extract(PacketPadding(3), date);
    }
    return result;
}

bool ActuatorBusPrivate::enableWatchdog(Actuator & actuator, uint32_t duration_ms, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }

    if (actuator.getProtocol() == Protocol::V2) {
        return false; // Not supported
    }

    Timeout timeout = timeout_ms;
    PacketBuilder::prepareWriteWatchdogParameters(sending_packet, actuator, duration_ms);
    bool success = send(sending_packet, receiving_packet, parser, timeout);
    if (success) {
        uint32_t response_duration;
        receiving_packet.extract(PacketPadding(3), response_duration);
        success &= (response_duration == duration_ms);
    }
    return success;
}

bool ActuatorBusPrivate::disableWatchdog(Actuator & actuator, uint16_t timeout_ms)
{
    return enableWatchdog(actuator, 0, timeout_ms);
}

bool ActuatorBusPrivate::getMotorModel(Actuator & actuator, ActuatorModel & model, uint16_t timeout_ms)
{
    if (!actuator.getImpl()->isRegistered) {
        return false;
    }

    uint8_t model_buffer[7] = {};
    Timeout timeout = timeout_ms;
    PacketBuilder::prepareReadMotorModel(sending_packet, actuator);
    bool result = send(sending_packet, receiving_packet, parser, timeout);
    if (result) {
        switch (actuator.getProtocol()) {
            case Protocol::V2:
                receiving_packet.extract(PacketPadding(20), model_buffer[0], model_buffer[1], model_buffer[2], model_buffer[3], model_buffer[4], model_buffer[5], model_buffer[6]);
                break;
            case Protocol::V3:
                receiving_packet.extract(model_buffer[0], model_buffer[1], model_buffer[2], model_buffer[3], model_buffer[4], model_buffer[5], model_buffer[6]);
                break;
            default:
                break;
        }
    }
    // Do something with this in the future?
    return false;
}