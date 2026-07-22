#include "ActuatorBusPrivate.h"
#include "HardwareSerialPortHandler.h"
#include "Packet.h"
#include "PacketBuilder.h"
#include "PacketParser.h"
#include "PacketParserV3.h"
#include "Timeout.h"
#include <MyActuator/ActuatorBus.h>
#include <MyActuator/PortHandler.h>
#include <stddef.h>
#include <stdint.h>

using namespace MyActuator;

ActuatorBus::ActuatorBus(PortHandler & port)
{
    this->impl = new ActuatorBusPrivate(port);
}

#if defined(ESP32)
ActuatorBus::ActuatorBus(HardwareSerial & port, const int8_t rxPin, const int8_t txPin, const int8_t dir_pin)
{
    this->impl = new ActuatorBusPrivate(port, rxPin, txPin, dir_pin);
}

#elif defined(ARDUINO)
ActuatorBus::ActuatorBus(HardwareSerial & port, const int8_t dir_pin)
{
    this->impl = new ActuatorBusPrivate(port, dir_pin);
}
#endif

ActuatorBus::~ActuatorBus()
{
    delete impl;
}

bool ActuatorBus::begin(int baudrate) //Baudrate baudrate)
{
    return impl->begin(baudrate);
}

bool ActuatorBus::add(Actuator & actuator, uint16_t timeout_ms)
{
    return impl->add(actuator, timeout_ms);
}

bool ActuatorBus::setCompliantPosition(Actuator & actuator, Angle angle, float speed, float torque, float kP, float kD, uint16_t timeout_ms)
{
    return impl->setCompliantPosition(actuator, angle, speed, torque, kP, kD, timeout_ms);
}

bool ActuatorBus::getPIDParameters(Actuator & actuator, PIDParameters & parameters, uint16_t timeout_ms)
{
    return impl->getPIDParameters(actuator, parameters, timeout_ms);
}

bool ActuatorBus::setPIDParameters(Actuator & actuator, const PIDParameters & parameters, bool save, uint16_t timeout_ms)
{
    return impl->setPIDParameters(actuator, parameters, save, timeout_ms);
}

// bool readAccelerationParameters(Actuator& actuator);
bool ActuatorBus::getAcceleration(Actuator & actuator, int32_t & acceleration, uint16_t timeout_ms)
{
    return impl->getAcceleration(actuator, acceleration, timeout_ms);
}

bool ActuatorBus::setMaxAcceleration(Actuator & actuator, uint32_t acceleration, OperatingMode controlMode, uint16_t timeout_ms)
{
    return impl->setMaxAcceleration(actuator, acceleration, controlMode, timeout_ms);
}

bool ActuatorBus::setMaxDeceleration(Actuator & actuator, uint32_t acceleration, OperatingMode controlMode, uint16_t timeout_ms)
{
    return impl->setMaxDeceleration(actuator, acceleration, controlMode, timeout_ms);
}

#if ENCODER_INTERACTION_ENABLED
bool ActuatorBus::getMultiTurnEncoderPosition(Actuator & actuator, int32_t & position, uint16_t timeout_ms)
{
    return impl->getMultiTurnEncoderPosition(actuator, position, timeout_ms);
}
#endif

#if ENCODER_INTERACTION_ENABLED
bool ActuatorBus::getMultiTurnEncoderHomePosition(Actuator & actuator, int32_t & position, uint16_t timeout_ms)
{
    return impl->getMultiTurnEncoderHomePosition(actuator, position, timeout_ms);
}
#endif

#if ENCODER_INTERACTION_ENABLED
bool ActuatorBus::getMultiTurnEncoderZeroOffset(Actuator & actuator, int32_t & offset, uint16_t timeout_ms)
{
    return impl->getMultiTurnEncoderZeroOffset(actuator, offset, timeout_ms);
}
#endif

#if ENCODER_INTERACTION_ENABLED
bool ActuatorBus::setMultiTurnEncoderZeroOffset(Actuator & actuator, int32_t offset, uint16_t timeout_ms)
{
    return impl->setMultiTurnEncoderZeroOffset(actuator, offset, timeout_ms);
}
#endif

#if ENCODER_INTERACTION_ENABLED
bool ActuatorBus::setCurrentMultiTurnPositionAsHome(Actuator & actuator, uint16_t timeout_ms)
{
    return impl->setCurrentMultiTurnPositionAsHome(actuator, timeout_ms);
}
#endif

#if ENCODER_INTERACTION_ENABLED
bool ActuatorBus::getSingleTurnEncoderPosition(Actuator & actuator, int16_t & position, int16_t & original_position, int16_t & zero_bias, uint16_t timeout_ms)
{
    return impl->getSingleTurnEncoderPosition(actuator, position, original_position, zero_bias, timeout_ms);
}
#endif

bool ActuatorBus::getMultiTurnAbsoluteAngle(Actuator & actuator, Angle & angle, uint16_t timeout_ms)
{
    return impl->getMultiTurnAbsoluteAngle(actuator, angle, timeout_ms);
}

bool ActuatorBus::getSingleTurnAngle(Actuator & actuator, Angle & angle, uint16_t timeout_ms)
{
    return impl->getSingleTurnAngle(actuator, angle, timeout_ms);
}

bool ActuatorBus::getVoltage(Actuator & actuator, float & voltage, uint16_t timeout_ms)
{
    return impl->getVoltage(actuator, voltage, timeout_ms);
}

bool ActuatorBus::getBrakeStatus(Actuator & actuator, bool & status, uint16_t timeout_ms)
{
    return impl->getBrakeStatus(actuator, status, timeout_ms);
}

bool ActuatorBus::getErrorFlags(Actuator & actuator, ErrorFlags & error_flags, uint16_t timeout_ms)
{
    return impl->getErrorFlags(actuator, error_flags, timeout_ms);
}

bool ActuatorBus::getControlModeStatus(Actuator & actuator, ControlModeStatus & status, uint16_t timeout_ms)
{
    return impl->getControlModeStatus(actuator, status, timeout_ms);
}

bool ActuatorBus::getPhaseCurrentStatus(Actuator & actuator, PhaseCurrentStatus & status, uint16_t timeout_ms)
{
    return impl->getPhaseCurrentStatus(actuator, status, timeout_ms);
}

bool ActuatorBus::shutdown(Actuator & actuator, uint16_t timeout_ms)
{
    return impl->shutdown(actuator, timeout_ms);
}

bool ActuatorBus::stop(Actuator & actuator, uint16_t timeout_ms)
{
    return impl->stop(actuator, timeout_ms);
}

bool ActuatorBus::setTorque(Actuator & actuator, int16_t torque, TorqueControlCommandStatus * status, uint16_t timeout_ms)
{
    return impl->setTorque(actuator, torque, status, timeout_ms);
}

bool ActuatorBus::setVelocity(Actuator & actuator, Velocity velocity, VelocityControlCommandStatus * status, uint16_t timeout_ms)
{
    return impl->setVelocity(actuator, velocity, status, timeout_ms);
}

bool ActuatorBus::setMultiTurnPosition(Actuator & actuator, Angle angle, Velocity max_speed, AbsolutePositionControlCommandStatus * status, uint16_t timeout_ms)
{
    return impl->setMultiTurnPosition(actuator, angle, max_speed, status, timeout_ms);
}

bool ActuatorBus::setSingleTurnPosition(Actuator & actuator, bool clockwise, Angle angle, Velocity max_speed, SingleTurnControlCommandStatus * status, uint16_t timeout_ms)
{
    return impl->setSingleTurnPosition(actuator, clockwise, angle, max_speed, status, timeout_ms);
}

bool ActuatorBus::setIncrementalPosition(Actuator & actuator, Angle angle, Velocity max_speed, uint16_t timeout_ms)
{
    return impl->setIncrementalPosition(actuator, angle, max_speed, timeout_ms);
}

bool ActuatorBus::getOperatingMode(Actuator & actuator, OperatingMode & mode, uint16_t timeout_ms)
{
    return impl->getOperatingMode(actuator, mode, timeout_ms);
}

bool ActuatorBus::getPowerConsumption(Actuator & actuator, uint16_t & power_consumption, uint16_t timeout_ms)
{
    return impl->getPowerConsumption(actuator, power_consumption, timeout_ms);
}

bool ActuatorBus::reset(Actuator & actuator, uint16_t timeout_ms)
{
    return impl->reset(actuator, timeout_ms);
}

bool ActuatorBus::releaseBrake(Actuator & actuator, uint16_t timeout_ms)
{
    return impl->releaseBrake(actuator, timeout_ms);
}

bool ActuatorBus::lockBrake(Actuator & actuator, uint16_t timeout_ms)
{
    return impl->lockBrake(actuator, timeout_ms);
}

bool ActuatorBus::getUptime(Actuator & actuator, uint32_t & uptime, uint16_t timeout_ms)
{
    return impl->getUptime(actuator, uptime, timeout_ms);
}

bool ActuatorBus::getFirmwareVersionDate(Actuator & actuator, uint32_t & date, uint16_t timeout_ms)
{
    return impl->getFirmwareVersionDate(actuator, date, timeout_ms);
}

bool ActuatorBus::enableWatchdog(Actuator & actuator, uint32_t duration_ms, uint16_t timeout_ms)
{
    return impl->enableWatchdog(actuator, duration_ms, timeout_ms);
}

bool ActuatorBus::disableWatchdog(Actuator & actuator, uint16_t timeout_ms)
{
    return impl->disableWatchdog(actuator, timeout_ms);
}

bool ActuatorBus::getMotorModel(Actuator & actuator, ActuatorModel & model, uint16_t timeout_ms)
{
    return impl->getMotorModel(actuator, model, timeout_ms);
}