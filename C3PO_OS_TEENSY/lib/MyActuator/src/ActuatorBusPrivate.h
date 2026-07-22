
#ifndef ACTUATORBUSPRIVATE_H
#define ACTUATORBUSPRIVATE_H

#include "AccelerationFunctionIndex.h"
#include "PacketParser.h"
#include "Timeout.h"
#include <MyActuator/MyActuator.h>
#include <stddef.h>
#include <stdint.h>

namespace MyActuator {
    class PortHandler;
    class ActuatorBusPrivate {
        private:
        PortHandler * port_handler = nullptr;
        PacketParser parser;
        uint8_t registered_device_count = 0;

        Packet sending_packet;
        Packet receiving_packet;

        bool send(Packet & outgoing, Packet & incoming, PacketParser & parser, Timeout & timeout);

        bool writeVolatilePIDParameters(Actuator & actuator, const PIDParameters & parameters, uint16_t timeout_ms);
        bool writeNonVolatilePIDParameters(Actuator & actuator, const PIDParameters & parameters, uint16_t timeout_ms);

        public:
#if defined(ESP32)
        ActuatorBusPrivate(HardwareSerial & port, const int8_t rxPin = -1, const int8_t txPin = -1, const int8_t dir_pin = -1);
#elif defined(ARDUINO)
        ActuatorBusPrivate(HardwareSerial & port, const int8_t dir_pin = -1);
#endif
        ActuatorBusPrivate(PortHandler & port);
        ~ActuatorBusPrivate();
        bool begin(int baudrate); //Baudrate baudrate);

        bool add(Actuator & actuator, uint16_t timeout_ms);

        bool setCompliantPosition(Actuator & actuator, Angle angle, float speed, float torque, float kP, float kD, uint16_t timeout_ms);
        bool getPIDParameters(Actuator & actuator, PIDParameters & parameters, uint16_t timeout_ms);

        bool setPIDParameters(Actuator & actuator, const PIDParameters & parameters, bool save, uint16_t timeout_ms);

        bool getAcceleration(Actuator & actuator, int32_t & acceleration, uint16_t timeout_ms);

        bool setMaxAcceleration(Actuator & actuator, uint32_t acceleration, OperatingMode controlMode, uint16_t timeout_ms);
        bool setMaxDeceleration(Actuator & actuator, uint32_t deceleration, OperatingMode controlMode, uint16_t timeout_ms);

#if ENCODER_INTERACTION_ENABLED
        bool getMultiTurnEncoderPosition(Actuator & actuator, int32_t & position, uint16_t timeout_ms);
        bool getMultiTurnEncoderHomePosition(Actuator & actuator, int32_t & position, uint16_t timeout_ms);
        bool getMultiTurnEncoderZeroOffset(Actuator & actuator, int32_t & offset, uint16_t timeout_ms);
        bool getSingleTurnEncoderPosition(Actuator & actuator, int16_t & position, int16_t & original_position, int16_t & zero_bias, uint16_t timeout_ms);

        bool setMultiTurnEncoderZeroOffset(Actuator & actuator, int32_t offset, uint16_t timeout_ms);
        bool setCurrentMultiTurnPositionAsHome(Actuator & actuator, uint16_t timeout_ms);
#endif

        bool getMultiTurnAbsoluteAngle(Actuator & actuator, Angle & angle, uint16_t timeout_ms);
        bool getSingleTurnAngle(Actuator & actuator, Angle & angle, uint16_t timeout_ms);

        bool getBrakeStatus(Actuator & actuator, bool & status, uint16_t timeout_ms);
        bool getVoltage(Actuator & actuator, float & voltage, uint16_t timeout_ms);
        bool getErrorFlags(Actuator & actuator, ErrorFlags & error_flags, uint16_t timeout_ms);
        bool getControlModeStatus(Actuator & actuator, ControlModeStatus & status, uint16_t timeout_ms);
        bool getPhaseCurrentStatus(Actuator & actuator, PhaseCurrentStatus & status, uint16_t timeout_ms);

        bool shutdown(Actuator & actuator, uint16_t timeout_ms);

        bool stop(Actuator & actuator, uint16_t timeout_ms);

        bool setTorque(Actuator & actuator, int16_t torque, TorqueControlCommandStatus * status, uint16_t timeout_ms);
        bool setVelocity(Actuator & actuator, Velocity speed, VelocityControlCommandStatus * status, uint16_t timeout_ms);

        bool setMultiTurnPosition(Actuator & actuator, Angle angle, Velocity max_speed, AbsolutePositionControlCommandStatus * status, uint16_t timeout_ms);
        bool setSingleTurnPosition(Actuator & actuator, bool clockwise, Angle position, Velocity max_speed, SingleTurnControlCommandStatus * status, uint16_t timeout_ms);
        bool setIncrementalPosition(Actuator & actuator, Angle angle, Velocity max_speed, uint16_t timeout_ms);

        bool getOperatingMode(Actuator & actuator, OperatingMode & mode, uint16_t timeout_ms);
        bool getPowerConsumption(Actuator & actuator, uint16_t & power_consumption, uint16_t timeout_ms);

        bool reset(Actuator & actuator, uint16_t timeout_ms);
        bool releaseBrake(Actuator & actuator, uint16_t timeout_ms);
        bool lockBrake(Actuator & actuator, uint16_t timeout_ms);

        bool getUptime(Actuator & actuator, uint32_t & uptime, uint16_t timeout_ms);
        bool getFirmwareVersionDate(Actuator & actuator, uint32_t & date, uint16_t timeout_ms);

        bool enableWatchdog(Actuator & actuator, uint32_t duration_ms, uint16_t timeout_ms);
        bool disableWatchdog(Actuator & actuator, uint16_t timeout_ms);

        bool getMotorModel(Actuator & actuator, ActuatorModel & model, uint16_t timeout_ms);
    };
};

#endif
