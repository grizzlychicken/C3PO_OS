#ifndef MYACTUATORBUS_H
#define MYACTUATORBUS_H

#include "ActuatorModel.h"
#include "Angle.h"
#include "Baudrate.h"
#include "ControlCommandStatus.h"
#include "ControlModeStatus.h"
#include "ErrorFlags.h"
#include "FeatureFlags.h"
#include "OperatingMode.h"
#include "PIDParameters.h"
#include "PhaseCurrentStatus.h"
#include "PortHandler.h"
#include "Velocity.h"
#include <stddef.h>
#include <stdint.h>

namespace MyActuator {
    class ActuatorBusPrivate;
    class PortHandler;
    class ActuatorBus {
        private:
        ActuatorBusPrivate * impl = nullptr;

        public:
#if defined(ESP32)
        ActuatorBus(HardwareSerial & port, const int8_t rxPin = -1, const int8_t txPin = -1, const int8_t dir_pin = -1);
#elif defined(ARDUINO)
        ActuatorBus(HardwareSerial & port, const int8_t dir_pin = -1);
#endif
        ActuatorBus(PortHandler & port);
        ~ActuatorBus();
        bool begin(int baudrate); //Baudrate baudrate);

        bool add(Actuator & actuator, uint16_t timeout_ms = 100);

        bool getPIDParameters(Actuator & actuator, PIDParameters & parameters, uint16_t timeout_ms = 100);
        bool setPIDParameters(Actuator & actuator, const PIDParameters & parameters, bool save, uint16_t timeout_ms = 100);

        bool getAcceleration(Actuator & actuator, int32_t & acceleration, uint16_t timeout_ms = 100);

        bool setMaxAcceleration(Actuator & actuator, uint32_t acceleration, OperatingMode controlMode, uint16_t timeout_ms = 100);
        bool setMaxDeceleration(Actuator & actuator, uint32_t deceleration, OperatingMode controlMode, uint16_t timeout_ms = 100);

#if ENCODER_INTERACTION_ENABLED
        bool getMultiTurnEncoderPosition(Actuator & actuator, int32_t & position, uint16_t timeout_ms = 100);
        bool getMultiTurnEncoderHomePosition(Actuator & actuator, int32_t & position, uint16_t timeout_ms = 100);
        bool getMultiTurnEncoderZeroOffset(Actuator & actuator, int32_t & offset, uint16_t timeout_ms = 100);
        bool getSingleTurnEncoderPosition(Actuator & actuator, int16_t & position, int16_t & original_position, int16_t & zero_bias, uint16_t timeout_ms = 100);

        bool setMultiTurnEncoderZeroOffset(Actuator & actuator, int32_t offset, uint16_t timeout_ms = 100);
        bool setCurrentMultiTurnPositionAsHome(Actuator & actuator, uint16_t timeout_ms = 100);
#endif

        bool getMultiTurnAbsoluteAngle(Actuator & actuator, Angle & angle, uint16_t timeout_ms = 100);
        bool getSingleTurnAngle(Actuator & actuator, Angle & angle, uint16_t timeout_ms = 100);

        bool getVoltage(Actuator & actuator, float & voltage, uint16_t timeout_ms = 100);
        bool getBrakeStatus(Actuator & actuator, bool & status, uint16_t timeout_ms = 100);
        bool getErrorFlags(Actuator & actuator, ErrorFlags & error_flags, uint16_t timeout_ms = 100);
        bool getControlModeStatus(Actuator & actuator, ControlModeStatus & status, uint16_t timeout_ms = 100);
        bool getPhaseCurrentStatus(Actuator & actuator, PhaseCurrentStatus & status, uint16_t timeout_ms = 100);

        bool shutdown(Actuator & actuator, uint16_t timeout_ms = 100);

        bool stop(Actuator & actuator, uint16_t timeout_ms = 100);

        bool setCompliantPosition(Actuator & actuator, Angle angle, float speed, float torque, float kP, float kD, uint16_t timeout_ms = 100);
        bool setTorque(Actuator & actuator, int16_t torque, TorqueControlCommandStatus * status = nullptr, uint16_t timeout_ms = 100);
        bool setVelocity(Actuator & actuator, Velocity speed, VelocityControlCommandStatus * status = nullptr, uint16_t timeout_ms = 100);

        bool setMultiTurnPosition(Actuator & actuator, Angle angle, Velocity max_speed, AbsolutePositionControlCommandStatus * status = nullptr, uint16_t timeout_ms = 100);
        bool setSingleTurnPosition(Actuator & actuator, bool clockwise, Angle position, Velocity max_speed, SingleTurnControlCommandStatus * status = nullptr, uint16_t timeout_ms = 100);
        bool setIncrementalPosition(Actuator & actuator, Angle angle, Velocity max_speed, uint16_t timeout_ms = 100);

        bool getOperatingMode(Actuator & actuator, OperatingMode & mode, uint16_t timeout_ms = 100);
        bool getPowerConsumption(Actuator & actuator, uint16_t & power_consumption, uint16_t timeout_ms = 100);

        bool reset(Actuator & actuator, uint16_t timeout_ms = 100);
        bool releaseBrake(Actuator & actuator, uint16_t timeout_ms = 100);
        bool lockBrake(Actuator & actuator, uint16_t timeout_ms = 100);

        bool getUptime(Actuator & actuator, uint32_t & uptime, uint16_t timeout_ms = 100);
        bool getFirmwareVersionDate(Actuator & actuator, uint32_t & date, uint16_t timeout_ms = 100);

        bool enableWatchdog(Actuator & actuator, uint32_t duration_ms, uint16_t timeout_ms = 100);
        bool disableWatchdog(Actuator & actuator, uint16_t timeout_ms = 100);

        bool getMotorModel(Actuator & actuator, ActuatorModel & model, uint16_t timeout_ms = 100);
    };
};

#endif /* MYACTUATORBUS_H */
