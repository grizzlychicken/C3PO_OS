
#include "Packet.h"
#include "PacketBuilder.h"
#include <MyActuator/MyActuator.h>
#include <MyActuator/TestSupport/TestSupport.h>
#include <unity.h>

using namespace MyActuator;
using namespace TestSupport;

inline void _mock_register_actuator(MockSerial & mockSerial, ActuatorBus & bus, Actuator & actuator, uint16_t line)
{
    // Fill the buffer with the response since the bus command call will block
    Packet mockResponse;
    PacketBuilder::prepareReadMotorModel(mockResponse, actuator);
    mockSerial.inputBuffer.push(mockResponse.packet_data, mockResponse.packet_length);
    UNITY_TEST_ASSERT(bus.add(actuator), line, nullptr);
}

#define mock_register_actuator(mockSerial, bus, actuator) \
    _mock_register_actuator(mockSerial, bus, actuator, __LINE__)

void test_register_actuator()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);

    // Fill the buffer with the response since the bus command call will block
    Packet deviceResponse(actuator, 0x12);
    deviceResponse.addParameters(PacketPadding(20)); // driver name
    deviceResponse.addParameters(PacketPadding(20)); // motor name
    deviceResponse.addParameters(0x30);              // Hardware version
    deviceResponse.addParameters(0x20);              // Firmware version

    serial.prepareActuatorResponse(deviceResponse);

    TEST_ASSERT_TRUE(bus.add(actuator));
}

void test_get_pid_parameters()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x30;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    PIDParameters mockGains;
    mockGains.current.kP = 0x55;
    mockGains.current.kI = 0x19;
    mockGains.velocity.kP = 0x55;
    mockGains.velocity.kI = 0x19;
    mockGains.position.kP = 0x55;
    mockGains.position.kI = 0x19;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mockGains.position, mockGains.velocity, mockGains.current);

    // Fill the buffer with the response since the bus command call will block
    serial.prepareActuatorResponse(mockResponse);

    PIDParameters actualGains;

    bool result = bus.getPIDParameters(actuator, actualGains);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(mockGains == actualGains);
}

void test_write_volatile_pid_parameters_success()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x31;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    PIDParameters mockGains;
    mockGains.current.kP = 0x55;
    mockGains.current.kI = 0x19;
    mockGains.velocity.kP = 0x55;
    mockGains.velocity.kI = 0x19;
    mockGains.position.kP = 0x55;
    mockGains.position.kI = 0x19;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mockGains.position, mockGains.velocity, mockGains.current);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setPIDParameters(actuator, mockGains, false);
    TEST_ASSERT_TRUE(result);
}

void test_write_volatile_pid_parameters_mismatch_failure()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x31;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    PIDParameters mockGains;
    mockGains.current.kP = 0x55;
    mockGains.current.kI = 0x19;
    mockGains.velocity.kP = 0x32; // wrong value
    mockGains.velocity.kI = 0x19;
    mockGains.position.kP = 0x55;
    mockGains.position.kI = 0x19;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mockGains.position, mockGains.velocity, mockGains.current);

    serial.prepareActuatorResponse(mockResponse);

    PIDParameters correctGains;
    correctGains.current.kP = 0x55;
    correctGains.current.kI = 0x19;
    correctGains.velocity.kP = 0x55;
    correctGains.velocity.kI = 0x19;
    correctGains.position.kP = 0x55;
    correctGains.position.kI = 0x19;

    bool result = bus.setPIDParameters(actuator, correctGains, false);
    TEST_ASSERT_FALSE(result); // Should fail because values written to the actuator do not match the values get back
}

void test_write_nonvolatile_pid_parameters_success()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x32;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    PIDParameters mockGains;
    mockGains.current.kP = 0x55;
    mockGains.current.kI = 0x19;
    mockGains.velocity.kP = 0x55;
    mockGains.velocity.kI = 0x19;
    mockGains.position.kP = 0x55;
    mockGains.position.kI = 0x19;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mockGains.position, mockGains.velocity, mockGains.current);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setPIDParameters(actuator, mockGains, true);
    TEST_ASSERT_TRUE(result);
}

void test_write_nonvolatile_pid_parameters_mismatch_failure()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x32;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    PIDParameters mockGains;
    mockGains.current.kP = 0x55;
    mockGains.current.kI = 0x19;
    mockGains.velocity.kP = 0x32; // wrong value
    mockGains.velocity.kI = 0x19;
    mockGains.position.kP = 0x55;
    mockGains.position.kI = 0x19;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mockGains.position, mockGains.velocity, mockGains.current);

    serial.prepareActuatorResponse(mockResponse);

    PIDParameters correctGains;
    correctGains.current.kP = 0x55;
    correctGains.current.kI = 0x19;
    correctGains.velocity.kP = 0x55;
    correctGains.velocity.kI = 0x19;
    correctGains.position.kP = 0x55;
    correctGains.position.kI = 0x19;

    bool result = bus.setPIDParameters(actuator, correctGains, true);
    TEST_ASSERT_FALSE(result); // Should fail because values written to the actuator do not match the values get back
}

void test_get_acceleration()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x33;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    int32_t mock_acceleration = 10000;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_acceleration);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.getAcceleration(actuator, mock_acceleration);
    TEST_ASSERT_TRUE(result);
}

void test_write_position_acceleration()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x34;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint32_t mock_acceleration = 10000;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_acceleration);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setMaxAcceleration(actuator, mock_acceleration, OperatingMode::Position);
    TEST_ASSERT_TRUE(result);
}

void test_write_position_deceleration()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x34;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint32_t mock_deceleration = 10000;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_deceleration);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setMaxDeceleration(actuator, mock_deceleration, OperatingMode::Position);
    TEST_ASSERT_TRUE(result);
}

void test_write_speed_acceleration()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x43;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    bool result = bus.setMaxAcceleration(actuator, 10000, OperatingMode::Velocity);
    TEST_ASSERT_FALSE(result);
}

void test_write_speed_deceleration()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x43;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    bool result = bus.setMaxDeceleration(actuator, 10000, OperatingMode::Velocity);
    TEST_ASSERT_FALSE(result);
}

#if ENCODER_INTERACTION_ENABLED
void test_get_multi_turn_encoder_position()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x90;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    const uint16_t mock_position = 1234;
    const uint16_t mock_origin = 5678;
    const uint16_t mock_offset = 9999;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_position, mock_origin, mock_offset);
    serial.prepareActuatorResponse(mockResponse);

    int32_t position;
    bool result = bus.getMultiTurnEncoderPosition(actuator, position);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_INT32(mock_position, position);
}
#endif

#if ENCODER_INTERACTION_ENABLED
void test_get_multi_turn_encoder_home_position()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x90;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    const uint16_t mock_position = 1234;
    const uint16_t mock_origin = 5678;
    const uint16_t mock_offset = 9999;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_position, mock_origin, mock_offset);
    serial.prepareActuatorResponse(mockResponse);

    int32_t origin;
    bool result = bus.getMultiTurnEncoderHomePosition(actuator, origin);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_INT32(mock_origin, origin);
}
#endif

#if ENCODER_INTERACTION_ENABLED
void test_get_multi_turn_encoder_zero_position()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x90;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    const uint16_t mock_position = 1234;
    const uint16_t mock_origin = 5678;
    const uint16_t mock_offset = 9999;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_position, mock_origin, mock_offset);
    serial.prepareActuatorResponse(mockResponse);

    int32_t offset;
    bool result = bus.getMultiTurnEncoderZeroOffset(actuator, offset);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_INT32(mock_offset, offset);
}
#endif

#if ENCODER_INTERACTION_ENABLED
void test_write_multi_turn_encoder_zero_offset()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x91;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    int32_t mock_position = 1000;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_position);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setMultiTurnEncoderZeroOffset(actuator, mock_position);
    TEST_ASSERT_TRUE(result);
}
#endif

#if ENCODER_INTERACTION_ENABLED
void test_write_current_multi_turn_position_as_home()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x19;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    Packet mockResponse(actuator, command);
    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setCurrentMultiTurnPositionAsHome(actuator);
    TEST_ASSERT_TRUE(result);
}
#endif

#if ENCODER_INTERACTION_ENABLED
void test_get_single_turn_encoder_position()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x90;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    int16_t mock_position = 12345;
    int16_t mock_original_position = 12345;
    int16_t mock_zero_bias = 12345;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_position, mock_original_position, mock_zero_bias);

    serial.prepareActuatorResponse(mockResponse);

    int16_t position = 0;
    int16_t original_position = 0;
    int16_t zero_bias = 0;
    bool result = bus.getSingleTurnEncoderPosition(actuator, position, original_position, zero_bias);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_INT16(mock_position, position);
    TEST_ASSERT_EQUAL_INT16(mock_original_position, original_position);
    TEST_ASSERT_EQUAL_INT16(mock_zero_bias, zero_bias);
}
#endif

void test_get_multi_turn_absolute_angle()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x92;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    const float mock_degrees = 180.0f;
    const int64_t mock_angle = mock_degrees * 100;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_angle);

    serial.prepareActuatorResponse(mockResponse);

    Angle angle;
    bool result = bus.getMultiTurnAbsoluteAngle(actuator, angle);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_FLOAT(mock_degrees, angle.toDegrees());
}

void test_get_single_turn_angle()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x94;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    const float mock_degrees = 180.0f;
    const uint16_t mock_angle = mock_degrees * 100;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_angle);

    serial.prepareActuatorResponse(mockResponse);

    Angle angle;
    bool result = bus.getSingleTurnAngle(actuator, angle);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_FLOAT(angle.toDegrees(), mock_degrees);
}

void test_get_motor_voltage()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x9A;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temp = 123;
    uint8_t mock_brake_active = 0x0;
    uint16_t mock_voltage = 234;
    uint16_t mock_error_flags = 345;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temp, PacketPadding(1), mock_voltage, PacketPadding(2), mock_error_flags);

    serial.prepareActuatorResponse(mockResponse);

    float voltage;
    bool result = bus.getVoltage(actuator, voltage);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_FLOAT(23.4, voltage);
}

void test_get_error_flags()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x9A;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temp = 123;
    uint8_t mock_brake_active = 0x0;
    uint8_t mock_voltage = 234;
    uint16_t mock_error_flags = (uint16_t)ErrorFlags::Flags::MOTOR_STALLED;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temp, PacketPadding(1), mock_brake_active, mock_voltage, mock_error_flags);

    serial.prepareActuatorResponse(mockResponse);

    ErrorFlags error_flags;
    bool result = bus.getErrorFlags(actuator, error_flags);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(error_flags[ErrorFlags::Flags::MOTOR_STALLED]);
}

void test_get_motor_status_two()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x9C;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temp = 123;
    int16_t mock_torque = 234;
    int16_t mock_speed = 15;
    uint16_t mock_angle = 123;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temp, mock_torque, mock_speed, mock_angle);

    serial.prepareActuatorResponse(mockResponse);

    ControlModeStatus status;
    bool result = bus.getControlModeStatus(actuator, status);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_INT8(mock_temp, status.temp);
    TEST_ASSERT_EQUAL_INT16(mock_torque, status.torque);
    TEST_ASSERT_EQUAL_FLOAT(15.0f, status.velocity.degreesPerSecond());
    TEST_ASSERT_EQUAL_FLOAT(123.0f, status.angle.toDegrees());
}

void test_get_motor_status_three()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x9D;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temp = 123;
    int16_t mock_phase_a_current = 0xAB;
    int16_t mock_phase_b_current = 0xFF;
    int16_t mock_phase_c_current = 0x04;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temp, mock_phase_a_current, mock_phase_b_current, mock_phase_c_current);

    serial.prepareActuatorResponse(mockResponse);

    PhaseCurrentStatus status;
    bool result = bus.getPhaseCurrentStatus(actuator, status);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_INT8(mock_temp, status.temp);
    TEST_ASSERT_EQUAL_INT16(mock_phase_a_current, status.phase_a_current);
    TEST_ASSERT_EQUAL_INT16(mock_phase_b_current, status.phase_b_current);
    TEST_ASSERT_EQUAL_INT16(mock_phase_c_current, status.phase_c_current);
}

void test_stop()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x81;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    Packet mockResponse(actuator, command);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.stop(actuator);
    TEST_ASSERT_TRUE(result);
}

void test_shutdown()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x80;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    Packet mockResponse(actuator, command);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.shutdown(actuator);
    TEST_ASSERT_TRUE(result);
}

void test_torque_control()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0xA1;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temperature = 0x12;
    int16_t mock_torque = 0x1234;
    int16_t mock_speed = 0x5678;
    int16_t mock_position = 0x9ABC;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temperature, mock_torque, mock_speed, mock_position);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setTorque(actuator, mock_torque);
    TEST_ASSERT_TRUE(result);
}

void test_torque_control_with_status()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0xA1;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temperature = 0x12;
    int16_t mock_torque = 0x1234;

    const float mock_speed_dps = 15.0f; // 15 degrees per second
    int16_t mock_speed = mock_speed_dps;

    const float mock_angle_deg = 90.0f;
    int16_t mock_position = mock_angle_deg;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temperature, mock_torque, mock_speed, mock_position);

    serial.prepareActuatorResponse(mockResponse);

    TorqueControlCommandStatus status;
    bool result = bus.setTorque(actuator, mock_torque, &status);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(mock_temperature, status.temp);
    TEST_ASSERT_EQUAL_INT16(mock_torque, status.torque);
    TEST_ASSERT_EQUAL_FLOAT(mock_speed_dps, status.speed.degreesPerSecond());
    TEST_ASSERT_EQUAL_FLOAT(mock_angle_deg, status.position.toDegrees());
}

void test_speed_control()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0xA2;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temperature = 0x12;
    int16_t mock_torque = 0x1234;

    const float mock_speed_dps = 15.0f; // 15 degrees per second
    int16_t mock_speed = mock_speed_dps * 100.0f;

    const float mock_angle_deg = 90.0f;
    int16_t mock_position = mock_angle_deg * 100.0f;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temperature, mock_torque, mock_speed, mock_position);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setVelocity(actuator, Velocity::degreesPerSecond(mock_speed_dps));
    TEST_ASSERT_TRUE(result);
}

void test_speed_control_with_status()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0xA2;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temperature = 0x12;
    int16_t mock_torque = 0x1234;

    const float mock_angle_deg = 90.0f;
    int16_t mock_position = mock_angle_deg;

    const float mock_speed_dps = 15.0f; // 15 degrees per second
    int16_t mock_speed = mock_speed_dps;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temperature, mock_torque, mock_speed, mock_position);

    serial.prepareActuatorResponse(mockResponse);

    VelocityControlCommandStatus status;
    bool result = bus.setVelocity(actuator, Velocity::degreesPerSecond(mock_speed_dps), &status);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(mock_temperature, status.temp);
    TEST_ASSERT_EQUAL_INT16(mock_torque, status.torque);
    TEST_ASSERT_EQUAL_INT16(mock_speed_dps, status.speed.degreesPerSecond());
    TEST_ASSERT_EQUAL_FLOAT(mock_angle_deg, status.position.toDegrees());
}

void test_absolute_position_control()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0xA4;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temperature = 0x12;
    int16_t mock_torque = 0x1234;

    const float mock_angle_deg = 90.0f;
    int16_t mock_position = mock_angle_deg * 100.0f;

    const float mock_speed_dps = 15.0f; // 15 degrees per second
    int16_t mock_speed = mock_speed_dps * 100.0f;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temperature, mock_torque, mock_speed, mock_position);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setMultiTurnPosition(actuator, Angle::fromDegrees(mock_angle_deg), Velocity::degreesPerSecond(mock_speed_dps));
    TEST_ASSERT_TRUE(result);
}

void test_absolute_position_control_with_status()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0xA4;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temperature = 0x12;
    int16_t mock_torque = 0x1234;

    const float mock_angle_deg = 90.0f;
    int16_t mock_position = mock_angle_deg * 100.0f;

    const float mock_speed_dps = 15.0f; // 15 degrees per second
    int16_t mock_speed = mock_speed_dps * 100.0f;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temperature, mock_torque, mock_speed, mock_position);

    serial.prepareActuatorResponse(mockResponse);

    AbsolutePositionControlCommandStatus status;
    bool result = bus.setMultiTurnPosition(actuator, Angle::fromDegrees(mock_angle_deg), Velocity::degreesPerSecond(mock_speed_dps), &status);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(mock_temperature, status.temp);
    TEST_ASSERT_EQUAL_INT16(mock_torque, status.torque);
    TEST_ASSERT_EQUAL_FLOAT(mock_speed, status.speed.degreesPerSecond());
    TEST_ASSERT_EQUAL_FLOAT(mock_position, status.position.toDegrees());
}

void test_single_turn_position_control()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0xA6;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temperature = 0x12;
    int16_t mock_torque = 0x1234;

    const float mock_angle_deg = 90.0f;
    int16_t mock_position = mock_angle_deg * 100.0f;

    const float mock_speed_dps = 15.0f; // 15 degrees per second
    int16_t mock_speed = mock_speed_dps * 100.0f;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temperature, mock_torque, mock_speed, mock_position);

    serial.prepareActuatorResponse(mockResponse);

    bool clockwise = true;
    bool result = bus.setSingleTurnPosition(actuator, clockwise, Angle::fromDegrees(mock_angle_deg), Velocity::degreesPerSecond(mock_speed_dps));
    TEST_ASSERT_TRUE(result);
}

void test_single_turn_position_control_with_status()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0xA6;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temperature = 0x12;
    int16_t mock_torque = 0x1234;

    const float mock_angle_deg = 90.0f;
    int16_t mock_position = mock_angle_deg;

    const float mock_speed_dps = 15.0f; // 15 degrees per second
    int16_t mock_speed = mock_speed_dps;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temperature, mock_torque, mock_speed, mock_position);

    serial.prepareActuatorResponse(mockResponse);

    bool clockwise = true;
    SingleTurnControlCommandStatus status;
    bool result = bus.setSingleTurnPosition(actuator, clockwise, Angle::fromDegrees(mock_angle_deg), Velocity::degreesPerSecond(mock_speed_dps), &status);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(mock_temperature, status.temp);
    TEST_ASSERT_EQUAL_INT16(mock_torque, status.torque);
    TEST_ASSERT_EQUAL_FLOAT(mock_speed_dps, status.speed.degreesPerSecond());
    TEST_ASSERT_EQUAL_UINT16(mock_angle_deg, status.encoderPosition);
}

void test_incremental_position_control()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0xA8;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temperature = 0x12;
    int16_t mock_torque = 0x1234;

    const float mock_angle_deg = 90.0f;
    int16_t mock_position = mock_angle_deg * 100.0f;

    const float mock_speed_dps = 15.0f; // 15 degrees per second
    int16_t mock_speed = mock_speed_dps * 100.0f;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temperature, mock_torque, mock_speed, mock_position);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setIncrementalPosition(actuator, Angle::fromDegrees(mock_angle_deg), Velocity::degreesPerSecond(mock_speed_dps));
    TEST_ASSERT_TRUE(result);
}

void test_get_operating_mode()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    OperatingMode mode;
    bool result = bus.getOperatingMode(actuator, mode);
    TEST_ASSERT_FALSE(result);
}

void test_get_power_consumption()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x71;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint16_t power_consumption;
    bool result = bus.getPowerConsumption(actuator, power_consumption);
    TEST_ASSERT_FALSE(result);
}

void test_reset()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x76;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    // no need to mock response, just check that the command is sent. Device does not respond.
    bool result = bus.reset(actuator);
    TEST_ASSERT_FALSE(result);
}

void test_release_brake()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x77;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    bool result = bus.releaseBrake(actuator);
    TEST_ASSERT_FALSE(result);
}

void test_lock_brake()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0x78;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    bool result = bus.lockBrake(actuator);
    TEST_ASSERT_FALSE(result);
}

void test_get_uptime()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0xB1;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint32_t uptime;
    bool result = bus.getUptime(actuator, uptime);
    TEST_ASSERT_FALSE(result);
}

void test_get_firmware_version_date()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0xB2;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint32_t date;
    bool result = bus.getFirmwareVersionDate(actuator, date);
    TEST_ASSERT_FALSE(result);
}

void test_enable_watchdog()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0xB3;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint32_t mock_duration_ms = 0x7890;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(PacketPadding(3), mock_duration_ms);

    serial.prepareActuatorResponse(mockResponse);

    uint32_t duration_ms = mock_duration_ms;
    bool result = bus.enableWatchdog(actuator, duration_ms);
    TEST_ASSERT_TRUE(result);
}

void test_enable_watchdog_mismatch_failure()
{
    Actuator actuator(0x4, ActuatorModel::RMD_X8, Protocol::V2);
    uint8_t command = 0xB3;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint32_t mock_duration_ms = 0x4531;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(PacketPadding(3), mock_duration_ms);

    serial.prepareActuatorResponse(mockResponse);

    uint32_t duration_ms = 0x7890;
    bool result = bus.enableWatchdog(actuator, duration_ms);
    TEST_ASSERT_FALSE(result);
}

void test_get_motor_model()
{
    // TODO: implement once we know more about the motor model response
}

int main(int argc, char ** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_register_actuator);
    RUN_TEST(test_get_pid_parameters);
    RUN_TEST(test_write_volatile_pid_parameters_success);
    RUN_TEST(test_write_volatile_pid_parameters_mismatch_failure);
    RUN_TEST(test_write_nonvolatile_pid_parameters_success);
    RUN_TEST(test_write_nonvolatile_pid_parameters_mismatch_failure);
    RUN_TEST(test_get_acceleration);
    RUN_TEST(test_write_position_acceleration);
    RUN_TEST(test_write_position_deceleration);
    RUN_TEST(test_write_speed_acceleration);
    RUN_TEST(test_write_speed_deceleration);

#if ENCODER_INTERACTION_ENABLED
    RUN_TEST(test_get_multi_turn_encoder_position);
    RUN_TEST(test_get_multi_turn_encoder_home_position);
    RUN_TEST(test_get_multi_turn_encoder_zero_position);
    RUN_TEST(test_write_multi_turn_encoder_zero_offset);
    RUN_TEST(test_write_current_multi_turn_position_as_home);
    RUN_TEST(test_get_single_turn_encoder_position);
#endif

    RUN_TEST(test_get_multi_turn_absolute_angle);
    RUN_TEST(test_get_single_turn_angle);
    RUN_TEST(test_get_motor_voltage);
    RUN_TEST(test_get_motor_status_two);
    RUN_TEST(test_get_motor_status_three);
    RUN_TEST(test_stop);
    RUN_TEST(test_shutdown);
    RUN_TEST(test_torque_control);
    RUN_TEST(test_torque_control_with_status);
    RUN_TEST(test_speed_control);
    RUN_TEST(test_speed_control_with_status);
    RUN_TEST(test_absolute_position_control);
    RUN_TEST(test_absolute_position_control_with_status);
    RUN_TEST(test_single_turn_position_control);
    RUN_TEST(test_single_turn_position_control_with_status);
    RUN_TEST(test_incremental_position_control);
    RUN_TEST(test_get_operating_mode);
    RUN_TEST(test_get_power_consumption);
    //  RUN_TEST(test_reset);
    //  RUN_TEST(test_release_brake);
    //  RUN_TEST(test_lock_brake);
    //  RUN_TEST(test_get_uptime);
    //  RUN_TEST(test_get_firmware_version_date);
    //  RUN_TEST(test_enable_watchdog);
    //  RUN_TEST(test_enable_watchdog_mismatch_failure);
    //  RUN_TEST(test_get_error_flags);

    UNITY_END();
}
