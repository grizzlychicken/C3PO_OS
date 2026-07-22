
#include "Packet.h"
#include "PacketBuilder.h"
#include "crc_functions.h"
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
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);

    // Fill the buffer with the response since the bus command call will block
    Packet deviceResponse;
    PacketBuilder::prepareReadMotorModel(deviceResponse, actuator);
    serial.prepareActuatorResponse(deviceResponse);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0xB5,             // command
                                         0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    // Verify data is gety before making call
    TEST_ASSERT_EQUAL_UINT8(expected_length, serial.inputBuffer.size());
    for (size_t i = 0; i < expected_length; i++) {
        TEST_ASSERT_EQUAL_UINT8(expected[i], serial.inputBuffer[i]);
    }

    TEST_ASSERT_TRUE(bus.add(actuator));

    // Check that the output buffer looks correct
    TEST_ASSERT_EQUAL_UINT8(expected_length, serial.outputBuffer.size());
    for (size_t i = 0; i < expected_length; i++) {
        TEST_ASSERT_EQUAL_UINT8(expected[i], serial.outputBuffer[i]);
    }
}

void test_set_motion_control_parameters()
{
    // TODO: implement
}

void test_get_pid_parameters()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
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
    mockResponse.addParameters(PacketPadding(1), mockGains.current, mockGains.velocity, mockGains.position);

    // Fill the buffer with the response since the bus command call will block
    serial.prepareActuatorResponse(mockResponse);

    PIDParameters actualGains;

    bool result = bus.getPIDParameters(actuator, actualGains);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(mockGains == actualGains);
}

void test_write_volatile_pid_parameters_success()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
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
    mockResponse.addParameters(PacketPadding(1), mockGains.current, mockGains.velocity, mockGains.position);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setPIDParameters(actuator, mockGains, false);
    TEST_ASSERT_TRUE(result);
}

void test_write_volatile_pid_parameters_mismatch_failure()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
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
    mockResponse.addParameters(PacketPadding(1), mockGains.current, mockGains.velocity, mockGains.position);

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
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
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
    mockResponse.addParameters(PacketPadding(1), mockGains.current, mockGains.velocity, mockGains.position);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setPIDParameters(actuator, mockGains, true);
    TEST_ASSERT_TRUE(result);
}

void test_write_nonvolatile_pid_parameters_mismatch_failure()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
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
    mockResponse.addParameters(PacketPadding(1), mockGains.current, mockGains.velocity, mockGains.position);

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
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x42;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    int32_t mock_acceleration = 10000;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(PacketPadding(3), mock_acceleration);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.getAcceleration(actuator, mock_acceleration);
    TEST_ASSERT_TRUE(result);
}

void test_write_position_acceleration()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x43;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    AccelerationFunctionIndex functionIndex = AccelerationFunctionIndex::PositionAcceleration;
    uint32_t mock_acceleration = 10000;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(functionIndex, PacketPadding(2), mock_acceleration);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setMaxAcceleration(actuator, mock_acceleration, OperatingMode::Position);
    TEST_ASSERT_TRUE(result);
}

void test_write_position_deceleration()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x43;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    AccelerationFunctionIndex functionIndex = AccelerationFunctionIndex::PositionDeceleration;
    uint32_t mock_deceleration = 10000;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(functionIndex, PacketPadding(2), mock_deceleration);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setMaxDeceleration(actuator, mock_deceleration, OperatingMode::Position);
    TEST_ASSERT_TRUE(result);
}

void test_write_speed_acceleration()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x43;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    AccelerationFunctionIndex functionIndex = AccelerationFunctionIndex::SpeedAcceleration;
    uint32_t mock_acceleration = 10000;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(functionIndex, PacketPadding(2), mock_acceleration);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setMaxAcceleration(actuator, mock_acceleration, OperatingMode::Velocity);
    TEST_ASSERT_TRUE(result);
}

void test_write_speed_deceleration()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x43;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    AccelerationFunctionIndex functionIndex = AccelerationFunctionIndex::SpeedDeceleration;
    uint32_t mock_deceleration = 10000;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(functionIndex, PacketPadding(2), mock_deceleration);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setMaxDeceleration(actuator, mock_deceleration, OperatingMode::Velocity);
    TEST_ASSERT_TRUE(result);
}

void test_get_multi_turn_encoder_position()
{
#if ENCODER_INTERACTION_ENABLED
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x60;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    int32_t mock_position = 12345;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(PacketPadding(3), mock_position);

    serial.prepareActuatorResponse(mockResponse);

    int32_t position;
    bool result = bus.getMultiTurnEncoderPosition(actuator, position);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_INT32(mock_position, position);
#endif
}

void test_get_multi_turn_encoder_home_position()
{
#if ENCODER_INTERACTION_ENABLED
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x61;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    int32_t mock_position = 12345;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(PacketPadding(3), mock_position);

    serial.prepareActuatorResponse(mockResponse);

    int32_t position;
    bool result = bus.getMultiTurnEncoderHomePosition(actuator, position);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_INT32(mock_position, position);
#endif
}

void test_get_multi_turn_encoder_zero_position()
{
#if ENCODER_INTERACTION_ENABLED
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x62;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    int32_t mock_position = 12345;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(PacketPadding(3), mock_position);

    serial.prepareActuatorResponse(mockResponse);

    int32_t position;
    bool result = bus.getMultiTurnEncoderZeroOffset(actuator, position);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_INT32(mock_position, position);
#endif
}

void test_write_multi_turn_encoder_zero_offset()
{
#if ENCODER_INTERACTION_ENABLED
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x63;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    int32_t mock_position = 12345;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(PacketPadding(3), mock_position);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setMultiTurnEncoderZeroOffset(actuator, mock_position);
    TEST_ASSERT_TRUE(result);
#endif
}

void test_write_current_multi_turn_position_as_home()
{
#if ENCODER_INTERACTION_ENABLED
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x64;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    int32_t mock_position = 12345;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(PacketPadding(3), mock_position);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setCurrentMultiTurnPositionAsHome(actuator, mock_position);
    TEST_ASSERT_TRUE(result);
#endif
}

void test_get_single_turn_encoder_position()
{
#if ENCODER_INTERACTION_ENABLED
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x90;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    int16_t mock_position = 12345;
    int16_t mock_original_position = 12345;
    int16_t mock_zero_bias = 12345;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(PacketPadding(1), mock_position, mock_original_position, mock_zero_bias);

    serial.prepareActuatorResponse(mockResponse);

    int16_t position = 0;
    int16_t original_position = 0;
    int16_t zero_bias = 0;
    bool result = bus.getSingleTurnEncoderPosition(actuator, position, original_position, zero_bias);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_INT16(mock_position, position);
    TEST_ASSERT_EQUAL_INT16(mock_original_position, original_position);
    TEST_ASSERT_EQUAL_INT16(mock_zero_bias, zero_bias);
#endif
}

void test_get_multi_turn_absolute_angle()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x92;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    float mock_angle_deg = 600.0f;
    int32_t mock_angle = mock_angle_deg * 100.0f;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(PacketPadding(3), mock_angle);

    serial.prepareActuatorResponse(mockResponse);

    Angle angle;
    bool result = bus.getMultiTurnAbsoluteAngle(actuator, angle);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_FLOAT(mock_angle_deg, angle.toDegrees());
}

void test_get_single_turn_angle()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x94;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    float mock_angle_deg = 80.0f;
    int16_t mock_angle = mock_angle_deg * 100.0f;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(PacketPadding(5), mock_angle);

    serial.prepareActuatorResponse(mockResponse);

    Angle angle;
    bool result = bus.getSingleTurnAngle(actuator, angle);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_FLOAT(mock_angle_deg, angle.toDegrees());
}

void test_get_motor_voltage()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
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
    mockResponse.addParameters(mock_temp, PacketPadding(1), mock_brake_active, mock_voltage, mock_error_flags);

    serial.prepareActuatorResponse(mockResponse);

    float voltage;
    bool result = bus.getVoltage(actuator, voltage);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_FLOAT(23.4, voltage);
}

void test_get_error_flags()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
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
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x9C;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temp = 123;
    int16_t mock_torque = 234;
    int16_t mock_speed = 345;
    int16_t mock_angle = 45;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temp, mock_torque, mock_speed, mock_angle);

    serial.prepareActuatorResponse(mockResponse);

    ControlModeStatus status;
    bool result = bus.getControlModeStatus(actuator, status);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_INT8(mock_temp, status.temp);
    TEST_ASSERT_EQUAL_INT16(mock_torque, status.torque);
    TEST_ASSERT_EQUAL_FLOAT(345.0f, status.velocity.degreesPerSecond());
    TEST_ASSERT_EQUAL_FLOAT(45.0f, status.angle.toDegrees());
}

void test_get_motor_status_three()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
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
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
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
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
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
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
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
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0xA1;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temperature = 0x12;
    int16_t mock_torque = 0x1234;

    float mock_speed_dps = 15.0f;
    int16_t mock_speed = mock_speed_dps; // No scaling for this status response

    float mock_position_deg = 180.0f;
    int16_t mock_position = mock_position_deg; // No scaling for this status response

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temperature, mock_torque, mock_speed, mock_position);

    serial.prepareActuatorResponse(mockResponse);

    TorqueControlCommandStatus status;
    bool result = bus.setTorque(actuator, mock_torque, &status);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(mock_temperature, status.temp);
    TEST_ASSERT_EQUAL_INT16(mock_torque, status.torque);
    TEST_ASSERT_EQUAL_FLOAT(mock_speed_dps, status.speed.degreesPerSecond());
    TEST_ASSERT_EQUAL_FLOAT(mock_position_deg, status.position.toDegrees());
}

void test_speed_control()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0xA2;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temperature = 0x12;
    int16_t mock_torque = 0x1234;

    float mock_speed_dps = 15.0f;
    int16_t mock_speed = mock_speed_dps * 100.0f;

    float mock_position_deg = 180.0f;
    int16_t mock_position = mock_position_deg * 100.0f;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temperature, mock_torque, mock_speed, mock_position);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setVelocity(actuator, Velocity::degreesPerSecond(mock_speed_dps));
    TEST_ASSERT_TRUE(result);
}

void test_speed_control_with_status()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0xA2;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temperature = 0x12;
    int16_t mock_torque = 0x1234;

    float mock_speed_dps = 15.0f;
    int16_t mock_speed = mock_speed_dps; // No scaling for this status response

    float mock_position_deg = 180.0f;
    int16_t mock_position = mock_position_deg; // No scaling for this status response

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temperature, mock_torque, mock_speed, mock_position);

    serial.prepareActuatorResponse(mockResponse);

    VelocityControlCommandStatus status;
    bool result = bus.setVelocity(actuator, Velocity::degreesPerSecond(mock_speed_dps), &status);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(mock_temperature, status.temp);
    TEST_ASSERT_EQUAL_INT16(mock_torque, status.torque);
    TEST_ASSERT_EQUAL_FLOAT(mock_speed_dps, status.speed.degreesPerSecond());
    TEST_ASSERT_EQUAL_FLOAT(mock_position_deg, status.position.toDegrees());
}

void test_absolute_position_control()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0xA4;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temperature = 0x12;
    int16_t mock_torque = 0x1234;

    float mock_speed_dps = 15.0f;
    int16_t mock_speed = mock_speed_dps * 100.0f;

    float mock_position_deg = 180.0f;
    int16_t mock_position = mock_position_deg * 100.0f;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temperature, mock_torque, mock_speed, mock_position);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setMultiTurnPosition(actuator, Angle::fromDegrees(mock_position_deg), Velocity::degreesPerSecond(mock_speed_dps));
    TEST_ASSERT_TRUE(result);
}

void test_absolute_position_control_with_status()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0xA4;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temperature = 0x12;
    int16_t mock_torque = 0x1234;

    float mock_speed_dps = 15.0f;
    int16_t mock_speed = mock_speed_dps; // No scaling for this status response

    float mock_position_deg = 180.0f;
    int16_t mock_position = mock_position_deg; // No scaling for this status response

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temperature, mock_torque, mock_speed, mock_position);

    serial.prepareActuatorResponse(mockResponse);

    uint16_t max_speed = 1000;
    AbsolutePositionControlCommandStatus status;
    bool result = bus.setMultiTurnPosition(actuator, Angle::fromDegrees(mock_position_deg), Velocity::degreesPerSecond(mock_speed_dps), &status);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(mock_temperature, status.temp);
    TEST_ASSERT_EQUAL_INT16(mock_torque, status.torque);
    TEST_ASSERT_EQUAL_FLOAT(mock_speed_dps, status.speed.degreesPerSecond());
    TEST_ASSERT_EQUAL_FLOAT(mock_position_deg, status.position.toDegrees());
}

void test_single_turn_position_control()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0xA6;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temperature = 0x12;
    int16_t mock_torque = 0x1234;

    float mock_speed_dps = 15.0f;
    int16_t mock_speed = mock_speed_dps * 100.0f;

    float mock_position_deg = 180.0f;
    int16_t mock_position = mock_position_deg * 100.0f;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temperature, mock_torque, mock_speed, mock_position);

    serial.prepareActuatorResponse(mockResponse);

    bool clockwise = true;
    bool result = bus.setSingleTurnPosition(actuator, clockwise, Angle::fromDegrees(mock_position_deg), Velocity::degreesPerSecond(mock_speed_dps));
    TEST_ASSERT_TRUE(result);
}

void test_single_turn_position_control_with_status()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0xA6;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temperature = 0x12;
    int16_t mock_torque = 0x1234;

    float mock_speed_dps = 15.0f;
    int16_t mock_speed = mock_speed_dps; // No scaling for this status response

    float mock_position_deg = 180.0f;
    uint16_t mock_position = mock_position_deg; // No scaling for this status response

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temperature, mock_torque, mock_speed, mock_position);

    serial.prepareActuatorResponse(mockResponse);

    bool clockwise = true;
    SingleTurnControlCommandStatus status;
    bool result = bus.setSingleTurnPosition(actuator, clockwise, Angle::fromDegrees(mock_position_deg), Velocity::degreesPerSecond(mock_speed_dps), &status);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(mock_temperature, status.temp);
    TEST_ASSERT_EQUAL_INT16(mock_torque, status.torque);
    TEST_ASSERT_EQUAL_FLOAT(mock_speed_dps, status.speed.degreesPerSecond());
    TEST_ASSERT_EQUAL_UINT16(mock_position, status.encoderPosition);
}

void test_incremental_position_control()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0xA8;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint8_t mock_temperature = 0x12;
    int16_t mock_torque = 0x1234;
    float mock_speed_dps = 15.0f;
    int16_t mock_speed = mock_speed_dps * 100.0f;

    float mock_position_deg = 180.0f;
    int32_t mock_position = mock_position_deg * 100.0f;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(mock_temperature, mock_torque, mock_speed, mock_position);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.setIncrementalPosition(actuator, Angle::fromDegrees(mock_position_deg), Velocity::degreesPerSecond(mock_speed_dps));
    TEST_ASSERT_TRUE(result);
}

void test_get_operating_mode()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x70;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    OperatingMode mock_mode = OperatingMode::Position;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(PacketPadding(6), mock_mode);

    serial.prepareActuatorResponse(mockResponse);

    OperatingMode mode;
    bool result = bus.getOperatingMode(actuator, mode);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(mock_mode, mode);
}

void test_get_power_consumption()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x71;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint16_t mock_power_consumption = 0x01;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(PacketPadding(5), mock_power_consumption);

    serial.prepareActuatorResponse(mockResponse);

    uint16_t power_consumption;
    bool result = bus.getPowerConsumption(actuator, power_consumption);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT16(mock_power_consumption, power_consumption);
}

void test_reset()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x76;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    // no need to mock response, just check that the command is sent. Device does not respond.
    bool result = bus.reset(actuator);
    TEST_ASSERT_TRUE(result);
}

void test_release_brake()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x77;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    Packet mockResponse(actuator, command);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.releaseBrake(actuator);
    TEST_ASSERT_TRUE(result);
}

void test_lock_brake()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0x78;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    Packet mockResponse(actuator, command);

    serial.prepareActuatorResponse(mockResponse);

    bool result = bus.lockBrake(actuator);
    TEST_ASSERT_TRUE(result);
}

void test_get_uptime()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0xB1;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint32_t mock_uptime = 0x7890;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(PacketPadding(3), mock_uptime);

    serial.prepareActuatorResponse(mockResponse);

    uint32_t uptime;
    bool result = bus.getUptime(actuator, uptime);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(mock_uptime, uptime);
}

void test_get_firmware_version_date()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    uint8_t command = 0xB2;

    MockSerial serial;
    ActuatorBus bus(serial);
    bus.begin(Baudrate::Baudrate_115_200);
    mock_register_actuator(serial, bus, actuator);

    uint32_t mock_date = 0x7890;

    Packet mockResponse(actuator, command);
    mockResponse.addParameters(PacketPadding(3), mock_date);

    serial.prepareActuatorResponse(mockResponse);

    uint32_t date;
    bool result = bus.getFirmwareVersionDate(actuator, date);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(mock_date, date);
}

void test_enable_watchdog()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
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
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
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
    RUN_TEST(test_set_motion_control_parameters);
    RUN_TEST(test_write_volatile_pid_parameters_success);
    RUN_TEST(test_write_volatile_pid_parameters_mismatch_failure);
    RUN_TEST(test_write_nonvolatile_pid_parameters_success);
    RUN_TEST(test_write_nonvolatile_pid_parameters_mismatch_failure);
    RUN_TEST(test_get_acceleration);
    RUN_TEST(test_write_position_acceleration);
    RUN_TEST(test_write_position_deceleration);
    RUN_TEST(test_write_speed_acceleration);
    RUN_TEST(test_write_speed_deceleration);
    RUN_TEST(test_get_multi_turn_encoder_position);
    RUN_TEST(test_get_multi_turn_encoder_home_position);
    RUN_TEST(test_get_multi_turn_encoder_zero_position);
    RUN_TEST(test_write_multi_turn_encoder_zero_offset);
    RUN_TEST(test_write_current_multi_turn_position_as_home);
    RUN_TEST(test_get_single_turn_encoder_position);
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
    RUN_TEST(test_reset);
    RUN_TEST(test_release_brake);
    RUN_TEST(test_lock_brake);
    RUN_TEST(test_get_uptime);
    RUN_TEST(test_get_firmware_version_date);
    RUN_TEST(test_enable_watchdog);
    RUN_TEST(test_enable_watchdog_mismatch_failure);
    RUN_TEST(test_get_error_flags);

    UNITY_END();
}
