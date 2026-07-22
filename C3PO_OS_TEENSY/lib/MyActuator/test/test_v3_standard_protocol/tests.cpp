
#include "Packet.h"
#include "PacketBuilder.h"
#include "crc_functions.h"
#include <MyActuator/MyActuator.h>
#include <stdint.h>
#include <unity.h>

using namespace MyActuator;

void test_read_pid_parameters()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    PacketBuilder::prepareReadPIDParameters(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x30,             // command
                                         0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_write_nonvolatile_pid_parameters()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PIDParameters parameters;
    parameters.current.kP = 85;
    parameters.current.kI = 25;
    parameters.velocity.kP = 85;
    parameters.velocity.kI = 25;
    parameters.position.kP = 85;
    parameters.position.kI = 25;

    PacketBuilder::prepareWriteNonVolatilePIDParameters(packet, actuator, parameters);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x32,             // command
                                         0x00, 0x55, 0x19, 0x55,
                                         0x19, 0x55, 0x19,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_write_volatile_pid_parameters()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PIDParameters parameters;
    parameters.current.kP = 85;
    parameters.current.kI = 25;
    parameters.velocity.kP = 85;
    parameters.velocity.kI = 25;
    parameters.position.kP = 85;
    parameters.position.kI = 25;

    PacketBuilder::prepareWriteVolatilePIDParameters(packet, actuator, parameters);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x31,             // command
                                         0x00, 0x55, 0x19, 0x55,
                                         0x19, 0x55, 0x19,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_read_acceleration_parameters()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    PacketBuilder::prepareReadAccelerationParameters(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x42,             // command
                                         0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_write_acceleration_parameters()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    uint32_t acceleration = 10000;
    auto functionIndex = AccelerationFunctionIndex::PositionAcceleration;
    PacketBuilder::prepareWriteAccelerationParameters(packet, actuator, functionIndex, acceleration);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x43,             // command
                                         (uint8_t)functionIndex,
                                         0x00, 0x00,
                                         (uint8_t)(acceleration >> 0),
                                         (uint8_t)(acceleration >> 8),
                                         (uint8_t)(acceleration >> 16),
                                         (uint8_t)(acceleration >> 24),
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_read_multi_turn_position()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareReadMultiTurnEncoderPosition(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x60,             // command
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_read_multi_turn_home_position()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareReadMultiTurnEncoderHomePosition(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x61,             // command
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_read_multi_turn_zero_offset()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareReadMultiTurnEncoderZeroOffset(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x62,             // command
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_write_multi_turn_zero_offset()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    int32_t offset = 10000;
    PacketBuilder::prepareWriteMultiTurnEncoderZeroOffset(packet, actuator, offset);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x63,             // command
                                         0x0, 0x0, 0x0,
                                         (uint8_t)(offset >> 0),
                                         (uint8_t)(offset >> 8),
                                         (uint8_t)(offset >> 16),
                                         (uint8_t)(offset >> 24),
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_write_current_multi_turn_position_as_home()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareWriteCurrentMultiTurnPositionAsHome(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x64,             // command
                                         0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_read_single_turn_encoder_position()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareReadSingleTurnEncoderPosition(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x90,             // command
                                         0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_read_multi_turn_absolute_angle()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareReadMultiTurnAbsoluteAngle(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x92,             // command
                                         0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_read_single_turn_angle()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareReadSingleTurnAngle(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x94,             // command
                                         0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_read_motor_status_one_and_error_flag()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareReadMotorStatusOneAndErrorFlag(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x9A,             // command
                                         0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_read_motor_status_two()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareReadMotorStatusTwo(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x9C,             // command
                                         0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_read_motor_status_three()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareReadMotorStatusThree(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x9D,             // command
                                         0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_write_motor_shutdown()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareMotorShutdown(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x80,             // command
                                         0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_write_motor_stop()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareMotorStop(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x81,             // command
                                         0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_write_torque_control()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    int16_t torque = 100;
    PacketBuilder::prepareTorqueControl(packet, actuator, torque);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0xA1,             // command
                                         0x0, 0x0, 0x0,
                                         (uint8_t)(torque >> 0),
                                         (uint8_t)(torque >> 8),
                                         0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_write_speed_control()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    Velocity speed = Velocity::degreesPerSecond(15.0);
    PacketBuilder::prepareVelocityControl(packet, actuator, speed);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0xA2,             // command
                                         0x0, 0x0, 0x0,
                                         (uint8_t)(1500 >> 0),
                                         (uint8_t)(1500 >> 8),
                                         (uint8_t)(1500 >> 16),
                                         (uint8_t)(1500 >> 24),
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_write_absolute_position_control()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    Velocity max_speed = Velocity::degreesPerSecond(15.0);
    Angle position = Angle::fromDegrees(180.0);
    PacketBuilder::prepareAbsolutePositionControl(packet, actuator, position, max_speed);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0xA4,             // command
                                         0x0,
                                         (uint8_t)(1500 >> 0),
                                         (uint8_t)(1500 >> 8),
                                         (uint8_t)(18000 >> 0),
                                         (uint8_t)(18000 >> 8),
                                         (uint8_t)(18000 >> 16),
                                         (uint8_t)(18000 >> 24),
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_write_single_turn_position_control()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    bool clockwise = true;
    Angle position = Angle::fromDegrees(180.0);
    Velocity max_speed = Velocity::degreesPerSecond(15.0);
    PacketBuilder::prepareSingleTurnPositionControl(packet, actuator, clockwise, position, max_speed);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {
        0x3E,             // header byte
        actuator.getID(), // motor id
        8,                // data length
        0xA6,             // command
        0x0,              // cw 0x0, ccw 0x01
        (uint8_t)(1500 >> 0),
        (uint8_t)(1500 >> 8),
        (uint8_t)(18000 >> 0),
        (uint8_t)(18000 >> 8),
        0x0, 0x0,
        0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_write_incremental_position_control()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    Velocity max_speed = Velocity::degreesPerSecond(15.0);
    Angle position = Angle::fromDegrees(180.0);
    PacketBuilder::prepareIncrementalPositionControl(packet, actuator, position, max_speed);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0xA8,             // command
                                         0x0,
                                         (uint8_t)(1500 >> 0),
                                         (uint8_t)(1500 >> 8),
                                         (uint8_t)(18000 >> 0),
                                         (uint8_t)(18000 >> 8),
                                         (uint8_t)(18000 >> 16),
                                         (uint8_t)(18000 >> 24),
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_read_operating_mode()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareReadOperatingMode(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x70,             // command
                                         0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_read_power_consumption()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareReadPowerConsumption(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x71,             // command
                                         0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_write_actuator_reset_command()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareResetCommand(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x76,             // command
                                         0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_write_actuator_brake_release_command()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareBrakeReleaseCommand(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x77,             // command
                                         0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_write_actuator_brake_lock_command()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareBrakeLockCommand(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0x78,             // command
                                         0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_read_uptime_command()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareReadUptimeCommand(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0xB1,             // command
                                         0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_read_version_date()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareReadFirmwareVersionDate(packet, actuator);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0xB2,             // command
                                         0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_write_baudrate()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    auto baudrate = Baudrate::Baudrate_1_000_000;
    PacketBuilder::prepareWriteBaudrateParameter(packet, actuator, baudrate);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E,             // header byte
                                         actuator.getID(), // motor id
                                         8,                // data length
                                         0xB4,             // command
                                         0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                         (uint8_t)baudrate,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_read_motor_model()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareReadMotorModel(packet, actuator);

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

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

void test_read_actuator_ids()
{
    Packet packet;

    PacketBuilder::prepareReadActuatorIDs(packet);

    const uint8_t expected_length = 13;
    uint8_t expected[expected_length] = {0x3E, // header byte
                                         0xCD, // multicast id
                                         8,    // data length
                                         0x79, // command
                                         0x0,
                                         0x1,  // read/write flag
                                         0x0, 0x0, 0x0, 0x0, 0x0,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, expected_length);
}

int main(int argc, char ** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_read_pid_parameters);
    RUN_TEST(test_write_volatile_pid_parameters);
    RUN_TEST(test_write_nonvolatile_pid_parameters);

    RUN_TEST(test_read_acceleration_parameters);
    RUN_TEST(test_write_acceleration_parameters);

    RUN_TEST(test_read_multi_turn_position);
    RUN_TEST(test_read_multi_turn_home_position);
    RUN_TEST(test_read_multi_turn_zero_offset);

    RUN_TEST(test_write_multi_turn_zero_offset);
    RUN_TEST(test_write_current_multi_turn_position_as_home);

    RUN_TEST(test_read_single_turn_encoder_position);

    RUN_TEST(test_read_multi_turn_absolute_angle);
    RUN_TEST(test_read_single_turn_angle);

    RUN_TEST(test_read_motor_status_one_and_error_flag);
    RUN_TEST(test_read_motor_status_two);
    RUN_TEST(test_read_motor_status_three);

    RUN_TEST(test_write_motor_shutdown);
    RUN_TEST(test_write_motor_stop);
    RUN_TEST(test_write_torque_control);
    RUN_TEST(test_write_speed_control);
    RUN_TEST(test_write_absolute_position_control);
    RUN_TEST(test_write_single_turn_position_control);
    RUN_TEST(test_write_incremental_position_control);

    RUN_TEST(test_read_operating_mode);
    RUN_TEST(test_read_power_consumption);
    RUN_TEST(test_write_actuator_reset_command);
    RUN_TEST(test_write_actuator_brake_release_command);
    RUN_TEST(test_write_actuator_brake_lock_command);

    RUN_TEST(test_read_uptime_command);
    RUN_TEST(test_read_version_date);
    RUN_TEST(test_write_baudrate);
    RUN_TEST(test_read_motor_model);

    RUN_TEST(test_read_actuator_ids);

    UNITY_END();
}