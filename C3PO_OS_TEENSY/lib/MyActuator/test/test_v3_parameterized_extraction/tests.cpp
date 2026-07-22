
#include "PacketBuilder.h"
#include <MyActuator/MyActuator.h>
#include <stdint.h>
#include <unity.h>

using namespace MyActuator;

void test_write_volatile_pid_parameters_extraction()
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

    PIDParameters actual_parameters;
    packet.extract(PacketPadding(1), actual_parameters.current, actual_parameters.velocity, actual_parameters.position);

    TEST_ASSERT_TRUE(parameters == actual_parameters);
}

void test_write_acceleration_parameters_extraction()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    uint32_t acceleration = 10000;
    AccelerationFunctionIndex functionIndex = AccelerationFunctionIndex::PositionAcceleration;
    PacketBuilder::prepareWriteAccelerationParameters(packet, actuator, functionIndex, acceleration);

    AccelerationFunctionIndex actual_functionIndex = AccelerationFunctionIndex::SpeedAcceleration;
    uint32_t actual_acceleration = 0;
    uint8_t spacer = 0;
    packet.extract(actual_functionIndex, PacketPadding(2), actual_acceleration);

    TEST_ASSERT_EQUAL_UINT8((uint8_t)functionIndex, (uint8_t)actual_functionIndex);
    TEST_ASSERT_EQUAL_UINT32(acceleration, actual_acceleration);
}

void test_write_multi_turn_zero_offset_extraction()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    int32_t offset = 10000;
    PacketBuilder::prepareWriteMultiTurnEncoderZeroOffset(packet, actuator, offset);

    int32_t actual_offset = 0;

    packet.extract(PacketPadding(3), actual_offset);

    TEST_ASSERT_EQUAL_UINT32(offset, actual_offset);
}

void test_write_absolute_position_control_extraction()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareAbsolutePositionControl(packet, actuator, Angle::fromDegrees(180.0), Velocity::degreesPerSecond(15.0));

    uint16_t actual_max_speed;
    uint32_t actual_position;

    packet.extract(PacketPadding(1), actual_max_speed, actual_position);

    uint16_t expected_max_speed = 1500;
    uint32_t expected_position = 18000;

    TEST_ASSERT_EQUAL_UINT16(expected_max_speed, actual_max_speed);
    TEST_ASSERT_EQUAL_UINT32(expected_position, actual_position);
}

void test_write_single_turn_position_control_extraction()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    bool clockwise = true;
    PacketBuilder::prepareSingleTurnPositionControl(packet, actuator, clockwise, Angle::fromDegrees(180.0), Velocity::degreesPerSecond(15.0));

    uint8_t actual_clockwise = 0xFF;
    uint16_t actual_position = 0;
    uint16_t actual_max_speed = 0;

    packet.extract(actual_clockwise, actual_max_speed, actual_position);

    uint8_t expected_clockwise = 0x0;
    uint16_t expected_position = 18000;
    uint16_t expected_max_speed = 1500;

    TEST_ASSERT_EQUAL_UINT8(expected_clockwise, actual_clockwise);
    TEST_ASSERT_EQUAL_UINT16(expected_position, actual_position);
    TEST_ASSERT_EQUAL_UINT16(expected_max_speed, actual_max_speed);
}

void test_write_incremental_position_control_extraction()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    PacketBuilder::prepareIncrementalPositionControl(packet, actuator, Angle::fromDegrees(180.0), Velocity::degreesPerSecond(15.0));

    uint16_t actual_max_speed = 0;
    int32_t actual_position = 0;

    packet.extract(PacketPadding(1), actual_max_speed, actual_position);

    uint16_t expected_max_speed = 1500;
    int32_t expected_position = 18000;

    TEST_ASSERT_EQUAL_UINT16(expected_max_speed, actual_max_speed);
    TEST_ASSERT_EQUAL_INT32(expected_position, actual_position);
}

void test_write_baudrate_extraction()
{
    Packet packet;
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);

    Baudrate baudrate = Baudrate::Baudrate_1_000_000;
    PacketBuilder::prepareWriteBaudrateParameter(packet, actuator, baudrate);

    Baudrate actual_baudrate = Baudrate::Baudrate_500_000; // random value that should change after extraction

    packet.extract(PacketPadding(6), actual_baudrate);

    TEST_ASSERT_EQUAL_UINT8(baudrate, actual_baudrate);
}

int main(int argc, char ** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_write_baudrate_extraction);
    RUN_TEST(test_write_multi_turn_zero_offset_extraction);
    RUN_TEST(test_write_absolute_position_control_extraction);
    RUN_TEST(test_write_single_turn_position_control_extraction);
    RUN_TEST(test_write_incremental_position_control_extraction);
    RUN_TEST(test_write_volatile_pid_parameters_extraction);
    RUN_TEST(test_write_acceleration_parameters_extraction);

    UNITY_END();
}