
#include "Packet.h"
#include "PacketBuilder.h"
#include "crc_functions.h"
#include <MyActuator/MyActuator.h>
#include <stdint.h>
#include <unity.h>

using namespace MyActuator;

void test_motion_packet()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    Packet packet;
    PacketBuilder::prepareMotionControl(packet, actuator, Angle::fromDegrees(90), 0.4f, 0.5f, 0.4f, 0.0f);

    const uint8_t expected_length = 26;
    uint8_t expected[expected_length] = {0x5A, 0xA5, 0xC3,
                                         20,
                                         219, 15, 201, 63,
                                         0xCD, 0xCC, 0xCC, 0x3E,
                                         0x00, 0x00, 0x00, 0x3F,
                                         0xCD, 0xCC, 0xCC, 0x3E,
                                         0x00, 0x00, 0x00, 0x00,
                                         0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, expected, expected_length - 2);
    expected[expected_length - 2] = (uint8_t)(crc_total >> 0);
    expected[expected_length - 1] = (uint8_t)(crc_total >> 8);

    TEST_ASSERT_EQUAL_UINT8(expected_length, packet.packet_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, packet.packet_data, packet.packet_length);
}

void test_motion_packet_extraction()
{
    Actuator actuator(0x4, ActuatorModel::X4, Protocol::V3);
    Packet packet;

    float speed = 0.4f;
    float torque = 0.5f;
    float kP = 0.4f;
    float kD = 0.0f;

    PacketBuilder::prepareMotionControl(packet, actuator, Angle::fromDegrees(90), speed, torque, kP, kD);

    float actual_position = 0.0f;
    float actual_speed = 0.0f;
    float actual_torque = 0.0f;
    float actual_kP = 0.0f;
    float actual_kD = 0.0f;

    packet.extract(actual_position, actual_speed, actual_torque, actual_kP, actual_kD);

    TEST_ASSERT_EQUAL_FLOAT(Angle::fromDegrees(90).toRadians(), actual_position);
    TEST_ASSERT_EQUAL_FLOAT(speed, actual_speed);
    TEST_ASSERT_EQUAL_FLOAT(torque, actual_torque);
    TEST_ASSERT_EQUAL_FLOAT(kP, actual_kP);
    TEST_ASSERT_EQUAL_FLOAT(kD, actual_kD);
}

int main(int argc, char ** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_motion_packet);
    RUN_TEST(test_motion_packet_extraction);
    UNITY_END();
}