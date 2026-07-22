#include "Packet.h"
#include "PacketParserV2.h"
#include <MyActuator/MyActuator.h>
#include <stdint.h>
#include <unity.h>

using namespace MyActuator;

void test_parse_response_zero_length()
{
    const uint8_t expected_length = 5;
    uint8_t mock_packet[expected_length] = {
        0x3E, // head
        0x30, // cmd
        0x01, // motor id
        0x00, // data length
        0x6F  // header checksum
    };        // No data segment

    uint8_t last_index = expected_length - 1;

    PacketParserV2 parser;
    for (uint8_t i = 0; i < last_index; i++) {
        TEST_ASSERT_FALSE(parser.consume(mock_packet[i]));
    }
    TEST_ASSERT_TRUE(parser.consume(mock_packet[last_index]));

    Packet packet;
    TEST_ASSERT_TRUE(parser.apply(packet));
    TEST_ASSERT_EQUAL(0x01, packet.motor_id);
    TEST_ASSERT_EQUAL(0x30, packet.command_id);
}

void test_write_pid_rom_response_parsing()
{
    const uint8_t expected_length = 12;
    uint8_t mock_packet[expected_length] = {
        0x3E,          // head
        0x32,          // cmd
        0x01,          // motor id
        0x06,          // data length
        0x77,          // header checksum
        0x55, 0x19,    // angle kP, kI
        0x55, 0x19,    // speed kP, kI
        0x55, 0x19,    // torque kP, kI
        (uint8_t)0x14A // data checksum
    };

    uint8_t last_index = expected_length - 1;

    PacketParserV2 parser;
    for (uint8_t i = 0; i < last_index; i++) {
        TEST_ASSERT_FALSE(parser.consume(mock_packet[i]));
    }
    TEST_ASSERT_TRUE(parser.consume(mock_packet[last_index]));

    Packet packet;
    TEST_ASSERT_TRUE(parser.apply(packet));
    TEST_ASSERT_EQUAL(0x01, packet.motor_id);
    TEST_ASSERT_EQUAL(0x32, packet.command_id);

    uint8_t expected_current_kP = 85;
    uint8_t expected_current_kI = 25;
    uint8_t expected_speed_kP = 85;
    uint8_t expected_speed_kI = 25;
    uint8_t expected_position_kP = 85;
    uint8_t expected_position_kI = 25;

    uint8_t actual_current_kP = 0;
    uint8_t actual_current_kI = 0;
    uint8_t actual_speed_kP = 0;
    uint8_t actual_speed_kI = 0;
    uint8_t actual_position_kP = 0;
    uint8_t actual_position_kI = 0;

    packet.extract(actual_current_kP, actual_current_kI, actual_speed_kP, actual_speed_kI, actual_position_kP, actual_position_kI);

    TEST_ASSERT_EQUAL(expected_current_kP, actual_current_kP);
    TEST_ASSERT_EQUAL(expected_current_kI, actual_current_kI);
    TEST_ASSERT_EQUAL(expected_speed_kP, actual_speed_kP);
    TEST_ASSERT_EQUAL(expected_speed_kI, actual_speed_kI);
    TEST_ASSERT_EQUAL(expected_position_kP, actual_position_kP);
    TEST_ASSERT_EQUAL(expected_position_kI, actual_position_kI);
}

int main(int argc, char ** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_parse_response_zero_length);
    RUN_TEST(test_write_pid_rom_response_parsing);
    UNITY_END();
}
