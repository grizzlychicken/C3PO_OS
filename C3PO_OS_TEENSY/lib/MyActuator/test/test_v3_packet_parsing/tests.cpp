#include "Packet.h"
#include "PacketParserV3.h"
#include "crc_functions.h"
#include <MyActuator/MyActuator.h>
#include <stdint.h>
#include <unity.h>

using namespace MyActuator;

void test_parse_response()
{
    const uint8_t expected_length = 13;
    uint8_t mock_packet[expected_length] = {
        0x3E,                                  // head
        0x01,                                  // motor id
        0x08,                                  // data length
        0x77,                                  // cmd
        0x00, 0x00, 0x00, 0x00, 0x0, 0x0, 0x0, // parameters
        0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, mock_packet, expected_length - 2);
    mock_packet[expected_length - 2] = (uint8_t)(crc_total >> 0);
    mock_packet[expected_length - 1] = (uint8_t)(crc_total >> 8);

    PacketParserV3 parser;
    for (uint8_t i = 0; i < 12; i++) {
        TEST_ASSERT_FALSE(parser.consume(mock_packet[i]));
    }
    TEST_ASSERT_TRUE(parser.consume(mock_packet[12]));
    Packet packet;
    TEST_ASSERT_TRUE(parser.apply(packet));
    TEST_ASSERT_EQUAL(0x01, packet.motor_id);
    TEST_ASSERT_EQUAL(0x77, packet.command_id);
}

void test_write_pid_rom_response_parsing()
{
    const uint8_t expected_length = 13;
    uint8_t mock_packet[expected_length] = {0x3E, // header byte
                                            0x01, // motor id
                                            8,    // data length
                                            0x32, // command
                                            0x00, // padding
                                            0x55, 0x19,
                                            0x55, 0x19,
                                            0x55, 0x19,
                                            0x0, 0x0};

    uint16_t crc_total = crc_sum_v3_begin();
    crc_sum_v3_append(crc_total, mock_packet, expected_length - 2);
    mock_packet[expected_length - 2] = (uint8_t)(crc_total >> 0);
    mock_packet[expected_length - 1] = (uint8_t)(crc_total >> 8);

    uint8_t last_index = expected_length - 1;

    PacketParserV3 parser;
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

    packet.extract(PacketPadding(1), actual_current_kP, actual_current_kI, actual_speed_kP, actual_speed_kI, actual_position_kP, actual_position_kI);

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
    RUN_TEST(test_parse_response);
    RUN_TEST(test_write_pid_rom_response_parsing);
    UNITY_END();
}
