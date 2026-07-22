#include "PacketParameter.h"
#include <stdint.h>
#include <unity.h>

using namespace MyActuator;

void test_encode_float()
{
    float value = 500.0;
    uint8_t buffer[4] = {0, 0, 0, 0};
    PacketParameter::encode(value, buffer);

    union {
        float value;
        uint8_t bytes[4];
    } converter;

    converter.value = value;
    TEST_ASSERT_EQUAL_UINT8_ARRAY(converter.bytes, buffer, 4);
}

void test_encode_int32_t()
{
    int32_t value = 500;
    uint8_t buffer[4] = {0, 0, 0, 0};
    PacketParameter::encode(value, buffer);

    uint8_t expected[4] = {(uint8_t)(value >> 0), (uint8_t)(value >> 8), (uint8_t)(value >> 16), (uint8_t)(value >> 24)};

    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, buffer, 4);
}

void test_encode_uint32_t()
{
    uint32_t value = 500;
    uint8_t buffer[4] = {0, 0, 0, 0};
    PacketParameter::encode(value, buffer);

    uint8_t expected[4] = {(uint8_t)(value >> 0), (uint8_t)(value >> 8), (uint8_t)(value >> 16), (uint8_t)(value >> 24)};

    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, buffer, 4);
}

void test_encode_int64_t()
{
    int64_t value = 500;
    uint8_t buffer[8] = {};
    PacketParameter::encode(value, buffer);

    uint8_t expected[8] = {(uint8_t)(value >> 0), (uint8_t)(value >> 8), (uint8_t)(value >> 16), (uint8_t)(value >> 24), (uint8_t)(value >> 32), (uint8_t)(value >> 40), (uint8_t)(value >> 48), (uint8_t)(value >> 56)};

    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, buffer, 8);
}

void test_encode_uint64_t()
{
    uint64_t value = 500;
    uint8_t buffer[8] = {};
    PacketParameter::encode(value, buffer);

    uint8_t expected[8] = {(uint8_t)(value >> 0), (uint8_t)(value >> 8), (uint8_t)(value >> 16), (uint8_t)(value >> 24), (uint8_t)(value >> 32), (uint8_t)(value >> 40), (uint8_t)(value >> 48), (uint8_t)(value >> 56)};

    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, buffer, 8);
}

void test_encode_uint16_t()
{
    uint16_t value = 500;
    uint8_t buffer[2] = {0, 0};
    PacketParameter::encode(value, buffer);

    uint8_t expected[2] = {(uint8_t)(value >> 0), (uint8_t)(value >> 8)};

    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, buffer, 2);
}

void test_encode_int16_t()
{
    int16_t value = 500;
    uint8_t buffer[2] = {0, 0};
    PacketParameter::encode(value, buffer);

    uint8_t expected[2] = {(uint8_t)(value >> 0), (uint8_t)(value >> 8)};

    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, buffer, 2);
}

void test_encode_int8_t()
{
    int8_t value = 127;
    uint8_t buffer = 0;
    PacketParameter::encode(value, &buffer);

    uint8_t expected = (uint8_t)value;
    TEST_ASSERT_EQUAL_UINT8(expected, buffer);
}

void test_encode_uint8_t()
{
    uint8_t value = 255;
    uint8_t buffer = 0;
    PacketParameter::encode(value, &buffer);

    uint8_t expected = (uint8_t)value;
    TEST_ASSERT_EQUAL_UINT8(expected, buffer);
}

void test_encode_bool()
{
    bool value = true;
    uint8_t buffer = 0;
    PacketParameter::encode(value, &buffer);

    uint8_t expected = (uint8_t)value;
    TEST_ASSERT_EQUAL_UINT8(expected, buffer);
}

void test_encode_Baudrate()
{
    Baudrate value = Baudrate::Baudrate_1_000_000;
    uint8_t buffer = 0;
    PacketParameter::encode(value, &buffer);

    uint8_t expected = (uint8_t)value;
    TEST_ASSERT_EQUAL_UINT8(expected, buffer);
}


void test_encode_AccelerationFunctionIndex()
{
    AccelerationFunctionIndex value = AccelerationFunctionIndex::PositionDeceleration;
    uint8_t buffer = 0;
    PacketParameter::encode(value, &buffer);

    uint8_t expected = (uint8_t)value;
    TEST_ASSERT_EQUAL_UINT8(expected, buffer);
}


int main(int argc, char ** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_encode_float);
    RUN_TEST(test_encode_int64_t);
    RUN_TEST(test_encode_uint64_t);
    RUN_TEST(test_encode_int32_t);
    RUN_TEST(test_encode_uint32_t);
    RUN_TEST(test_encode_uint16_t);
    RUN_TEST(test_encode_int16_t);
    RUN_TEST(test_encode_int8_t);
    RUN_TEST(test_encode_uint8_t);
    RUN_TEST(test_encode_bool);
    RUN_TEST(test_encode_Baudrate);
    RUN_TEST(test_encode_AccelerationFunctionIndex);

    UNITY_END();
}
