#include "PacketParameter.h"
#include <stdint.h>
#include <unity.h>

using namespace MyActuator;

void test_decode_float()
{
    float value = 500.0;
    uint8_t buffer[4] = {0, 0, 0, 0};
    PacketParameter::encode(value, buffer);
    float decoded_value = 0.0;
    PacketParameter::decode(decoded_value, buffer);

    TEST_ASSERT_EQUAL_FLOAT(value, decoded_value);
}

void test_decode_int32_t()
{
    int32_t value = 500;
    uint8_t buffer[4] = {0, 0, 0, 0};
    PacketParameter::encode(value, buffer);
    int32_t decoded_value = 0;
    PacketParameter::decode(decoded_value, buffer);

    TEST_ASSERT_EQUAL_INT32(value, decoded_value);
}

void test_decode_uint32_t()
{
    uint32_t value = 500;
    uint8_t buffer[4] = {0, 0, 0, 0};
    PacketParameter::encode(value, buffer);
    uint32_t decoded_value = 0;
    PacketParameter::decode(decoded_value, buffer);

    TEST_ASSERT_EQUAL_UINT32(value, decoded_value);
}

void test_decode_int64_t()
{
    int64_t value = 500;
    uint8_t buffer[8] = {};
    PacketParameter::encode(value, buffer);
    int64_t decoded_value = 0;
    PacketParameter::decode(decoded_value, buffer);

    TEST_ASSERT_EQUAL_INT32(value, decoded_value);
}

void test_decode_uint64_t()
{
    uint64_t value = 500;
    uint8_t buffer[8] = {};
    PacketParameter::encode(value, buffer);
    uint64_t decoded_value = 0;
    PacketParameter::decode(decoded_value, buffer);

    TEST_ASSERT_EQUAL_UINT32(value, decoded_value);
}

void test_decode_uint16_t()
{
    uint16_t value = 500;
    uint8_t buffer[4] = {0, 0, 0, 0};
    PacketParameter::encode(value, buffer);
    uint16_t decoded_value = 0;
    PacketParameter::decode(decoded_value, buffer);

    TEST_ASSERT_EQUAL_UINT16(value, decoded_value);
}

void test_decode_int16_t()
{
    int16_t value = 500;
    uint8_t buffer[4] = {0, 0, 0, 0};
    PacketParameter::encode(value, buffer);
    int16_t decoded_value = 0;
    PacketParameter::decode(decoded_value, buffer);

    TEST_ASSERT_EQUAL_INT16(value, decoded_value);
}

void test_decode_int8_t()
{
    int8_t value = 127;
    uint8_t buffer[4] = {0, 0, 0, 0};
    PacketParameter::encode(value, buffer);
    int8_t decoded_value = 0;
    PacketParameter::decode(decoded_value, buffer);

    TEST_ASSERT_EQUAL_INT8(value, decoded_value);
}

void test_decode_uint8_t()
{
    uint8_t value = 255;
    uint8_t buffer[4] = {0, 0, 0, 0};
    PacketParameter::encode(value, buffer);
    uint8_t decoded_value = 0;
    PacketParameter::decode(decoded_value, buffer);

    TEST_ASSERT_EQUAL_UINT8(value, decoded_value);
}

void test_decode_bool()
{
    bool value = true;
    uint8_t buffer[4] = {0, 0, 0, 0};
    PacketParameter::encode(value, buffer);
    bool decoded_value = false;
    PacketParameter::decode(decoded_value, buffer);

    TEST_ASSERT_EQUAL_UINT8((uint8_t)value, (uint8_t)decoded_value);
}

void test_decode_Baudrate()
{
    Baudrate value = Baudrate::Baudrate_1_000_000;
    uint8_t buffer[4] = {0, 0, 0, 0};
    PacketParameter::encode(value, buffer);
    Baudrate decoded_value = Baudrate::Baudrate_500_000;
    PacketParameter::decode(decoded_value, buffer);

    TEST_ASSERT_EQUAL_UINT8((uint8_t)value, (uint8_t)decoded_value);
}

void test_decode_AccelerationFunctionIndex()
{
    AccelerationFunctionIndex value = AccelerationFunctionIndex::PositionAcceleration;
    uint8_t buffer[4] = {0, 0, 0, 0};
    PacketParameter::encode(value, buffer);
    AccelerationFunctionIndex decoded_value = AccelerationFunctionIndex::PositionDeceleration;
    PacketParameter::decode(decoded_value, buffer);

    TEST_ASSERT_EQUAL_UINT8((uint8_t)value, (uint8_t)decoded_value);
}


int main(int argc, char ** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_decode_float);
    RUN_TEST(test_decode_uint64_t);
    RUN_TEST(test_decode_int64_t);
    RUN_TEST(test_decode_int32_t);
    RUN_TEST(test_decode_uint32_t);
    RUN_TEST(test_decode_uint16_t);
    RUN_TEST(test_decode_int16_t);
    RUN_TEST(test_decode_int8_t);
    RUN_TEST(test_decode_uint8_t);
    RUN_TEST(test_decode_bool);
    RUN_TEST(test_decode_Baudrate);
    RUN_TEST(test_decode_AccelerationFunctionIndex);

    UNITY_END();
}
