#include <MyActuator/MyActuator.h>
#include <stdint.h>
#include <unity.h>

using namespace MyActuator;

void test_angle_conversion()
{
    float degrees = 180.0f;
    auto angle_deg = Angle::fromDegrees(degrees);

    TEST_ASSERT_EQUAL_FLOAT((float)M_PI, angle_deg.toRadians());
    TEST_ASSERT_EQUAL_FLOAT(degrees, angle_deg.toDegrees());

    float radians = M_PI;
    auto angle_rad = Angle::fromRadians(radians);

    TEST_ASSERT_EQUAL_FLOAT(180.0f, angle_rad.toDegrees());
    TEST_ASSERT_EQUAL_FLOAT(radians, angle_rad.toRadians());
}

int main(int argc, char ** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_angle_conversion);

    UNITY_END();
}
