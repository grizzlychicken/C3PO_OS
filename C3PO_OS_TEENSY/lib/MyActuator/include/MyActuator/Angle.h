#ifndef INCLUDE_MYACTUATOR_ANGLE
#define INCLUDE_MYACTUATOR_ANGLE
#include <math.h>
#include <stddef.h>
#include <stdint.h>

namespace MyActuator {

    struct Angle {
        private:
        float rads;
        Angle(float radians)
        {
            this->rads = radians;
        }

        public:
        Angle()
        {
            this->rads = 0.0f;
        }

        float toRadians()
        {
            return rads;
        }

        float toDegrees()
        {
            return rads * 180.0f / (float)M_PI;
        }

        static Angle fromRadians(float radians)
        {
            return Angle(radians);
        }

        static Angle fromDegrees(float degrees)
        {
            return Angle(degrees * (float)M_PI / 180.0f);
        }

        inline bool operator==(const Angle & rhs)
        {
            return rads == rhs.rads;
        }
    };
};

#endif // INCLUDE_MYACTUATOR_ANGLE
