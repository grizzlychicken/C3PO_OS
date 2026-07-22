#ifndef INCLUDE_MYACTUATOR_ANGLEVELOCITY
#define INCLUDE_MYACTUATOR_ANGLEVELOCITY

#include <math.h>
#include <stddef.h>
#include <stdint.h>

namespace MyActuator {
    struct Velocity {
        private:
        float radsPerSec;
        Velocity(float radiansPerSecond)
        {
            this->radsPerSec = radiansPerSecond;
        }

        public:
        Velocity()
        {
            this->radsPerSec = 0;
        }

        float radiansPerSecond()
        {
            return radsPerSec;
        }

        float degreesPerSecond()
        {
            return radsPerSec * 180.0f / (float)M_PI;
        }

        static Velocity radiansPerSecond(float radiansPerSecond)
        {
            return Velocity(radiansPerSecond);
        }

        static Velocity degreesPerSecond(float degreesPerSecond)
        {
            return Velocity(degreesPerSecond * (float)M_PI / 180.0f);
        }

        inline bool operator==(const Velocity & rhs)
        {
            return radsPerSec == rhs.radsPerSec;
        }
    };
};
#endif // INCLUDE_MYACTUATOR_ANGLEVELOCITY
