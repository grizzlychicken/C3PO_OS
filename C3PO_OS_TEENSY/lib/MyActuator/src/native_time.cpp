
#ifdef NATIVE_BUILD

#include "native_time.h"
#include <stddef.h>
#include <stdint.h>
#include <sys/time.h>

uint32_t MyActuator::millis()
{
    struct timeval time;
    gettimeofday(&time, nullptr);
    uint32_t millis = (time.tv_sec * 1000) + (time.tv_usec / 1000);
    return millis;
}

#endif