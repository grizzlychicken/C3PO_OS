
#ifndef NATIVETIME_H
#define NATIVETIME_H

#include <stdint.h>

#ifdef NATIVE_BUILD

namespace MyActuator {
    uint32_t millis();
}

#endif

#endif /* NATIVETIME_H */
