#ifndef TIMEOUT_H
#define TIMEOUT_H

#include <stdint.h>

#ifdef ARDUINO
#include <Arduino.h>
#else
#include "native_time.h"
#endif // !ARDUINO

namespace MyActuator {
    class Timeout {
        private:
        uint32_t start_time_ms;
        uint32_t timeout_ms;

        public:
        Timeout(uint32_t timeout_ms)
        {
            this->timeout_ms = timeout_ms;
            this->start_time_ms = millis();
        }

        Timeout(const Timeout & other)
        {
            this->timeout_ms = other.timeout_ms;
            this->start_time_ms = other.start_time_ms;
        }

        bool isExpired()
        {
            return (millis() - start_time_ms) > timeout_ms;
        }
    };
};
#endif /* TIMEOUT_H */
