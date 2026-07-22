#ifndef PORTHANDLER_H
#define PORTHANDLER_H

#include <stddef.h>
#include <stdint.h>

namespace MyActuator {
    class PortHandler {
        public:
        PortHandler();

        virtual void begin(uint32_t baud) = 0;
        virtual void end() = 0;
        virtual size_t available() = 0;
        virtual int16_t read() = 0;
        virtual size_t write(uint8_t) = 0;
        virtual size_t write(uint8_t * buf, size_t len) = 0;
        bool isPortOpen();
        void setPortOpen(bool);

        private:
        bool is_port_open;
    };
};

#endif /* PORTHANDLER_H */
