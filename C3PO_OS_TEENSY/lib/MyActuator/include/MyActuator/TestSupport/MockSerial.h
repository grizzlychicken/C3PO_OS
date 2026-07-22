#ifndef MOCKSERIAL_H
#define MOCKSERIAL_H
#include "../PortHandler.h"
#include "CircularBuffer.h"
#include <stddef.h>
#include <stdint.h>

namespace MyActuator {
    class Packet;
    namespace TestSupport {
        class MockSerial : public MyActuator::PortHandler {
            public:
            CircularBuffer<uint8_t, 1024> inputBuffer;  // Buffer that holds the data that will be read from the serial port
            CircularBuffer<uint8_t, 1024> outputBuffer; // Buffer that holds the data that will be written to the serial port

            virtual void begin(uint32_t baud) override;
            virtual void end() override;
            virtual size_t available(void) override;
            virtual int16_t read() override;
            virtual size_t write(uint8_t) override;
            virtual size_t write(uint8_t * buf, size_t len) override;

            virtual uint32_t getBaud() const;

            void prepareActuatorResponse(Packet & packet);

            MockSerial()
                : MyActuator::PortHandler()
            {
            }

            ~MockSerial()
            {
            }
        };
    };
};
#endif /* MOCKSERIAL_H */
