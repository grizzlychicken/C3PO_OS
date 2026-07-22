#ifndef HARDWARESERIALPORTHANDLER_H
#define HARDWARESERIALPORTHANDLER_H

#include <MyActuator/PortHandler.h>
#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <cstddef>
#include <cstdint>
#endif

namespace MyActuator {

    class HardwareSerialPortHandler : public PortHandler {
        public:
#if defined(ESP32)
        HardwareSerialPortHandler(HardwareSerial & port, const int8_t rxPin = -1, const int8_t txPin = -1, const int8_t dir_pin = -1)
            : MyActuator::PortHandler()
            , port_(port)
            , dir_pin_(dir_pin)
            , baud_(115200)
        {
            this->rxPin = rxPin;
            this->txPin = txPin;
        }
#elif defined(ARDUINO)
        HardwareSerialPortHandler(HardwareSerial & port, const int8_t dir_pin = -1)
            : MyActuator::PortHandler()
            , port_(port)
            , dir_pin_(dir_pin)
            , baud_(115200)
        {
        }
#else
        HardwareSerialPortHandler()
            : MyActuator::PortHandler()
            , dir_pin_(-1) {
                  // Shouldn't be called
              };
#endif

        virtual void begin(uint32_t baudrate) override;
        virtual void end() override;

        virtual size_t available() override;
        virtual int16_t read() override;
        virtual size_t write(uint8_t) override;
        virtual size_t write(uint8_t * buf, size_t len) override;

        virtual uint32_t getBaud() const;

        private:
        const int8_t dir_pin_;
        uint32_t baud_;

#if defined(ARDUINO)
        HardwareSerial & port_;
#endif

#if defined(ESP32)
        int8_t rxPin = -1;
        int8_t txPin = -1;
#endif
    };
};

#endif /* HARDWARESERIALPORTHANDLER_H */
