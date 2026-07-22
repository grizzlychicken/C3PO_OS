
#include "HardwareSerialPortHandler.h"

using namespace MyActuator;

void HardwareSerialPortHandler::begin(uint32_t baudrate)
{
#if defined(ARDUINO)
    baud_ = baudrate;
    port_.begin(baud_, SERIAL_8N1);//, rxPin, txPin);

    if (dir_pin_ != -1) {
        pinMode(dir_pin_, OUTPUT);
        digitalWrite(dir_pin_, LOW);
        while (digitalRead(dir_pin_) != LOW) {
            // Wait for the pin to go low
        }
    }

    setPortOpen(true);
#endif
}

void HardwareSerialPortHandler::end(void)
{
#if defined(ARDUINO)
    port_.end();
    setPortOpen(false);
#endif
}

size_t HardwareSerialPortHandler::available()
{
#if defined(ARDUINO)
    return port_.available();
#else
    return 0;
#endif
}

int16_t HardwareSerialPortHandler::read()
{
#if defined(ARDUINO)
    return port_.read();
#else
    return -1;
#endif
}

size_t HardwareSerialPortHandler::write(uint8_t c)
{
#if defined(ARDUINO)
    size_t ret = 0;
    if (dir_pin_ != -1) {
        digitalWrite(dir_pin_, HIGH);
        while (digitalRead(dir_pin_) != HIGH) {
            // Wait for the pin to go high
        }
    }

    ret = port_.write(c);

    if (dir_pin_ != -1) {
        port_.flush();
        digitalWrite(dir_pin_, LOW);
        while (digitalRead(dir_pin_) != LOW) {
            // Wait for the pin to go low
        }
    }

    return ret;
#else
    return 0;
#endif
}

size_t HardwareSerialPortHandler::write(uint8_t * buf, size_t len)
{
#if defined(ARDUINO)
    size_t ret;
    if (dir_pin_ != -1) {
        digitalWrite(dir_pin_, HIGH);
        while (digitalRead(dir_pin_) != HIGH) {
            // Wait for the pin to go high
        }
    }

    ret = port_.write(buf, len);

    if (dir_pin_ != -1) {
        port_.flush();
        digitalWrite(dir_pin_, LOW);
        while (digitalRead(dir_pin_) != LOW) {
            // Wait for the pin to go low
        }
    }

    return ret;
#else
    return 0;
#endif
}

uint32_t HardwareSerialPortHandler::getBaud() const
{
    return baud_;
}
