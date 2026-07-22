#include <Arduino.h>

// #include <HardwareSerial.h>
#include "Definitions.h" // Include the definitions library
#include <SerialFunctions.hpp>
#include <ServoFunctions.hpp>

void setup()
{
    Serial.begin(115200);
    Serial.println("Booting ServoESP32 Torso");
    // Initialize serial communication for RS485
    RS485.begin(115200); //, SERIAL_8N1, 16, 17); // RX on GPIO16, TX on GPIO17
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH);
    digitalWrite(18, HIGH);
    // delay(200);
    RS485.println("Booting ESP32 Torso 485");

    digitalWrite(5, LOW);
    digitalWrite(18, LOW);

    SetupSerial();
    SetupServo();
}

void loop()
{
    CheckSerialCommands();
    // Check if data is available on the RS485 bus
    Check485Serial();
    ServoLoop();
}