#include <Arduino.h>

#include <Servo.h>
#include <HardwareSerial.h>
#include <RS485.h>
#include "Definitions.h"  // Include the definitions library
#include <ESP32Servo.h>

// Define the board ID for this sketch
#define BOARD_ID TORSO_ID
//#define BOARD_ID SHOULDER_ID
//#define BOARD_ID ARM_ID
//#define BOARD_ID HEAD_ID

// Create an array of Servo objects
Servo servos[NUM_TORSO_SERVOS];

// Initialize RS485 communication
HardwareSerial RS485(1); // Use UART1 for RS485

void setup() {
    // Initialize serial communication for RS485
    RS485.begin(9600, SERIAL_8N1, 16, 17); // RX on GPIO16, TX on GPIO17

    // Attach servos to pins
    for (int i = 0; i < NUM_TORSO_SERVOS; i++) {
        servos[i].attach(torsoServoPins[i]);
    }
}

void loop() {
    // Check if data is available on the RS485 bus
    if (RS485.available() >= 2) { // We need at least 2 bytes (servoIndex and position)
        // Read the servo index and position from the RS485 bus
        int servoIndex = RS485.read();
        int position = RS485.read();

        // Set the servo to the received position
        if (servoIndex >= 0 && servoIndex < NUM_TORSO_SERVOS) {
            servos[servoIndex].write(position);
        }
    }
}
