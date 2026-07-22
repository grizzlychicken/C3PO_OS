// RS485 Serial Port Functions
// 485 used Serial Port 4
// TX4 - D17
// RX4 - D16
// RTS4 - D41
// RTS sets the Direction for half duplex operation.  Low - Receive Mode, High - Transmit
#include <Arduino.h>


#define HeadSerial Serial3



void SetupSerial();
void SerialLoop();

void TxServoPosition(uint8_t Addr, uint8_t ServoNum, int ServoPositionuS);
void TxServoAttach(uint8_t Addr, uint8_t ServoNum);
void TxServoDetach(uint8_t Addr, uint8_t ServoNum);

void CheckSerialCommands();
