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

void SetHeadAudioLevel(uint8_t Level);
void TxHeadAudioLevel(uint8_t Level);
void SetVisorPosition(int VisorPositionDeg);
void TxVisorPosition(int VisorPositionDeg);


void CheckSerialCommands();
void Check485Serial();
void ParseData(char RxChar[64]);
int charToInt(char c);