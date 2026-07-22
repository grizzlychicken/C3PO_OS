// RS485 Serial Port Functions
// 485 used Serial Port 4
// TX4 - D17
// RX4 - D16
// RTS4 - D41
// RTS sets the Direction for half duplex operation.  Low - Receive Mode, High - Transmit
#include <Arduino.h>



void SBUSsetup();
void SBUSloop();

bool SBUSOK();
uint16_t ChannelData(uint16_t channelnum);
void ToggleSBUSDebug();