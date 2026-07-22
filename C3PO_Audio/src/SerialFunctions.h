#include <Arduino.h>
#include <string>


// Serial Functions
#include "PitchShiftEffect.h"

// #define Serial485 Serial7

#define Serial485 Serial1

void SetupSerial() ;
void SerialLoop() ;
void TransmitPeakLevel(int PeakLevel) ;
void TransmitIsPlayingA(uint8_t IsPlaying) ;
void CheckSerialCommands();
void CheckAudioSerial();
void ParseData(char ReceivedChar[32]) ;
void ParseData2(char InputBuffer[32]) ;