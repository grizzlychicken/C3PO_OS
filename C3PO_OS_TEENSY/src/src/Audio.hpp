#include <Arduino.h>
#include "definitions.h"
#include <string>
#include <src/SBUSFunctions.hpp>
#include <src/SerialFunctions.hpp>

#define AUDIOSERIAL Serial3
#define AUDIORTS 12

void AudioSetup();
void AudioLoop();

void UpdateVolume();
void PlayWavA();
void PlayWavA(int num);
void PlayNextWavA();
void PlayNextWav5000A();
void PlayWavB(int num);
void SetAutoPlayWav(bool PlayWav);
void SetAutoPlayVocal();
void TxAudioAllStop();
void TxVoiceVolume(int Volume);
void TxWavAVolume(int Volume);
void TxWavBVolume(int Volume);
void WavAisPlaying();
void QueryAudioPeak();
void SetCurrentTrack(uint16_t Track);
void SetAudioVoiceOn();
void SetAudioVoiceOff();
uint8_t GetAudioVoiceState();
void SetAudioPitch(int pitch);

void AudioSerialTransmit(String text);

void VocalUpdateMasterVolume(int Volume);
void CheckAudioSerial();
// void ParseData(char ReceivedChar[32]);
void ParseData(char RxChar[64]);
