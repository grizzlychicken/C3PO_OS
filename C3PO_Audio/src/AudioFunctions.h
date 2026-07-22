#include "PitchShiftEffect.h"
#include <Arduino.h>
#include <string>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>


void SetupAudio();
void createFileList() ;
void SetPitch(float PitchShiftAmount);
float GetCurrentPitchSetting();
int GetAudioLevel();
void SetMixerGain(int MixerChannel, int Gain);
void SetVolumeV(int vol);
void SetVolumeA(int vol);
void SetVolumeB(int vol);
void PlayFileA(const char *FileNameNum);
void PlayFileB(const char *FileNameNum);
void CheckWavAPlaying();
void StopFileA();
void StopFileB();
void StopAll();

void PeakDetection();