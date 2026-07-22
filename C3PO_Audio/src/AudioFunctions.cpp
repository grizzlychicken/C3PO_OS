// Audio Functions
#include "AudioFunctions.h"
#include "SerialFunctions.h"

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioInputI2S i2s1;            // xy=92.27275085449219,85.00000381469727
AudioPlaySdWav playWavA;       // xy=96.5,240
AudioPlaySdWav playWavB;       // xy=96.50000381469727,316.00000190734863
AudioEffectFade fadeAR;        // xy=284.4999771118164,261.0000009536743
AudioEffectFade fadeBL;        // xy=284.4999771118164,299.00000190734863
AudioEffectFade fadeBR;        // xy=284.4999771118164,337.00000286102295
AudioEffectFade fadeAL;        // xy=285.4999809265137,229.0000114440918
AudioEffectFade fadeVL;        // xy=287.4999771118164,56.00001859664917
AudioEffectFade fadeVR;        // xy=288.49999237060547,95.0000057220459
AudioMixer4 voice_mixer;       // xy=469.2727966308594,97.00001525878906
AudioAnalyzePeak peak1;        // xy=630.2727737426758,25.000015258789062
PitchShiftEffect pitchEffect1; // xy=644.2727737426758,96.27273178100586
AudioMixer4 mixerR;            // xy=828.5000076293945,300.99998474121094
AudioMixer4 mixerL;            // xy=834.5000038146973,173.0000114440918
AudioOutputI2S audioOutput;    // xy=1022.5000152587891,233.00003051757812
AudioConnection patchCord1(i2s1, 0, fadeVL, 0);
AudioConnection patchCord2(i2s1, 1, fadeVR, 0);
AudioConnection patchCord3(playWavA, 0, fadeAL, 0);
AudioConnection patchCord4(playWavA, 1, fadeAR, 0);
AudioConnection patchCord5(playWavB, 0, fadeBL, 0);
AudioConnection patchCord6(playWavB, 1, fadeBR, 0);
AudioConnection patchCord7(fadeAR, 0, mixerR, 1);
AudioConnection patchCord8(fadeBL, 0, mixerL, 2);
AudioConnection patchCord9(fadeBR, 0, mixerR, 2);
AudioConnection patchCord10(fadeAL, 0, mixerL, 1);
AudioConnection patchCord11(fadeVL, 0, voice_mixer, 0);
AudioConnection patchCord12(fadeVR, 0, voice_mixer, 1);
AudioConnection patchCord13(voice_mixer, pitchEffect1);
AudioConnection patchCord14(voice_mixer, peak1);
AudioConnection patchCord15(pitchEffect1, 0, mixerL, 0);
AudioConnection patchCord16(pitchEffect1, 0, mixerR, 0);
AudioConnection patchCord18(mixerL, 0, audioOutput, 0);
AudioConnection patchCord17(mixerR, 0, audioOutput, 1);
AudioControlSGTL5000 sgtl5000_1; // xy=971.2727355957031,556.9999847412109
// GUItool: end automatically generated code

#define VOICECHANNEL 0
#define WAVACHANNEL 1
#define WAVBCHANNEL 2

int BOARDLED = LED_BUILTIN;

int gVolumeV = 50;
int gVolumeA = 50;
int gVolumeB = 50;
float gCurrentPitch = 1.4f;
int gAudioLevelPeak = 0;
const int myInput = AUDIO_INPUT_LINEIN;
bool AudioQuiet = 1;
elapsedMillis Peakfps;
elapsedMillis fps;

// Vector to hold filenames
std::vector<String> fileNames;

// void SetMixerGain(int MixerChannel, int Gain)
// {
//     float TargetGain = Gain * 0.01f;
//     main_mixer.gain(MixerChannel, TargetGain);
//     // mixer2.gain(MixerChannel, TargetGain);
// }

void SetupAudio()
{
    AudioMemory(24);
    sgtl5000_1.enable();
    sgtl5000_1.audioPreProcessorEnable(); // Seemed to help with crackling a bit, so I left it
    sgtl5000_1.inputSelect(myInput);
    sgtl5000_1.volume(0.8);
    sgtl5000_1.lineOutLevel(15);
    sgtl5000_1.dacVolumeRamp();
    sgtl5000_1.enhanceBassEnable();

    SetMixerGain(VOICECHANNEL, gVolumeV);
    SetMixerGain(WAVACHANNEL, gVolumeA);
    SetMixerGain(WAVBCHANNEL, gVolumeB);
    if (!SD.begin(BUILTIN_SDCARD))
    {
        Serial.println("SD Card failed, or not present");
    }

    createFileList();
}

void createFileList()
{
    File root = SD.open("/");
    File entry = root.openNextFile();

    while (entry)
    {
        String fileName = entry.name();
        if (fileName.endsWith(".wav"))
        {
            fileNames.push_back(fileName);
        }
        entry = root.openNextFile();
    }

    Serial.println("File list created:");
    for (const auto &fileName : fileNames)
    {
        Serial.println(fileName);
    }

    root.close();
}

void SetPitch(float PitchShiftAmount)
{
    gCurrentPitch = PitchShiftAmount;
    pitchEffect1.setPitchAmount(gCurrentPitch);
    Serial.print("Pitch Setting: ");
    Serial.println(gCurrentPitch);
}

float GetCurrentPitchSetting()
{
    return gCurrentPitch;
}

int GetAudioLevel()
{
    return gAudioLevelPeak;
}

void SetMixerGain(int MixerChannel, int Gain)
{
    float TargetGain = Gain * 0.01f;
    mixerL.gain(MixerChannel, TargetGain);
    mixerR.gain(MixerChannel, TargetGain);
}

void SetVolumeV(int vol)
{
    gVolumeV = vol;

    if (gVolumeV > 100)
        gVolumeV = 100;
    Serial.printf("VV: %03d \r\n", gVolumeV);
    // Set the gain
    SetMixerGain(VOICECHANNEL, gVolumeV);
}

void SetVolumeA(int vol)
{
    gVolumeA = vol;
    if (gVolumeA > 100)
        gVolumeA = 100;
    Serial.printf("VA: %01d \r\n", gVolumeA);
    // Set the gain
    SetMixerGain(WAVACHANNEL, gVolumeA);
}

void SetVolumeB(int vol)
{
    gVolumeB = vol;
    if (gVolumeB > 100)
        gVolumeB = 100;
    Serial.printf("VB: %01d \r\n", gVolumeB);
    // Set the gain
    SetMixerGain(WAVBCHANNEL, gVolumeB);
}

void PlayFileA(const char *FileNameNum)
{
       
    auto it = std::find_if(fileNames.begin(), fileNames.end(), [&](const String& fileName) {
        return fileName.startsWith(FileNameNum);
    });

    if (it != fileNames.end()) {
        String fileName = *it;
        Serial.println("Playing file: " + fileName);
        playWavA.play(fileName.c_str());
    } else {
        Serial.println("No matching file found.");
    }

}

void PlayFileB(const char *FileNameNum)
{
    auto it = std::find_if(fileNames.begin(), fileNames.end(), [&](const String& fileName) {
        return fileName.startsWith(FileNameNum);
    });

    if (it != fileNames.end()) {
        String fileName = *it;
        Serial.println("Playing file: " + fileName);
        playWavB.play(fileName.c_str());
    } else {
        Serial.println("No matching file found.");
    }
    
    //    Serial.print("Playing file B: ");
    // Serial.println(filename);

    // // Start playing the file.
    // playWavB.play(filename);
}

void CheckWavAPlaying()
{
    if (playWavA.isPlaying())
    {
        // check again
        if (playWavA.isPlaying())
        {
            TransmitIsPlayingA(1);
            Serial.println("IsPlaying!");
        }
        else
        {
            TransmitIsPlayingA(0);
            Serial.println("IsNotPlaying!");
        }
    }
    else
    {
        if (playWavA.isPlaying())
        {
            TransmitIsPlayingA(1);
            Serial.println("IsPlaying!");
        }
        else
        {
            TransmitIsPlayingA(0);
            Serial.println("IsNotPlaying!");
        }
    }
}

void StopFileA()
{
    playWavA.stop();
}

void StopFileB()
{
    playWavB.stop();
}

void StopAll()
{
    playWavA.stop();
    playWavB.stop();
}

void PeakDetection()
{
    if (Peakfps > 30)
    {
        Peakfps = 0;
        if (peak1.available())
        {

            gAudioLevelPeak = peak1.read() * 70;
            // Serial.println("Peak available");
            // Serial.print("Peak Read: ");
            // Serial.println(AudioLevelPeak);

            if (gAudioLevelPeak > 1)
            {
                AudioQuiet = 0;
                //  Serial.print(AudioLevelPeak); Serial.print(" |");
                // for (int cnt = 0; cnt < AudioLevelPeak; cnt++) {
                //     				Serial.print(">");
                //     			}
                //     			Serial.println();

                TransmitPeakLevel(gAudioLevelPeak);
            }
            else
            {
                if (AudioQuiet == 0)
                {

                    TransmitPeakLevel(0);
                    AudioQuiet = 1;
                }
            }
        }
    }
}
