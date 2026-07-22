
#include "PitchShiftEffect.h"
#include "SerialFunctions.h"
#include "AudioFunctions.h"
#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing...");

    SetupSerial();
    SetupAudio();

    // Set volume
    float pitch_amount = 1.4f;
    SetPitch(pitch_amount);

    digitalWrite(34, HIGH);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
}

void loop()
{
    SerialLoop();
    PeakDetection();
}
