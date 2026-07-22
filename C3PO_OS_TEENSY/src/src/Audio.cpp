// Functions to talk to Audio Board

#include <src/Audio.hpp>

uint8_t AudioVoiceState = AUDIOVOICEOFF;
unsigned long VocalAudioAutoMillis = millis();
unsigned long VocalAudioAutoWait = 3000;

uint16_t gCurrentWavTrack = 1000;
const uint16_t MaxWavTracks = 1034;
uint16_t gCurrentWavTrack5000 = 5001;
const uint16_t MaxWavTracks5000 = 5007;
uint16_t gCurrentVoiceAudioVolume = 8;
uint16_t gTargetVoiceAudioVolume = 8;
uint16_t gCurrentWavAAudioVolume = 6;
uint16_t gCurrentWavBAudioVolume = 6;
uint16_t gCurrentMasterVolume = 5; // 0 to 9 range;
uint16_t gCurrentPitch = 120;      // pitch scale factor, float x 100
unsigned long AudioPrevMillis = millis();
unsigned long AudioVocalPlayTime = millis();
uint16_t AudioVocalDelay = 5000;
// bool MuseEnabled = 0;
bool bAutoPlayWav = false;
bool bAutoPlayVocal = false;
int gWavAisPlaying = 0;
int gWavAFinished = 0;
extern int gWavAisPlaying;   // 0 = not playing, >0 = playing (smoothed counter)


// Serial
char gAudio_rx_byte = 0;
const byte maxDataLength = 30; // maxDataLength is the maximum length allowed for received data.
char AudioRxChars[64];
String RxString = "";
bool newData = false; // newData is used to determine if there is a new command
bool recvInProgress = false;
byte Audiondx = 0;
const char startMarker = '<';
const char endMarker = '>';

elapsedMillis AudioSerialElapsed = 0;
elapsedMillis QueryPeakElapsed = 0;

void AudioSetup()
{
    AUDIOSERIAL.begin(115200);
    // AUDIOSERIAL.transmitterEnable(AUDIORTS);

    Serial.println("Setup Audio");

    Serial.print("Current Vocal Volume: ");
    Serial.println(gCurrentVoiceAudioVolume);

    AUDIOSERIAL.println("Vocalizer start");

    VocalUpdateMasterVolume(gCurrentMasterVolume);
    TxVoiceVolume(gCurrentVoiceAudioVolume);
    TxWavAVolume(gCurrentWavAAudioVolume);
    TxWavBVolume(gCurrentWavBAudioVolume);
}

void AudioLoop()
{
    // Audio Processing Loop
    CheckAudioSerial();

    // if (QueryPeakElapsed > 100)
    // {
    // //    QueryAudioPeak();
    // }

    if ((millis() - AudioPrevMillis) > 200)
    {

        AudioPrevMillis = millis();

        if (bAutoPlayWav)
        {
            WavAisPlaying();
            //   Serial.print("Wav A playing: ");
            //   Serial.println(gWavAisPlaying);
            // if no wave is playing - play a file.
            if (!gWavAisPlaying)
            {
                if (!gWavAFinished)
                { // Previous one finished
                    PlayNextWavA();
                    PlayWavA();
                    // PlayWavA();
                    // PlayWavA();
                    gWavAFinished = 1;
                }
            }
            else
                gWavAFinished = 0;
        }

        if (bAutoPlayVocal)
        {

            if (millis() - AudioVocalPlayTime > AudioVocalDelay)
            {
                AudioVocalPlayTime = millis();
                AudioVocalDelay = random(60000, 180000);
                PlayWavB(random(2001, 2032));
            }
        }
        // Serial.print("AudioVoiceState: ");
        // Serial.println(AudioVoiceState);

        if (AudioVoiceState == AUDIOVOICEOFF)
        {
            // Serial.println("Voice Audio OFF");
            if (gTargetVoiceAudioVolume > 0)
            {
                Serial.println("Voice Audio Off");
                TxVoiceVolume(0);
                gTargetVoiceAudioVolume = 0;
            }
            //  SetHeadAudioLevel(0);
        }
        if (AudioVoiceState == AUDIOVOICEON)
        {
            // Serial.println("Voice Audio On");
            if (gTargetVoiceAudioVolume == 0)
            {
                Serial.println("Voice Audio ON");
                gTargetVoiceAudioVolume = gCurrentVoiceAudioVolume;
                TxVoiceVolume(gCurrentVoiceAudioVolume);
            }
        }
    }
}

void UpdateVolume()
{
    uint16_t volume;

    if (ChannelData(cVOICEVOLUME) >= 175)
        volume = map(ChannelData(cVOICEVOLUME), 175, 1812, 1, 99);
    else
        volume = 0;
    if (gCurrentVoiceAudioVolume != volume)
    {
        gCurrentVoiceAudioVolume = volume;
        // volume = map(CurrentVocalAudioVolume, 150, 1850, 0, 9);
        Serial.print("CH: ");
        Serial.print(ChannelData(cVOICEVOLUME));
        Serial.print("VoiceVolume: ");
        Serial.println(volume);
        // Don't update Voice volume is Voice off
        if (AudioVoiceState == AUDIOVOICEON)
            TxVoiceVolume(volume);
        TxWavBVolume(volume);
    }
    if (ChannelData(cWAVVOLUME) >= 175)
        volume = map(ChannelData(cWAVVOLUME), 175, 1812, 1, 99);
    else
        volume = 0;
    if (gCurrentWavAAudioVolume != volume)
    {
        gCurrentWavAAudioVolume = volume;
        Serial.print("CH: ");
        Serial.print(ChannelData(cWAVVOLUME));
        Serial.print(" WavAVolume: ");
        Serial.println(volume);
        TxWavAVolume(volume);
    }
}

// Play a Random wav file on Bank A
void PlayWavA()
{

    AUDIOSERIAL.print("<CA");
    AUDIOSERIAL.printf("%04d", gCurrentWavTrack);
    Serial.printf("%04d", gCurrentWavTrack);
    AUDIOSERIAL.print(">");
}

void PlayWavA(int num)
{
    gCurrentWavTrack = num;
    AUDIOSERIAL.print("<CA");
    AUDIOSERIAL.printf("%04d", gCurrentWavTrack);
    Serial.printf("%04d", gCurrentWavTrack);
    AUDIOSERIAL.print(">");
}

void PlayNextWavA()
{
    if (gCurrentWavTrack < MaxWavTracks)
        gCurrentWavTrack++;
    else
        gCurrentWavTrack = 1001;
    AUDIOSERIAL.print("<CA");
    AUDIOSERIAL.printf("%04d", gCurrentWavTrack);
    Serial.printf("%04d", gCurrentWavTrack);
    AUDIOSERIAL.print(">");
}

void PlayNextWav5000A()
{
    if (gCurrentWavTrack5000 < MaxWavTracks5000)
        gCurrentWavTrack5000++;
    else
        gCurrentWavTrack5000 = 5001;
    AUDIOSERIAL.print("<CA");
    AUDIOSERIAL.printf("%04d", gCurrentWavTrack5000);
    Serial.printf("%04d", gCurrentWavTrack5000);
    AUDIOSERIAL.print(">");
}

void PlayWavB(int num)
{
    gCurrentWavTrack = num;
    AUDIOSERIAL.print("<CB");
    AUDIOSERIAL.printf("%04d", gCurrentWavTrack);
    Serial.printf("%04d", gCurrentWavTrack);
    AUDIOSERIAL.print(">");
}

// Send all Stop command
void TxAudioAllStop()
{
    AudioSerialTransmit("PSX");
}

void TxVoiceVolume(int Volume)
{

    AUDIOSERIAL.print("<PVV");
    AUDIOSERIAL.printf("%02d", Volume);
    AUDIOSERIAL.print(">");
}
void TxWavAVolume(int Volume)
{

    AUDIOSERIAL.print("<PVA");
    AUDIOSERIAL.printf("%02d", Volume);
    AUDIOSERIAL.print(">");
}
void TxWavBVolume(int Volume)
{

    AUDIOSERIAL.print("<PVB");
    AUDIOSERIAL.printf("%02d", Volume);
    AUDIOSERIAL.print(">");
}

// Update Master Volume
void VocalUpdateMasterVolume(int Volume)
{

    AUDIOSERIAL.print("<PVM");
    AUDIOSERIAL.print(Volume);
    AUDIOSERIAL.print(">");
}

void AudioSerialTransmit(String text)
{
    // sendCache = text;
    AUDIOSERIAL.println(("<" + text + ">").c_str());
}

void WavAisPlaying()
{
    AUDIOSERIAL.print("<QPA>");
}

void QueryAudioPeak()
{
    AUDIOSERIAL.print("<QPP>");
}

void CheckAudioSerial()
{
    if (AudioSerialElapsed > 5)
    {

        // if (AUDIOSERIAL.available() > 0)
        // is a character available?
        for (int x = 0; x < min(AUDIOSERIAL.available(), 16); x++)
        {
            gAudio_rx_byte = AUDIOSERIAL.read(); // get the character
            //  Serial.print(" x");
            // Serial.print(x);
            // Serial.print(gAudio_rx_byte);
            if (recvInProgress == true)
            {
                if (gAudio_rx_byte != endMarker)
                {
                    if (Audiondx < maxDataLength)
                    {
                        AudioRxChars[Audiondx] = gAudio_rx_byte;
                        Audiondx++;
                        RxString += gAudio_rx_byte;
                    }
                }
                else
                {
                    AudioRxChars[Audiondx] = '\0'; // terminate the string
                    recvInProgress = false;
                    Audiondx = 0;
                    newData = true;
                }
            }
            else if (gAudio_rx_byte == startMarker)
            {
                recvInProgress = true;
            }
        } // end: if (Serial.available() > 0)
        if (newData)
        {
            ParseData(AudioRxChars);
            // ParseDataString(RxString);
            // Serial.println(receivedChars);
            // Serial.println(RxString);
            RxString = "";
            newData = false;
        }
        AudioSerialElapsed = 0;
    }
}

void ParseData(char RxChar[64])
{
    char Command;

    Command = RxChar[0];

    switch (Command)
    {
    case 'Q': // Mode
        if (RxChar[1] == 'V')
        {
            if (RxChar[2] == 'P') // Receive Peak Audio
            {
                // Serial.print("QVP - ");
                // Serial.println(atoi(RxChar + 3));
                // Pass along the audio level to the head
                // SetHeadAudioLevel(atoi(RxChar + 3));
            }
        }
        if (RxChar[1] == 'P')
        {
            if (RxChar[2] == 'A') // Is Playing
            {
                uint8_t isplaying = atoi(RxChar + 3);

                if (isplaying > 0)
                {
                    if (gWavAisPlaying < 3)
                        gWavAisPlaying++;
                }
                else
                {
                    if (gWavAisPlaying > 0)
                        gWavAisPlaying--;
                }
                //  Serial.print("QPA: ");
                // Serial.println(isplaying);
            }
        }
        break;

    default:
        break;
    }
}

void SetAutoPlayWav(bool PlayWav)
{
    bAutoPlayWav = PlayWav;
    Serial.print("Set Auto Play Wav ");
    Serial.println(PlayWav);
}

void SetAutoPlayVocal()
{
    if (bAutoPlayVocal)
    {
        bAutoPlayVocal = false;
        Serial.println("Set Auto Play Vocal Off ");
    }
    else
    {
        bAutoPlayVocal = true;
        Serial.println("Set Auto Play Vocal On ");
        PlayWavB(random(2001,2033));
    }
}

void SetCurrentTrack(uint16_t Track)
{
    gCurrentWavTrack = Track;
}

void SetAudioVoiceOn()
{
    AudioVoiceState = AUDIOVOICEON;
    Serial.println("Voice Audio ON");
    gTargetVoiceAudioVolume = gCurrentVoiceAudioVolume;
    TxVoiceVolume(gCurrentVoiceAudioVolume);
}

void SetAudioVoiceOff()
{
    AudioVoiceState = AUDIOVOICEOFF;
    Serial.println("Voice Audio OFF");
    gTargetVoiceAudioVolume = 0;
    TxVoiceVolume(0);
}

uint8_t GetAudioVoiceState()
{
    return AudioVoiceState;
}

void SetAudioPitch(int pitch)
{
    if (gCurrentPitch != pitch)
    {
        gCurrentPitch = pitch;
        AUDIOSERIAL.print("<PP");
        AUDIOSERIAL.print(pitch);
        AUDIOSERIAL.print(">");

        Serial.print("Set Pitch: ");
        Serial.println(pitch);
    }
}