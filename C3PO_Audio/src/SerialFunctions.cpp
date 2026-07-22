// Serial Functions
#include "SerialFunctions.h"
#include "AudioFunctions.h"
elapsedMillis SerialElapsed = 0;
elapsedMillis SerialAudioElapsed = 0;

// Serial

const byte maxDataLength = 32; // maxDataLength is the maximum length allowed for received data.
char receivedChars[maxDataLength + 1];
String RxString = "";
bool newData = false; // newData is used to determine if there is a new command
bool recvInProgress = false;
byte RxIndex = 0;
const char startMarker = '<';
const char endMarker = '>';

void SetupSerial()
{
    Serial485.begin(115200);
    Serial485.transmitterEnable(41);
}

void SerialLoop()
{
    CheckSerialCommands();
    CheckAudioSerial();
}

void TransmitPeakLevel(int PeakLevel)
{
    Serial485.print("<QVP");
    Serial485.print(PeakLevel);
    Serial485.print(">");

//    Serial.print("QVP ");
//    Serial.println(PeakLevel);
}

void TransmitIsPlayingA(uint8_t IsPlaying)
{
    Serial485.print("<QPA");
    Serial485.print(IsPlaying);
    Serial485.print(">");

    Serial.print("QPA ");
    Serial.println(IsPlaying);
}

void CheckSerialCommands()
{
    char rx_byte = 0;

    if (SerialElapsed > 25)
    {
        // Serial Port Command Processing
        if (Serial.available() > 0)
        {                            // is a character available?
            rx_byte = Serial.read(); // get the character

            // check if a number was received

            if (rx_byte == 'a')
            {
                PlayFileA("2001 c3po_lines_iamc3po short.wav");
                Serial.print("PlayFileA");
                Serial.println("2001.wav");
            }
            if (rx_byte == 'b')
            {
                PlayFileB("1032.wav");
                Serial.print("PlayFileB");
                Serial.println("1032.wav");
            }
            if (rx_byte == 'A')
            {
                StopFileA();
            }
            if (rx_byte == 'B')
            {
                StopFileB();
            }

            if (rx_byte == 'x')
            {
                StopAll();
            }
            if (rx_byte == '1')
            {
                SetPitch(0.7f);
            }
            if (rx_byte == '2')
            {
                SetPitch(1.0f);
            }
            if (rx_byte == '3')
            {
                SetPitch(1.4f);
            }
            if (rx_byte == '4')
            {
                SetPitch(2.0f);
            }
            if (rx_byte == '+')
            {
                SetPitch(GetCurrentPitchSetting() + .01);
            }

            if (rx_byte == '-')
            {
                SetPitch(GetCurrentPitchSetting() - .01);
            }

        } // end: if (Serial.available() > 0)
        SerialElapsed = 0;
    }
}

void CheckAudioSerial()
{
    if (SerialAudioElapsed > 0)
    {
        char Audio_rx_byte = 0;
         // is a character available?
        for (int x = 0; x < min(Serial485.available(), 16); x++) // if (Serial485.available() > 0)
        {
           
            Audio_rx_byte = Serial485.read(); // get the character
            // Serial.println(Audio_rx_byte);
            if (recvInProgress == true)
            {
                if (Audio_rx_byte != endMarker)
                {
                    if (RxIndex < maxDataLength)
                    {
                        receivedChars[RxIndex] = Audio_rx_byte;
                        RxIndex++;
                        RxString += Audio_rx_byte;
                    }
                }
                else
                {
                    receivedChars[RxIndex] = '\0'; // terminate the string
                    recvInProgress = false;
                    RxIndex = 0;
                    newData = true;
                }
            }
            else if (Audio_rx_byte == startMarker)
            {
                recvInProgress = true;
            }
        } // end: if (Serial.available() > 0)
        if (newData)
        {
            ParseData(receivedChars);
            // ParseData2(receivedChars);
            // ParseDataString(RxString);
            // Serial.println(receivedChars);
            // Serial.println(RxString);
            RxString = "";
            newData = false;
        }
    }
    //	SerialMillis = millis();
    //}
}

void ParseData(char ReceivedChar[32])
{
    char Command;

    // Serial485.print("Received Char ");
    Serial.println(ReceivedChar);

    Command = ReceivedChar[0];

    switch (Command)
    {
    case 'Q': // Query
              // Query for Wav A Playing
        if (ReceivedChar[1] == 'P')
        {
            if (ReceivedChar[2] == 'A')
            {
                Serial.print("QPA");
                CheckWavAPlaying();
            }
        }

        break;
        // Query for Peak
        // if (ReceivedChar[1] == 'P')
        // {
        //     if (ReceivedChar[2] == 'P')
        //     {
        //         Serial.print("QPP");
        //         TransmitPeakLevel(GetAudioLevel());
        //     }
        // }

        break;

    case 'P':
    {
        // Stop Playback
        if (receivedChars[1] == 'S')
        {
            if (receivedChars[2] == 'A')
            {
                StopFileA();
            }
            if (receivedChars[2] == 'B')
            {
                StopFileB();
            }
            if (receivedChars[2] == 'X')
            {
                StopAll();
            }
        }

        // Volume
        if (receivedChars[1] == 'V')
        {
            Serial.println("Set Volume ");
            if (receivedChars[2] == 'V')
            {
                SetVolumeV(atoi(receivedChars + 3));
            }
            else if (receivedChars[2] == 'A')
            {
                SetVolumeA(atoi(receivedChars + 3));
            }
            else if (receivedChars[2] == 'B')
            {
                SetVolumeB(atoi(receivedChars + 3));
            }
        }

        // Pitch
        if (receivedChars[1] == 'P')
        {
            Serial.print("Set Pitch");
            Serial.println(atoi(receivedChars + 2) * .01f);
            SetPitch(atoi(receivedChars + 2) * .01f);
        }
    }
    break;
    // SD card Playback
    case 'C':
    {

        if (receivedChars[1] == 'A')
        {
            Serial.print("Play Wav A: ");
            int number = atoi(receivedChars + 2); // Skip first two digits and convert the remaining part to an integer

            // Create the filename string
            char filename[10]; // Adjust the size based on your maximum expected number length
           // sprintf(filename, "%04d.wav", number);
            sprintf(filename, "%04d", number);

            PlayFileA(filename);
        }
        else if (receivedChars[1] == 'B')
        {
            Serial.print("Play Wav B: ");

            int number = atoi(receivedChars + 2); // Skip first two digits and convert the remaining part to an integer

            // Create the filename string
            char filename[10]; // Adjust the size based on your maximum expected number length
            sprintf(filename, "%04d", number);

            PlayFileB(filename);
        }
    }
    break;

    default:
        break;
    }
}

// void ParseData2(char InputBuffer[32])
// {

//     //	int index = 0;
//     Serial.println("Parsing Data 2");
//     Serial.println(InputBuffer);
//     // Check if the input starts with 'C' and 'A'
//     if (InputBuffer[0] == 'C' && InputBuffer[1] == 'A')
//     {
//         // Extract the number part from the input
//         int number = atoi(InputBuffer + 2); // Skip 'C' and 'A' and convert the remaining part to an integer

//         // Create the filename string
//         char filename[10]; // Adjust the size based on your maximum expected number length
//         sprintf(filename, "%04d.wav", number);

//         // Call the playwava function with the generated filename
//         PlayFileA(filename);
//     }
// }