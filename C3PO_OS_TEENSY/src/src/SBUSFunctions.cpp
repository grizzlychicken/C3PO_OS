// SBUS related functions and controls
// Requires SBUS library - https://github.com/bolderflight/SBUS
// Library modified to allow for invert for Teensy 4.

#include <Arduino.h>
#include "SBUS.h"
#include "definitions.h"

// a SBUS object, which is on hardware
// serial port 1
SBUS rcvr(Serial1);
int z = 0;

#define SBUSChannels 24
// channel, fail safe, and lost frames data

uint16_t channelsTemp[SBUSChannels];
uint16_t channels[SBUSChannels];
bool failSafe;
bool lostFrame;
bool SBUS_OK = 0;
uint8_t SBUSokCount = MAXSBUSOK;

bool SBUSDebugEnabled = false;

elapsedMillis SBUSelapsed = 0;

void SBUSsetup()
{
  // begin the SBUS communication
  Serial.println("Setup SBUS");
  rcvr.begin();
}

void SBUSloop()
{
  // Serial.println("SBus Loop ");
  if (SBUSelapsed > 10)
  {
    // look for a good SBUS packet from the receiver
    if (rcvr.read(&channelsTemp[0], &failSafe, &lostFrame))
    {

      // Turn on SBUS serial outputs
if(SBUSDebugEnabled){
    for (int i = 0; i < SBUSChannels; i++) {
      Serial.print("C");
      Serial.print(i);
      Serial.print(" ");
      Serial.print(channelsTemp[i]);
      Serial.print(" ");

    }
    Serial.print(failSafe);
    Serial.print(" ");
    Serial.print(lostFrame);
    Serial.println(" ");
} //#endif
    }
    if (!failSafe && !lostFrame)
    {
      for (int i = 0; i < SBUSChannels; i++)
      {
        // check if any channels are < sbus min or greater then sbus max, if so, toss the frame
        if ((channelsTemp[i] < SBUSMIN) || (channelsTemp[i] > SBUSMAX))
        {
          lostFrame = 1;
          if (SBUSokCount > 0)
            SBUSokCount--;
        }
      }
      if (!lostFrame)
      {
        for (int i = 0; i < SBUSChannels; i++)
        { // copy data across if lost frame is ok.
          channels[i] = channelsTemp[i];
          if (SBUSokCount >= MAXSBUSOK)
            SBUSokCount = MAXSBUSOK;
          else
            SBUSokCount++;
        }
      }
    }
    else if (SBUSokCount > 0)
      SBUSokCount--;
    SBUSelapsed = 0;
  }
}

bool SBUSOK()
{
  if (SBUSokCount >= MAXSBUSOK)
    SBUS_OK = 1;
  else
    SBUS_OK = 0;
  Serial.println("SBUS not ok");
  return SBUS_OK;
}

uint16_t ChannelData(uint16_t channelnum)
{
  return channels[channelnum];
}

void ToggleSBUSDebug(){
  SBUSDebugEnabled = !SBUSDebugEnabled;
  Serial.print("SBUS Enabled = ");
  Serial.println(SBUSDebugEnabled);
}