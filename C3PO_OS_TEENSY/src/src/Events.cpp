// Radio Event triggering
// Mapping radio switches etc. to functions.
//  Requires Scheduler Library.  Modified Library by TZ

#include <src/Events.hpp>
#include <src/MotionControl.hpp>
#include <src/GeneratedCodeanimations.h>
#include <src/BottangoCore.h>

// ---- Debug rate limiter for Events.cpp (prevents Serial spam) ----
// Uses 32 independent "keys" to rate-limit messages.
static inline bool dbgAllow(uint8_t key, uint32_t intervalMs = 250)
{
    static uint32_t last[32] = {0};
    uint32_t now = millis();
    if ((uint32_t)(now - last[key]) >= intervalMs)
    {
        last[key] = now;
        return true;
    }
    return false;
}

uint8_t RunMode = 0;
uint8_t gVoicePitchMode = PITCHOFF;
uint16_t ButtonState = 0;
bool ButtonPressed = false;   // Button Pressed indicator for button bank
bool ButtonPressedTR = false; // Button Pressed indicator for Trim Rud
bool ButtonPressedTR2 = false; // Button Pressed indicator for Trim2 Rud
bool ButtonLongPressed = false;
bool ButtonLongPressedTR = false;
bool ButtonLongPressedTR2 = false;
byte animationIndex = 0;

int ButtonCount = 0;   // Counter for Button Bank
int ButtonCountTR = 0; // Counter for Trim Rud
int ButtonCountTR2 = 0; // Counter for Trim2 Rud

// int previousTrim1 = 0;
// int previousTrim2 = 0;

elapsedMillis ElapsedEvent = 0;

// Scheduler scheduler = Scheduler();      //create a scheduler
// typedef void *func;
// void CheckLongPress(void (*)());

void ProcessEventsLoop()
{

    if (ElapsedEvent > 100)
    {

        ProcessButtons();
        UpdateVolume();
        CheckHeadMode();
        CheckMotorHome();
        // ProcessTrim1Switches();
        // ProcessTrimBackSwitches();
        CheckVoiceAudioMode();
        // CheckShoulderMode();
        //  CheckPitchAudioMode();
        ElapsedEvent = 0;
    }
    // scheduler.update();                 //update the scheduler, maybe it is time to execute a function?
}

// Function to read the button channel, and assign a button # to it based on the nominal value + a buffer either side.
uint16_t GetButtonState()

{
    //  Serial.println(ChannelData(cBUTTONSWITCH));
    return ChannelData(cBUTTONSWITCH);
}
// Function to read the button channel, and assign a button # to it based on the nominal value + a buffer either side.
uint16_t GetButtonBank2State()

{
    //  Serial.println(ChannelData(cBUTTONSWITCH));
    return ChannelData(cTRIMSWITCH);
}

// Function to read the button channel, and assign a button # to it based on the nominal value + a buffer either side.
uint16_t GetButtonBank3State()

{
    //  Serial.println(ChannelData(cBUTTONSWITCH));
    return ChannelData(cTRIMBACKSWITCH);
}

// Function to check the button, and active Bank A or Bank B function
// void CheckButtonPress(void (*funcA)(), void (*funcB)())
// {
//     Serial.println("CheckButtonPress");
//     // Check if first time button pressed.
//     if (!ButtonPressed)
//     {
//         // Check which bank to activate
//         // if (ChannelData(cHOMEMOTORS) < 1000)
//         funcA();
//         //  else
//         //     funcB();
//         Serial.println("ButtonPress");
//         ButtonPressed = true;
//     }
void CheckButtonPress(void (*funcA)(), void (*funcB)())
{
    // Only act/print on the FIRST detection of a press (debounced by ButtonPressed flag)
    if (!ButtonPressed)
    {
#if DBG_EVENTS
        if (dbgAllow(3, 50)) Serial.println("Button bank press");
#endif
        funcA();
        ButtonPressed = true;
    }
}

    // if (abs(cBUTTONSWITCH - ButtonOff) <= 5)
    
    // {
    //     Serial.println("Button Released, Resetting TR2");
    //     ButtonPressed = false;
    // }


void CheckLongPress(void (*func)(), bool continuous)
{

    ButtonCount++;
    if (ButtonCount >= LongPressCount && !ButtonLongPressed)
    {
        func();
        if (!continuous)
            ButtonLongPressed = true;
    }
}
// void CheckButtonPressTR(void (*funcA)(), void (*funcB)())
// {
//     Serial.println("CheckButtonPressTR");
//     // Check if first time button pressed.
//     if (!ButtonPressedTR)
//     {
//         // Check which bank to activate
//         // if (ChannelData(cHOMEMOTORS) < 1000)
//         funcA();
//         //  else
//         //     funcB();
//         Serial.println("ButtonPressTR");
//         ButtonPressedTR = true;
//     }
  void CheckButtonPressTR(void (*funcA)(), void (*funcB)())
{
    if (!ButtonPressedTR)
    {
#if DBG_EVENTS
        if (dbgAllow(4, 50)) Serial.println("Trim bank press");
#endif
        funcA();
        ButtonPressedTR = true;
    }
}

    //     if (abs(cTRIMSWITCH - trimcenter) <= 5)
    // {
    //     Serial.println("Button Released, Resetting TR2");
    //     ButtonPressedTR = false;
    // }


void CheckLongPressTR(void (*func)(), bool continuous)
{

    ButtonCountTR++;
    if (ButtonCountTR >= LongPressCount && !ButtonLongPressedTR)
    {
        func();
        if (!continuous)
            ButtonLongPressedTR = true;
    }
}
// void CheckButtonPressTR2(void (*funcA)(), void (*funcB)())
// {
//     Serial.println("CheckButtonPressTR2");
//     // Check if first time button pressed.
//     if (!ButtonPressedTR2)
//     {
//         // Check which bank to activate
//         // if (ChannelData(cHOMEMOTORS) < 1000)
//         funcA();
//         //  else
//         //     funcB();
//         Serial.println("ButtonPressTR2");
//         ButtonPressedTR2 = 1;
//     }

    void CheckButtonPressTR2(void (*funcA)(), void (*funcB)())
{
    if (!ButtonPressedTR2)
    {
#if DBG_EVENTS
        if (dbgAllow(5, 50)) Serial.println("Trim bank 2 press");
#endif
        funcA();
        ButtonPressedTR2 = true;
    }
}

        // Reset when trim switch returns to neutral
    // if (abs(cTRIMBACKSWITCH - trimcenter2) <= 5)
    // {
    //     Serial.println("Button Released, Resetting TR2");
    //     ButtonPressedTR2 = false;
    // }


void CheckLongPressTR2(void (*func)(), bool continuous)
{

    ButtonCountTR2++;
    if (ButtonCountTR2 >= LongPressCount && !ButtonLongPressedTR2)
    {
        func();
        if (!continuous)
            ButtonLongPressedTR2 = true;
    }
}

// Function to determine which button was pressed and run the function associated with the click
void ProcessButtons()
{
    // Check button bank presses
    switch (GetButtonState())
    {

        // Buttons Off
    case (ButtonOff - ButtonDeadband)...(ButtonOff + ButtonDeadband):
        ButtonPressed = 0;
        ButtonLongPressed = 0;
        ButtonCount = 0;
        //     Serial.println("Buttons Off");
        break;

        // Button 1
    case (CButton1 - ButtonDeadband)...(CButton1 + ButtonDeadband):
        CheckButtonPress(Button1Click, Button1ClickB);
        CheckLongPress(Button1LongClick, 0);
        break;

        // Button 2
    case (CButton2 - ButtonDeadband)...(CButton2 + ButtonDeadband):
        CheckButtonPress(Button2Click, Button2ClickB);
        CheckLongPress(Button2LongClick, 0);
        break;

        // Button 3
    case (CButton3 - ButtonDeadband)...(CButton3 + ButtonDeadband):
        CheckButtonPress(Button3Click, Button3ClickB);
        CheckLongPress(Button3LongClick, 0);
        break;

        // Button 4
    case (CButton4 - ButtonDeadband)...(CButton4 + ButtonDeadband):
        CheckButtonPress(Button4Click, Button4ClickB);
        CheckLongPress(Button4LongClick, 0);
        break;

        // Button 5
    case (CButton5 - ButtonDeadband)...(CButton5 + ButtonDeadband):
        CheckButtonPress(Button5Click, Button5ClickB);
        CheckLongPress(Button5LongClick, 0);
        break;

        // Button 6
    case (CButton6 - ButtonDeadband)...(CButton6 + ButtonDeadband):
        CheckButtonPress(Button6Click, Button6ClickB);
        CheckLongPress(Button6LongClick, 0);
        break;
        // Button 7
    case (CButton7 - ButtonDeadband)...(CButton7 + ButtonDeadband):
        CheckButtonPress(Button7Click, Button7ClickB);
        CheckLongPress(Button7LongClick, 0);
        break;

    // Button 8
    case (CButton8 - ButtonDeadband)...(CButton8 + ButtonDeadband):
        CheckButtonPress(Button8Click, Button8ClickB);
        CheckLongPress(Button8LongClick, 0);
        break;
    }
    // button check for bank 2 buttons trim switches

   if (GetRCBottangoMode() == RC_MODE)
   { switch (GetButtonBank2State())
    {
    case (trimcenter - ButtonDeadband)...(trimcenter + ButtonDeadband):
        ButtonPressedTR = 0;
        ButtonLongPressedTR = 0;
        ButtonCountTR = 0;
        // ElbowCenter();         // Reset both elbows to mid if no trim switch is pressed
        // Serial.println("Buttons Off");
        break;

        // Trim slider #6 left push
    case (CTrim6L - ButtonDeadband)...(CTrim6L + ButtonDeadband):
        CheckButtonPressTR(Trim6LClick, Trim6LClickB);
        CheckLongPressTR(Trim6LClickLongClick, 0);
        break;

        // Trim slider #6 right push
    case (CTrim6R - ButtonDeadband)...(CTrim6R + ButtonDeadband):
        CheckButtonPressTR(Trim6RClick, Trim6RClickB);
        CheckLongPressTR(Trim6RClickLongClick, 0);
        break;

        // Trim slider #5 left push
    case (CTrim5L - ButtonDeadband)...(CTrim5L + ButtonDeadband):
        CheckButtonPressTR(Trim5LClick, Trim5LClickB);
        CheckLongPressTR(Trim5LClickLongClick, 0);
        break;
        // Trim slider #5 right push
    case (CTrim5R - ButtonDeadband)...(CTrim5R + ButtonDeadband):
        CheckButtonPressTR(Trim5RClick, Trim5RClickB);
        CheckLongPressTR(Trim5LClickLongClick, 0);
        break;


//     // Trim slider #1 left push - bend right elbow
//     case (CTrim1L - ButtonDeadband)...(CTrim1L + ButtonDeadband):
//         // Serial.println("Trim button 1 left - bend right elbow");
//         RightElbowPositionMax();
//         break;

//     // Trim slider #1 right push - extend right elbow
//     case (CTrim1R - ButtonDeadband)...(CTrim1R + ButtonDeadband):
//         // Serial.println("Trim button 1 right - extend right elbow");
//         RightElbowPositionMin();

//         break;

//     // Trim slider #5 left push - bend left elbow
//     case (CTrim4L - ButtonDeadband)...(CTrim4L + ButtonDeadband):
//         // Serial.println("Trim button 5 left - bend left elbow");
//         LeftElbowPositionMax();

//         break;

//     // Trim slider #5 right push - extend left elbow
//     case (CTrim4R - ButtonDeadband)...(CTrim4R + ButtonDeadband):
//         // Serial.println("Trim button 5 right - extend left elbow");
//         LeftElbowPositionMin();

//         break;
    }
   }
     if (GetRCBottangoMode() == RC_MODE)
    {    switch (GetButtonBank3State())
    { 
        case (trimcenter2 - ButtonDeadband)...(trimcenter2 + ButtonDeadband):
        ButtonPressedTR2 = 0;
        ButtonLongPressedTR2 = 0;
        ButtonCountTR2 = 0;
        // ForearmCenter();       // Reset both Forearms to mid if no trim switch is pressed
        //     Serial.println("Buttons Off");
        break;
        
    //     // Trim slider #2 left push - bend right Forearm
    // case (CTrim2U - ButtonDeadband)...(CTrim2U + ButtonDeadband):
    //     // Serial.println("Trim button 1 left - rotate right Forearm");
    //     RightForearmPositionMax();
    //     break;

    // // Trim slider #2 right push - extend right Forearm
    // case (CTrim2D - ButtonDeadband)...(CTrim2D + ButtonDeadband):
    //     // Serial.println("Trim button 1 right - rotate right Forearm");
    //     RightForearmPositionMin();

    //     break;

    // // Trim slider #3 left push - bend left Forearm
    // case (CTrim3U - ButtonDeadband)...(CTrim3U + ButtonDeadband):
    //     // Serial.println("Trim button 5 left - rotate left Forearm");
    //     LeftForearmPositionMax();

    //     break;

    // // Trim slider #5 right push - extend left Forearm
    // case (CTrim3D - ButtonDeadband)...(CTrim3D + ButtonDeadband):
    //     // Serial.println("Trim button 5 right - rotate left Forearm");
    //     LeftForearmPositionMin();

    //     break;

            // Finger button
    case (CSWI - ButtonDeadband)...(CSWI + ButtonDeadband):
        // Serial.println("Finger button L CLick");
        CheckButtonPressTR2(FingerButtonLClick, FingerButtonLClickB);
        CheckLongPressTR2(FingerButtonLClickLongClick, 0);
    //    FingerButtonLClick();

        break;
    // Finger button
    case (CTSWJ - ButtonDeadband)...(CTSWJ + ButtonDeadband):
        // Serial.println("Finger Button R click");
        CheckButtonPressTR2(FingerButtonRClick, FingerButtonRClickB);
        CheckLongPressTR2(FingerButtonRClickLongClick, 0);
    //    FingerButtonLClick();

        break;
    }
    }
    if (GetRCBottangoMode() == ANIMATION_MODE)
    { 
       switch (GetButtonBank2State())
      { 
        case (trimcenter - ButtonDeadband)...(trimcenter + ButtonDeadband):
        ButtonPressedTR = 0;
        ButtonLongPressedTR = 0;
        ButtonCountTR = 0;
                //     Serial.println("debug here");
        break;
     
       // Debounced button check for animations
case (CTrim1L - ButtonDeadband) ... (CTrim1L + ButtonDeadband):
CheckButtonPressTR(Animation1, Animation1);
       //     Serial.println("debug here");
break;

case (CTrim1R - ButtonDeadband) ... (CTrim1R + ButtonDeadband):
CheckButtonPressTR(Animation2, Animation2);
        //     Serial.println("debug here");
break;

case (CTrim6L - ButtonDeadband) ... (CTrim6L + ButtonDeadband):
CheckButtonPressTR(Animation3, Animation3);
        //     Serial.println("debug here");
break;

case (CTrim6R - ButtonDeadband) ... (CTrim6R + ButtonDeadband):
CheckButtonPressTR(Animation4, Animation4);
        //     Serial.println("debug here");
break;

// case (CSWI - ButtonDeadband) ... (CSWI + ButtonDeadband):
// CheckButtonPressTR2(Animation5, Animation5);
//                 //     Serial.println("debug here");
// break;

// case (CTSWJ - ButtonDeadband) ... (CTSWJ + ButtonDeadband):
//   CheckButtonPressTR2(Animation6, Animation6);
//                 //     Serial.println("debug here");
// break;

case (CTrim4L - ButtonDeadband) ... (CTrim4L + ButtonDeadband):
CheckButtonPressTR(Animation11, Animation11);
        //     Serial.println("debug here");
break;

case (CTrim4R - ButtonDeadband) ... (CTrim4R + ButtonDeadband):
CheckButtonPressTR(Animation12, Animation12);
        //     Serial.println("debug here");
break;

case (CTrim5L - ButtonDeadband) ... (CTrim5L + ButtonDeadband):
CheckButtonPressTR(Animation9, Animation9);
        //     Serial.println("debug here");
break;

case (CTrim5R - ButtonDeadband) ... (CTrim5R + ButtonDeadband):
CheckButtonPressTR(Animation10, Animation10);
        //     Serial.println("debug here");
break;

}

switch (GetButtonBank3State())
{ 
case (trimcenter - ButtonDeadband)...(trimcenter + ButtonDeadband):
ButtonPressedTR2 = 0;
ButtonLongPressedTR2 = 0;
ButtonCountTR2 = 0;

//     Serial.println("Buttons Off");
break;

// Debounced button check for animations
case (CTrim2U - ButtonDeadband)...(CTrim2U + ButtonDeadband):
CheckButtonPressTR2(Animation5, Animation5);
        //     Serial.println("debug here");
break;

case (CTrim2D - ButtonDeadband) ... (CTrim2D + ButtonDeadband):
CheckButtonPressTR2(Animation6, Animation6);
        //     Serial.println("debug here");
break;

case (CTrim3U - ButtonDeadband) ... (CTrim3U + ButtonDeadband):
CheckButtonPressTR2(Animation7, Animation7);
        //     Serial.println("debug here");
break;

case (CTrim3D - ButtonDeadband) ... (CTrim3D + ButtonDeadband):
CheckButtonPressTR2(Animation8, Animation8);
        //     Serial.println("debug here");
break;
    // case (CTrim4L - ButtonDeadband) ... (CTrim4L + ButtonDeadband):
    // 
    //     CheckButtonPressTR(Animation7, Animation7);
    //                 //     Serial.println("debug here");
        // break;
    // case (CTrim4R - ButtonDeadband) ... (CTrim4R + ButtonDeadband):
    // 
    //     CheckButtonPressTR(Animation8, Animation8);
    //                 //     Serial.println("debug here");
        // break;
    // case (CTrim5L - ButtonDeadband) ... (CTrim5L + ButtonDeadband):
    // 
    //     CheckButtonPressTR(Animation9, Animation9);
    //                 //     Serial.println("debug here");
        // break;
    // case (CTrim5R - ButtonDeadband) ... (CTrim5R + ButtonDeadband):
    // 
    //     CheckButtonPressTR(Animation10, Animation10);
    //                 //     Serial.println("debug here");
        // break;
   }
  }
}



void Animation1() { animationIndex = 1; Serial.println("Animation 1 triggered, animationIndex = 1"); }
void Animation2() { animationIndex = 2; Serial.println("Animation 2 triggered, animationIndex = 2"); }
void Animation3() { animationIndex = 3; Serial.println("Animation 3 triggered, animationIndex = 3"); }
void Animation4() { animationIndex = 4; Serial.println("Animation 4 triggered, animationIndex = 4"); }
void Animation5() { animationIndex = 5; Serial.println("Animation 5 triggered, animationIndex = 5"); }
void Animation6() { animationIndex = 6; Serial.println("Animation 6 triggered, animationIndex = 6"); }
void Animation7() { animationIndex = 7; Serial.println("Animation 7 triggered, animationIndex = 7"); }
void Animation8() { animationIndex = 8; Serial.println("Animation 8 triggered, animationIndex = 8"); }
void Animation9() { animationIndex = 9; Serial.println("Animation 9 triggered, animationIndex = 9"); }
void Animation10() { animationIndex = 10; Serial.println("Animation 10 triggered, animationIndex = 10"); }
void Animation11() { animationIndex = 11; Serial.println("Animation 11 triggered, animationIndex = 11"); }
void Animation12() { animationIndex = 12; Serial.println("Animation 12 triggered, animationIndex = 12"); }
void Animation14() { animationIndex = 14; Serial.println("Animation setup triggered, animationIndex = 14"); }

// Function to process trim bank 3 switch inputs and trigger animations
// void ProcessTrim1Switches()
// { if (GetRCBottangoMode() == ANIMATION_MODE)
//    {
//     int trim1Value = ChannelData(cTRIMSWITCH);  // Read trim switch value

//     if (trim1Value != previousTrim1)  // Only update if changed
//     {
//         Serial.print("Trim 1 value: ");
//         Serial.println(trim1Value);
//         previousTrim1 = trim1Value;  // Update previous value
//     }
//     else
//     {
//         return; // Exit early if no change
//     }

 
// // Function to process trim bank 3 switch inputs and trigger animations
// void ProcessTrimBackSwitches()
// {  if (GetRCBottangoMode() == ANIMATION_MODE)
//    {
//     int trim2Value = ChannelData(cTRIMBACKSWITCH);  // Read trim switch value

//     if (trim2Value != previousTrim2)  // Only update if changed
//     {
//         Serial.print("Trim 2 value: ");
//         Serial.println(trim2Value);
//         previousTrim2 = trim2Value;  // Update previous value
//     }
//     else
//     {
//         return; // Exit early if no change
//     }


// }

    void Button1Click()
    {

        // scheduler.stopall();
        Serial.println("Button 1 Click");
        SetAutoPlayWav(false);
        TxAudioAllStop();
        // BottangoCore::commandStreamProvider->stop();
    }

    void Button1ClickB()
    {

        // scheduler.stopall();

        SetAutoPlayWav(false);
        TxAudioAllStop();
    }

    void Button1LongClick()
    {

        Serial.println("Button A1 Long Click");
    }

    void Button2Click()
    {
        Serial.println("Button 2 Click");
        // gAutoPlayWav = 0;
        //**    gCurrentWavTrack = random(1, MaxWavTracks + 1);
        PlayWavA(1032); // Imperial March
    }

    void Button2ClickB()
    {
        // PlayNextWavA();
    }

    void Button2LongClick()
    {
       PlayWavA(1031); 

    }

    void Button3Click()
    {
        Serial.println("Button 3 Click");
        PlayWavA(2001); // I am c3p0
    }

    void Button3ClickB()
    {
        // Play Imperial March
        // SetCurrentTrack(2);
        // PlayWavA(2);
 
    }
    void Button3LongClick()
    {
         PlayNextWavA();

    }

    void Button4Click()
    {
        Serial.println("Button 4 Click");
        PlayNextWav5000A();
    }

    void Button4ClickB()
    {
    }
    void Button4LongClick()
    {

    }
    void Button5Click()
    {
        Serial.println("Button 5 Click");
        PlayWavA(3001); // Model Notes
    }

    void Button5ClickB()
    {
    }
    void Button5LongClick()
    {

    }
    void Button6Click()
    {
        Serial.println("Button 6 Click");
        PlayNextWavA();
    }

    void Button6ClickB()
    {
    }

    void Button6LongClick()
    {
        SetAutoPlayWav(true);
    }

    void Button7Click()
    {
        Serial.println("Button 7 Click");
        SetAutoPlayVocal();
    }

    void Button7ClickB()
    {
    }

    void Button7LongClick()
    {
        // SetAutoPlayWav(true);
    }
void Button8Click()
{
    Serial.println("Button 8 Click");

    const int FIRST_TRACK = 2001;
    const int LAST_TRACK  = 2032;

    static int currentTrack = FIRST_TRACK;

    // Stop any currently playing B channel audio immediately
    TxAudioAllStop();   // <-- if your audio lib uses a different stop call, tell me

    Serial.print("Play sequential song: ");
    Serial.println(currentTrack);

    PlayWavB(currentTrack);

    // Advance to next track
    currentTrack++;
    if (currentTrack > LAST_TRACK)
        currentTrack = FIRST_TRACK;
}
    // void Button8Click()
    // {
    //     Serial.println("Button 8 Click");

    //     int songNum = random(2001, 2032);
    //     Serial.print("play Rando song: ");
    //     Serial.print(songNum);

    //     PlayWavB(songNum);
    // }

    void Button8ClickB()
    {
    }

    void Button8LongClick()
    {
        // SetAutoPlayWav(true);
    }

    // bank B trim button functions
    void Trim6LClick()
    {
        Serial.println("trim button 6 push left");
        ;
    }

    void Trim6LClickB()
    {
    }

    void Trim6LClickLongClick()
    {
    }

    void Trim6RClick()
    {
        Serial.println("trim button 6 push right");
    }

    void Trim6RClickB()
    {
    }

    void Trim6RClickLongClick()
    {
    }

    void Trim5LClick()
    {
        Serial.println("trim button 5 push left");
        ;
    }

    void Trim5LClickB()
    {
    }

    void Trim5LClickLongClick()
    {
    }

    void Trim5RClick()
    {
        Serial.println("trim button 5 push right");
        ;
    }

    void Trim5RClickB()
    {
    }

    void Trim5RClickLongClick()
    {
    }

    void Trim1LClick()
    {
        Serial.println("trim button 1 push left");
        ;
    }

    void Trim1LClickB()
    {
    }

    void Trim1LClickLongClick()
    {
    }

    void Trim1RClick()
    {
        Serial.println("trim button 1 push right");
    }

    void Trim1RClickB()
    {
    }

    void Trim1RClickLongClick()
    {
    }

    void Trim4LClick()
    {
        Serial.println("trim button 4 push left");
        ;
    }

    void Trim4LClickB()
    {
    }

    void Trim4LClickLongClick()
    {
    }

    void Trim4RClick()
    {
        Serial.println("trim button 4 push right");
        ;
    }

    void Trim4RClickB()
    {
    }

  void Trim4RClickLongClick()
    {
    }  
  void FingerButtonLClick()
    {
        PlayWavA(2025); 
    }
  void FingerButtonRClick()
    {
         PlayWavA(2008);       
    }
    void FingerButtonLClickB()
    {

    }
    void FingerButtonLClickLongClick()
    {
         PlayWavA(2020);  
    }

    void FingerButtonRClickB()
    {

    }
    void FingerButtonRClickLongClick()
    {
         PlayWavA(2004);  
    }
   void CheckRCBottangoMode()
    {
        // Serial.println("check head mode");
        // Check button bank presses
        switch (ChannelData(cRCBOTTANGOMODESW))
        {

            // RC Control Mode
        case (SBUSMIN - ButtonDeadband)...(SBUSMIN + ButtonDeadband):

            if (GetRCBottangoMode() != RC_MODE)
            {
                SetRCBottangoMode_RC();
                Serial.println("RC_MODE ");
            }
            break;



            // Bottango Animation Mode
        case (SBUSMAX - ButtonDeadband)...(SBUSMAX + ButtonDeadband):

            if (GetRCBottangoMode() != ANIMATION_MODE)
            {
                SetRCBottangoMode_Animation();
                Serial.println("ANIMATION_MODE");
            }
            break;
        default:
            break;
        }
    }

    void CheckHeadMode()
    {
        // Serial.println("check head mode");
        // Check button bank presses
        switch (ChannelData(cCONTROLMODE))
        {

            // Head Idle
        case (SBUSMIN - ButtonDeadband)...(SBUSMIN + ButtonDeadband):

            if (GetHeadMode() != HEADIDLE)
            {
                SetHeadRunModeIdle();
                Serial.println("Control Mode Idle");
            }
            break;

            // Head Auto
        case (SBUSCENTER - ButtonDeadband)...(SBUSCENTER + ButtonDeadband):
            if (GetHeadMode() != HEADAUTO)
            {
                SetHeadRunModeAuto();
                Serial.println("Control Mode Auto");
            }
            break;

            // Head Control
        case (SBUSMAX - ButtonDeadband)...(SBUSMAX + ButtonDeadband):

            if (GetHeadMode() != HEADCONTROL)
            {
                SetHeadRunModeControl();
                Serial.println("Control Mode Control");
            }
            break;
        default:
            break;
        }
    }

    // void CheckShoulderMode()
    // {
    //     // Serial.println("check head mode");
    //     // Check button bank presses
    //     switch (ChannelData(cSHOULDERMODESWITCH))
    //     {

    //         // Shoulder Left Mode
    //     case (SBUSMIN - ButtonDeadband)...(SBUSMIN + ButtonDeadband):

    //         if (GetShoulderMode() != SHOULDERSWITCHLEFT)
    //         {
    //             SetShoulderMode(SHOULDERSWITCHLEFT);
    //             Serial.println("Shoulder Mode Left");
    //         }
    //         break;

    //         // Shoulder Right Mode
    //     case (SBUSCENTER - ButtonDeadband)...(SBUSCENTER + ButtonDeadband):
    //         if (GetShoulderMode() != SHOULDERSWITCHRIGHT)
    //         {
    //             SetShoulderMode(SHOULDERSWITCHRIGHT);
    //             Serial.println("Shoulder Mode Right");
    //         }
    //         break;

    //         // Shoulder Both Mode
    //     case (SBUSMAX - ButtonDeadband)...(SBUSMAX + ButtonDeadband):

    //         if (GetShoulderMode() != SHOULDERSWITCHBOTH)
    //         {
    //             SetShoulderMode(SHOULDERSWITCHBOTH);
    //             Serial.println("Shoulder Mode Both");
    //         }
    //         break;
    //     default:
    //         break;
    //     }
    // }
void CheckMotorHome()
{
    static bool lastHomeHigh = false;

    bool homeHigh = (ChannelData(cHOMEMOTORS) > SBUSCENTER);

    // Rising edge: OFF -> ON
    if (homeHigh && !lastHomeHigh)
    {
        if (GetRCBottangoMode() == RC_MODE)
        {
            ServoAttach(0);
        }
        else if (GetRCBottangoMode() == ANIMATION_MODE)
        {
            Animation14();
        }
    }

    lastHomeHigh = homeHigh;
}

    // void CheckMotorHome()
    // {
    //    if (GetRCBottangoMode() == RC_MODE)        
    //      if (ChannelData(cHOMEMOTORS) > SBUSCENTER)
    //     {
    //         ServoAttach(0);
    //         // TxServoAttach(HEAD_ID, 0);
    //         // TxServoAttach(SHOULDER_ID, 0);
    //         // TxServoAttach(ARM_ID, 0);
    //         // TxServoAttach(TORSO_ID, 0);
    //         // TxServoAttach(HEAD_ID, 0);
    //     }
    //            if (GetRCBottangoMode() == ANIMATION_MODE)        
    //      if (ChannelData(cHOMEMOTORS) > SBUSCENTER)
    //     {
    //         Animation14();
    //         // TxServoAttach(HEAD_ID, 0);
    //         // TxServoAttach(SHOULDER_ID, 0);
    //         // TxServoAttach(ARM_ID, 0);
    //         // TxServoAttach(TORSO_ID, 0);
    //         // TxServoAttach(HEAD_ID, 0);
    //     }
    // }

    void CheckVoiceAudioMode()
    {

        // Check button bank presses
        switch (ChannelData(cPITCHSW))
        {

            // Voice Audio Off
        case (SBUSMIN - ButtonDeadband)...(SBUSMIN + ButtonDeadband):
            // Serial.println("audio voice off");
            if (gVoicePitchMode != PITCHOFF)
            {
                SetAudioVoiceOff();
                gVoicePitchMode = PITCHOFF;
            }
            break;

        // Voice Audio On pitch 1.4
        case (SBUSCENTER - ButtonDeadband)...(SBUSCENTER + ButtonDeadband):
            if (gVoicePitchMode != PITCHHIGH)
            {
                SetAudioVoiceOn();
                SetAudioPitch(100); // 1.4f
                gVoicePitchMode = PITCHHIGH;
            }
            break;

            // Voice Audio On
        case (SBUSMAX - ButtonDeadband)...(SBUSMAX + ButtonDeadband):
            if (gVoicePitchMode != PITCHLOW)
            {
                SetAudioVoiceOn();
                SetAudioPitch(80); // 0.7f
                gVoicePitchMode = PITCHLOW;
            }
            break;

        default:
            break;
        }
    }
    // void handleAnimationControl()
    // {
    //     // // Check for specific trim switch inputs to trigger animations
    //     // int trimValue = ChannelData(cTRIMBACKSWITCH); // Assuming cTRIMSWITCH is defined for your trim inputs

    //     // // Serial.print("Trim value: ");
    //     // // Serial.println(trimValue);

    //     // // Trigger animations based on trim switch values
    //     // if (trimValue >= (CTrim2U - ButtonDeadband) && trimValue <= (CTrim2U + ButtonDeadband))
    //     // {
    //     //     triggerAnimation(1, false); // Animation ID for CTrim2U
    //     //     Serial.println("Triggered Animation for CTrim2U");
    //     //     Serial.println(trimValue);
    //     // }
    //     // else if (trimValue >= (CTrim2D - ButtonDeadband) && trimValue <= (CTrim2D + ButtonDeadband))
    //     // {
    //     //     triggerAnimation(2, false); // Animation ID for CTrim2D
    //     //     Serial.println("Triggered Animation for CTrim2D");
    //     //     Serial.println(trimValue);
    //     // }
    //     // else if (trimValue >= (CTrim3U - ButtonDeadband) && trimValue <= (CTrim3U + ButtonDeadband))
    //     // {
    //     //     triggerAnimation(3, false); // Animation ID for CTrim3U
    //     //     Serial.println("Triggered Animation for CTrim3U");
    //     //     Serial.println(trimValue);
    //     // }
    //     // else if (trimValue >= (CTrim3D - ButtonDeadband) && trimValue <= (CTrim3D + ButtonDeadband))
    //     // {
    //     //     triggerAnimation(4, false); // Animation ID for CTrim3D
    //     //     Serial.println("Triggered Animation for CTrim3D");
    //     //     Serial.println(trimValue);
    //     // }
    //     // else if (trimValue >= (CSWI - ButtonDeadband) && trimValue <= (CSWI + ButtonDeadband))
    //     // {
    //     //     triggerAnimation(5, false); // Animation ID for CSWI
    //     //     Serial.println("Triggered Animation for CSWI");
    //     //     Serial.println(trimValue);
    //     // }
    //     // else if (trimValue >= (CTSWJ - ButtonDeadband) && trimValue <= (CTSWJ + ButtonDeadband))
    //     // {
    //     //     triggerAnimation(6, false); // Animation ID for CTSWJ
    //     //     Serial.println("Triggered Animation for CTSWJ");
    //     //     Serial.println(trimValue);
    //     // }
    //     // // Update and play animations
    //     // GeneratedCommandStreams::updatePlayStatus();
    // }











// // Radio Event triggering
// // Mapping radio switches etc. to functions.
// //  Requires Scheduler Library.  Modified Library by TZ

// #include <src/Events.hpp>
// #include <src/MotionControl.hpp>
// #include <src/GeneratedCodeanimations.h>
// #include <src/BottangoCore.h>

// uint8_t RunMode = 0;
// uint8_t gVoicePitchMode = PITCHOFF;
// uint16_t ButtonState = 0;
// bool ButtonPressed = false;   // Button Pressed indicator for button bank
// bool ButtonPressedTR = false; // Button Pressed indicator for Trim Rud
// bool ButtonPressedTR2 = false; // Button Pressed indicator for Trim2 Rud
// bool ButtonLongPressed = false;
// bool ButtonLongPressedTR = false;
// bool ButtonLongPressedTR2 = false;
// byte animationIndex = 0;

// int ButtonCount = 0;   // Counter for Button Bank
// int ButtonCountTR = 0; // Counter for Trim Rud
// int ButtonCountTR2 = 0; // Counter for Trim2 Rud

// // int previousTrim1 = 0;
// // int previousTrim2 = 0;

// elapsedMillis ElapsedEvent = 0;

// // Scheduler scheduler = Scheduler();      //create a scheduler
// // typedef void *func;
// // void CheckLongPress(void (*)());

// void ProcessEventsLoop()
// {

//     if (ElapsedEvent > 100)
//     {

//         ProcessButtons();
//         UpdateVolume();
//         CheckHeadMode();
//         CheckMotorHome();
//         // ProcessTrim1Switches();
//         // ProcessTrimBackSwitches();
//         CheckVoiceAudioMode();
//         // CheckShoulderMode();
//         //  CheckPitchAudioMode();
//         ElapsedEvent = 0;
//     }
//     // scheduler.update();                 //update the scheduler, maybe it is time to execute a function?
// }

// // Function to read the button channel, and assign a button # to it based on the nominal value + a buffer either side.
// uint16_t GetButtonState()

// {
//     //  Serial.println(ChannelData(cBUTTONSWITCH));
//     return ChannelData(cBUTTONSWITCH);
// }
// // Function to read the button channel, and assign a button # to it based on the nominal value + a buffer either side.
// uint16_t GetButtonBank2State()

// {
//     //  Serial.println(ChannelData(cBUTTONSWITCH));
//     return ChannelData(cTRIMSWITCH);
// }

// // Function to read the button channel, and assign a button # to it based on the nominal value + a buffer either side.
// uint16_t GetButtonBank3State()

// {
//     //  Serial.println(ChannelData(cBUTTONSWITCH));
//     return ChannelData(cTRIMBACKSWITCH);
// }

// // Function to check the button, and active Bank A or Bank B function
// void CheckButtonPress(void (*funcA)(), void (*funcB)())
// {
//     Serial.println("CheckButtonPress");
//     // Check if first time button pressed.
//     if (!ButtonPressed)
//     {
//         // Check which bank to activate
//         // if (ChannelData(cHOMEMOTORS) < 1000)
//         funcA();
//         //  else
//         //     funcB();
//         Serial.println("ButtonPress");
//         ButtonPressed = true;
//     }
 
//     // if (abs(cBUTTONSWITCH - ButtonOff) <= 5)
    
//     // {
//     //     Serial.println("Button Released, Resetting TR2");
//     //     ButtonPressed = false;
//     // }
// }

// void CheckLongPress(void (*func)(), bool continuous)
// {

//     ButtonCount++;
//     if (ButtonCount >= LongPressCount && !ButtonLongPressed)
//     {
//         func();
//         if (!continuous)
//             ButtonLongPressed = true;
//     }
// }
// void CheckButtonPressTR(void (*funcA)(), void (*funcB)())
// {
//     Serial.println("CheckButtonPressTR");
//     // Check if first time button pressed.
//     if (!ButtonPressedTR)
//     {
//         // Check which bank to activate
//         // if (ChannelData(cHOMEMOTORS) < 1000)
//         funcA();
//         //  else
//         //     funcB();
//         Serial.println("ButtonPressTR");
//         ButtonPressedTR = true;
//     }
  
//     //     if (abs(cTRIMSWITCH - trimcenter) <= 5)
//     // {
//     //     Serial.println("Button Released, Resetting TR2");
//     //     ButtonPressedTR = false;
//     // }
// }

// void CheckLongPressTR(void (*func)(), bool continuous)
// {

//     ButtonCountTR++;
//     if (ButtonCountTR >= LongPressCount && !ButtonLongPressedTR)
//     {
//         func();
//         if (!continuous)
//             ButtonLongPressedTR = true;
//     }
// }
// void CheckButtonPressTR2(void (*funcA)(), void (*funcB)())
// {
//     Serial.println("CheckButtonPressTR2");
//     // Check if first time button pressed.
//     if (!ButtonPressedTR2)
//     {
//         // Check which bank to activate
//         // if (ChannelData(cHOMEMOTORS) < 1000)
//         funcA();
//         //  else
//         //     funcB();
//         Serial.println("ButtonPressTR2");
//         ButtonPressedTR2 = 1;
//     }
//         // Reset when trim switch returns to neutral
//     // if (abs(cTRIMBACKSWITCH - trimcenter2) <= 5)
//     // {
//     //     Serial.println("Button Released, Resetting TR2");
//     //     ButtonPressedTR2 = false;
//     // }
// }

// void CheckLongPressTR2(void (*func)(), bool continuous)
// {

//     ButtonCountTR2++;
//     if (ButtonCountTR2 >= LongPressCount && !ButtonLongPressedTR2)
//     {
//         func();
//         if (!continuous)
//             ButtonLongPressedTR2 = true;
//     }
// }

// // Function to determine which button was pressed and run the function associated with the click
// void ProcessButtons()
// {
//     // Check button bank presses
//     switch (GetButtonState())
//     {

//         // Buttons Off
//     case (ButtonOff - ButtonDeadband)...(ButtonOff + ButtonDeadband):
//         ButtonPressed = 0;
//         ButtonLongPressed = 0;
//         ButtonCount = 0;
//         //     Serial.println("Buttons Off");
//         break;

//         // Button 1
//     case (CButton1 - ButtonDeadband)...(CButton1 + ButtonDeadband):
//         CheckButtonPress(Button1Click, Button1ClickB);
//         CheckLongPress(Button1LongClick, 0);
//         break;

//         // Button 2
//     case (CButton2 - ButtonDeadband)...(CButton2 + ButtonDeadband):
//         CheckButtonPress(Button2Click, Button2ClickB);
//         CheckLongPress(Button2LongClick, 0);
//         break;

//         // Button 3
//     case (CButton3 - ButtonDeadband)...(CButton3 + ButtonDeadband):
//         CheckButtonPress(Button3Click, Button3ClickB);
//         CheckLongPress(Button3LongClick, 0);
//         break;

//         // Button 4
//     case (CButton4 - ButtonDeadband)...(CButton4 + ButtonDeadband):
//         CheckButtonPress(Button4Click, Button4ClickB);
//         CheckLongPress(Button4LongClick, 0);
//         break;

//         // Button 5
//     case (CButton5 - ButtonDeadband)...(CButton5 + ButtonDeadband):
//         CheckButtonPress(Button5Click, Button5ClickB);
//         CheckLongPress(Button5LongClick, 0);
//         break;

//         // Button 6
//     case (CButton6 - ButtonDeadband)...(CButton6 + ButtonDeadband):
//         CheckButtonPress(Button6Click, Button6ClickB);
//         CheckLongPress(Button6LongClick, 0);
//         break;
//         // Button 7
//     case (CButton7 - ButtonDeadband)...(CButton7 + ButtonDeadband):
//         CheckButtonPress(Button7Click, Button7ClickB);
//         CheckLongPress(Button7LongClick, 0);
//         break;

//     // Button 8
//     case (CButton8 - ButtonDeadband)...(CButton8 + ButtonDeadband):
//         CheckButtonPress(Button8Click, Button8ClickB);
//         CheckLongPress(Button8LongClick, 0);
//         break;
//     }
//     // button check for bank 2 buttons trim switches

//    if (GetRCBottangoMode == RC_MODE)
//    { switch (GetButtonBank2State())
//     {
//     case (trimcenter - ButtonDeadband)...(trimcenter + ButtonDeadband):
//         ButtonPressedTR = 0;
//         ButtonLongPressedTR = 0;
//         ButtonCountTR = 0;
//         // ElbowCenter();         // Reset both elbows to mid if no trim switch is pressed
//         // Serial.println("Buttons Off");
//         break;

//         // Trim slider #6 left push
//     case (CTrim6L - ButtonDeadband)...(CTrim6L + ButtonDeadband):
//         CheckButtonPressTR(Trim6LClick, Trim6LClickB);
//         CheckLongPressTR(Trim6LClickLongClick, 0);
//         break;

//         // Trim slider #6 right push
//     case (CTrim6R - ButtonDeadband)...(CTrim6R + ButtonDeadband):
//         CheckButtonPressTR(Trim6RClick, Trim6RClickB);
//         CheckLongPressTR(Trim6RClickLongClick, 0);
//         break;

//         // Trim slider #5 left push
//     case (CTrim5L - ButtonDeadband)...(CTrim5L + ButtonDeadband):
//         CheckButtonPressTR(Trim5LClick, Trim5LClickB);
//         CheckLongPressTR(Trim5LClickLongClick, 0);
//         break;
//         // Trim slider #5 right push
//     case (CTrim5R - ButtonDeadband)...(CTrim5R + ButtonDeadband):
//         CheckButtonPressTR(Trim5RClick, Trim5RClickB);
//         CheckLongPressTR(Trim5LClickLongClick, 0);
//         break;


// //     // Trim slider #1 left push - bend right elbow
// //     case (CTrim1L - ButtonDeadband)...(CTrim1L + ButtonDeadband):
// //         // Serial.println("Trim button 1 left - bend right elbow");
// //         RightElbowPositionMax();
// //         break;

// //     // Trim slider #1 right push - extend right elbow
// //     case (CTrim1R - ButtonDeadband)...(CTrim1R + ButtonDeadband):
// //         // Serial.println("Trim button 1 right - extend right elbow");
// //         RightElbowPositionMin();

// //         break;

// //     // Trim slider #5 left push - bend left elbow
// //     case (CTrim4L - ButtonDeadband)...(CTrim4L + ButtonDeadband):
// //         // Serial.println("Trim button 5 left - bend left elbow");
// //         LeftElbowPositionMax();

// //         break;

// //     // Trim slider #5 right push - extend left elbow
// //     case (CTrim4R - ButtonDeadband)...(CTrim4R + ButtonDeadband):
// //         // Serial.println("Trim button 5 right - extend left elbow");
// //         LeftElbowPositionMin();

// //         break;
//     }
//    }
//      if (GetRCBottangoMode() == RC_MODE)
//     {    switch (GetButtonBank3State())
//     { 
//         case (trimcenter2 - ButtonDeadband)...(trimcenter2 + ButtonDeadband):
//         ButtonPressedTR2 = 0;
//         ButtonLongPressedTR2 = 0;
//         ButtonCountTR2 = 0;
//         // ForearmCenter();       // Reset both Forearms to mid if no trim switch is pressed
//         //     Serial.println("Buttons Off");
//         break;
        
//     //     // Trim slider #2 left push - bend right Forearm
//     // case (CTrim2U - ButtonDeadband)...(CTrim2U + ButtonDeadband):
//     //     // Serial.println("Trim button 1 left - rotate right Forearm");
//     //     RightForearmPositionMax();
//     //     break;

//     // // Trim slider #2 right push - extend right Forearm
//     // case (CTrim2D - ButtonDeadband)...(CTrim2D + ButtonDeadband):
//     //     // Serial.println("Trim button 1 right - rotate right Forearm");
//     //     RightForearmPositionMin();

//     //     break;

//     // // Trim slider #3 left push - bend left Forearm
//     // case (CTrim3U - ButtonDeadband)...(CTrim3U + ButtonDeadband):
//     //     // Serial.println("Trim button 5 left - rotate left Forearm");
//     //     LeftForearmPositionMax();

//     //     break;

//     // // Trim slider #5 right push - extend left Forearm
//     // case (CTrim3D - ButtonDeadband)...(CTrim3D + ButtonDeadband):
//     //     // Serial.println("Trim button 5 right - rotate left Forearm");
//     //     LeftForearmPositionMin();

//     //     break;

//             // Finger button
//     case (CSWI - ButtonDeadband)...(CSWI + ButtonDeadband):
//         // Serial.println("Finger button L CLick");
//         CheckButtonPressTR2(FingerButtonLClick, FingerButtonLClickB);
//         CheckLongPressTR2(FingerButtonLClickLongClick, 0);
//     //    FingerButtonLClick();

//         break;
//     // Finger button
//     case (CTSWJ - ButtonDeadband)...(CTSWJ + ButtonDeadband):
//         // Serial.println("Finger Button R click");
//         CheckButtonPressTR2(FingerButtonRClick, FingerButtonRClickB);
//         CheckLongPressTR2(FingerButtonRClickLongClick, 0);
//     //    FingerButtonLClick();

//         break;
//     }
//     }
//     if (GetRCBottangoMode() == ANIMATION_MODE)
//     { 
//        switch (GetButtonBank2State())
//       { 
//         case (trimcenter - ButtonDeadband)...(trimcenter + ButtonDeadband):
//         ButtonPressedTR = 0;
//         ButtonLongPressedTR = 0;
//         ButtonCountTR = 0;
//                 //     Serial.println("debug here");
//         break;
     
//        // Debounced button check for animations
// case (CTrim1L - ButtonDeadband) ... (CTrim1L + ButtonDeadband):
// CheckButtonPressTR(Animation1, Animation1);
//        //     Serial.println("debug here");
// break;

// case (CTrim1R - ButtonDeadband) ... (CTrim1R + ButtonDeadband):
// CheckButtonPressTR(Animation2, Animation2);
//         //     Serial.println("debug here");
// break;

// case (CTrim6L - ButtonDeadband) ... (CTrim6L + ButtonDeadband):
// CheckButtonPressTR(Animation3, Animation3);
//         //     Serial.println("debug here");
// break;

// case (CTrim6R - ButtonDeadband) ... (CTrim6R + ButtonDeadband):
// CheckButtonPressTR(Animation4, Animation4);
//         //     Serial.println("debug here");
// break;

// // case (CSWI - ButtonDeadband) ... (CSWI + ButtonDeadband):
// // CheckButtonPressTR2(Animation5, Animation5);
// //                 //     Serial.println("debug here");
// // break;

// // case (CTSWJ - ButtonDeadband) ... (CTSWJ + ButtonDeadband):
// //   CheckButtonPressTR2(Animation6, Animation6);
// //                 //     Serial.println("debug here");
// // break;

// case (CTrim4L - ButtonDeadband) ... (CTrim4L + ButtonDeadband):
// CheckButtonPressTR(Animation11, Animation11);
//         //     Serial.println("debug here");
// break;

// case (CTrim4R - ButtonDeadband) ... (CTrim4R + ButtonDeadband):
// CheckButtonPressTR(Animation12, Animation12);
//         //     Serial.println("debug here");
// break;

// case (CTrim5L - ButtonDeadband) ... (CTrim5L + ButtonDeadband):
// CheckButtonPressTR(Animation9, Animation9);
//         //     Serial.println("debug here");
// break;

// case (CTrim5R - ButtonDeadband) ... (CTrim5R + ButtonDeadband):
// CheckButtonPressTR(Animation10, Animation10);
//         //     Serial.println("debug here");
// break;

// }

// switch (GetButtonBank3State())
// { 
// case (trimcenter - ButtonDeadband)...(trimcenter + ButtonDeadband):
// ButtonPressedTR2 = 0;
// ButtonLongPressedTR2 = 0;
// ButtonCountTR2 = 0;

// //     Serial.println("Buttons Off");
// break;

// // Debounced button check for animations
// case (CTrim2U - ButtonDeadband)...(CTrim2U + ButtonDeadband):
// CheckButtonPressTR2(Animation5, Animation5);
//         //     Serial.println("debug here");
// break;

// case (CTrim2D - ButtonDeadband) ... (CTrim2D + ButtonDeadband):
// CheckButtonPressTR2(Animation6, Animation6);
//         //     Serial.println("debug here");
// break;

// case (CTrim3U - ButtonDeadband) ... (CTrim3U + ButtonDeadband):
// CheckButtonPressTR2(Animation7, Animation7);
//         //     Serial.println("debug here");
// break;

// case (CTrim3D - ButtonDeadband) ... (CTrim3D + ButtonDeadband):
// CheckButtonPressTR2(Animation8, Animation8);
//         //     Serial.println("debug here");
// break;
//     // case (CTrim4L - ButtonDeadband) ... (CTrim4L + ButtonDeadband):
//     // 
//     //     CheckButtonPressTR(Animation7, Animation7);
//     //                 //     Serial.println("debug here");
//         // break;
//     // case (CTrim4R - ButtonDeadband) ... (CTrim4R + ButtonDeadband):
//     // 
//     //     CheckButtonPressTR(Animation8, Animation8);
//     //                 //     Serial.println("debug here");
//         // break;
//     // case (CTrim5L - ButtonDeadband) ... (CTrim5L + ButtonDeadband):
//     // 
//     //     CheckButtonPressTR(Animation9, Animation9);
//     //                 //     Serial.println("debug here");
//         // break;
//     // case (CTrim5R - ButtonDeadband) ... (CTrim5R + ButtonDeadband):
//     // 
//     //     CheckButtonPressTR(Animation10, Animation10);
//     //                 //     Serial.println("debug here");
//         // break;
//    }
//   }
// }



// void Animation1() { animationIndex = 1; Serial.println("Animation 1 triggered, animationIndex = 1"); }
// void Animation2() { animationIndex = 2; Serial.println("Animation 2 triggered, animationIndex = 2"); }
// void Animation3() { animationIndex = 3; Serial.println("Animation 3 triggered, animationIndex = 3"); }
// void Animation4() { animationIndex = 4; Serial.println("Animation 4 triggered, animationIndex = 4"); }
// void Animation5() { animationIndex = 5; Serial.println("Animation 5 triggered, animationIndex = 5"); }
// void Animation6() { animationIndex = 6; Serial.println("Animation 6 triggered, animationIndex = 6"); }
// void Animation7() { animationIndex = 7; Serial.println("Animation 7 triggered, animationIndex = 7"); }
// void Animation8() { animationIndex = 8; Serial.println("Animation 8 triggered, animationIndex = 8"); }
// void Animation9() { animationIndex = 9; Serial.println("Animation 9 triggered, animationIndex = 9"); }
// void Animation10() { animationIndex = 10; Serial.println("Animation 10 triggered, animationIndex = 10"); }
// void Animation11() { animationIndex = 11; Serial.println("Animation 11 triggered, animationIndex = 11"); }
// void Animation12() { animationIndex = 12; Serial.println("Animation 12 triggered, animationIndex = 12"); }
// void Animation14() { animationIndex = 14; Serial.println("Animation setup triggered, animationIndex = 14"); }

// // Function to process trim bank 3 switch inputs and trigger animations
// // void ProcessTrim1Switches()
// // { if (GetRCBottangoMode() == ANIMATION_MODE)
// //    {
// //     int trim1Value = ChannelData(cTRIMSWITCH);  // Read trim switch value

// //     if (trim1Value != previousTrim1)  // Only update if changed
// //     {
// //         Serial.print("Trim 1 value: ");
// //         Serial.println(trim1Value);
// //         previousTrim1 = trim1Value;  // Update previous value
// //     }
// //     else
// //     {
// //         return; // Exit early if no change
// //     }

 
// // // Function to process trim bank 3 switch inputs and trigger animations
// // void ProcessTrimBackSwitches()
// // {  if (GetRCBottangoMode() == ANIMATION_MODE)
// //    {
// //     int trim2Value = ChannelData(cTRIMBACKSWITCH);  // Read trim switch value

// //     if (trim2Value != previousTrim2)  // Only update if changed
// //     {
// //         Serial.print("Trim 2 value: ");
// //         Serial.println(trim2Value);
// //         previousTrim2 = trim2Value;  // Update previous value
// //     }
// //     else
// //     {
// //         return; // Exit early if no change
// //     }


// // }

//     void Button1Click()
//     {

//         // scheduler.stopall();
//         Serial.println("Button 1 Click");
//         SetAutoPlayWav(false);
//         TxAudioAllStop();
//         BottangoCore::commandStreamProvider->stop();
//     }

//     void Button1ClickB()
//     {

//         // scheduler.stopall();

//         SetAutoPlayWav(false);
//         TxAudioAllStop();
//     }

//     void Button1LongClick()
//     {

//         Serial.println("Button A1 Long Click");
//     }

//     void Button2Click()
//     {
//         Serial.println("Button 2 Click");
//         // gAutoPlayWav = 0;
//         //**    gCurrentWavTrack = random(1, MaxWavTracks + 1);
//         PlayWavA(1032); // Imperial March
//     }

//     void Button2ClickB()
//     {
//         // PlayNextWavA();
//     }

//     void Button2LongClick()
//     {
//        PlayWavA(1031); 

//     }

//     void Button3Click()
//     {
//         Serial.println("Button 3 Click");
//         PlayWavA(2001); // I am c3p0
//     }

//     void Button3ClickB()
//     {
//         // Play Imperial March
//         // SetCurrentTrack(2);
//         // PlayWavA(2);
 
//     }
//     void Button3LongClick()
//     {
//          PlayNextWavA();

//     }

//     void Button4Click()
//     {
//         Serial.println("Button 4 Click");
//         PlayNextWav5000A();
//     }

//     void Button4ClickB()
//     {
//     }
//     void Button4LongClick()
//     {

//     }
//     void Button5Click()
//     {
//         Serial.println("Button 5 Click");
//         PlayWavA(3001); // Model Notes
//     }

//     void Button5ClickB()
//     {
//     }
//     void Button5LongClick()
//     {

//     }
//     void Button6Click()
//     {
//         Serial.println("Button 6 Click");
//         PlayNextWavA();
//     }

//     void Button6ClickB()
//     {
//     }

//     void Button6LongClick()
//     {
//         SetAutoPlayWav(true);
//     }

//     void Button7Click()
//     {
//         Serial.println("Button 7 Click");
//         SetAutoPlayVocal();
//     }

//     void Button7ClickB()
//     {
//     }

//     void Button7LongClick()
//     {
//         // SetAutoPlayWav(true);
//     }

//     void Button8Click()
//     {
//         Serial.println("Button 8 Click");

//         int songNum = random(2001, 2032);
//         Serial.print("play Rando song: ");
//         Serial.print(songNum);

//         PlayWavB(songNum);
//     }

//     void Button8ClickB()
//     {
//     }

//     void Button8LongClick()
//     {
//         // SetAutoPlayWav(true);
//     }

//     // bank B trim button functions
//     void Trim6LClick()
//     {
//         Serial.println("trim button 6 push left");
//         ;
//     }

//     void Trim6LClickB()
//     {
//     }

//     void Trim6LClickLongClick()
//     {
//     }

//     void Trim6RClick()
//     {
//         Serial.println("trim button 6 push right");
//     }

//     void Trim6RClickB()
//     {
//     }

//     void Trim6RClickLongClick()
//     {
//     }

//     void Trim5LClick()
//     {
//         Serial.println("trim button 5 push left");
//         ;
//     }

//     void Trim5LClickB()
//     {
//     }

//     void Trim5LClickLongClick()
//     {
//     }

//     void Trim5RClick()
//     {
//         Serial.println("trim button 5 push right");
//         ;
//     }

//     void Trim5RClickB()
//     {
//     }

//     void Trim5RClickLongClick()
//     {
//     }

//     void Trim1LClick()
//     {
//         Serial.println("trim button 1 push left");
//         ;
//     }

//     void Trim1LClickB()
//     {
//     }

//     void Trim1LClickLongClick()
//     {
//     }

//     void Trim1RClick()
//     {
//         Serial.println("trim button 1 push right");
//     }

//     void Trim1RClickB()
//     {
//     }

//     void Trim1RClickLongClick()
//     {
//     }

//     void Trim4LClick()
//     {
//         Serial.println("trim button 4 push left");
//         ;
//     }

//     void Trim4LClickB()
//     {
//     }

//     void Trim4LClickLongClick()
//     {
//     }

//     void Trim4RClick()
//     {
//         Serial.println("trim button 4 push right");
//         ;
//     }

//     void Trim4RClickB()
//     {
//     }

//   void Trim4RClickLongClick()
//     {
//     }  
//   void FingerButtonLClick()
//     {
//         PlayWavA(2025); 
//     }
//   void FingerButtonRClick()
//     {
//          PlayWavA(2008);       
//     }
//     void FingerButtonLClickB()
//     {

//     }
//     void FingerButtonLClickLongClick()
//     {
//          PlayWavA(2020);  
//     }

//     void FingerButtonRClickB()
//     {

//     }
//     void FingerButtonRClickLongClick()
//     {
//          PlayWavA(2004);  
//     }
//    void CheckRCBottangoMode()
//     {
//         // Serial.println("check head mode");
//         // Check button bank presses
//         switch (ChannelData(cRCBOTTANGOMODESW))
//         {

//             // RC Control Mode
//         case (SBUSMIN - ButtonDeadband)...(SBUSMIN + ButtonDeadband):

//             if (GetRCBottangoMode() != RC_MODE)
//             {
//                 SetRCBottangoMode_RC();
//                 Serial.println("RC_MODE ");
//             }
//             break;



//             // Bottango Animation Mode
//         case (SBUSMAX - ButtonDeadband)...(SBUSMAX + ButtonDeadband):

//             if (GetRCBottangoMode() != ANIMATION_MODE)
//             {
//                 SetRCBottangoMode_Animation();
//                 Serial.println("ANIMATION_MODE");
//             }
//             break;
//         default:
//             break;
//         }
//     }

//     void CheckHeadMode()
//     {
//         // Serial.println("check head mode");
//         // Check button bank presses
//         switch (ChannelData(cCONTROLMODE))
//         {

//             // Head Idle
//         case (SBUSMIN - ButtonDeadband)...(SBUSMIN + ButtonDeadband):

//             if (GetHeadMode() != HEADIDLE)
//             {
//                 SetHeadRunModeIdle();
//                 Serial.println("Control Mode Idle");
//             }
//             break;

//             // Head Auto
//         case (SBUSCENTER - ButtonDeadband)...(SBUSCENTER + ButtonDeadband):
//             if (GetHeadMode() != HEADAUTO)
//             {
//                 SetHeadRunModeAuto();
//                 Serial.println("Control Mode Auto");
//             }
//             break;

//             // Head Control
//         case (SBUSMAX - ButtonDeadband)...(SBUSMAX + ButtonDeadband):

//             if (GetHeadMode() != HEADCONTROL)
//             {
//                 SetHeadRunModeControl();
//                 Serial.println("Control Mode Control");
//             }
//             break;
//         default:
//             break;
//         }
//     }

//     // void CheckShoulderMode()
//     // {
//     //     // Serial.println("check head mode");
//     //     // Check button bank presses
//     //     switch (ChannelData(cSHOULDERMODESWITCH))
//     //     {

//     //         // Shoulder Left Mode
//     //     case (SBUSMIN - ButtonDeadband)...(SBUSMIN + ButtonDeadband):

//     //         if (GetShoulderMode() != SHOULDERSWITCHLEFT)
//     //         {
//     //             SetShoulderMode(SHOULDERSWITCHLEFT);
//     //             Serial.println("Shoulder Mode Left");
//     //         }
//     //         break;

//     //         // Shoulder Right Mode
//     //     case (SBUSCENTER - ButtonDeadband)...(SBUSCENTER + ButtonDeadband):
//     //         if (GetShoulderMode() != SHOULDERSWITCHRIGHT)
//     //         {
//     //             SetShoulderMode(SHOULDERSWITCHRIGHT);
//     //             Serial.println("Shoulder Mode Right");
//     //         }
//     //         break;

//     //         // Shoulder Both Mode
//     //     case (SBUSMAX - ButtonDeadband)...(SBUSMAX + ButtonDeadband):

//     //         if (GetShoulderMode() != SHOULDERSWITCHBOTH)
//     //         {
//     //             SetShoulderMode(SHOULDERSWITCHBOTH);
//     //             Serial.println("Shoulder Mode Both");
//     //         }
//     //         break;
//     //     default:
//     //         break;
//     //     }
//     // }

//     void CheckMotorHome()
//     {
//        if (GetRCBottangoMode() == RC_MODE)        
//          if (ChannelData(cHOMEMOTORS) > SBUSCENTER)
//         {
//             ServoAttach(0);
//             // TxServoAttach(HEAD_ID, 0);
//             // TxServoAttach(SHOULDER_ID, 0);
//             // TxServoAttach(ARM_ID, 0);
//             // TxServoAttach(TORSO_ID, 0);
//             // TxServoAttach(HEAD_ID, 0);
//         }
//                if (GetRCBottangoMode() == ANIMATION_MODE)        
//          if (ChannelData(cHOMEMOTORS) > SBUSCENTER)
//         {
//             Animation14();
//             // TxServoAttach(HEAD_ID, 0);
//             // TxServoAttach(SHOULDER_ID, 0);
//             // TxServoAttach(ARM_ID, 0);
//             // TxServoAttach(TORSO_ID, 0);
//             // TxServoAttach(HEAD_ID, 0);
//         }
//     }

//     void CheckVoiceAudioMode()
//     {

//         // Check button bank presses
//         switch (ChannelData(cPITCHSW))
//         {

//             // Voice Audio Off
//         case (SBUSMIN - ButtonDeadband)...(SBUSMIN + ButtonDeadband):
//             // Serial.println("audio voice off");
//             if (gVoicePitchMode != PITCHOFF)
//             {
//                 SetAudioVoiceOff();
//                 gVoicePitchMode = PITCHOFF;
//             }
//             break;

//         // Voice Audio On pitch 1.4
//         case (SBUSCENTER - ButtonDeadband)...(SBUSCENTER + ButtonDeadband):
//             if (gVoicePitchMode != PITCHHIGH)
//             {
//                 SetAudioVoiceOn();
//                 SetAudioPitch(100); // 1.4f
//                 gVoicePitchMode = PITCHHIGH;
//             }
//             break;

//             // Voice Audio On
//         case (SBUSMAX - ButtonDeadband)...(SBUSMAX + ButtonDeadband):
//             if (gVoicePitchMode != PITCHLOW)
//             {
//                 SetAudioVoiceOn();
//                 SetAudioPitch(80); // 0.7f
//                 gVoicePitchMode = PITCHLOW;
//             }
//             break;

//         default:
//             break;
//         }
//     }
//     // void handleAnimationControl()
//     // {
//     //     // // Check for specific trim switch inputs to trigger animations
//     //     // int trimValue = ChannelData(cTRIMBACKSWITCH); // Assuming cTRIMSWITCH is defined for your trim inputs

//     //     // // Serial.print("Trim value: ");
//     //     // // Serial.println(trimValue);

//     //     // // Trigger animations based on trim switch values
//     //     // if (trimValue >= (CTrim2U - ButtonDeadband) && trimValue <= (CTrim2U + ButtonDeadband))
//     //     // {
//     //     //     triggerAnimation(1, false); // Animation ID for CTrim2U
//     //     //     Serial.println("Triggered Animation for CTrim2U");
//     //     //     Serial.println(trimValue);
//     //     // }
//     //     // else if (trimValue >= (CTrim2D - ButtonDeadband) && trimValue <= (CTrim2D + ButtonDeadband))
//     //     // {
//     //     //     triggerAnimation(2, false); // Animation ID for CTrim2D
//     //     //     Serial.println("Triggered Animation for CTrim2D");
//     //     //     Serial.println(trimValue);
//     //     // }
//     //     // else if (trimValue >= (CTrim3U - ButtonDeadband) && trimValue <= (CTrim3U + ButtonDeadband))
//     //     // {
//     //     //     triggerAnimation(3, false); // Animation ID for CTrim3U
//     //     //     Serial.println("Triggered Animation for CTrim3U");
//     //     //     Serial.println(trimValue);
//     //     // }
//     //     // else if (trimValue >= (CTrim3D - ButtonDeadband) && trimValue <= (CTrim3D + ButtonDeadband))
//     //     // {
//     //     //     triggerAnimation(4, false); // Animation ID for CTrim3D
//     //     //     Serial.println("Triggered Animation for CTrim3D");
//     //     //     Serial.println(trimValue);
//     //     // }
//     //     // else if (trimValue >= (CSWI - ButtonDeadband) && trimValue <= (CSWI + ButtonDeadband))
//     //     // {
//     //     //     triggerAnimation(5, false); // Animation ID for CSWI
//     //     //     Serial.println("Triggered Animation for CSWI");
//     //     //     Serial.println(trimValue);
//     //     // }
//     //     // else if (trimValue >= (CTSWJ - ButtonDeadband) && trimValue <= (CTSWJ + ButtonDeadband))
//     //     // {
//     //     //     triggerAnimation(6, false); // Animation ID for CTSWJ
//     //     //     Serial.println("Triggered Animation for CTSWJ");
//     //     //     Serial.println(trimValue);
//     //     // }
//     //     // // Update and play animations
//     //     // GeneratedCommandStreams::updatePlayStatus();
//     // }
