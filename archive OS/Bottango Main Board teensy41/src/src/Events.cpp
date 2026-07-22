// Radio Event triggering
// Mapping radio switches etc. to functions.
//  Requires Scheduler Library.  Modified Library by TZ

#include <src/Events.hpp>
#include <src/MotionControl.hpp>

uint8_t RunMode = 0;
uint8_t gVoicePitchMode = PITCHOFF;
uint16_t ButtonState = 0;
bool ButtonPressed = false;   // Button Pressed indicator for button bank
bool ButtonPressedTR = false; // Button Pressed indicator for Trim Rud
bool ButtonPressedTR2 = false; // Button Pressed indicator for Trim2 Rud
bool ButtonLongPressed = false;
bool ButtonLongPressedTR = false;
bool ButtonLongPressedTR2 = false;

int ButtonCount = 0;   // Counter for Button Bank
int ButtonCountTR = 0; // Counter for Trim Rud
int ButtonCountTR2 = 0; // Counter for Trim2 Rud

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
void CheckButtonPress(void (*funcA)(), void (*funcB)())
{
    // Serial.println("CheckButtonPress");
    // Check if first time button pressed.
    if (!ButtonPressed)
    {
        // Check which bank to activate
        // if (ChannelData(cBUTTONBANK) < 1000)
        funcA();
        //  else
        //     funcB();
        Serial.println("ButtonPress");
        ButtonPressed = 1;
    }
}

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
        // CheckLongPress(Button3LongClick, 0);
        break;

        // Button 4
    case (CButton4 - ButtonDeadband)...(CButton4 + ButtonDeadband):
        CheckButtonPress(Button4Click, Button4ClickB);
        // CheckLongPress(Button4LongClick, 0);
        break;

        // Button 5
    case (CButton5 - ButtonDeadband)...(CButton5 + ButtonDeadband):
        CheckButtonPress(Button5Click, Button5ClickB);
        // CheckLongPress(Button5LongClick, 0);
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
    switch (GetButtonBank2State())
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
        CheckButtonPress(Trim6LClick, Trim6LClickB);
        CheckLongPress(Trim6LClickLongClick, 0);
        break;

        // Trim slider #6 right push
    case (CTrim6R - ButtonDeadband)...(CTrim6R + ButtonDeadband):
        CheckButtonPress(Trim6RClick, Trim6RClickB);
        CheckLongPress(Trim6RClickLongClick, 0);
        break;

        // Trim slider #5 left push
    case (CTrim5L - ButtonDeadband)...(CTrim5L + ButtonDeadband):
        CheckButtonPress(Trim5LClick, Trim5LClickB);
        CheckLongPress(Trim5LClickLongClick, 0);
        break;
        // Trim slider #5 right push
    case (CTrim5R - ButtonDeadband)...(CTrim5R + ButtonDeadband):
        CheckButtonPress(Trim5RClick, Trim5RClickB);
        CheckLongPress(Trim5LClickLongClick, 0);
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
        switch (GetButtonBank3State())
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
        CheckButtonPress(FingerButtonLClick, FingerButtonLClickB);
        CheckLongPress(FingerButtonLClickLongClick, 0);
    //    FingerButtonLClick();

        break;
    // Finger button
    case (CTSWJ - ButtonDeadband)...(CTSWJ + ButtonDeadband):
        // Serial.println("Finger Button R click");
        CheckButtonPress(FingerButtonRClick, FingerButtonRClickB);
        CheckLongPress(FingerButtonRClickLongClick, 0);
    //    FingerButtonLClick();

        break;

    }
}
    void Button1Click()
    {

        // scheduler.stopall();
        Serial.println("Button 1 Click");
        SetAutoPlayWav(false);
        TxAudioAllStop();
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

    void Button5Click()
    {
        Serial.println("Button 5 Click");
        PlayWavA(3001); // Model Notes
    }

    void Button5ClickB()
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

        int songNum = random(2001, 2032);
        Serial.print("play Rando song: ");
        Serial.print(songNum);

        PlayWavB(songNum);
    }

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

        if (ChannelData(cHOMEMOTORS) > SBUSCENTER)
        {
            ServoAttach(0);
            // TxServoAttach(HEAD_ID, 0);
            // TxServoAttach(SHOULDER_ID, 0);
            // TxServoAttach(ARM_ID, 0);
            // TxServoAttach(TORSO_ID, 0);
            // TxServoAttach(HEAD_ID, 0);
        }
    }

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
                SetAudioPitch(120); // 1.4f
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
    void handleAnimationControl()
    {
        // Check for specific trim switch inputs to trigger animations
        int trimValue = ChannelData(cTRIMBACKSWITCH); // Assuming cTRIMSWITCH is defined for your trim inputs

        // Serial.print("Trim value: ");
        // Serial.println(trimValue);

        // Trigger animations based on trim switch values
        if (trimValue >= (CTrim2U - ButtonDeadband) && trimValue <= (CTrim2U + ButtonDeadband))
        {
            triggerAnimation(1, false); // Animation ID for CTrim2U
            Serial.println("Triggered Animation for CTrim2U");
            Serial.println(trimValue);
        }
        else if (trimValue >= (CTrim2D - ButtonDeadband) && trimValue <= (CTrim2D + ButtonDeadband))
        {
            triggerAnimation(2, false); // Animation ID for CTrim2D
            Serial.println("Triggered Animation for CTrim2D");
            Serial.println(trimValue);
        }
        else if (trimValue >= (CTrim3U - ButtonDeadband) && trimValue <= (CTrim3U + ButtonDeadband))
        {
            triggerAnimation(3, false); // Animation ID for CTrim3U
            Serial.println("Triggered Animation for CTrim3U");
            Serial.println(trimValue);
        }
        else if (trimValue >= (CTrim3D - ButtonDeadband) && trimValue <= (CTrim3D + ButtonDeadband))
        {
            triggerAnimation(4, false); // Animation ID for CTrim3D
            Serial.println("Triggered Animation for CTrim3D");
            Serial.println(trimValue);
        }
        else if (trimValue >= (CSWI - ButtonDeadband) && trimValue <= (CSWI + ButtonDeadband))
        {
            triggerAnimation(5, false); // Animation ID for CSWI
            Serial.println("Triggered Animation for CSWI");
            Serial.println(trimValue);
        }
        else if (trimValue >= (CTSWJ - ButtonDeadband) && trimValue <= (CTSWJ + ButtonDeadband))
        {
            triggerAnimation(6, false); // Animation ID for CTSWJ
            Serial.println("Triggered Animation for CTSWJ");
            Serial.println(trimValue);
        }
        // Update and play animations
        GeneratedCommandStreams::updatePlayStatus();
    }
