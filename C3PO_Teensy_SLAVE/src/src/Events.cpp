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

byte animationIndex = 0;

elapsedMillis ElapsedEvent = 0;

// Scheduler scheduler = Scheduler();      //create a scheduler
// typedef void *func;
// void CheckLongPress(void (*)());

void ProcessEventsLoop()
{

    if (ElapsedEvent > 100)
    {

        ProcessButtons();
        // UpdateVolume();
        CheckHeadMode();
        CheckMotorHome();
        // ProcessTrim1Switches();
        // ProcessTrimBackSwitches();
        // CheckVoiceAudioMode();
        CheckShoulderMode();
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
    Serial.println("CheckButtonPress");
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

void CheckButtonPressTR(void (*funcA)(), void (*funcB)())
{
    Serial.println("CheckButtonPressTR");
    // Check if first time button pressed.
    if (!ButtonPressedTR)
    {
        // Check which bank to activate
        // if (ChannelData(cBUTTONBANK) < 1000)
        funcA();
        //  else
        //     funcB();
        Serial.println("ButtonPressTR");
        ButtonPressedTR = 1;
    }
}

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
void CheckButtonPressTR2(void (*funcA)(), void (*funcB)())
{
    Serial.println("CheckButtonPressTR2");
    // Check if first time button pressed.
    if (!ButtonPressedTR2)
    {
        // Check which bank to activate
        // if (ChannelData(cBUTTONBANK) < 1000)
        funcA();
        //  else
        //     funcB();
        Serial.println("ButtonPressTR2");
        ButtonPressedTR2 = 1;
    }
}

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
{  if (GetRCBottangoMode() == RC_MODE)
    // Check button bank presses
    switch (GetButtonState())
{}
  if (GetRCBottangoMode() == RC_MODE)
    switch (GetButtonBank2State())
    {
    case (trimcenter - ButtonDeadband)...(trimcenter + ButtonDeadband):
        ButtonPressedTR = 0;
        ButtonLongPressedTR = 0;
        ButtonCountTR = 0;
        ElbowCenter();         // Reset both elbows to mid if no trim switch is pressed
        WristCenter();
        //     Serial.println("Buttons Off");
        break;

        // Trim slider #6 left push
    case (CTrim6L - ButtonDeadband)...(CTrim6L + ButtonDeadband):
         CheckButtonPressTR(RightWristPositionMax,RightWristPositionMax);
        ;
        break;

        // Trim slider #6 right push
    case (CTrim6R - ButtonDeadband)...(CTrim6R + ButtonDeadband):
         CheckButtonPressTR(RightWristPositionMin,RightWristPositionMin);
        
        break;

        // Trim slider #5 left push
    case (CTrim5L - ButtonDeadband)...(CTrim5L + ButtonDeadband):
    CheckButtonPressTR(LeftWristPositionMax,LeftWristPositionMax);
     
        break;

        // Trim slider #5 right push
    case (CTrim5R - ButtonDeadband)...(CTrim5R + ButtonDeadband):
    CheckButtonPressTR(LeftWristPositionMin,LeftWristPositionMin);
        
        break;

    // Trim slider #1 left push - bend right elbow
    case (CTrim1L - ButtonDeadband)...(CTrim1L + ButtonDeadband):
    CheckButtonPressTR(RightElbowPositionMax,RightElbowPositionMax);
        
        break;

    // Trim slider #1 right push - extend right elbow
    case (CTrim1R - ButtonDeadband)...(CTrim1R + ButtonDeadband):
    CheckButtonPressTR(RightElbowPositionMin,RightElbowPositionMin);

        break;

    // Trim slider #4 left push - bend left elbow
    case (CTrim4L - ButtonDeadband)...(CTrim4L + ButtonDeadband):
    CheckButtonPressTR(LeftElbowPositionMax,LeftElbowPositionMax);
        

        break;

    // Trim slider #4 right push - extend left elbow
    case (CTrim4R - ButtonDeadband)...(CTrim4R + ButtonDeadband):
    CheckButtonPressTR(LeftElbowPositionMin,LeftElbowPositionMin);
        

        break;

    }
      if (GetRCBottangoMode() == RC_MODE)
    switch (GetButtonBank3State())
    { 
        case (trimcenter2 - ButtonDeadband)...(trimcenter2 + ButtonDeadband):
        ButtonPressedTR = 0;
        ButtonLongPressedTR = 0;
        ButtonCountTR = 0;
        ForearmCenter();       // Reset both Forearms to mid if no trim switch is pressed
        //     Serial.println("Buttons Off");
        break;
        
        // Trim slider #2 left push - bend right Forearm
    case (CTrim2U - ButtonDeadband)...(CTrim2U + ButtonDeadband):
        CheckButtonPressTR2(RightForearmPositionMax,RightForearmPositionMax);
        
        break;

    // Trim slider #2 right push - extend right Forearm
    case (CTrim2D - ButtonDeadband)...(CTrim2D + ButtonDeadband):
    CheckButtonPressTR2(RightForearmPositionMin,RightForearmPositionMin);
        

        break;

    // Trim slider #3 left push - bend left Forearm
    case (CTrim3U - ButtonDeadband)...(CTrim3U + ButtonDeadband):
    CheckButtonPressTR2(LeftForearmPositionMax,LeftForearmPositionMax);
        

        break;

    // Trim slider #5 right push - extend left Forearm
    case (CTrim3D - ButtonDeadband)...(CTrim3D + ButtonDeadband):
    CheckButtonPressTR2(LeftForearmPositionMin,LeftForearmPositionMin);

        break;


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

// case (CSWI - ButtonDeadband) ... (CSWI + ButtonDeadband):
//         CheckButtonPressTR2(Animation11, Animation11);
//                 //     Serial.println("debug here");
//     break;

// case (CTSWJ - ButtonDeadband) ... (CTSWJ + ButtonDeadband):
//         CheckButtonPressTR2(Animation12, Animation12);
//                 //     Serial.println("debug here");
//     break;

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
// if (GetRCBottangoMode() == ANIMATION_MODE)
// // // Check for specific trim switch inputs to trigger animations
//         int trimValue = ChannelData(cTRIMBACKSWITCH); // Assuming cTRIMSWITCH is defined for your trim inputs

//         // Serial.print("Trim value: ");
//         // Serial.println(trimValue);

//         // Trigger animations based on trim switch values
//         if (trimValue >= (CTrim2U - ButtonDeadband) && trimValue <= (CTrim2U + ButtonDeadband))
//         {
//            CheckButtonPress(Animation1, Animation1);
//             Serial.println("Triggered Animation for CTrim2U");
//             Serial.println(trimValue);
//         }
//         else if (trimValue >= (CTrim2D - ButtonDeadband) && trimValue <= (CTrim2D + ButtonDeadband))
//         {
//             animationIndex = 2; 
//             // looping = true;// Animation ID for CTrim2D
//             Serial.println("Triggered Animation for CTrim2D");
//             Serial.println(trimValue);
//         }
//         else if (trimValue >= (CTrim3U - ButtonDeadband) && trimValue <= (CTrim3U + ButtonDeadband))
//         {
//             animationIndex = 3; 
//             // looping = true; // Animation ID for CTrim3U
//             Serial.println("Triggered Animation for CTrim3U");
//             Serial.println(trimValue);
//         }
//         else if (trimValue >= (CTrim3D - ButtonDeadband) && trimValue <= (CTrim3D + ButtonDeadband))
//         {
//             animationIndex = 4; 
//             // looping = true;// Animation ID for CTrim3D
//             Serial.println("Triggered Animation for CTrim3D");
//             Serial.println(trimValue);
//         }
//         else if (trimValue >= (CSWI - ButtonDeadband) && trimValue <= (CSWI + ButtonDeadband))
//         {
//             animationIndex = 5; 
//             // looping = true;// Animation ID for CSWI
//             Serial.println("Triggered Animation for CSWI");
//             Serial.println(trimValue);
//         }
//         else if (trimValue >= (CTSWJ - ButtonDeadband) && trimValue <= (CTSWJ + ButtonDeadband))
//         {
//             animationIndex = 6; 
//             // looping = true;// Animation ID for CTSWJ
//             Serial.println("Triggered Animation for CTSWJ");
//             Serial.println(trimValue);
// }
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
void Animation11() { animationIndex = 11; Serial.println("Animation 10 triggered, animationIndex = 11"); }
void Animation12() { animationIndex = 12; Serial.println("Animation 10 triggered, animationIndex = 12"); }
void Animation13() { animationIndex = 13; Serial.println("Animation 10 triggered, animationIndex = 13"); }

// Function to process trim bank 3 switch inputs and trigger animations
// void ProcessTrim1Switches()
// {
//     int trim1Value = ChannelData(cTRIMSWITCH);  // Read trim switch value

//     if (trim1Value != previousTrim1)  // Only update if changed
//     {
//         Serial.print("Trim value: ");
//         Serial.println(trim1Value);
//         previousTrim1 = trim1Value;  // Update previous value
//     }
//     else
//     {
//         return; // Exit early if no change
//     }

//     // Debounced button check for animations
//     if (trim1Value >= (CTrim1L - ButtonDeadband) && trim1Value <= (CTrim1L + ButtonDeadband))
//     {
//         CheckButtonPressTR(Animation1, Animation1);
//     }
//     else if (trim1Value >= (CTrim1R - ButtonDeadband) && trim1Value <= (CTrim1R + ButtonDeadband))
//     {
//         CheckButtonPressTR(Animation2, Animation2);
//     }
//     else if (trim1Value >= (CTrim6L - ButtonDeadband) && trim1Value <= (CTrim6L + ButtonDeadband))
//     {
//         CheckButtonPressTR(Animation3, Animation3);
//     }
//     else if (trim1Value >= (CTrim6R - ButtonDeadband) && trim1Value <= (CTrim6R + ButtonDeadband))
//     {
//         CheckButtonPressTR2(Animation4, Animation4);
//     }
//     // else if (trim1Value >= (CSWI - ButtonDeadband) && trim1Value <= (CSWI + ButtonDeadband))
//     // {
//     //     CheckButtonPressTR2(Animation5, Animation5);
//     // }
//     // else if (trim1Value >= (CTSWJ - ButtonDeadband) && trim1Value <= (CTSWJ + ButtonDeadband))
//     // {
//     //     CheckButtonPressTR2(Animation6, Animation6);
//     // }
//     else if (trim1Value >= (CTrim4L - ButtonDeadband) && trim1Value <= (CTrim4L + ButtonDeadband))
//     {
//         CheckButtonPressTR(Animation7, Animation7);
//     }
//     else if (trim1Value >= (CTrim4R - ButtonDeadband) && trim1Value <= (CTrim4R + ButtonDeadband))
//     {
//         CheckButtonPressTR(Animation8, Animation8);
//     }
//     else if (trim1Value >= (CTrim5L - ButtonDeadband) && trim1Value <= (CTrim5L + ButtonDeadband))
//     {
//         CheckButtonPressTR(Animation9, Animation9);
//     }
//     else if (trim1Value >= (CTrim5R - ButtonDeadband) && trim1Value <= (CTrim5R + ButtonDeadband))
//     {
//         CheckButtonPressTR(Animation10, Animation10);
//     }
// }
// // Function to process trim bank 3 switch inputs and trigger animations
// void ProcessTrimBackSwitches()
// {
//     int trim2Value = ChannelData(cTRIMBACKSWITCH);  // Read trim switch value

//     if (trim2Value != previousTrim2)  // Only update if changed
//     {
//         Serial.print("Trim value: ");
//         Serial.println(trim2Value);
//         previousTrim2 = trim2Value;  // Update previous value
//     }
//     else
//     {
//         return; // Exit early if no change
//     }

//     // Debounced button check for animations
//     if (trim2Value >= (CTrim2U - ButtonDeadband) && trim2Value <= (CTrim2U + ButtonDeadband))
//     {
//         CheckButtonPressTR2(Animation5, Animation5);
//     }
//     else if (trim2Value >= (CTrim2D - ButtonDeadband) && trim2Value <= (CTrim2D + ButtonDeadband))
//     {
//         CheckButtonPressTR2(Animation6, Animation6);
//     }
//     else if (trim2Value >= (CTrim3U - ButtonDeadband) && trim2Value <= (CTrim3U + ButtonDeadband))
//     {
//         CheckButtonPressTR2(Animation3, Animation3);
//     }
//     else if (trim2Value >= (CTrim3D - ButtonDeadband) && trim2Value <= (CTrim3D + ButtonDeadband))
//     {
//         CheckButtonPressTR2(Animation4, Animation4);
//     }
//     else if (trim2Value >= (CSWI - ButtonDeadband) && trim2Value <= (CSWI + ButtonDeadband))
//     {
//         CheckButtonPressTR2(Animation11, Animation11);
//     }
//     else if (trim2Value >= (CTSWJ - ButtonDeadband) && trim2Value <= (CTSWJ + ButtonDeadband))
//     {
//         CheckButtonPressTR2(Animation12, Animation12);
//     }
    // else if (trim2Value >= (CTrim4L - ButtonDeadband) && trim2Value <= (CTrim4L + ButtonDeadband))
    // {
    //     CheckButtonPressTR(Animation7, Animation7);
    // }
    // else if (trim2Value >= (CTrim4R - ButtonDeadband) && trim2Value <= (CTrim4R + ButtonDeadband))
    // {
    //     CheckButtonPressTR(Animation8, Animation8);
    // }
    // else if (trim2Value >= (CTrim5L - ButtonDeadband) && trim2Value <= (CTrim5L + ButtonDeadband))
    // {
    //     CheckButtonPressTR(Animation9, Animation9);
    // }
    // else if (trim2Value >= (CTrim5R - ButtonDeadband) && trim2Value <= (CTrim5R + ButtonDeadband))
    // {
    //     CheckButtonPressTR(Animation10, Animation10);
    // }


   
    // bank B trim button functions
    void Trim6LClick()
    {
        // Serial.println("trim button 6 push left");
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
        // Serial.println("trim button 6 push right");
    }

    void Trim6RClickB()
    {
    }

    void Trim6RClickLongClick()
    {
    }

    void Trim5LClick()
    {
        // Serial.println("trim button 5 push left");
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
        // Serial.println("trim button 5 push right");
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
        // Serial.println("trim button 1 push left");
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
        // Serial.println("trim button 1 push right");
    }

    void Trim1RClickB()
    {
    }

    void Trim1RClickLongClick()
    {
    }

    void Trim4LClick()
    {
        // Serial.println("trim button 4 push left");
        
    }

    void Trim4LClickB()
    {
    }

    void Trim4LClickLongClick()
    {
    }

    void Trim4RClick()
    {
        // Serial.println("trim button 4 push right");
       
    }

    void Trim4RClickB()
    {
    }

    void Trim4RClickLongClick()
    {
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

    void CheckShoulderMode()
    {
        // Serial.println("check head mode");
        // Check button bank presses
        switch (ChannelData(cSHOULDERMODESWITCH))
        {

            // Shoulder Left Mode
        case (SBUSMIN - ButtonDeadband)...(SBUSMIN + ButtonDeadband):

            if (GetShoulderMode() != SHOULDERSWITCHLEFT)
            {
                SetShoulderMode(SHOULDERSWITCHLEFT);
                Serial.println("Shoulder Mode Left");
            }
            break;

            // Shoulder Right Mode
        case (SBUSCENTER - ButtonDeadband)...(SBUSCENTER + ButtonDeadband):
            if (GetShoulderMode() != SHOULDERSWITCHRIGHT)
            {
                SetShoulderMode(SHOULDERSWITCHRIGHT);
                Serial.println("Shoulder Mode Right");
            }
            break;

            // Shoulder Both Mode
        case (SBUSMAX - ButtonDeadband)...(SBUSMAX + ButtonDeadband):

            if (GetShoulderMode() != SHOULDERSWITCHBOTH)
            {
                SetShoulderMode(SHOULDERSWITCHBOTH);
                Serial.println("Shoulder Mode Both");
            }
            break;
        default:
            break;
        }
    }

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

    