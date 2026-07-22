
#include <Arduino.h>
#include "Definitions.h"
#include <src/Audio.hpp>
#include <src/MotionControl.hpp>
// #include "RCtoBottangoSwitch.h"


// #include <Scheduler.h>

void ProcessEventsLoop();
// void ProcessTrim1Switches();
// void ProcessTrimBackSwitches();
uint16_t GetButtonState();
uint16_t GetButtonBank2State();
uint16_t GetButtonBank3State();
void CheckButtonPress(void (*funcA)(), void (*funcB)());
void CheckLongPress(void (*func)(), bool continuous);
void CheckButtonPressTR(void (*funcA)(), void (*funcB)());
void CheckLongPressTR(void (*func)(), bool continuous);
void CheckButtonPressTR2(void (*funcA)(), void (*funcB)());
void CheckLongPressTR2(void (*func)(), bool continuous);
void ProcessButtons();
void CheckHeadMode();
// void CheckShoulderMode();
void CheckVoiceAudioMode();
void CheckMotorHome();
void CheckPitchAudioMode();

void Button1Click();
void Button1ClickB();
void Button1LongClick();
void Button2Click();
void Button2ClickB();
void Button2LongClick();
void Button3Click();
void Button3ClickB();
void Button3LongClick();
void Button4Click();
void Button4ClickB();
void Button4LongClick();
void Button5Click();
void Button5ClickB();
void Button5LongClick();
void Button6Click();
void Button6ClickB();
void Button6LongClick();
void Button7Click();
void Button7ClickB();
void Button7LongClick();
void Button8Click();
void Button8ClickB();
void Button8LongClick();

void Trim6LClick();
void Trim6LClickB();
void Trim6LClickLongClick();
void Trim6RClick();
void Trim6RClickB();
void Trim6RClickLongClick();
void Trim5LClick();
void Trim5LClickB();
void Trim5LClickLongClick();
void Trim5RClick();
void Trim5RClickB();
void Trim5RClickLongClick();
// void Trim1LClick();
// void Trim1LClickB();
// void Trim1LClickLongClick();
// void Trim1RClick();
// void Trim1RClickB();
// void Trim1RClickLongClick();
// void Trim4LClick();
// void Trim4LClickB();
// void Trim4LClickLongClick();
// void Trim4RClick();
// void Trim4RClickB();
// void Trim4RClickLongClick();
void FingerButtonLClick();
void FingerButtonRClick();
void FingerButtonLClickB();
void FingerButtonLClickLongClick();
void FingerButtonRClickB();
void FingerButtonRClickLongClick();
void CheckRCBottangoMode();
void Animation1();
void Animation2();
void Animation3();
void Animation4();
void Animation5();
void Animation6();
void Animation7();
void Animation8();
void Animation9();
void Animation10();
void Animation11();
void Animation12();
void Animation14();
// void handleAnimationControl();
