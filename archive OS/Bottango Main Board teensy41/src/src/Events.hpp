
#include <Arduino.h>
#include "Definitions.h"
#include <src/Audio.hpp>
#include <src/MotionControl.hpp>
#include "src/RCtoBottangoSwitch.h"

// #include <Scheduler.h>

void ProcessEventsLoop();
uint16_t GetButtonState();
uint16_t GetButtonBank2State();
void CheckButtonPress(void (*funcA)(), void (*funcB)());
void CheckLongPress(void (*func)(), bool continuous);
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
void Button4Click();
void Button4ClickB();
void Button5Click();
void Button5ClickB();
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

void handleAnimationControl();
