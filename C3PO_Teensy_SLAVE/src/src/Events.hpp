
#include <Arduino.h>
#include "Definitions.h"
// #include <Audio.hpp>
#include <src/MotionControl.hpp>
// #include "RCtoBottangoSwitch.h"

//#include <Scheduler.h>


void ProcessEventsLoop();
uint16_t GetButtonState();
uint16_t GetButtonBank2State();
uint16_t GetButtonBank3State();
void CheckButtonPress(void (*funcA)(), void (*funcB)());
void CheckLongPress(void (*func)(), bool continuous);
void ProcessButtons();
// void ProcessTrim1Switches();
// void ProcessTrimBackSwitches();

void CheckHeadMode();
void CheckShoulderMode();
void CheckMotorHome();
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
void Animation13();

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
void Trim1LClick();
void Trim1LClickB();
void Trim1LClickLongClick();
void Trim1RClick();
void Trim1RClickB();
void Trim1RClickLongClick();
void Trim4LClick();
void Trim4LClickB();
void Trim4LClickLongClick();
void Trim4RClick();
void Trim4RClickB();
void Trim4RClickLongClick();

void CheckRCBottangoMode();
// void handleAnimationControl();




