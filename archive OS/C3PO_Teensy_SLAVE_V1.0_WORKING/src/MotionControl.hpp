#include <Arduino.h>
#include <SerialFunctions.hpp>
#include <SBUSFunctions.hpp>

//#define Serial485 Serial7
//#define RTS4 41

// Forward declaration of ServoParameters struct (adjust if this is elsewhere in your code)
struct ServoParameters;

uint8_t GetHeadMode();
uint8_t GetShoulderMode();
void SetupMotion();
void MotionLoop();
void UpdateServos();

// void setupEasingCurve();  // Pre-calculate easing values
// void ProcessAutoHeadMovement(uint8_t servoIndex, int midPosition, int deviation);
// void StopAllMotion();

// void CalculateHeadPosition(float pitch, float roll);
// void CalculateTorsoLeanLR(float Lean);
// void CalculateTorsoLeanFB(float Lean);

void SetHeadRunModeAuto();
void SetHeadRunModeIdle();
void SetHeadRunModeControl();

void SetShoulderRunModeLeft();
void SetShoulderRunModeRight();
void SetShoulderRunModeBoth();
void SetShoulderMode(uint8_t mode);
bool isExternalRotationModeLeft();
bool isExternalRotationMode();
bool isExternalRotationModeRight();

void ElbowCenter();
void LeftElbowPositionMin();
void LeftElbowPositionMax();
void RightElbowPositionMax();
void RightElbowPositionMin();
void elbowmotionloop();
void updateServoPosition(ServoParameters &servo);  // Updated function declaration

void ForearmCenter();
void LeftForearmPositionMin();
void LeftForearmPositionMax();
void RightForearmPositionMax();
void RightForearmPositionMin();
void ForearmMotionloop();

void WristCenter();
void LeftWristPositionMin();
void LeftWristPositionMax();
void RightWristPositionMax();
void RightWristPositionMin();
void WristMotionloop();

//void SetAutoMotor(uint8_t MotorID, float TargetMin, float TargetMax, float VelocityMin, float VelocityMax);
//void CheckMotorPosition(uint8_t MotorID);
//void CheckHeadYawPosition();
//void SetAutoYaw();

// void TestAllServos(void);

void ServoAttach(uint8_t ServoNum);
// void ServoDetach(uint8_t ServoNum);
void SetServoPosition(int ServoID, float target_position);
