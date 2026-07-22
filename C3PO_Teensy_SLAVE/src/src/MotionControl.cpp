// MotionControl.cpp
// Functions to control Servos and other Motions
// Comminicating to ESP32s over RS-485
// Arranged as 4 banks of Servos



#include <src/MotionControl.hpp>
#include <Teensy_PWM.h>
#include <Arduino.h>
#include "Definitions.h"
#include <math.h>
#include <src/BottangoCore.h>
//#include <PWMServo.h>
#include <servo.h>
#include <src/Events.hpp>
#include "BasicCommands.h"


uint8_t gHeadRunMode = HEADIDLE;
uint8_t gShoulderSwitchMode = SHOULDERSWITCHLEFT;
uint8_t gRCBottangoMode = RC_MODE;
uint8_t currentMode = RC_MODE;

bool bottangoDeregistered = false; // Track whether Bottango has been deregistered

// Variables to track if any trim button is pressed
bool rightElbowActive = false;
bool leftElbowActive = false;
bool rightForearmActive = false;
bool leftForearmActive = false;
bool rightWristActive = false;
bool leftWristActive = false;

#define MOTIONFRAMERATE 25 // ms for Motor Control Loop

unsigned long MotionMillis = millis();
elapsedMillis gAutoHeadElapsed[6];  // Elapsed timer for updating the head
unsigned long gAutoHeadInterval[6]; // Random movement interval of pitch roll yaw
const int deviation = 150;          // Adjust this value to control the subtlety of movements
const int easingSteps = 100;        // Number of steps for easing
float easingCurve[easingSteps];
unsigned long lastUpdateTime = 0;
const unsigned long servoUpdateInterval = 20; // Update servos every 20ms (50Hz)

// Smooth step easing function (can replace with easeInOut or any other function)
float smoothStep(float t)
{
    return t * t * (3 - 2 * t); // Adjust formula if you want different easing effects
}


// Shoulder Servo
Servo servoShoulderPitch1Right;
Servo servoShoulderPitch2Right;
Servo servoShoulderPitch1Left;
Servo servoShoulderPitch2Left;
Servo servoShoulderRollRight;
Servo servoShoulderRollLeft;

// Arm Servos
Servo servoElbowPitchRight;
Servo servoElbowPitchLeft;
Servo servoWristPitchRight;
Servo servoWristPitchLeft;
Servo servoWristRollRight;
Servo servoWristRollLeft;

// Define struct to contain the motor and parameters
struct ServoParameters
{
    // uint8_t servoBank; // servo bank that servo references
    uint8_t servoID;
    Servo sMotor;
    float center_position;  // center position of motor in degrees
    float current_position; // angle in degrees
    float target_position;  // angle in degrees
    float speed;            // degrees per second
    float easing;
    float MinDegrees; // Min degree limit
    float MaxDegrees; // Maximum degree limit
    bool isAttached;
    uint8_t servoBank; // when setting up multiple teensy to control motion with one teensy addr =1
    unsigned long moveStartMs;
    float start_position;
    float moveTimeMs;     // like your gAutoHeadMoveTime
};

// Add new servos to ServoParameters


ServoParameters sShoulderPitch1Right = {iShoulderPitch1Right, servoShoulderPitch1Right, Shoulder_Pitch1_Right_MID, Shoulder_Pitch1_Right_MID, Shoulder_Pitch1_Right_MID, 0, 0, Shoulder_Pitch1_Right_MIN, Shoulder_Pitch1_Right_MAX, false, TEENSY_SLAVE1,0,0,0};
ServoParameters sShoulderPitch2Right = {iShoulderPitch2Right, servoShoulderPitch2Right, Shoulder_Pitch2_Right_MID, Shoulder_Pitch2_Right_MID, Shoulder_Pitch2_Right_MID, 0, 0, Shoulder_Pitch2_Right_MIN, Shoulder_Pitch2_Right_MAX, false, TEENSY_SLAVE1,0,0,0};
ServoParameters sShoulderPitch1Left = {iShoulderPitch1Left, servoShoulderPitch1Left, Shoulder_Pitch1_Left_MID, Shoulder_Pitch1_Left_MID, Shoulder_Pitch1_Left_MID, 0, 0, Shoulder_Pitch1_Left_MIN, Shoulder_Pitch1_Left_MAX, false, TEENSY_SLAVE1,0,0,0};
ServoParameters sShoulderPitch2Left = {iShoulderPitch2Left, servoShoulderPitch2Left, Shoulder_Pitch2_Left_MID, Shoulder_Pitch2_Left_MID, Shoulder_Pitch2_Left_MID, 0, 0, Shoulder_Pitch2_Left_MIN, Shoulder_Pitch2_Left_MAX, false, TEENSY_SLAVE1,0,0,0};
ServoParameters sShoulderRollRight = {iShoulderRollRight, servoShoulderRollRight, Shoulder_Roll_Right_MID, Shoulder_Roll_Right_MID, Shoulder_Roll_Right_MID, 0, 0, Shoulder_Roll_Right_MIN, Shoulder_Roll_Right_MAX, false, TEENSY_SLAVE1,0,0,0};
ServoParameters sShoulderRollLeft = {iShoulderRollLeft, servoShoulderRollLeft, Shoulder_Roll_Left_MID, Shoulder_Roll_Left_MID, Shoulder_Roll_Left_MID, 0, 0, Shoulder_Roll_Left_MIN, Shoulder_Roll_Left_MAX, false, TEENSY_SLAVE1};

ServoParameters sElbowPitchRight = {iElbowPitchRight, servoElbowPitchRight, Elbow_Pitch_Right_MID, Elbow_Pitch_Right_MID, Elbow_Pitch_Right_MID, 0.01, 0.01, Elbow_Pitch_Right_MIN, Elbow_Pitch_Right_MAX, false, TEENSY_SLAVE1,0,0,0};
ServoParameters sElbowPitchLeft = {iElbowPitchLeft, servoElbowPitchLeft, Elbow_Pitch_Left_MID, Elbow_Pitch_Left_MID, Elbow_Pitch_Left_MID, 0.1, 0.1, Elbow_Pitch_Left_MIN, Elbow_Pitch_Left_MAX, false, TEENSY_SLAVE1,0,0,0};
ServoParameters sWristPitchRight = {iWristPitchRight, servoWristPitchRight, Wrist_Pitch_Right_MID, Wrist_Pitch_Right_MID, Wrist_Pitch_Right_MID, 0, 0, Wrist_Pitch_Right_MIN, Wrist_Pitch_Right_MAX, false, TEENSY_SLAVE1,0,0,0};
ServoParameters sWristPitchLeft = {iWristPitchLeft, servoWristPitchLeft, Wrist_Pitch_Left_MID, Wrist_Pitch_Left_MID, Wrist_Pitch_Left_MID, 0, 0, Wrist_Pitch_Left_MIN, Wrist_Pitch_Left_MAX, false, TEENSY_SLAVE1,0,0,0};
ServoParameters sWristRollRight = {iWristRollRight, servoWristRollRight, Wrist_Roll_Right_MID, Wrist_Roll_Right_MID, Wrist_Roll_Right_MID, 1, 0.1, Wrist_Roll_Right_MIN, Wrist_Roll_Right_MAX, false, TEENSY_SLAVE1,0,0,0};
ServoParameters sWristRollLeft = {iWristRollLeft, servoWristRollLeft, Wrist_Roll_Left_MID, Wrist_Roll_Left_MID, Wrist_Roll_Left_MID, 1, 0.1, Wrist_Roll_Left_MIN, Wrist_Roll_Left_MAX, false, TEENSY_SLAVE1,0,0,0};

ServoParameters ServoList[] = {


    sShoulderPitch1Right,
    sShoulderPitch2Right,
    sShoulderPitch1Left,
    sShoulderPitch2Left,
    sShoulderRollRight,
    sShoulderRollLeft,

    sElbowPitchRight,
    sElbowPitchLeft,
     sWristPitchRight,
     sWristPitchLeft,
     sWristRollRight,
      sWristRollLeft,
};

// Total number of Servos
const int NUM_SERVOS = sizeof(ServoList) / sizeof(ServoList[0]);

uint8_t ServoPins[] = {

    pinShoulderPitch1Right,
    pinShoulderPitch2Right,
    pinShoulderPitch1Left,
    pinShoulderPitch2Left,
    pinShoulderRollRight,
    pinShoulderRollLeft,

    pinElbowPitchRight,
    pinElbowPitchLeft,
     pinWristPitchRight,
      pinWristPitchLeft,
     pinWristRollRight,
     pinWristRollLeft,


};

void SetupMotion()
{

    Serial.println("Setting up Servos");
    pinMode(10, OUTPUT);
    digitalWrite(10, HIGH); // Turn D1-D3 as outputs.
    delay(25);

    // Set all servos to center position

        // TxServoPosition(ServoList[i].servoBank, ServoList[i].servoID, ServoList[i].center_position);

    // setupEasingCurve();  // Pre-calculate easing values
    // attach servos
    // ServoAttach(0);
    Serial.println("Setup Servos Complete");
}
// void setupEasingCurve() {
//     for (int i = 0; i < easingSteps; i++) {
//         float t = (float)i / (easingSteps - 1);  // Normalize `i` to [0,1] range
//         easingCurve[i] = smoothStep(t);
//     }
// }
void MotionLoop()
{
    // float PitchAngle;
    // float RollAngle;


if (millis() - MotionMillis > MOTIONFRAMERATE)
{    MotionMillis = millis();
    if (gRCBottangoMode == RC_MODE)
    {
        if (!bottangoDeregistered)  // Ensure it runs only once
        {
            BottangoCore::effectorPool.deregisterAll();
            bottangoDeregistered = true; // Mark as deregistered
        }

        if (gHeadRunMode == HEADCONTROL)
        {
            // Elbow functions
            elbowmotionloop();
            ForearmMotionloop();
            WristMotionloop();

            // SHOULDER FUNCTIONS
            if (gShoulderSwitchMode == SHOULDERSWITCHLEFT)
            {
                SetShoulderRunModeLeft();
            }
            else if (gShoulderSwitchMode == SHOULDERSWITCHRIGHT)
            {
                SetShoulderRunModeRight();
            }
            else if (gShoulderSwitchMode == SHOULDERSWITCHBOTH)
            {
                SetShoulderRunModeBoth();
            }

            // Update all servos
            UpdateServos();
        }

        if (gHeadRunMode == HEADIDLE)
        {
            //  ServoDetach(0); // Detach servos
        }

        if (millis() - lastUpdateTime >= servoUpdateInterval)
        {
            lastUpdateTime = millis(); // Store the current time
        }
    }
    else if (gRCBottangoMode == ANIMATION_MODE)
    {
        // bottangoDeregistered = false;  // Reset flag when switching to ANIMATION_MODE
        // GeneratedCodeAnimations::AnimationControl();
    }
}
}

// void UpdateServos()
// {
//     unsigned long now = millis();

//     for (uint8_t i = 0; i < NUM_SERVOS; i++)
//     {
//         float cur = ServoList[i].current_position;
//         float tgt = ServoList[i].target_position;

//         // If we're basically already there, hold
//         if (fabs(tgt - cur) < 1.0f)
//         {
//             ServoList[i].current_position = tgt;
//             continue;
//         }

//         // If this is a new move (start not set), latch start
//         if (ServoList[i].moveStartMs == 0)
//         {
//             ServoList[i].moveStartMs = now;
//             ServoList[i].start_position = cur;

//             // per-servo ramp time (ms) - YOU TUNE THIS
//             ServoList[i].moveTimeMs = 500;  // example default
//         }

//         float t = (float)(now - ServoList[i].moveStartMs) / ServoList[i].moveTimeMs;
//         if (t > 1.0f) t = 1.0f;

//         float eased = smoothStep(t);  // accel/decel shape

//         float out = ServoList[i].start_position + (tgt - ServoList[i].start_position) * eased;

//         ServoList[i].sMotor.writeMicroseconds((int)out);
//         ServoList[i].current_position = out;

//         // End move
//         if (t >= 1.0f)
//         {
//             ServoList[i].moveStartMs = 0; // ready for next target change
//             ServoList[i].current_position = tgt;
//         }
//     }
// }

void UpdateServos()
{
    // if (gHeadRunMode == HEADCONTROL)
    // {
        for (uint8_t i = 0; i < NUM_SERVOS; i++)
        {
            if (abs(ServoList[i].target_position - ServoList[i].current_position) > 1)
            {
                SetServoPosition(i, ServoList[i].target_position);
                ServoList[i].current_position = ServoList[i].target_position;
            }
        }
    // }
}

// void StopAllMotion()
// {
//     ServoDetach(0);
//     // Implement Stop Motion function logic
// }

void SetHeadRunModeAuto()
{
    gHeadRunMode = HEADAUTO;
    Serial.println("HeadMode Auto");
}

void SetHeadRunModeIdle()
{
    gHeadRunMode = HEADIDLE;
    Serial.println("HeadMode Idle");
    //StopAllMotion();
}

void SetHeadRunModeControl()
{
    gHeadRunMode = HEADCONTROL;
    Serial.println("HeadMode Control");
}

uint8_t GetHeadMode()
{
    return gHeadRunMode;
}
void SetRCBottangoMode_RC()
{
    gRCBottangoMode = RC_MODE;
    BottangoCore::commandStreamProvider->stop(); 
    Serial.println("RC MODE");

}

void SetRCBottangoMode_Animation()
{
    gRCBottangoMode = ANIMATION_MODE;
    Serial.println("ANIMATION MODE");

}

uint8_t GetRCBottangoMode()
{
    return gRCBottangoMode;
}

void SetShoulderRunModeLeft()
{
    // Flexion (FL) - Shoulder Roll for Left Shoulder
    ServoList[iShoulderRollLeft].target_position = map(ChannelData(cSHFL), SBUSNEUTRAL, SBUSMAX, ServoList[iShoulderRollLeft].center_position, ServoList[iShoulderRollLeft].MaxDegrees);


    // Initialize target positions to neutral
    int targetPitch1Left = Shoulder_Pitch1_Left_MID;
    int targetPitch2Left = Shoulder_Pitch2_Left_MID;

    // Read abduction and external rotation input values
    int abductionValue = ChannelData(cSHER);
    int externalRotationValue = ChannelData(cSHAB);

    // Calculate abduction target positions for Left Pitch 1 and Pitch 2
    if (abductionValue > SBUSNEUTRAL) {
        targetPitch1Left = map(abductionValue, SBUSNEUTRAL, SBUSMAX, Shoulder_Pitch1_Left_MID, Shoulder_Pitch1_Left_MAX);
        targetPitch2Left = map(abductionValue, SBUSNEUTRAL, SBUSMAX, Shoulder_Pitch2_Left_MID, Shoulder_Pitch2_Left_MAX);
    }

    // Apply external rotation on top of abduction positions
    if (isExternalRotationMode()) {
        int rotationOffsetPitch1Left = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, -600);
        int rotationOffsetPitch2Left = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0,  600);

        targetPitch1Left += rotationOffsetPitch1Left;
        targetPitch2Left += rotationOffsetPitch2Left;
    }

    // Update the target positions for Left Shoulder
    ServoList[iShoulderPitch1Left].target_position = targetPitch1Left;
    ServoList[iShoulderPitch2Left].target_position = targetPitch2Left;
}

// Helper function to determine if the shoulder is in external rotation mode (left)
bool isExternalRotationModeLeft() 
{
    // Get the value from the left shoulder external rotation channel (cSHER)
    int erInputLeft = ChannelData(cSHAB);

    // Define the "quiet" range around 992 ± 100 for the left shoulder
    const int quietLowerBound = 892;
    const int quietUpperBound = 1092;

    // Return true if external rotation is active, meaning the input is outside the quiet range
    return (erInputLeft < quietLowerBound || erInputLeft > quietUpperBound);
}


void SetShoulderRunModeRight()
{
    // Flexion (FL) - Shoulder Roll for Right Shoulder
     ServoList[iShoulderRollRight].target_position = map(ChannelData(cSHFL), SBUSNEUTRAL, SBUSMAX, ServoList[iShoulderRollRight].center_position, ServoList[iShoulderRollRight].MaxDegrees);

    // Initialize target positions to neutral
    int targetPitch1Right = Shoulder_Pitch1_Right_MID;
    int targetPitch2Right = Shoulder_Pitch2_Right_MID;

    // Read abduction and external rotation input values
    int abductionValue = ChannelData(cSHER);
    int externalRotationValue = ChannelData(cSHAB);

    // Calculate abduction target positions for Right Pitch 1 and Pitch 2
    if (abductionValue > SBUSNEUTRAL) {
        targetPitch1Right = map(abductionValue, SBUSNEUTRAL, SBUSMAX, Shoulder_Pitch1_Right_MID, Shoulder_Pitch1_Right_MAX);
        targetPitch2Right = map(abductionValue, SBUSNEUTRAL, SBUSMAX, Shoulder_Pitch2_Right_MID, Shoulder_Pitch2_Right_MAX);
    }

    // Apply external rotation on top of abduction positions
    if (isExternalRotationMode()) {
        int rotationOffsetPitch1Right = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, 600);
        int rotationOffsetPitch2Right = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, -600);

        targetPitch1Right += rotationOffsetPitch1Right;
        targetPitch2Right += rotationOffsetPitch2Right;
    }

    // Update the target positions for Right Shoulder
    ServoList[iShoulderPitch1Right].target_position = targetPitch1Right;
    ServoList[iShoulderPitch2Right].target_position = targetPitch2Right;
}


// Helper function to determine if the shoulder is in external rotation mode
bool isExternalRotationModeRight() 
{
    // Get the value from the external rotation channel (cSHER)
    int erInput = ChannelData(cSHAB);

    // Define the "quiet" range around 992 ± 100
    const int quietLowerBound = 892;
    const int quietUpperBound = 1092;

    // Return true if external rotation is active, meaning the input is outside the quiet range
    return (erInput < quietLowerBound || erInput > quietUpperBound);
}


// 
void SetShoulderRunModeBoth()
{
    // Set flexion (FL) for both shoulders - Shoulder Roll
    ServoList[iShoulderRollRight].target_position = map(ChannelData(cSHFL), SBUSNEUTRAL, SBUSMAX, ServoList[iShoulderRollRight].center_position, ServoList[iShoulderRollRight].MaxDegrees);
    ServoList[iShoulderRollLeft].target_position = map(ChannelData(cSHFL), SBUSNEUTRAL, SBUSMAX, ServoList[iShoulderRollLeft].center_position, ServoList[iShoulderRollLeft].MaxDegrees);

    // Initialize target positions to neutral for both shoulders
    int targetPitch1Right = Shoulder_Pitch1_Right_MID;
    int targetPitch2Right = Shoulder_Pitch2_Right_MID;
    int targetPitch1Left = Shoulder_Pitch1_Left_MID;
    int targetPitch2Left = Shoulder_Pitch2_Left_MID;

    // Read abduction and external rotation input values
    int abductionValue = ChannelData(cSHER);
    int externalRotationValue = ChannelData(cSHAB);

    // Calculate abduction target positions for both shoulders
    if (abductionValue > SBUSNEUTRAL) {
        targetPitch1Left = map(abductionValue, SBUSNEUTRAL, SBUSMAX, Shoulder_Pitch1_Left_MID, Shoulder_Pitch1_Left_MAX);
        targetPitch2Left = map(abductionValue, SBUSNEUTRAL, SBUSMAX, Shoulder_Pitch2_Left_MID, Shoulder_Pitch2_Left_MAX);

        targetPitch1Right = map(abductionValue, SBUSNEUTRAL, SBUSMAX, Shoulder_Pitch1_Right_MID, Shoulder_Pitch1_Right_MAX);
        targetPitch2Right = map(abductionValue, SBUSNEUTRAL, SBUSMAX, Shoulder_Pitch2_Right_MID, Shoulder_Pitch2_Right_MAX);
    }

    // Apply external rotation on top of abduction positions for both shoulders
    if (isExternalRotationMode()) {
        int rotationOffsetPitch1Right = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, 600);
        int rotationOffsetPitch2Right = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, -600);
        
        int rotationOffsetPitch1Left = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, -600);
        int rotationOffsetPitch2Left = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, 600);

        targetPitch1Right += rotationOffsetPitch1Right;
        targetPitch2Right += rotationOffsetPitch2Right;
        
        targetPitch1Left += rotationOffsetPitch1Left;
        targetPitch2Left += rotationOffsetPitch2Left;
    }

    // Update the target positions for both shoulders
    ServoList[iShoulderPitch1Right].target_position = targetPitch1Right;
    ServoList[iShoulderPitch2Right].target_position = targetPitch2Right;
    ServoList[iShoulderPitch1Left].target_position = targetPitch1Left;
    ServoList[iShoulderPitch2Left].target_position = targetPitch2Left;
}


// Helper function to determine if the shoulder is in external rotation mode
bool isExternalRotationMode() 
{
    // Get the value from the external rotation channel (cSHER)
    int erInput = ChannelData(cSHAB);

    // Define the "quiet" range around 992 ± 100
    const int quietLowerBound = 892;
    const int quietUpperBound = 1092;

    // Return true if external rotation is active, meaning the input is outside the quiet range
    return (erInput < quietLowerBound || erInput > quietUpperBound);
}

uint8_t GetShoulderMode()
{
    return gShoulderSwitchMode;
}

void SetShoulderMode(uint8_t mode)
{
    gShoulderSwitchMode = mode;
}


void elbowmotionloop()
{
    // Update the right elbow servo
    if (!rightElbowActive)
    {
        ServoList[iElbowPitchRight].target_position = Elbow_Pitch_Right_MID;
    }
    // updateServoPosition(ServoList[iElbowPitchRight]);

    // Update the left elbow servo
    if (!leftElbowActive)
    {
        ServoList[iElbowPitchLeft].target_position = Elbow_Pitch_Left_MID;
    }
    // updateServoPosition(ServoList[iElbowPitchLeft]);
}

void ElbowCenter()
{
    rightElbowActive = false;
    leftElbowActive = false;
    elbowmotionloop();

}

void LeftElbowPositionMin()
{
    ServoList[iElbowPitchLeft].target_position = Elbow_Pitch_Left_MIN;
    Serial.print(" SP: ");
    Serial.print(iElbowPitchLeft);
    Serial.print(" ");
    Serial.println(ServoList[iElbowPitchLeft].target_position);
    // ServoList[iElbowPitchLeft].sMotor.writeMicroseconds(ServoList[iElbowPitchLeft].target_position);
    leftElbowActive = true;
    // elbowmotionloop();
}

void LeftElbowPositionMax()
{
    ServoList[iElbowPitchLeft].target_position = Elbow_Pitch_Left_MAX;
    Serial.print(" SP: ");
    Serial.print(iElbowPitchLeft);
    Serial.print(" ");
    Serial.println(ServoList[iElbowPitchLeft].target_position);
    // ServoList[iElbowPitchLeft].sMotor.writeMicroseconds(ServoList[iElbowPitchLeft].target_position);
    leftElbowActive = true;
    // elbowmotionloop();
}

void RightElbowPositionMax()
{
    ServoList[iElbowPitchRight].target_position = Elbow_Pitch_Right_MAX;
    Serial.print(" SP: ");
    Serial.print(iElbowPitchRight);
    Serial.print(" ");
    Serial.println(ServoList[iElbowPitchRight].target_position);
    // ServoList[iElbowPitchRight].sMotor.writeMicroseconds(ServoList[iElbowPitchRight].target_position);
    rightElbowActive = true;
    // elbowmotionloop();
}
void RightElbowPositionMin()
{
    ServoList[iElbowPitchRight].target_position = Elbow_Pitch_Right_MIN;
    Serial.print(" SP: ");
    Serial.print(iElbowPitchRight);
    Serial.print(" ");
    Serial.println(ServoList[iElbowPitchRight].target_position);
    // ServoList[iElbowPitchRight].sMotor.writeMicroseconds(ServoList[iElbowPitchRight].target_position);
    rightElbowActive = true;
    // elbowmotionloop();
}

void ForearmMotionloop()
{

    // If no trim button is actively pressed, re-center the elbows
    if (!rightForearmActive)
    {
        // Serial.println("Re-center right elbow");
        ServoList[iWristPitchRight].target_position = Wrist_Pitch_Right_MID;
    }

    if (!leftForearmActive)
    {
        // Serial.println("Re-center left Wrist");
        ServoList[iWristPitchLeft].target_position = Wrist_Pitch_Left_MID;
    }


}
void ForearmCenter()
{
    rightForearmActive = false;
    leftForearmActive = false;
    ForearmMotionloop();
}

void LeftForearmPositionMin()
{
    ServoList[iWristPitchLeft].target_position = Wrist_Pitch_Left_MIN;
    Serial.print(" SP: ");
    Serial.print(iWristPitchLeft);
    Serial.print(" ");
    Serial.println(ServoList[iWristPitchLeft].target_position);
    // ServoList[iWristPitchLeft].sMotor.writeMicroseconds(ServoList[iWristPitchLeft].target_position);
    leftForearmActive = true;
    // ForearmMotionloop();
}

void LeftForearmPositionMax()
{
    ServoList[iWristPitchLeft].target_position = Wrist_Pitch_Left_MAX;
    Serial.print(" SP: ");
    Serial.print(iWristPitchLeft);
    Serial.print(" ");
    Serial.println(ServoList[iWristPitchLeft].target_position);
    // ServoList[iWristPitchLeft].sMotor.writeMicroseconds(ServoList[iWristPitchLeft].target_position);
    leftForearmActive = true;
    // ForearmMotionloop();
}

void RightForearmPositionMax()
{
    ServoList[iWristPitchRight].target_position = Wrist_Pitch_Right_MAX;
    Serial.print(" SP: ");
    Serial.print(iWristPitchRight);
    Serial.print(" ");
    Serial.println(ServoList[iWristPitchRight].target_position);
    // ServoList[iWristPitchRight].sMotor.writeMicroseconds(ServoList[iWristPitchRight].target_position);
    rightForearmActive = true;
    // ForearmMotionloop();
}
void RightForearmPositionMin()
{
    ServoList[iWristPitchRight].target_position = Wrist_Pitch_Right_MIN;
    Serial.print(" SP: ");
    Serial.print(iWristPitchRight);
    Serial.print(" ");
    Serial.println(ServoList[iWristPitchRight].target_position);
    // ServoList[iWristPitchRight].sMotor.writeMicroseconds(ServoList[iWristPitchRight].target_position);
    rightForearmActive = true;
    // ForearmMotionloop();
}

void WristMotionloop()
{

    // If no trim button is actively pressed, re-center the elbows
    if (!rightWristActive)
    {
        // Serial.println("Re-center right elbow");
        ServoList[iWristRollRight].target_position = Wrist_Roll_Right_MID;
    }

    if (!leftWristActive)
    {
        // Serial.println("Re-center left Wrist");
        ServoList[iWristRollLeft].target_position = Wrist_Roll_Left_MID;
    }


}
void WristCenter()
{
    rightWristActive = false;
    leftWristActive = false;
    WristMotionloop();

}

void LeftWristPositionMin()
{
    ServoList[iWristRollLeft].target_position = Wrist_Roll_Left_MIN;
    Serial.print(" SP: ");
    Serial.print(iWristRollLeft);
    Serial.print(" ");
    Serial.println(ServoList[iWristRollLeft].target_position);
    // ServoList[iWristRollLeft].sMotor.writeMicroseconds(ServoList[iWristRollLeft].target_position);
    leftWristActive = true;
    // WristMotionloop();
}

void LeftWristPositionMax()
{
    ServoList[iWristRollLeft].target_position = Wrist_Roll_Left_MAX;
    Serial.print(" SP: ");
    Serial.print(iWristRollLeft);
    Serial.print(" ");
    Serial.println(ServoList[iWristRollLeft].target_position);
    // ServoList[iWristRollLeft].sMotor.writeMicroseconds(ServoList[iWristRollLeft].target_position);
    leftWristActive = true;
    // WristMotionloop();
}

void RightWristPositionMax()
{
    ServoList[iWristRollRight].target_position = Wrist_Roll_Right_MAX;
    Serial.print(" SP: ");
    Serial.print(iWristRollRight);
    Serial.print(" ");
    Serial.println(ServoList[iWristRollRight].target_position);
    // ServoList[iWristRollRight].sMotor.writeMicroseconds(ServoList[iWristRollRight].target_position);
    rightWristActive = true;
    // WristMotionloop();
}
void RightWristPositionMin()
{
    ServoList[iWristRollRight].target_position = Wrist_Roll_Right_MIN;
    Serial.print(" SP: ");
    Serial.print(iWristRollRight);
    Serial.print(" ");
    Serial.println(ServoList[iWristRollRight].target_position);
    // ServoList[iWristRollRight].sMotor.writeMicroseconds(ServoList[iWristRollRight].target_position);
    rightWristActive = true;
    // WristMotionloop();
}

void updateServoPosition(ServoParameters &servo)
{
    // Calculate the step size based on speed and easing factor
    float step = servo.speed * servo.easing;
    Serial.println(" easing = ");
    Serial.println(servo.easing);

    // Adjust the current position toward the target position
    if (servo.current_position < servo.target_position)
    {
        servo.current_position += step;
        if (servo.current_position > servo.target_position)
            servo.current_position = servo.target_position;
    }
    else if (servo.current_position > servo.target_position)
    {
        servo.current_position -= step;
        if (servo.current_position < servo.target_position)
            servo.current_position = servo.target_position;
    }

    // Write the updated position to the servo
    servo.sMotor.writeMicroseconds(static_cast<int>(servo.current_position));
}


void ServoAttach(uint8_t ServoNum)
{

    if (ServoNum == 0)
    { // attach all
        for (uint8_t x = 0; x < NUM_SERVOS; x++)
        {
            ServoList[x].sMotor.attach(ServoPins[x]);

            Serial.print(" attach debug sMotor ID: ");//debug code for servo attach
            Serial.println(x);
            Serial.print("Servo pin number: ");
            Serial.println(ServoPins[x]);

            ServoList[x].isAttached = 1;
            ServoList[x].sMotor.writeMicroseconds(ServoList[x].target_position);
            Serial.print("Servo target position: ");
            Serial.println(ServoList[x].target_position);
            delay(50);
        }
            Serial.println("Servo setup complete ");
    }
    else
    {
        ServoList[ServoNum].sMotor.attach(ServoPins[ServoNum]);
            Serial.print(" attach debug ServoNum ID: ");//debug code for servo attach
            Serial.println(ServoNum);
            Serial.print("Servo pin number: ");
            Serial.println(ServoPins[ServoNum]);
        ServoList[ServoNum].isAttached = true;
        ServoList[ServoNum].sMotor.writeMicroseconds(ServoList[ServoNum].target_position);
    }
       Serial.println("Servo attach complete ");
}

void ServoDetach(uint8_t ServoNum)
{
    if (ServoNum == 0)
    { // detach all
        for (int i = 0; i < NUM_SERVOS; i++)
        {
            ServoList[i].sMotor.detach();
            ServoList[i].isAttached = 0;
        }
    }
    else
    {
        ServoList[ServoNum].sMotor.detach();
        ServoList[ServoNum].isAttached = false;
    }
}

void SetServoPosition(int ServoID, float target_position)
{
    Serial.print(" SP: ");
    Serial.print(ServoID);
    Serial.print(" ");
    Serial.println(target_position);
    ServoList[ServoID].sMotor.writeMicroseconds(target_position);
}
