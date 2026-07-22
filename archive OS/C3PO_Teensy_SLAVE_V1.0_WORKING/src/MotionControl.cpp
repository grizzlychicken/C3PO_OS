// MotionControl.cpp
// Functions to control Servos and other Motions
// Comminicating to ESP32s over RS-485
// Arranged as 4 banks of Servos

#include <MotionControl.hpp>
#include <Arduino.h>
#include "Definitions.h"
#include <math.h>
// #include <PWMServo.h>
#include <Servo.h>
#include <Events.hpp>


uint8_t gHeadRunMode = HEADIDLE;
uint8_t gShoulderSwitchMode = SHOULDERSWITCHLEFT;


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

// Now controlled by main board
// // Head Servos
// Servo servoHeadRotate;
// Servo servoHeadPitch1;
// Servo servoHeadPitch2;
// Servo servoHeadPitch3;

// // Torso Servos
// Servo servoTorsoLRRight;
// Servo servoTorsoLRLeft;
// Servo servoTorsoFBRight;
// Servo servoTorsoFBLeft;
// Servo servoTorsoHipShift;
// Servo servoTorsoRotate;

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
};

// Add new servos to ServoParameters

// ServoParameters sTorsoLRRight = {iTorsoLRRight, servoTorsoLRRight, Torso_LR_Right_MID, Torso_LR_Right_MID, Torso_LR_Right_MID, 0, 0, Torso_LR_Right_MIN, Torso_LR_Right_MAX, false, TEENSY_MAIN};
// ServoParameters sTorsoLRLeft = {iTorsoLRLeft, servoTorsoLRLeft, Torso_LR_Left_MID,  Torso_LR_Left_MID,  Torso_LR_Left_MID,  0,  0,  Torso_LR_Left_MIN,   Torso_LR_Left_MAX, false, TEENSY_MAIN };
// ServoParameters sTorsoFBRight = {iTorsoFBRight, servoTorsoFBRight, Torso_FB_Right_MID, Torso_FB_Right_MID, Torso_FB_Right_MID, 0, 0, Torso_FB_Right_MIN, Torso_FB_Right_MAX, false, TEENSY_MAIN};
// ServoParameters sTorsoFBLeft = {iTorsoFBLeft, servoTorsoFBLeft,  Torso_FB_Left_MID, Torso_FB_Left_MID, Torso_FB_Left_MID, 0, 0, Torso_FB_Left_MIN, Torso_FB_Left_MAX, false, TEENSY_MAIN};
// ServoParameters sTorsoHipShift = {iTorsoHipShift, servoTorsoHipShift, Torso_Hip_Shift_MID, Torso_Hip_Shift_MID, Torso_Hip_Shift_MID, 0, 0, Torso_Hip_Shift_MIN, Torso_Hip_Shift_MAX, false, TEENSY_MAIN};
// ServoParameters sTorsoRotate = {iTorsoRotate, servoTorsoRotate, Torso_Rotate_MID, Torso_Rotate_MID, Torso_Rotate_MID, 0, 0, Torso_Rotate_MIN, Torso_Rotate_MAX, false, TEENSY_MAIN};

ServoParameters sShoulderPitch1Right = {iShoulderPitch1Right, servoShoulderPitch1Right, Shoulder_Pitch1_Right_MID, Shoulder_Pitch1_Right_MID, Shoulder_Pitch1_Right_MID, 0, 0, Shoulder_Pitch1_Right_MIN, Shoulder_Pitch1_Right_MAX, false, TEENSY_SLAVE1};
ServoParameters sShoulderPitch2Right = {iShoulderPitch2Right, servoShoulderPitch2Right, Shoulder_Pitch2_Right_MID, Shoulder_Pitch2_Right_MID, Shoulder_Pitch2_Right_MID, 0, 0, Shoulder_Pitch2_Right_MIN, Shoulder_Pitch2_Right_MAX, false, TEENSY_SLAVE1};
ServoParameters sShoulderPitch1Left = {iShoulderPitch1Left, servoShoulderPitch1Left, Shoulder_Pitch1_Left_MID, Shoulder_Pitch1_Left_MID, Shoulder_Pitch1_Left_MID, 0, 0, Shoulder_Pitch1_Left_MIN, Shoulder_Pitch1_Left_MAX, false, TEENSY_SLAVE1};
ServoParameters sShoulderPitch2Left = {iShoulderPitch2Left, servoShoulderPitch2Left, Shoulder_Pitch2_Left_MID, Shoulder_Pitch2_Left_MID, Shoulder_Pitch2_Left_MID, 0, 0, Shoulder_Pitch2_Left_MIN, Shoulder_Pitch2_Left_MAX, false, TEENSY_SLAVE1};
ServoParameters sShoulderRollRight = {iShoulderRollRight, servoShoulderRollRight, Shoulder_Roll_Right_MID, Shoulder_Roll_Right_MID, Shoulder_Roll_Right_MID, 0, 0, Shoulder_Roll_Right_MIN, Shoulder_Roll_Right_MAX, false, TEENSY_SLAVE1};
ServoParameters sShoulderRollLeft = {iShoulderRollLeft, servoShoulderRollLeft, Shoulder_Roll_Left_MID, Shoulder_Roll_Left_MID, Shoulder_Roll_Left_MID, 0, 0, Shoulder_Roll_Left_MIN, Shoulder_Roll_Left_MAX, false, TEENSY_SLAVE1};

ServoParameters sElbowPitchRight = {iElbowPitchRight, servoElbowPitchRight, Elbow_Pitch_Right_MID, Elbow_Pitch_Right_MID, Elbow_Pitch_Right_MID, 0.01, 0.01, Elbow_Pitch_Right_MIN, Elbow_Pitch_Right_MAX, false, TEENSY_SLAVE1};
ServoParameters sElbowPitchLeft = {iElbowPitchLeft, servoElbowPitchLeft, Elbow_Pitch_Left_MID, Elbow_Pitch_Left_MID, Elbow_Pitch_Left_MID, 0.1, 0.1, Elbow_Pitch_Left_MIN, Elbow_Pitch_Left_MAX, false, TEENSY_SLAVE1};
ServoParameters sWristPitchRight = {iWristPitchRight, servoWristPitchRight, Wrist_Pitch_Right_MID, Wrist_Pitch_Right_MID, Wrist_Pitch_Right_MID, 0, 0, Wrist_Pitch_Right_MIN, Wrist_Pitch_Right_MAX, false, TEENSY_SLAVE1};
ServoParameters sWristPitchLeft = {iWristPitchLeft, servoWristPitchLeft, Wrist_Pitch_Left_MID, Wrist_Pitch_Left_MID, Wrist_Pitch_Left_MID, 0, 0, Wrist_Pitch_Left_MIN, Wrist_Pitch_Left_MAX, false, TEENSY_SLAVE1};
ServoParameters sWristRollRight = {iWristRollRight, servoWristRollRight, Wrist_Roll_Right_MID, Wrist_Roll_Right_MID, Wrist_Roll_Right_MID, 1, 0.1, Wrist_Roll_Right_MIN, Wrist_Roll_Right_MAX, false, TEENSY_SLAVE1};
ServoParameters sWristRollLeft = {iWristRollLeft, servoWristRollLeft, Wrist_Roll_Left_MID, Wrist_Roll_Left_MID, Wrist_Roll_Left_MID, 1, 0.1, Wrist_Roll_Left_MIN, Wrist_Roll_Left_MAX, false, TEENSY_SLAVE1};

// ServoParameters sHeadRotate = {iHeadRotate, servoHeadRotate, Head_Rotate_MID, Head_Rotate_MID, Head_Rotate_MID, 0, 0, Head_Rotate_MIN, Head_Rotate_MAX, false, TEENSY_MAIN};
// ServoParameters sHeadPitch1 = {iHeadPitch1, servoHeadPitch1, Head_Pitch1_MID, Head_Pitch1_MID,Head_Pitch1_MID, 0, 0, Head_Pitch1_MIN, Head_Pitch1_MAX, false, TEENSY_MAIN};
// ServoParameters sHeadPitch2 = {iHeadPitch2, servoHeadPitch2, Head_Pitch2_MID, Head_Pitch2_MID, Head_Pitch2_MID, 0, 0, Head_Pitch2_MIN, Head_Pitch2_MAX, false, TEENSY_MAIN};
// ServoParameters sHeadPitch3 = {iHeadPitch3, servoHeadPitch3, Head_Pitch3_MID, Head_Pitch3_MID, Head_Pitch3_MID, 0, 0, Head_Pitch3_MIN, Head_Pitch3_MAX, false, TEENSY_MAIN};

ServoParameters ServoList[] = {
    // sTorsoLRRight,
    // sTorsoLRLeft,
    // sTorsoFBRight,
    // sTorsoFBLeft,
    // sTorsoHipShift,
    // sTorsoRotate,

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

    // sHeadRotate,
    // sHeadPitch1,
    // sHeadPitch2,
    // sHeadPitch3,
};

// Total number of Servos
const int NUM_SERVOS = sizeof(ServoList) / sizeof(ServoList[0]);

uint8_t ServoPins[] = {
    // pinTorsoLRRight,
    // pinTorsoLRLeft,
    // pinTorsoFBRight,
    // pinTorsoFBLeft,
    // pinTorsoHipShift,
    // pinTorsoRotate,

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

    // pinHeadRotate,
    // pinHeadPitch1,
    // pinHeadPitch2,
    // pinHeadPitch3,

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
    {
        if (gHeadRunMode == HEADCONTROL)
        {
            // HEAD FUNCTIONS
            // PitchAngle = (ChannelData(cHEADPITCH) - SBUSCENTER) / 2;
            // RollAngle = (ChannelData(cHEADROLL) - SBUSCENTER) / 2;
            // CalculateHeadPosition(PitchAngle, RollAngle);

            // // Set Head Yaw
            // ServoList[iHeadRotate].target_position = map(ChannelData(cHEADYAW), SBUSMIN, SBUSMAX, ServoList[iHeadRotate].MinDegrees, ServoList[iHeadRotate].MaxDegrees);

            // // TORSO FUNCTIONS
            // CalculateTorsoLeanFB((ChannelData(cTORSOFB) - SBUSCENTER) / 2);
            // CalculateTorsoLeanLR((ChannelData(cTORSOLR) - SBUSCENTER) / 2);

            // // Set Torso Rotate
            // ServoList[iTorsoRotate].target_position = map(ChannelData(cTORSOROT), SBUSMIN, SBUSMAX, ServoList[iTorsoRotate].MinDegrees, ServoList[iTorsoRotate].MaxDegrees);

            // // Set Hip Shift
            // ServoList[iTorsoHipShift].target_position = map(ChannelData(cTORSOHIPSHIFT), SBUSMIN, SBUSMAX, ServoList[iTorsoHipShift].MinDegrees, ServoList[iTorsoHipShift].MaxDegrees);

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
//             if (gHeadRunMode == HEADAUTO)
//             {
//                 // ServoAttach(0);  // Ensure servos are attached

//                 // Process random head movements
//                 for (uint8_t x = 0; x <= 3; x++)
//                 {
//                     // Update each axis if enough time has passed
//                     if (gAutoHeadElapsed[x] > gAutoHeadInterval[x])
//                     {
//                         switch (x)
//                         {
//                         case 0: // Yaw (Head Rotate)
//                             ProcessAutoHeadMovement(iHeadRotate, Head_Rotate_MID, deviation);
//                             gAutoHeadInterval[x] = random(8000, 15000); // Randomize next interval
//                             break;
//                         case 1: // Pitch1
//                             ProcessAutoHeadMovement(iHeadPitch1, Head_Pitch1_MID, deviation);
//                             gAutoHeadInterval[x] = random(3000, 18000); // Randomize next interval
//                             break;
//                         case 2: // Pitch2
//                             ProcessAutoHeadMovement(iHeadPitch2, Head_Pitch2_MID, deviation);
//                             gAutoHeadInterval[x] = random(15000, 25000); // Randomize next interval
//                             break;
//                         case 3: // Pitch3
//                             ProcessAutoHeadMovement(iHeadPitch3, Head_Pitch3_MID, deviation);
//                             gAutoHeadInterval[x] = random(15000, 25000); // Randomize next interval
//                             break;
//                         }

//                         // Reset the timer for this axis
//                         gAutoHeadElapsed[x] = 0;
//                     }
//                 }

//                 // Gradually move each servo toward its target position with easing
//                 for (int i = 0; i < 4; i++)
//                 {
//                     int servoIndex;
//                     if (i == 0)
//                     {
//                         servoIndex = iHeadRotate;
//                     }
//                     else if (i == 1)
//                     {
//                         servoIndex = iHeadPitch1;
//                     }
//                     else if (i == 2)
//                     {
//                         servoIndex = iHeadPitch2;
//                     }
//                     else
//                     {
//                         servoIndex = iHeadPitch3;
//                     }

//                     if (ServoList[servoIndex].current_position != ServoList[servoIndex].target_position)
//                     {
//                         // Calculate easing progress based on elapsed time and easingSteps
//                         float easingProgress = (float)(millis() - gAutoHeadElapsed[i]) / gAutoHeadInterval[i]; // Easing progress [0,1]
//                         int easingStepIndex = (int)(easingProgress * (easingSteps - 1));                       // Get index in easing curve array
//                         easingStepIndex = constrain(easingStepIndex, 0, easingSteps - 1);                      // Ensure index is in bounds

//                         // Calculate eased position
//                         float easedFactor = easingCurve[easingStepIndex];
//                         float easedPosition = ServoList[servoIndex].current_position +
//                                               (ServoList[servoIndex].target_position - ServoList[servoIndex].current_position) * easedFactor;

//                         // Map eased position to microseconds
//                         int currentMicroseconds = map(easedPosition, ServoList[servoIndex].MinDegrees, ServoList[servoIndex].MaxDegrees, 1000, 2000);

//                         // Set the servo position in microseconds
//                         ServoList[servoIndex].sMotor.writeMicroseconds(currentMicroseconds);

//                         // Update the current position to reflect the eased value
//                         ServoList[servoIndex].current_position = map(currentMicroseconds, 1000, 2000, ServoList[servoIndex].MinDegrees, ServoList[servoIndex].MaxDegrees);
//                     }
//                 }
//             }
//         }
//     }
// }

// void ProcessAutoHeadMovement(uint8_t servoIndex, int midPosition, int deviation)
// {
//     // Generate a random new target only when needed
//     int newTarget = random(midPosition - deviation, midPosition + deviation);

//     // Check if the new target differs from the current target
//     if (newTarget != ServoList[servoIndex].target_position)
//     {
//         ServoList[servoIndex].target_position = newTarget;

//         // Optionally, print the target value for debugging
//         Serial.print("New target for servo ");
//         Serial.print(servoIndex);
//         Serial.print(": ");
//         Serial.println(newTarget);
        



void UpdateServos()
{
    if (gHeadRunMode == HEADCONTROL)
    {
        for (uint8_t i = 0; i < NUM_SERVOS; i++)
        {
            if (abs(ServoList[i].target_position - ServoList[i].current_position) > 1)
            {
                SetServoPosition(i, ServoList[i].target_position);
                ServoList[i].current_position = ServoList[i].target_position;
            }
        }
    }
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

// Mix the pitch and roll together to calculate the required servo positions.
// 120° servo mix

// void CalculateHeadPosition(float pitch, float roll)
// {
//     ServoList[iHeadPitch1].target_position = constrain(ServoList[iHeadPitch1].center_position - pitch, ServoList[iHeadPitch1].MinDegrees, ServoList[iHeadPitch1].MaxDegrees);
//     ServoList[iHeadPitch2].target_position = constrain(ServoList[iHeadPitch2].center_position - 0.5 * pitch + 0.866 * roll, ServoList[iHeadPitch2].MinDegrees, ServoList[iHeadPitch2].MaxDegrees);
//     ServoList[iHeadPitch3].target_position = constrain(ServoList[iHeadPitch3].center_position - 0.5 * pitch - 0.866 * roll, ServoList[iHeadPitch3].MinDegrees, ServoList[iHeadPitch3].MaxDegrees);

// #if 0
//     Serial.print("P: ");
//     Serial.print(pitch);
//     Serial.print(" R: ");
//     Serial.print(roll);
//     Serial.print(" SP1 ");
//     Serial.print(ServoList[iHeadPitch1].target_position);
//     Serial.print(" SP2 ");
//     Serial.print(ServoList[iHeadPitch2].target_position);
//     Serial.print(" SP3 ");
//     Serial.println(ServoList[iHeadPitch3].target_position);
// #endif
// }

// // Mix the Left Right Lean Servos
// void CalculateTorsoLeanLR(float Lean)
// {
//     ServoList[iTorsoLRLeft].target_position = constrain(ServoList[iTorsoLRLeft].center_position + Lean, ServoList[iTorsoLRLeft].MinDegrees, ServoList[iTorsoLRLeft].MaxDegrees);
//     ServoList[iTorsoLRRight].target_position = constrain(ServoList[iTorsoLRRight].center_position + Lean, ServoList[iTorsoLRRight].MinDegrees, ServoList[iTorsoLRRight].MaxDegrees);

// #if 0
//     Serial.print("L: ");
//     Serial.print(Lean);
//     Serial.print(" SLL ");
//     Serial.print(sTorsoLRLeft.target_position);
//     Serial.print(" SLR ");
//     Serial.println(sTorsoLRRight.target_position);
// #endif
// }

// // Mix the Forward/Back Lean Servos
// void CalculateTorsoLeanFB(float Lean)
// {
//     ServoList[iTorsoFBLeft].target_position = constrain(ServoList[iTorsoFBLeft].center_position + Lean, ServoList[iTorsoFBLeft].MinDegrees, ServoList[iTorsoFBLeft].MaxDegrees);
//     ServoList[iTorsoFBRight].target_position = constrain(ServoList[iTorsoFBRight].center_position - Lean, ServoList[iTorsoFBRight].MinDegrees, ServoList[iTorsoFBRight].MaxDegrees);

// #if 0
//     Serial.print("L: ");
//     Serial.print(Lean);
//     Serial.print(" SFBL ");
//     Serial.print(ServoList[iTorsoFBLeft].target_position);
//     Serial.print(" SFBR ");
//     Serial.println(ServoList[iTorsoFBRight].target_position);
// #endif
// }


// void SetShoulderRunModeLeft()
// {
//     // Set the shoulder mode to left
//     gShoulderSwitchMode = SHOULDERSWITCHLEFT;

//     // Flexion Function (Shoulder Roll)
//     ServoList[iShoulderRollLeft].target_position = map(ChannelData(cSHFL), SBUSMIN, SBUSMAX, ServoList[iShoulderRollLeft].MinDegrees, ServoList[iShoulderRollLeft].MaxDegrees);

//     // Abduction Function (Shoulder Pitch 1 and 2 work in opposite directions)
//     int abductionValueLeft = ChannelData(cSHAB);
//     int targetPitch1Left = 1460;  // Default neutral for Pitch 1
//     int targetPitch2Left = 1130;  // Default neutral for Pitch 2

//     // If abduction input is active (positive direction)
//     if (abductionValueLeft > SBUSNEUTRAL) {
//         targetPitch1Left = map(abductionValueLeft, SBUSNEUTRAL, SBUSMAX, 1460, 2280);  // Abduction range for Pitch 1
//         targetPitch2Left = map(abductionValueLeft, SBUSNEUTRAL, SBUSMAX, 1130, 641);   // Abduction range for Pitch 2 (opposite)
//     }

//     // External Rotation Function - Both servos move together
//     int externalRotationValueLeft = ChannelData(cSHER);

//     if (externalRotationValueLeft > SBUSNEUTRAL && isExternalRotationModeLeft()) {
//         // If external rotation mode is active
//         targetPitch1Left = map(externalRotationValueLeft, SBUSNEUTRAL, SBUSMAX, 1460, 640);  // External rotation for Pitch 1
//         targetPitch2Left = map(externalRotationValueLeft, SBUSNEUTRAL, SBUSMAX, 1130, 640);  // External rotation for Pitch 2 (same direction)
//     }

//     // Set the target positions for Pitch 1 and Pitch 2 servos
//     ServoList[iShoulderPitch1Left].target_position = targetPitch1Left;
//     ServoList[iShoulderPitch2Left].target_position = targetPitch2Left;
// }

// // Helper function to determine if the shoulder is in external rotation mode (left)
// bool isExternalRotationModeLeft() 
// {
//     // Get the value from the left shoulder external rotation channel (cSHER)
//     int erInputLeft = ChannelData(cSHER);

//     // Define the "quiet" range around 992 ± 100 for the left shoulder
//     const int quietLowerBound = 892;
//     const int quietUpperBound = 1092;

//     // Return true if external rotation is active, meaning the input is outside the quiet range
//     return (erInputLeft < quietLowerBound || erInputLeft > quietUpperBound);
// }


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

// void SetShoulderRunModeRight() 
// {
//     gShoulderSwitchMode = SHOULDERSWITCHRIGHT;

//     // Shoulder Roll - Flexion Control
//     ServoList[iShoulderRollRight].target_position = map(ChannelData(cSHFL), SBUSMIN, SBUSMAX, ServoList[iShoulderRollRight].MinDegrees, ServoList[iShoulderRollRight].MaxDegrees);

//     // Abduction Function (Shoulder Pitch 1 and 2 work in opposite directions)
//     int abductionValueRight = ChannelData(cSHAB);
//     int targetPitch1Right = 1420;  // Default neutral for Pitch 1
//     int targetPitch2Right = 2000;  // Default neutral for Pitch 2

//     if (abductionValueRight > SBUSNEUTRAL) {  
//         // When the stick is pushed in the positive direction (abduction)
//         targetPitch1Right = map(abductionValueRight, SBUSNEUTRAL, SBUSMAX, 1420, 2400);  // Shoulder Pitch 1 (neutral to full abduction)
//         targetPitch2Right = map(abductionValueRight, SBUSNEUTRAL, SBUSMAX, 2000, 640); // Shoulder Pitch 2 (opposite)
//     }

//     // External Rotation Function (Both servos should move together in the same direction)
//     int externalRotationValueRight = ChannelData(cSHER);

//     if (externalRotationValueRight > SBUSNEUTRAL && isExternalRotationModeRight()) {  
//         // When the stick is pushed in the external rotation direction (992 to 1811)
//         targetPitch1Right = map(externalRotationValueRight, SBUSNEUTRAL, SBUSMAX, 1420, 2200);  // Pitch 1 external rotation
//         targetPitch2Right = map(externalRotationValueRight, SBUSNEUTRAL, SBUSMAX, 2000, 2400);  // Pitch 2 external rotation
//     }

//     // Update the target positions for Pitch 1 and Pitch 2 servos
//     ServoList[iShoulderPitch1Right].target_position = targetPitch1Right;
//     ServoList[iShoulderPitch2Right].target_position = targetPitch2Right;
    
// }

// // Helper function to determine if the shoulder is in external rotation mode
// bool isExternalRotationModeRight() 
// {
//     // Get the value from the external rotation channel (cSHER)
//     int erInput = ChannelData(cSHER);

//     // Define the "quiet" range around 992 ± 100
//     const int quietLowerBound = 892;
//     const int quietUpperBound = 1092;

//     // Return true if external rotation is active, meaning the input is outside the quiet range
//     return (erInput < quietLowerBound || erInput > quietUpperBound);
// }


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
// void TestAllServos(void)
// {
//     Serial.println("testing all servos...");
//     Serial.println("Set all to min");
//     for (int i = 0; i < NUM_SERVOS; i++)
//     {
//         delay(1000);
//         TxServoPosition(ServoList[i].servoBank, ServoList[i].servoID, ServoList[i].MinDegrees);
//     }
//     delay(2000);
//     Serial.println("Set all to max");
//     for (int i = 0; i < NUM_SERVOS; i++)
//     {
//         delay(1000);
//         TxServoPosition(ServoList[i].servoBank, ServoList[i].servoID, ServoList[i].MaxDegrees);
//     }
//     delay(2000);
//     Serial.println("Set all to mid");
//     for (int i = 0; i < NUM_SERVOS; i++)
//     {
//         delay(1000);
//         TxServoPosition(ServoList[i].servoBank, ServoList[i].servoID, ServoList[i].center_position);
//     }
// }

void elbowmotionloop()
{
    // Update the right elbow servo
    if (!rightElbowActive)
    {
        ServoList[iElbowPitchRight].target_position = Elbow_Pitch_Right_MID;
    }
    updateServoPosition(ServoList[iElbowPitchRight]);

    // Update the left elbow servo
    if (!leftElbowActive)
    {
        ServoList[iElbowPitchLeft].target_position = Elbow_Pitch_Left_MID;
    }
    updateServoPosition(ServoList[iElbowPitchLeft]);
}

void ElbowCenter()
{
    rightElbowActive = false;
    leftElbowActive = false;
    elbowmotionloop();

    // ServoList[iElbowPitchRight].target_position = Elbow_Pitch_Right_MID;
    // // Serial.print(" SP: ");
    // // Serial.print(iElbowPitchRight);
    // // Serial.print(" ");
    // // Serial.println(ServoList[iElbowPitchRight].target_position);
    // ServoList[iElbowPitchRight].sMotor.writeMicroseconds(ServoList[iElbowPitchRight].target_position);

    // ServoList[iElbowPitchLeft].target_position = Elbow_Pitch_Left_MID;
    // // Serial.print(" SP: ");
    // // Serial.print(iElbowPitchLeft);
    // // Serial.print(" ");
    // // Serial.println(ServoList[iElbowPitchLeft].target_position);
    // ServoList[iElbowPitchLeft].sMotor.writeMicroseconds(ServoList[iElbowPitchLeft].target_position);
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
    elbowmotionloop();
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
    elbowmotionloop();
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
    elbowmotionloop();
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
    elbowmotionloop();
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

    ServoList[iWristPitchRight].target_position = Wrist_Pitch_Right_MID;
    // Serial.print(" SP: ");
    // Serial.print(iWristPitchRight);
    // Serial.print(" ");
    // Serial.println(ServoList[iWristPitchRight].target_position);
    ServoList[iWristPitchRight].sMotor.writeMicroseconds(ServoList[iWristPitchRight].target_position);

    ServoList[iWristPitchLeft].target_position = Wrist_Pitch_Left_MID;
    // Serial.print(" SP: ");
    // Serial.print(iWristPitchLeft);
    // Serial.print(" ");
    // Serial.println(ServoList[iWristPitchLeft].target_position);
    ServoList[iWristPitchLeft].sMotor.writeMicroseconds(ServoList[iWristPitchLeft].target_position);
}

void LeftForearmPositionMin()
{
    ServoList[iWristPitchLeft].target_position = Wrist_Pitch_Left_MIN;
    Serial.print(" SP: ");
    Serial.print(iWristPitchLeft);
    Serial.print(" ");
    Serial.println(ServoList[iWristPitchLeft].target_position);
    ServoList[iWristPitchLeft].sMotor.writeMicroseconds(ServoList[iWristPitchLeft].target_position);
    leftForearmActive = true;
    ForearmMotionloop();
}

void LeftForearmPositionMax()
{
    ServoList[iWristPitchLeft].target_position = Wrist_Pitch_Left_MAX;
    Serial.print(" SP: ");
    Serial.print(iWristPitchLeft);
    Serial.print(" ");
    Serial.println(ServoList[iWristPitchLeft].target_position);
    ServoList[iWristPitchLeft].sMotor.writeMicroseconds(ServoList[iWristPitchLeft].target_position);
    leftForearmActive = true;
    ForearmMotionloop();
}

void RightForearmPositionMax()
{
    ServoList[iWristPitchRight].target_position = Wrist_Pitch_Right_MAX;
    Serial.print(" SP: ");
    Serial.print(iWristPitchRight);
    Serial.print(" ");
    Serial.println(ServoList[iWristPitchRight].target_position);
    ServoList[iWristPitchRight].sMotor.writeMicroseconds(ServoList[iWristPitchRight].target_position);
    rightForearmActive = true;
    ForearmMotionloop();
}
void RightForearmPositionMin()
{
    ServoList[iWristPitchRight].target_position = Wrist_Pitch_Right_MIN;
    Serial.print(" SP: ");
    Serial.print(iWristPitchRight);
    Serial.print(" ");
    Serial.println(ServoList[iWristPitchRight].target_position);
    ServoList[iWristPitchRight].sMotor.writeMicroseconds(ServoList[iWristPitchRight].target_position);
    rightForearmActive = true;
    ForearmMotionloop();
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

    ServoList[iWristRollRight].target_position = Wrist_Roll_Right_MID;
    // Serial.print(" SP: ");
    // Serial.print(iWristRollRight);
    // Serial.print(" ");
    // Serial.println(ServoList[iWristRollRight].target_position);
    ServoList[iWristRollRight].sMotor.writeMicroseconds(ServoList[iWristRollRight].target_position);

    ServoList[iWristRollLeft].target_position = Wrist_Roll_Left_MID;
    // Serial.print(" SP: ");
    // Serial.print(iWristRollLeft);
    // Serial.print(" ");
    // Serial.println(ServoList[iWristRollLeft].target_position);
    ServoList[iWristRollLeft].sMotor.writeMicroseconds(ServoList[iWristRollLeft].target_position);
}

void LeftWristPositionMin()
{
    ServoList[iWristRollLeft].target_position = Wrist_Roll_Left_MIN;
    Serial.print(" SP: ");
    Serial.print(iWristRollLeft);
    Serial.print(" ");
    Serial.println(ServoList[iWristRollLeft].target_position);
    ServoList[iWristRollLeft].sMotor.writeMicroseconds(ServoList[iWristRollLeft].target_position);
    leftWristActive = true;
    WristMotionloop();
}

void LeftWristPositionMax()
{
    ServoList[iWristRollLeft].target_position = Wrist_Roll_Left_MAX;
    Serial.print(" SP: ");
    Serial.print(iWristRollLeft);
    Serial.print(" ");
    Serial.println(ServoList[iWristRollLeft].target_position);
    ServoList[iWristRollLeft].sMotor.writeMicroseconds(ServoList[iWristRollLeft].target_position);
    leftWristActive = true;
    WristMotionloop();
}

void RightWristPositionMax()
{
    ServoList[iWristRollRight].target_position = Wrist_Roll_Right_MAX;
    Serial.print(" SP: ");
    Serial.print(iWristRollRight);
    Serial.print(" ");
    Serial.println(ServoList[iWristRollRight].target_position);
    ServoList[iWristRollRight].sMotor.writeMicroseconds(ServoList[iWristRollRight].target_position);
    rightWristActive = true;
    WristMotionloop();
}
void RightWristPositionMin()
{
    ServoList[iWristRollRight].target_position = Wrist_Roll_Right_MIN;
    Serial.print(" SP: ");
    Serial.print(iWristRollRight);
    Serial.print(" ");
    Serial.println(ServoList[iWristRollRight].target_position);
    ServoList[iWristRollRight].sMotor.writeMicroseconds(ServoList[iWristRollRight].target_position);
    rightWristActive = true;
    WristMotionloop();
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

// void ServoDetach(uint8_t ServoNum)
// {
//     if (ServoNum == 0)
//     { // detach all
//         for (int i = 0; i < NUM_SERVOS; i++)
//         {
//             ServoList[i].sMotor.detach();
//             ServoList[i].isAttached = 0;
//         }
//     }
//     else
//     {
//         ServoList[ServoNum].sMotor.detach();
//         ServoList[ServoNum].isAttached = false;
//     }
// }

void SetServoPosition(int ServoID, float target_position)
{
    Serial.print(" SP: ");
    Serial.print(ServoID);
    Serial.print(" ");
    Serial.println(target_position);
    ServoList[ServoID].sMotor.writeMicroseconds(target_position);
}
