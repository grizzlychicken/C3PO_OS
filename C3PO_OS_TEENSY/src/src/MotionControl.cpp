// MotionControl.cpp
// Functions to control Servos and other Motions
// Comminicating to ESP32s over RS-485
// Arranged as 4 banks of Servos

#include <src/MotionControl.hpp>
#include <Teensy_PWM.h>
#include <Arduino.h>
#include "Definitions.h"
#include <math.h>
#include "BottangoCore.h"
//#include <PWMServo.h>
#include <servo.h>
#include <src/Events.hpp>
#include <src/BasicCommands.h>

uint8_t gHeadRunMode = HEADIDLE;
uint8_t gRCBottangoMode = RC_MODE;
uint8_t currentMode = RC_MODE;
// uint8_t gShoulderSwitchMode = SHOULDERSWITCHLEFT;
bool animationModeSetupDone = false; // Flag to track if setup has run
// Track when easing started for each axis (0..3)
unsigned long gAutoHeadEasingStart[4] = {0, 0, 0, 0};
// unsigned long gAutoHeadMoveTime[4] = {0, 0, 0, 0}; // ms to reach target (easing duration)
unsigned long gAutoHeadMoveTime[4] = { 450, 650, 650, 650 };// ms to reach target (easing duration)

// Variables to track if any trim button is pressed
// bool rightElbowActive = false;
// bool leftElbowActive = false;

#define MOTIONFRAMERATE 25 // ms for Motor Control Loop

unsigned long gAutoHeadLastUpdate[4] = {0, 0, 0, 0};
unsigned long MotionMillis = millis();
elapsedMillis gAutoHeadElapsed[6];  // Elapsed timer for updating the head
unsigned long gAutoHeadInterval[6]; // Random movement interval of pitch roll yaw
const int deviation = 150;          // Adjust this value to control the subtlety of movements
const int easingSteps = 100;        // Number of steps for easing
float easingCurve[easingSteps];
unsigned long lastUpdateTime = 0;
const unsigned long servoUpdateInterval = 20; // Update servos every 20ms (50Hz)

static inline bool dbgAllow(uint8_t key, uint32_t intervalMs = DBG_RATE_MS)
{
    static uint32_t last[32] = {0}; // 32 independent rate limit keys
    uint32_t now = millis();
    if ((uint32_t)(now - last[key]) >= intervalMs)
    {
        last[key] = now;
        return true;
    }
    return false;
}


// Smooth step easing function (can replace with easeInOut or any other function)
float smoothStep(float t)
{
    return t * t * (3 - 2 * t); // Adjust formula if you want different easing effects
}


// Head Servos
Servo servoHeadRotate;
Servo servoHeadPitch1;
Servo servoHeadPitch2;
Servo servoHeadPitch3;

// Torso Servos
Servo servoTorsoLRRight;
Servo servoTorsoLRLeft;
Servo servoTorsoFBRight;
Servo servoTorsoFBLeft;
Servo servoTorsoHipShift;
Servo servoTorsoRotate;

// Controlled by slave board
// // Shoulder Servo
// Servo servoShoulderPitch1Right;
// Servo servoShoulderPitch2Right;
// Servo servoShoulderPitch1Left;
// Servo servoShoulderPitch2Left;
// Servo servoShoulderRollRight;
// Servo servoShoulderRollLeft;

// // Arm Servos
// Servo servoElbowPitchRight;
// Servo servoElbowPitchLeft;
// Servo servoWristPitchRight;
// Servo servoWristPitchLeft;
// Servo servoWristRollRight;
// Servo servoWristRollLeft;

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

ServoParameters sTorsoLRRight = {iTorsoLRRight, servoTorsoLRRight, Torso_LR_Right_MID, Torso_LR_Right_MID, Torso_LR_Right_MID, 0, 0, Torso_LR_Right_MIN, Torso_LR_Right_MAX, false, TEENSY_MAIN};
ServoParameters sTorsoLRLeft = {iTorsoLRLeft, servoTorsoLRLeft, Torso_LR_Left_MID,  Torso_LR_Left_MID,  Torso_LR_Left_MID,  0,  0,  Torso_LR_Left_MIN,   Torso_LR_Left_MAX, false, TEENSY_MAIN };
ServoParameters sTorsoFBRight = {iTorsoFBRight, servoTorsoFBRight, Torso_FB_Right_MID, Torso_FB_Right_MID, Torso_FB_Right_MID, 0, 0, Torso_FB_Right_MIN, Torso_FB_Right_MAX, false, TEENSY_MAIN};
ServoParameters sTorsoFBLeft = {iTorsoFBLeft, servoTorsoFBLeft,  Torso_FB_Left_MID, Torso_FB_Left_MID, Torso_FB_Left_MID, 0, 0, Torso_FB_Left_MIN, Torso_FB_Left_MAX, false, TEENSY_MAIN};
ServoParameters sTorsoHipShift = {iTorsoHipShift, servoTorsoHipShift, Torso_Hip_Shift_MID, Torso_Hip_Shift_MID, Torso_Hip_Shift_MID, 0, 0, Torso_Hip_Shift_MIN, Torso_Hip_Shift_MAX, false, TEENSY_MAIN};
ServoParameters sTorsoRotate = {iTorsoRotate, servoTorsoRotate, Torso_Rotate_MID, Torso_Rotate_MID, Torso_Rotate_MID, 0, 0, Torso_Rotate_MIN, Torso_Rotate_MAX, false, TEENSY_MAIN};

// ServoParameters sShoulderPitch1Right = {iShoulderPitch1Right, servoShoulderPitch1Right, Shoulder_Pitch1_Right_MID, Shoulder_Pitch1_Right_MID, Shoulder_Pitch1_Right_MID, 0, 0, Shoulder_Pitch1_Right_MIN, Shoulder_Pitch1_Right_MAX, false, TEENSY_SLAVE1};
// ServoParameters sShoulderPitch2Right = {iShoulderPitch2Right, servoShoulderPitch2Right, Shoulder_Pitch2_Right_MID, Shoulder_Pitch2_Right_MID, Shoulder_Pitch2_Right_MID, 0, 0, Shoulder_Pitch2_Right_MIN, Shoulder_Pitch2_Right_MAX, false, TEENSY_SLAVE1};
// ServoParameters sShoulderPitch1Left = {iShoulderPitch1Left, servoShoulderPitch1Left, Shoulder_Pitch1_Left_MID, Shoulder_Pitch1_Left_MID, Shoulder_Pitch1_Left_MID, 0, 0, Shoulder_Pitch1_Left_MIN, Shoulder_Pitch1_Left_MAX, false, TEENSY_SLAVE1};
// ServoParameters sShoulderPitch2Left = {iShoulderPitch2Left, servoShoulderPitch2Left, Shoulder_Pitch2_Left_MID, Shoulder_Pitch2_Left_MID, Shoulder_Pitch2_Left_MID, 0, 0, Shoulder_Pitch2_Left_MIN, Shoulder_Pitch2_Left_MAX, false, TEENSY_SLAVE1};
// ServoParameters sShoulderRollRight = {iShoulderRollRight, servoShoulderRollRight, Shoulder_Roll_Right_MID, Shoulder_Roll_Right_MID, Shoulder_Roll_Right_MID, 0, 0, Shoulder_Roll_Right_MIN, Shoulder_Roll_Right_MAX, false, TEENSY_SLAVE1};
// ServoParameters sShoulderRollLeft = {iShoulderRollLeft, servoShoulderRollLeft, Shoulder_Roll_Left_MID, Shoulder_Roll_Left_MID, Shoulder_Roll_Left_MID, 0, 0, Shoulder_Roll_Left_MIN, Shoulder_Roll_Left_MAX, false, TEENSY_SLAVE1};

// ServoParameters sElbowPitchRight = {iElbowPitchRight, servoElbowPitchRight, Elbow_Pitch_Right_MID, Elbow_Pitch_Right_MID, Elbow_Pitch_Right_MID, 0, 0, Elbow_Pitch_Right_MIN, Elbow_Pitch_Right_MAX, false, TEENSY_SLAVE1};
// ServoParameters sElbowPitchLeft = {iElbowPitchLeft, servoElbowPitchLeft, Elbow_Pitch_Left_MID, Elbow_Pitch_Left_MID, Elbow_Pitch_Left_MID, 0, 0, Elbow_Pitch_Left_MIN, Elbow_Pitch_Left_MAX, false, TEENSY_SLAVE1};
// ServoParameters sWristPitchRight = {iWristPitchRight, servoWristPitchRight, Wrist_Pitch_Right_MID, Wrist_Pitch_Right_MID, Wrist_Pitch_Right_MID, 0, 0, Wrist_Pitch_Right_MIN, Wrist_Pitch_Right_MAX, false, TEENSY_SLAVE1};
// ServoParameters sWristPitchLeft = {iWristPitchLeft, servoWristPitchLeft, Wrist_Pitch_Left_MID, Wrist_Pitch_Left_MID, Wrist_Pitch_Left_MID, 0, 0, Wrist_Pitch_Left_MIN, Wrist_Pitch_Left_MAX, false, TEENSY_SLAVE1};
// ServoParameters sWristRollRight = {iWristRollRight, servoWristRollRight, Wrist_Roll_Right_MID, Wrist_Roll_Right_MID, Wrist_Roll_Right_MID, 0, 0, Wrist_Roll_Right_MIN, Wrist_Roll_Right_MAX, false, TEENSY_SLAVE1};
// ServoParameters sWristRollLeft = {iWristRollLeft, servoWristRollLeft, Wrist_Roll_Left_MID, Wrist_Roll_Left_MID, Wrist_Roll_Left_MID, 0, 0, Wrist_Roll_Left_MIN, Wrist_Roll_Left_MAX, false, TEENSY_SLAVE1};

ServoParameters sHeadRotate = {iHeadRotate, servoHeadRotate, Head_Rotate_MID, Head_Rotate_MID, Head_Rotate_MID, 0, 0, Head_Rotate_MIN, Head_Rotate_MAX, false, TEENSY_MAIN};
ServoParameters sHeadPitch1 = {iHeadPitch1, servoHeadPitch1, Head_Pitch1_MID, Head_Pitch1_MID,Head_Pitch1_MID, 0, 0, Head_Pitch1_MIN, Head_Pitch1_MAX, false, TEENSY_MAIN};
ServoParameters sHeadPitch2 = {iHeadPitch2, servoHeadPitch2, Head_Pitch2_MID, Head_Pitch2_MID, Head_Pitch2_MID, 0, 0, Head_Pitch2_MIN, Head_Pitch2_MAX, false, TEENSY_MAIN};
ServoParameters sHeadPitch3 = {iHeadPitch3, servoHeadPitch3, Head_Pitch3_MID, Head_Pitch3_MID, Head_Pitch3_MID, 0, 0, Head_Pitch3_MIN, Head_Pitch3_MAX, false, TEENSY_MAIN};

ServoParameters ServoList[] = {
    sTorsoLRRight,
    sTorsoLRLeft,
    sTorsoFBRight,
    sTorsoFBLeft,
    sTorsoHipShift,
    sTorsoRotate,

    // sShoulderPitch1Right,
    // sShoulderPitch2Right,
    // sShoulderPitch1Left,
    // sShoulderPitch2Left,
    // sShoulderRollRight,
    // sShoulderRollLeft,

    // sElbowPitchRight,
    // sElbowPitchLeft,
    //  sWristPitchRight,
    //  sWristPitchLeft,
    //  sWristRollRight,
    //   sWristRollLeft,

    sHeadRotate,
    sHeadPitch1,
    sHeadPitch2,
    sHeadPitch3,
};

// Total number of Servos
const int NUM_SERVOS = sizeof(ServoList) / sizeof(ServoList[0]);

// Safety limits for debug/protection only. These do not affect behavior unless
// a target exceeds safe bounds or a frame produces an abnormal jump.
static const int SERVO_HARD_MIN_US = 900;
static const int SERVO_HARD_MAX_US = 2100;
static const int SERVO_MAX_STEP_PER_LOOP_US = 1200;

static float gServoVelocityUsPerSec[NUM_SERVOS] = {0.0f};
static int gServoPrevPositionUs[NUM_SERVOS] = {0};
static bool gServoPrevPositionInit = false;

static int ClampPulseForServo(uint8_t servoID, float pulseUs)
{
    int minUs = (int)ServoList[servoID].MinDegrees;
    int maxUs = (int)ServoList[servoID].MaxDegrees;

    minUs = max(minUs, SERVO_HARD_MIN_US);
    maxUs = min(maxUs, SERVO_HARD_MAX_US);

    if (minUs > maxUs)
    {
        int safeMid = (SERVO_HARD_MIN_US + SERVO_HARD_MAX_US) / 2;
        return constrain((int)pulseUs, safeMid, safeMid);
    }

    return constrain((int)pulseUs, minUs, maxUs);
}

static int ClampStepFromCurrent(uint8_t servoID, float targetUs)
{
    int curUs = (int)ServoList[servoID].current_position;
    int tgtUs = ClampPulseForServo(servoID, targetUs);
    int delta = tgtUs - curUs;

    if (delta > SERVO_MAX_STEP_PER_LOOP_US)
        delta = SERVO_MAX_STEP_PER_LOOP_US;
    else if (delta < -SERVO_MAX_STEP_PER_LOOP_US)
        delta = -SERVO_MAX_STEP_PER_LOOP_US;

    return ClampPulseForServo(servoID, curUs + delta);
}

static void UpdateVelocityEstimates(float dtSec)
{
    if (dtSec <= 0.0001f)
        return;

    for (uint8_t i = 0; i < NUM_SERVOS; i++)
    {
        int nowUs = (int)ServoList[i].current_position;
        int prevUs = gServoPrevPositionUs[i];
        gServoVelocityUsPerSec[i] = (nowUs - prevUs) / dtSec;
        gServoPrevPositionUs[i] = nowUs;
    }
}

static void EmitDebugMotionTelemetry20Hz()
{
#if DEBUG_MOTION
    static uint32_t lastPrintMs = 0;
    uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastPrintMs) < 50) // 20 Hz
        return;
    lastPrintMs = nowMs;

#if DEBUG_MOTION_CSV
#if DEBUG_MOTION_CSV_HEADER
    static bool headerPrinted = false;
    if (!headerPrinted)
    {
        Serial.print("ms,mode_rc,mode_head,raw_yaw,raw_pitch,raw_roll,raw_tfb,raw_tlr,raw_trot,raw_thip");
        for (uint8_t i = 0; i < NUM_SERVOS; i++)
        {
            Serial.print(",s");
            Serial.print((int)i);
            Serial.print("_t,s");
            Serial.print((int)i);
            Serial.print("_c,s");
            Serial.print((int)i);
            Serial.print("_v");
        }
        Serial.println();
        headerPrinted = true;
    }
#endif

    Serial.print((unsigned long)nowMs);
    Serial.print(",");
    Serial.print((int)gRCBottangoMode);
    Serial.print(",");
    Serial.print((int)gHeadRunMode);
    Serial.print(",");
    Serial.print((int)ChannelData(cHEADYAW));
    Serial.print(",");
    Serial.print((int)ChannelData(cHEADPITCH));
    Serial.print(",");
    Serial.print((int)ChannelData(cHEADROLL));
    Serial.print(",");
    Serial.print((int)ChannelData(cTORSOFB));
    Serial.print(",");
    Serial.print((int)ChannelData(cTORSOLR));
    Serial.print(",");
    Serial.print((int)ChannelData(cTORSOROT));
    Serial.print(",");
    Serial.print((int)ChannelData(cTORSOHIPSHIFT));

    for (uint8_t i = 0; i < NUM_SERVOS; i++)
    {
        Serial.print(",");
        Serial.print((int)ServoList[i].target_position);
        Serial.print(",");
        Serial.print((int)ServoList[i].current_position);
        Serial.print(",");
        Serial.print((int)gServoVelocityUsPerSec[i]);
    }
    Serial.println();
#else

    Serial.print("MDBG mode(rc/head)=");
    Serial.print((int)gRCBottangoMode);
    Serial.print("/");
    Serial.print((int)gHeadRunMode);

    Serial.print(" raw[yaw/pitch/roll/tfb/tlr/trot/thip]=");
    Serial.print((int)ChannelData(cHEADYAW));
    Serial.print("/");
    Serial.print((int)ChannelData(cHEADPITCH));
    Serial.print("/");
    Serial.print((int)ChannelData(cHEADROLL));
    Serial.print("/");
    Serial.print((int)ChannelData(cTORSOFB));
    Serial.print("/");
    Serial.print((int)ChannelData(cTORSOLR));
    Serial.print("/");
    Serial.print((int)ChannelData(cTORSOROT));
    Serial.print("/");
    Serial.print((int)ChannelData(cTORSOHIPSHIFT));

    for (uint8_t i = 0; i < NUM_SERVOS; i++)
    {
        Serial.print(" | s");
        Serial.print((int)i);
        Serial.print(" t=");
        Serial.print((int)ServoList[i].target_position);
        Serial.print(" c=");
        Serial.print((int)ServoList[i].current_position);
        Serial.print(" v=");
        Serial.print((int)gServoVelocityUsPerSec[i]);
    }
    Serial.println();
#endif
#endif
}

uint8_t ServoPins[] = {
    pinTorsoLRRight,
    pinTorsoLRLeft,
    pinTorsoFBRight,
    pinTorsoFBLeft,
    pinTorsoHipShift,
    pinTorsoRotate,

    // pinShoulderPitch1Right,
    // pinShoulderPitch2Right,
    // pinShoulderPitch1Left,
    // pinShoulderPitch2Left,
    // pinShoulderRollRight,
    // pinShoulderRollLeft,

    // pinElbowPitchRight,
    // pinElbowPitchLeft,
    //  pinWristPitchRight,
    //   pinWristPitchLeft,
    //  pinWristRollRight,
    //  pinWristRollLeft,

    pinHeadRotate,
    pinHeadPitch1,
    pinHeadPitch2,
    pinHeadPitch3,

};

void SetupMotion()
{

    Serial.println("Setting up Servos");
    pinMode(10, OUTPUT);
    digitalWrite(10, HIGH); // Turn D1-D3 as outputs.
    delay(25);

    // Set all servos to center position

        // TxServoPosition(ServoList[i].servoBank, ServoList[i].servoID, ServoList[i].center_position);

    setupEasingCurve();  // Pre-calculate easing values
    // attach servos
    // ServoAttach(0);
    Serial.println("Setup Servos Complete");
}
void setupEasingCurve() {
    for (int i = 0; i < easingSteps; i++) {
        float t = (float)i / (easingSteps - 1);  // Normalize `i` to [0,1] range
        easingCurve[i] = smoothStep(t);
    }
}

void MotionLoop()
{
    float PitchAngle;
    float RollAngle;

    if (millis() - MotionMillis >= MOTIONFRAMERATE)
    {
        MotionMillis = millis(); // <-- FIX: re-arm the gate every frame
                                 //    toggleControlMode();

        if (gRCBottangoMode == RC_MODE)
        {
            if (gHeadRunMode == HEADCONTROL)
            {
                // HEAD FUNCTIONS
                PitchAngle = (ChannelData(cHEADPITCH) - SBUSCENTER) / 2;
                RollAngle  = (ChannelData(cHEADROLL)  - SBUSCENTER) / 2;
                CalculateHeadPosition(PitchAngle, RollAngle);

                // Set Head Yaw
                ServoList[iHeadRotate].target_position =
                    map(ChannelData(cHEADYAW), SBUSMIN, SBUSMAX,
                        ServoList[iHeadRotate].MinDegrees, ServoList[iHeadRotate].MaxDegrees);

                // TORSO FUNCTIONS
                CalculateTorsoLeanFB((ChannelData(cTORSOFB) - SBUSCENTER) / 2);
                CalculateTorsoLeanLR((ChannelData(cTORSOLR) - SBUSCENTER) / 2);

                // Set Torso Rotate
                ServoList[iTorsoRotate].target_position =
                    map(ChannelData(cTORSOROT), SBUSMIN, SBUSMAX,
                        ServoList[iTorsoRotate].MinDegrees, ServoList[iTorsoRotate].MaxDegrees);

                // Set Hip Shift
                ServoList[iTorsoHipShift].target_position =
                    map(ChannelData(cTORSOHIPSHIFT), SBUSMIN, SBUSMAX,
                        ServoList[iTorsoHipShift].MinDegrees, ServoList[iTorsoHipShift].MaxDegrees);

                // Elbow functions
                // elbowmotionloop();

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

            if (gHeadRunMode == HEADAUTO)
                {
                    // Attach once (idempotent attach will also prevent reattach spam)
                    if (!ServoList[iHeadRotate].isAttached)
                    {
                        ServoAttach(0);
                    }

                    // -----------------------------
                    // AUTO HEAD TUNING (centralized)
                    // -----------------------------
                    const unsigned long normalTargetMinMs = 1550;
                    const unsigned long normalTargetMaxMs = 2000;

                    const unsigned long talkTargetMinMs   = 220;
                    const unsigned long talkTargetMaxMs   = 520;

                    // Move-to-target time (ms) – smaller = faster motion
                    // yaw, pitch1, pitch2, pitch3
                    // gAutoHeadMoveTime[] lives globally
                    // -----------------------------

                    // -----------------------------
                    // NEW: Burst / Pause / Sound-burst state machine
                    // -----------------------------
                    enum AutoHeadState : uint8_t
                    {
                        AHS_BURST,
                        AHS_PAUSE,
                        AHS_SOUND_BURST
                    };
                    static AutoHeadState autoState = AHS_BURST;

                    static uint8_t burstMovesRemaining = 0; // how many "moves" left in this burst
                    static uint8_t burstCounter = 0;        // counts completed bursts
                    static uint8_t burstsUntilSound = 0;    // countdown to next sound burst
                    static unsigned long pauseUntilMs = 0;  // when pause ends

                    // Sound tracking (uses gWavAisPlaying which Audio.cpp updates when <QPA> responses arrive)
                    extern int gWavAisPlaying;
                    static unsigned long lastQpaMs = 0;
                    static uint8_t soundDoneStableCount = 0;

                    // Helper: start a normal burst
                    auto startBurst = [&]()
                    {
                        // 2–4 quick moves per burst (tweak if you want more/less)
                        burstMovesRemaining = (uint8_t)random(4, 6); // 2,3,4
                        autoState = AHS_BURST;
                    };

                    // Helper: start a pause
                    auto startPause = [&]()
                    {
                        pauseUntilMs = millis() + (unsigned long)random(2000, 5001); // 5–10s
                        autoState = AHS_PAUSE;
                    };

                    // Helper: start a sound burst (movement lasts as long as sound)
                    auto startSoundBurst = [&]()
                    {
                        // Trigger a C3PO voice line (you already use 2001..2032 elsewhere)
                        int track = random(2001, 2033); // inclusive lower, exclusive upper
                        PlayWavA(track);

                        // While sound is playing, we keep generating targets quickly
                        burstMovesRemaining = 255; // effectively "infinite" until sound ends
                        soundDoneStableCount = 0;
                        autoState = AHS_SOUND_BURST;
                    };

                    // Init sound schedule if needed: every 3–4 bursts
                    if (burstsUntilSound == 0)
                    {
                        burstsUntilSound = (uint8_t)random(3, 5); // 3 or 4
                    }

                    // Handle PAUSE state
                    if (autoState == AHS_PAUSE)
                    {
                        if (millis() >= pauseUntilMs)
                        {
                            // Countdown to sound burst
                            if (burstsUntilSound > 0)
                                burstsUntilSound--;

                            // If this is the burst that should have sound -> start sound burst
                            if (burstsUntilSound == 0)
                            {
                                startSoundBurst();
                                burstsUntilSound = (uint8_t)random(3, 5); // schedule next one now
                            }
                            else
                            {
                                startBurst();
                            }
                        }
                        // During pause, do nothing (hold last targets)
                    }
                    else
                    {
                        // -----------------------------
                        // Target generation timing
                        // -----------------------------
                        // (ONLY CHANGE HERE: use centralized tuning constants)
                        unsigned long targetMinMs = (autoState == AHS_SOUND_BURST) ? talkTargetMinMs : normalTargetMinMs;
                        unsigned long targetMaxMs = (autoState == AHS_SOUND_BURST) ? talkTargetMaxMs : normalTargetMaxMs;

                        // If we haven't started a burst yet (first time), start one.
                        if (burstMovesRemaining == 0 && autoState != AHS_PAUSE)
                        {
                            startBurst();
                        }

                        // Update each axis if enough time has passed (your existing per-axis logic)
                        for (uint8_t x = 0; x <= 3; x++)
                        {
                            // If interval not initialized, seed it
                            if (gAutoHeadInterval[x] < targetMinMs || gAutoHeadInterval[x] > 60000UL)
                            {
                                gAutoHeadInterval[x] = random(targetMinMs, targetMaxMs);
                                gAutoHeadLastUpdate[x] = millis();
                            }

                            if (millis() - gAutoHeadLastUpdate[x] >= gAutoHeadInterval[x])
                            {
                                switch (x)
                                {
                                case 0: // Yaw (Head Rotate)
                                    ProcessAutoHeadMovement(iHeadRotate, Head_Rotate_MID, deviation);
                                    break;

                                case 1: // Pitch1
                                    ProcessAutoHeadMovement(iHeadPitch1, Head_Pitch1_MID, deviation);
                                    break;

                                case 2: // Pitch2
                                    ProcessAutoHeadMovement(iHeadPitch2, Head_Pitch2_MID, deviation);
                                    break;

                                case 3: // Pitch3
                                    ProcessAutoHeadMovement(iHeadPitch3, Head_Pitch3_MID, deviation);
                                    break;
                                }

                                // Next target timing for this axis
                                gAutoHeadInterval[x] = random(targetMinMs, targetMaxMs);
                                gAutoHeadLastUpdate[x] = millis();
                            }
                        }

                        // -----------------------------
                        // Easing move (your existing easing section)
                        // -----------------------------
                        //
                        // 1) HEAD easing (UNCHANGED LOGIC, just axis < 4)
                        //
                        for (int axis = 0; axis < 4; axis++)   // 0-3 head ONLY
                        {
                            int servoIndex;
                            if (axis == 0)
                                servoIndex = iHeadRotate;
                            else if (axis == 1)
                                servoIndex = iHeadPitch1;
                            else if (axis == 2)
                                servoIndex = iHeadPitch2;
                            else
                                servoIndex = iHeadPitch3;

                            int curUs = (int)ServoList[servoIndex].current_position;
                            int tgtUs = (int)ServoList[servoIndex].target_position;
                            if (curUs == tgtUs)
                                continue;

                            // IMPORTANT: use your "move time" (faster), not the random target interval (slower)
                            extern unsigned long gAutoHeadMoveTime[4];
                            unsigned long interval = gAutoHeadMoveTime[axis];
                            if (interval < 1)
                                interval = 1;

                            unsigned long start = gAutoHeadEasingStart[axis];
                            if (start == 0)
                                start = millis();

                            float easingProgress = (float)(millis() - start) / (float)interval;
                            if (easingProgress > 1.0f)
                                easingProgress = 1.0f;

                            int easingStepIndex = (int)(easingProgress * (easingSteps - 1));
                            easingStepIndex = constrain(easingStepIndex, 0, easingSteps - 1);

                            float easedFactor = easingCurve[easingStepIndex];

                            int easedUs = curUs + (int)((tgtUs - curUs) * easedFactor);
                            easedUs = constrain(easedUs,
                                                (int)ServoList[servoIndex].MinDegrees,
                                                (int)ServoList[servoIndex].MaxDegrees);

                            ServoList[servoIndex].sMotor.writeMicroseconds(easedUs);
                            ServoList[servoIndex].current_position = easedUs;

                            if (easingProgress >= 1.0f)
                            {
                                ServoList[servoIndex].sMotor.writeMicroseconds(tgtUs);
                                ServoList[servoIndex].current_position = tgtUs;
                            }
                        }

                        //
                        // 2) TORSO easing (your existing torso writes, moved OUTSIDE the head loop)
                        //    This keeps your current tuning values the same.
                        //
                        {
                            // ---- Torso Rotate (axis 4 equivalent) ----
                            {
                                int servoIndex = iTorsoRotate;

                                int curUs = (int)ServoList[servoIndex].current_position;
                                int tgtUs = (int)ServoList[servoIndex].target_position;

                                if (curUs != tgtUs)
                                {
                                    unsigned long interval = 600; // <-- unchanged from your code
                                    if (interval < 1) interval = 1;

                                    static unsigned long torsoEasingStartRot = 0;
                                    if (torsoEasingStartRot == 0) torsoEasingStartRot = millis();

                                    float easingProgress = (float)(millis() - torsoEasingStartRot) / (float)interval;
                                    if (easingProgress > 1.0f) easingProgress = 1.0f;

                                    int easingStepIndex = (int)(easingProgress * (easingSteps - 1));
                                    easingStepIndex = constrain(easingStepIndex, 0, easingSteps - 1);
                                    float easedFactor = easingCurve[easingStepIndex];

                                    int easedUs = curUs + (int)((tgtUs - curUs) * easedFactor);
                                    easedUs = constrain(easedUs,
                                                        (int)ServoList[servoIndex].MinDegrees,
                                                        (int)ServoList[servoIndex].MaxDegrees);

                                    ServoList[servoIndex].sMotor.writeMicroseconds(easedUs);
                                    ServoList[servoIndex].current_position = easedUs;

                                    if (easingProgress >= 1.0f)
                                    {
                                        ServoList[servoIndex].sMotor.writeMicroseconds(tgtUs);
                                        ServoList[servoIndex].current_position = tgtUs;
                                    }
                                }
                            }

                            // ---- Hip Shift (axis 5 equivalent) ----
                            {
                                int servoIndex = iTorsoHipShift;

                                int curUs = (int)ServoList[servoIndex].current_position;
                                int tgtUs = (int)ServoList[servoIndex].target_position;

                                if (curUs != tgtUs)
                                {
                                    unsigned long interval = 600; // <-- unchanged from your code
                                    if (interval < 1) interval = 1;

                                    static unsigned long torsoEasingStartHip = 0;
                                    if (torsoEasingStartHip == 0) torsoEasingStartHip = millis();

                                    float easingProgress = (float)(millis() - torsoEasingStartHip) / (float)interval;
                                    if (easingProgress > 1.0f) easingProgress = 1.0f;

                                    int easingStepIndex = (int)(easingProgress * (easingSteps - 1));
                                    easingStepIndex = constrain(easingStepIndex, 0, easingSteps - 1);
                                    float easedFactor = easingCurve[easingStepIndex];

                                    int easedUs = curUs + (int)((tgtUs - curUs) * easedFactor);
                                    easedUs = constrain(easedUs,
                                                        (int)ServoList[servoIndex].MinDegrees,
                                                        (int)ServoList[servoIndex].MaxDegrees);

                                    ServoList[servoIndex].sMotor.writeMicroseconds(easedUs);
                                    ServoList[servoIndex].current_position = easedUs;

                                    if (easingProgress >= 1.0f)
                                    {
                                        ServoList[servoIndex].sMotor.writeMicroseconds(tgtUs);
                                        ServoList[servoIndex].current_position = tgtUs;
                                    }
                                }
                            }

                            // ---- Torso Forward/Back pair (axis 6 equivalent) ----
                            {
                                int idxL = iTorsoFBLeft;
                                int idxR = iTorsoFBRight;

                                unsigned long interval = 600; // <-- unchanged from your code
                                if (interval < 1)
                                    interval = 1;

                                static unsigned long torsoEasingStartFB = 0;
                                if (torsoEasingStartFB == 0)
                                    torsoEasingStartFB = millis();

                                int curL = (int)ServoList[idxL].current_position;
                                int tgtL = (int)ServoList[idxL].target_position;

                                int curR = (int)ServoList[idxR].current_position;
                                int tgtR = (int)ServoList[idxR].target_position;

                                if (curL != tgtL || curR != tgtR)
                                {
                                    float easingProgress = (float)(millis() - torsoEasingStartFB) / (float)interval;
                                    if (easingProgress > 1.0f)
                                        easingProgress = 1.0f;

                                    int easingStepIndex = (int)(easingProgress * (easingSteps - 1));
                                    easingStepIndex = constrain(easingStepIndex, 0, easingSteps - 1);
                                    float easedFactor = easingCurve[easingStepIndex];

                                    // L
                                    if (curL != tgtL)
                                    {
                                        int easedUsL = curL + (int)((tgtL - curL) * easedFactor);
                                        easedUsL = constrain(easedUsL,
                                                             (int)ServoList[idxL].MinDegrees,
                                                             (int)ServoList[idxL].MaxDegrees);
                                        ServoList[idxL].sMotor.writeMicroseconds(easedUsL);
                                        ServoList[idxL].current_position = easedUsL;
                                        if (easingProgress >= 1.0f)
                                            ServoList[idxL].current_position = tgtL;
                                    }

                                    // R
                                    if (curR != tgtR)
                                    {
                                        int easedUsR = curR + (int)((tgtR - curR) * easedFactor);
                                        easedUsR = constrain(easedUsR,
                                                             (int)ServoList[idxR].MinDegrees,
                                                             (int)ServoList[idxR].MaxDegrees);
                                        ServoList[idxR].sMotor.writeMicroseconds(easedUsR);
                                        ServoList[idxR].current_position = easedUsR;
                                        if (easingProgress >= 1.0f)
                                            ServoList[idxR].current_position = tgtR;
                                    }

                                    if (easingProgress >= 1.0f)
                                    {
                                        ServoList[idxL].sMotor.writeMicroseconds(tgtL);
                                        ServoList[idxL].current_position = tgtL;

                                        ServoList[idxR].sMotor.writeMicroseconds(tgtR);
                                        ServoList[idxR].current_position = tgtR;
                                    }
                                }
                            }
                        }

                        // -----------------------------
                        // Decide when a "move" is done (burst accounting)
                        // -----------------------------
                        bool allAtTarget =
                            ((int)ServoList[iHeadRotate].current_position == (int)ServoList[iHeadRotate].target_position) &&
                            ((int)ServoList[iHeadPitch1].current_position == (int)ServoList[iHeadPitch1].target_position) &&
                            ((int)ServoList[iHeadPitch2].current_position == (int)ServoList[iHeadPitch2].target_position) &&
                            ((int)ServoList[iHeadPitch3].current_position == (int)ServoList[iHeadPitch3].target_position);

                        // For SOUND_BURST: keep going until audio finishes.
                        if (autoState == AHS_SOUND_BURST)
                        {
                            // Poll QPA occasionally so gWavAisPlaying stays current (AudioLoop may not be polling it)
                            if (millis() - lastQpaMs > 200)
                            {
                                WavAisPlaying(); // sends <QPA>
                                lastQpaMs = millis();
                            }

                            // When we see "not playing" stable a few times, end sound burst and pause.
                            if (gWavAisPlaying <= 0)
                            {
                                if (soundDoneStableCount < 10)
                                    soundDoneStableCount++;
                                if (soundDoneStableCount >= 3) // ~3 polls = ~600ms stable
                                {
                                    startPause();
                                    burstCounter++;
                                }
                            }
                            else
                            {
                                soundDoneStableCount = 0;
                            }
                        }
                        else
                        {
                            // Normal burst: when a full move finishes, decrement remaining moves.
                            if (allAtTarget && burstMovesRemaining > 0)
                            {
                                burstMovesRemaining--;

                                // If burst is done, pause.
                                if (burstMovesRemaining == 0)
                                {
                                    startPause();
                                    burstCounter++;
                                }
                            }
                        }
                    } // <-- closes: else (autoState != AHS_PAUSE)
                }     // <-- closes: if (gHeadRunMode == HEADAUTO)
            }
    if (gRCBottangoMode == ANIMATION_MODE)
                {
                    // GeneratedCodeAnimations::AnimationControl();
                }
            // <-- closes: if (millis() - lastUpdateTime >= servoUpdateInterval)
        }     // <-- closes: if (gRCBottangoMode == RC_MODE)
    }         // <-- closes: if (millis() - MotionMillis >= MOTIONFRAMERATE)
}             // <-- closes: MotionLoop()

void ProcessAutoHeadMovement(uint8_t servoIndex, int midPosition, int deviation)
        {
            int newTarget = random(midPosition - deviation, midPosition + deviation);
            newTarget = constrain(newTarget,
                                  (int)ServoList[servoIndex].MinDegrees,
                                  (int)ServoList[servoIndex].MaxDegrees);

            if (newTarget != (int)ServoList[servoIndex].target_position)
            {
                ServoList[servoIndex].target_position = newTarget;

                // Start easing timer for the matching axis (assumes your 4 head servos)
                uint8_t axis = 0;
                if (servoIndex == iHeadRotate)
                    axis = 0;
                else if (servoIndex == iHeadPitch1)
                    axis = 1;
                else if (servoIndex == iHeadPitch2)
                    axis = 2;
                else if (servoIndex == iHeadPitch3)
                    axis = 3;

                gAutoHeadEasingStart[axis] = millis();
            }
        }

        // void ProcessAutoHeadMovement(uint8_t axisIndex, uint8_t servoIndex, int midPosition, int deviation)
        // {
        //     // Generate a random new target only when needed
        //     int newTarget = random(midPosition - deviation, midPosition + deviation);
        //     newTarget = constrain(newTarget, ServoList[servoIndex].MinDegrees, ServoList[servoIndex].MaxDegrees);

        //     // Only update if changed
        //     if (newTarget != (int)ServoList[servoIndex].target_position)
        //     {
        //         ServoList[servoIndex].target_position = newTarget;

        //         // Reset easing timer for THIS AXIS so easingProgress starts at 0
        //         gAutoHeadElapsed[axisIndex] = 0;

        //         // Optional debug
        //         // Serial.printf("New target axis %u servo %u: %d\n", axisIndex, servoIndex, newTarget);
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
        //     }
        // }

        void UpdateServos()
        {
            if (gHeadRunMode == HEADCONTROL)
            {
                uint32_t nowMs = millis();
                static uint32_t lastUpdateMs = 0;

                if (!gServoPrevPositionInit)
                {
                    for (uint8_t i = 0; i < NUM_SERVOS; i++)
                    {
                        gServoPrevPositionUs[i] = (int)ServoList[i].current_position;
                    }
                    gServoPrevPositionInit = true;
                    lastUpdateMs = nowMs;
                }

                float dtSec = (float)(nowMs - lastUpdateMs) / 1000.0f;
                if (dtSec < 0.001f)
                    dtSec = 0.001f;
                if (dtSec > 0.250f)
                    dtSec = 0.250f;

                for (uint8_t i = 0; i < NUM_SERVOS; i++)
                {
                    if (abs(ServoList[i].target_position - ServoList[i].current_position) > 1)
                    {
                        int safeTargetUs = ClampStepFromCurrent(i, ServoList[i].target_position);
                        SetServoPosition(i, safeTargetUs);
                        ServoList[i].current_position = safeTargetUs;
                    }
                }

                UpdateVelocityEstimates(dtSec);
                EmitDebugMotionTelemetry20Hz();
                lastUpdateMs = nowMs;
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
            // StopAllMotion();
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
        // Mix the pitch and roll together to calculate the required servo positions.
        // 120° servo mix
void CalculateHeadPosition(float pitch, float roll)
{
    const float R = 0.866f;  // sqrt(3)/2 for 120° mix

    // BACK apex servo (Pitch3): moves with pitch
    ServoList[iHeadPitch3].target_position =
        constrain(ServoList[iHeadPitch3].center_position - pitch,
                  ServoList[iHeadPitch3].MinDegrees,
                  ServoList[iHeadPitch3].MaxDegrees);

    // FRONT LEFT (Pitch1): half pitch opposite + roll
    ServoList[iHeadPitch1].target_position =
        constrain(ServoList[iHeadPitch1].center_position - 0.5f * pitch + R * roll,
                  ServoList[iHeadPitch1].MinDegrees,
                  ServoList[iHeadPitch1].MaxDegrees);

    // FRONT RIGHT (Pitch2): half pitch opposite - roll
    ServoList[iHeadPitch2].target_position =
        constrain(ServoList[iHeadPitch2].center_position - 0.5f * pitch - R * roll,
                  ServoList[iHeadPitch2].MinDegrees,
                  ServoList[iHeadPitch2].MaxDegrees);



        // void CalculateHeadPosition(float pitch, float roll)
        // {
        //     ServoList[iHeadPitch1].target_position = constrain(ServoList[iHeadPitch1].center_position - pitch, ServoList[iHeadPitch1].MinDegrees, ServoList[iHeadPitch1].MaxDegrees);
        //     ServoList[iHeadPitch2].target_position = constrain(ServoList[iHeadPitch2].center_position - 0.5 * pitch + 0.866 * roll, ServoList[iHeadPitch2].MinDegrees, ServoList[iHeadPitch2].MaxDegrees);
        //     ServoList[iHeadPitch3].target_position = constrain(ServoList[iHeadPitch3].center_position - 0.5 * pitch - 0.866 * roll, ServoList[iHeadPitch3].MinDegrees, ServoList[iHeadPitch3].MaxDegrees);

#if 0
    Serial.print("P: ");
    Serial.print(pitch);
    Serial.print(" R: ");
    Serial.print(roll);
    Serial.print(" SP1 ");
    Serial.print(ServoList[iHeadPitch1].target_position);
    Serial.print(" SP2 ");
    Serial.print(ServoList[iHeadPitch2].target_position);
    Serial.print(" SP3 ");
    Serial.println(ServoList[iHeadPitch3].target_position);
#endif
}

// Mix the Left Right Lean Servos
void CalculateTorsoLeanLR(float Lean)
{
    ServoList[iTorsoLRLeft].target_position = constrain(ServoList[iTorsoLRLeft].center_position + Lean, ServoList[iTorsoLRLeft].MinDegrees, ServoList[iTorsoLRLeft].MaxDegrees);
    ServoList[iTorsoLRRight].target_position = constrain(ServoList[iTorsoLRRight].center_position + Lean, ServoList[iTorsoLRRight].MinDegrees, ServoList[iTorsoLRRight].MaxDegrees);

#if 0
    Serial.print("L: ");
    Serial.print(Lean);
    Serial.print(" SLL ");
    Serial.print(sTorsoLRLeft.target_position);
    Serial.print(" SLR ");
    Serial.println(sTorsoLRRight.target_position);
#endif
}

// Mix the Forward/Back Lean Servos
void CalculateTorsoLeanFB(float Lean)
{
    ServoList[iTorsoFBLeft].target_position = constrain(ServoList[iTorsoFBLeft].center_position + Lean, ServoList[iTorsoFBLeft].MinDegrees, ServoList[iTorsoFBLeft].MaxDegrees);
    ServoList[iTorsoFBRight].target_position = constrain(ServoList[iTorsoFBRight].center_position - Lean, ServoList[iTorsoFBRight].MinDegrees, ServoList[iTorsoFBRight].MaxDegrees);

#if 0
    Serial.print("L: ");
    Serial.print(Lean);
    Serial.print(" SFBL ");
    Serial.print(ServoList[iTorsoFBLeft].target_position);
    Serial.print(" SFBR ");
    Serial.println(ServoList[iTorsoFBRight].target_position);
#endif
}

// ***SHOULDER AND ELBOW FUNCTIONS NOW PROCESSED ON SLAVE BOARD***
// // void SetShoulderRunModeLeft()
// // {
// //     // Set the shoulder mode to left
// //     gShoulderSwitchMode = SHOULDERSWITCHLEFT;

// //     // Flexion Function (Shoulder Roll)
// //     ServoList[iShoulderRollLeft].target_position = map(ChannelData(cSHFL), SBUSMIN, SBUSMAX, ServoList[iShoulderRollLeft].MinDegrees, ServoList[iShoulderRollLeft].MaxDegrees);

// //     // Abduction Function (Shoulder Pitch 1 and 2 work in opposite directions)
// //     int abductionValueLeft = ChannelData(cSHAB);
// //     int targetPitch1Left = 1460;  // Default neutral for Pitch 1
// //     int targetPitch2Left = 1130;  // Default neutral for Pitch 2

// //     // If abduction input is active (positive direction)
// //     if (abductionValueLeft > SBUSNEUTRAL) {
// //         targetPitch1Left = map(abductionValueLeft, SBUSNEUTRAL, SBUSMAX, 1460, 2280);  // Abduction range for Pitch 1
// //         targetPitch2Left = map(abductionValueLeft, SBUSNEUTRAL, SBUSMAX, 1130, 641);   // Abduction range for Pitch 2 (opposite)
// //     }

// //     // External Rotation Function - Both servos move together
// //     int externalRotationValueLeft = ChannelData(cSHER);

// //     if (externalRotationValueLeft > SBUSNEUTRAL && isExternalRotationModeLeft()) {
// //         // If external rotation mode is active
// //         targetPitch1Left = map(externalRotationValueLeft, SBUSNEUTRAL, SBUSMAX, 1460, 640);  // External rotation for Pitch 1
// //         targetPitch2Left = map(externalRotationValueLeft, SBUSNEUTRAL, SBUSMAX, 1130, 640);  // External rotation for Pitch 2 (same direction)
// //     }

// //     // Set the target positions for Pitch 1 and Pitch 2 servos
// //     ServoList[iShoulderPitch1Left].target_position = targetPitch1Left;
// //     ServoList[iShoulderPitch2Left].target_position = targetPitch2Left;
// // }

// // // Helper function to determine if the shoulder is in external rotation mode (left)
// // bool isExternalRotationModeLeft() 
// // {
// //     // Get the value from the left shoulder external rotation channel (cSHER)
// //     int erInputLeft = ChannelData(cSHER);

// //     // Define the "quiet" range around 992 ± 100 for the left shoulder
// //     const int quietLowerBound = 892;
// //     const int quietUpperBound = 1092;

// //     // Return true if external rotation is active, meaning the input is outside the quiet range
// //     return (erInputLeft < quietLowerBound || erInputLeft > quietUpperBound);
// // }


// void SetShoulderRunModeLeft()
// {
//     // Flexion (FL) - Shoulder Roll for Left Shoulder
//     ServoList[iShoulderRollLeft].target_position = map(ChannelData(cSHFL), SBUSMIN, SBUSMAX, ServoList[iShoulderRollLeft].MinDegrees, ServoList[iShoulderRollLeft].MaxDegrees);

//     // Initialize target positions to neutral
//     int targetPitch1Left = 1460;
//     int targetPitch2Left = 1130;

//     // Read abduction and external rotation input values
//     int abductionValue = ChannelData(cSHAB);
//     int externalRotationValue = ChannelData(cSHER);

//     // Calculate abduction target positions for Left Pitch 1 and Pitch 2
//     if (abductionValue > SBUSNEUTRAL) {
//         targetPitch1Left = map(abductionValue, SBUSNEUTRAL, SBUSMAX, 1460, 2280);
//         targetPitch2Left = map(abductionValue, SBUSNEUTRAL, SBUSMAX, 1130, 641);
//     }

//     // Apply external rotation on top of abduction positions
//     if (isExternalRotationMode()) {
//         int rotationOffsetPitch1Left = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, 200);
//         int rotationOffsetPitch2Left = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, -200);

//         targetPitch1Left += rotationOffsetPitch1Left;
//         targetPitch2Left += rotationOffsetPitch2Left;
//     }

//     // Update the target positions for Left Shoulder
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

// // void SetShoulderRunModeRight() 
// // {
// //     gShoulderSwitchMode = SHOULDERSWITCHRIGHT;

// //     // Shoulder Roll - Flexion Control
// //     ServoList[iShoulderRollRight].target_position = map(ChannelData(cSHFL), SBUSMIN, SBUSMAX, ServoList[iShoulderRollRight].MinDegrees, ServoList[iShoulderRollRight].MaxDegrees);

// //     // Abduction Function (Shoulder Pitch 1 and 2 work in opposite directions)
// //     int abductionValueRight = ChannelData(cSHAB);
// //     int targetPitch1Right = 1420;  // Default neutral for Pitch 1
// //     int targetPitch2Right = 2000;  // Default neutral for Pitch 2

// //     if (abductionValueRight > SBUSNEUTRAL) {  
// //         // When the stick is pushed in the positive direction (abduction)
// //         targetPitch1Right = map(abductionValueRight, SBUSNEUTRAL, SBUSMAX, 1420, 2400);  // Shoulder Pitch 1 (neutral to full abduction)
// //         targetPitch2Right = map(abductionValueRight, SBUSNEUTRAL, SBUSMAX, 2000, 640); // Shoulder Pitch 2 (opposite)
// //     }

// //     // External Rotation Function (Both servos should move together in the same direction)
// //     int externalRotationValueRight = ChannelData(cSHER);

// //     if (externalRotationValueRight > SBUSNEUTRAL && isExternalRotationModeRight()) {  
// //         // When the stick is pushed in the external rotation direction (992 to 1811)
// //         targetPitch1Right = map(externalRotationValueRight, SBUSNEUTRAL, SBUSMAX, 1420, 2200);  // Pitch 1 external rotation
// //         targetPitch2Right = map(externalRotationValueRight, SBUSNEUTRAL, SBUSMAX, 2000, 2400);  // Pitch 2 external rotation
// //     }

// //     // Update the target positions for Pitch 1 and Pitch 2 servos
// //     ServoList[iShoulderPitch1Right].target_position = targetPitch1Right;
// //     ServoList[iShoulderPitch2Right].target_position = targetPitch2Right;
    
// // }

// // // Helper function to determine if the shoulder is in external rotation mode
// // bool isExternalRotationModeRight() 
// // {
// //     // Get the value from the external rotation channel (cSHER)
// //     int erInput = ChannelData(cSHER);

// //     // Define the "quiet" range around 992 ± 100
// //     const int quietLowerBound = 892;
// //     const int quietUpperBound = 1092;

// //     // Return true if external rotation is active, meaning the input is outside the quiet range
// //     return (erInput < quietLowerBound || erInput > quietUpperBound);
// // }


// void SetShoulderRunModeRight()
// {
//     // Flexion (FL) - Shoulder Roll for Right Shoulder
//     ServoList[iShoulderRollRight].target_position = map(ChannelData(cSHFL), SBUSMIN, SBUSMAX, ServoList[iShoulderRollRight].MinDegrees, ServoList[iShoulderRollRight].MaxDegrees);

//     // Initialize target positions to neutral
//     int targetPitch1Right = 1420;
//     int targetPitch2Right = 2000;

//     // Read abduction and external rotation input values
//     int abductionValue = ChannelData(cSHAB);
//     int externalRotationValue = ChannelData(cSHER);

//     // Calculate abduction target positions for Right Pitch 1 and Pitch 2
//     if (abductionValue > SBUSNEUTRAL) {
//         targetPitch1Right = map(abductionValue, SBUSNEUTRAL, SBUSMAX, 1420, 2600);
//         targetPitch2Right = map(abductionValue, SBUSNEUTRAL, SBUSMAX, 2000, 640);
//     }

//     // Apply external rotation on top of abduction positions
//     if (isExternalRotationMode()) {
//         int rotationOffsetPitch1Right = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, 100);
//         int rotationOffsetPitch2Right = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, 1200);

//         targetPitch1Right += rotationOffsetPitch1Right;
//         targetPitch2Right += rotationOffsetPitch2Right;
//     }

//     // Update the target positions for Right Shoulder
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


// // 
// void SetShoulderRunModeBoth()
// {
//     // Set flexion (FL) for both shoulders - Shoulder Roll
//     ServoList[iShoulderRollRight].target_position = map(ChannelData(cSHFL), SBUSMIN, SBUSMAX, ServoList[iShoulderRollRight].MinDegrees, ServoList[iShoulderRollRight].MaxDegrees);
//     ServoList[iShoulderRollLeft].target_position = map(ChannelData(cSHFL), SBUSMIN, SBUSMAX, ServoList[iShoulderRollLeft].MinDegrees, ServoList[iShoulderRollLeft].MaxDegrees);

//     // Initialize target positions to neutral for both shoulders
//     int targetPitch1Right = 1420;
//     int targetPitch2Right = 2000;
//     int targetPitch1Left = 1460;
//     int targetPitch2Left = 1130;

//     // Read abduction and external rotation input values
//     int abductionValue = ChannelData(cSHAB);
//     int externalRotationValue = ChannelData(cSHER);

//     // Calculate abduction target positions for both shoulders
//     if (abductionValue > SBUSNEUTRAL) {
//         targetPitch1Left = map(abductionValue, SBUSNEUTRAL, SBUSMAX, 1460, 2280);
//         targetPitch2Left = map(abductionValue, SBUSNEUTRAL, SBUSMAX, 1130, 641);

//         targetPitch1Right = map(abductionValue, SBUSNEUTRAL, SBUSMAX, 1420, 640);
//         targetPitch2Right = map(abductionValue, SBUSNEUTRAL, SBUSMAX, 2000, 2400);
//     }

//     // Apply external rotation on top of abduction positions for both shoulders
//     if (isExternalRotationMode()) {
//         int rotationOffsetPitch1Right = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, 200);
//         int rotationOffsetPitch2Right = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, -200);
        
//         int rotationOffsetPitch1Left = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, 200);
//         int rotationOffsetPitch2Left = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, -200);

//         targetPitch1Right += rotationOffsetPitch1Right;
//         targetPitch2Right += rotationOffsetPitch2Right;
        
//         targetPitch1Left += rotationOffsetPitch1Left;
//         targetPitch2Left += rotationOffsetPitch2Left;
//     }

//     // Update the target positions for both shoulders
//     ServoList[iShoulderPitch1Right].target_position = targetPitch1Right;
//     ServoList[iShoulderPitch2Right].target_position = targetPitch2Right;
//     ServoList[iShoulderPitch1Left].target_position = targetPitch1Left;
//     ServoList[iShoulderPitch2Left].target_position = targetPitch2Left;
// }


// // Helper function to determine if the shoulder is in external rotation mode
// bool isExternalRotationMode() 
// {
//     // Get the value from the external rotation channel (cSHER)
//     int erInput = ChannelData(cSHER);

//     // Define the "quiet" range around 992 ± 100
//     const int quietLowerBound = 892;
//     const int quietUpperBound = 1092;

//     // Return true if external rotation is active, meaning the input is outside the quiet range
//     return (erInput < quietLowerBound || erInput > quietUpperBound);
// }

// uint8_t GetShoulderMode()
// {
//     return gShoulderSwitchMode;
// }

// void SetShoulderMode(uint8_t mode)
// {
//     gShoulderSwitchMode = mode;
// }
// // void TestAllServos(void)
// // {
// //     Serial.println("testing all servos...");
// //     Serial.println("Set all to min");
// //     for (int i = 0; i < NUM_SERVOS; i++)
// //     {
// //         delay(1000);
// //         TxServoPosition(ServoList[i].servoBank, ServoList[i].servoID, ServoList[i].MinDegrees);
// //     }
// //     delay(2000);
// //     Serial.println("Set all to max");
// //     for (int i = 0; i < NUM_SERVOS; i++)
// //     {
// //         delay(1000);
// //         TxServoPosition(ServoList[i].servoBank, ServoList[i].servoID, ServoList[i].MaxDegrees);
// //     }
// //     delay(2000);
// //     Serial.println("Set all to mid");
// //     for (int i = 0; i < NUM_SERVOS; i++)
// //     {
// //         delay(1000);
// //         TxServoPosition(ServoList[i].servoBank, ServoList[i].servoID, ServoList[i].center_position);
// //     }
// // }

// void elbowmotionloop()
// {

//     // If no trim button is actively pressed, re-center the elbows
//     if (!rightElbowActive)
//     {
//         // Serial.println("Re-center right elbow");
//         ServoList[iElbowPitchRight].target_position = Elbow_Pitch_Right_MID;
//     }

//     if (!leftElbowActive)
//     {
//         // Serial.println("Re-center left elbow");
//         ServoList[iElbowPitchLeft].target_position = Elbow_Pitch_Left_MID;
//     }


// }
// void ElbowCenter()
// {
//     rightElbowActive = false;
//     leftElbowActive = false;
//     elbowmotionloop();

//     ServoList[iElbowPitchRight].target_position = Elbow_Pitch_Right_MID;
//     // Serial.print(" SP: ");
//     // Serial.print(iElbowPitchRight);
//     // Serial.print(" ");
//     // Serial.println(ServoList[iElbowPitchRight].target_position);
//     ServoList[iElbowPitchRight].sMotor.writeMicroseconds(ServoList[iElbowPitchRight].target_position);

//     ServoList[iElbowPitchLeft].target_position = Elbow_Pitch_Left_MID;
//     // Serial.print(" SP: ");
//     // Serial.print(iElbowPitchLeft);
//     // Serial.print(" ");
//     // Serial.println(ServoList[iElbowPitchLeft].target_position);
//     ServoList[iElbowPitchLeft].sMotor.writeMicroseconds(ServoList[iElbowPitchLeft].target_position);
// }

// void LeftElbowPositionMin()
// {
//     ServoList[iElbowPitchLeft].target_position = Elbow_Pitch_Left_MIN;
//     Serial.print(" SP: ");
//     Serial.print(iElbowPitchLeft);
//     Serial.print(" ");
//     Serial.println(ServoList[iElbowPitchLeft].target_position);
//     ServoList[iElbowPitchLeft].sMotor.writeMicroseconds(ServoList[iElbowPitchLeft].target_position);
//     leftElbowActive = true;
//     elbowmotionloop();
// }

// void LeftElbowPositionMax()
// {
//     ServoList[iElbowPitchLeft].target_position = Elbow_Pitch_Left_MAX;
//     Serial.print(" SP: ");
//     Serial.print(iElbowPitchLeft);
//     Serial.print(" ");
//     Serial.println(ServoList[iElbowPitchLeft].target_position);
//     ServoList[iElbowPitchLeft].sMotor.writeMicroseconds(ServoList[iElbowPitchLeft].target_position);
//     leftElbowActive = true;
//     elbowmotionloop();
// }

// void RightElbowPositionMax()
// {
//     ServoList[iElbowPitchRight].target_position = Elbow_Pitch_Right_MAX;
//     Serial.print(" SP: ");
//     Serial.print(iElbowPitchRight);
//     Serial.print(" ");
//     Serial.println(ServoList[iElbowPitchRight].target_position);
//     ServoList[iElbowPitchRight].sMotor.writeMicroseconds(ServoList[iElbowPitchRight].target_position);
//     rightElbowActive = true;
//     elbowmotionloop();
// }
// void RightElbowPositionMin()
// {
//     ServoList[iElbowPitchRight].target_position = Elbow_Pitch_Right_MIN;
//     Serial.print(" SP: ");
//     Serial.print(iElbowPitchRight);
//     Serial.print(" ");
//     Serial.println(ServoList[iElbowPitchRight].target_position);
//     ServoList[iElbowPitchRight].sMotor.writeMicroseconds(ServoList[iElbowPitchRight].target_position);
//     rightElbowActive = true;
//     elbowmotionloop();
// }
void ServoAttach(uint8_t ServoNum)
{
    #if DBG_ATTACH
    if (dbgAllow(1, 500))  // key 1, once per 500ms
    {
        Serial.print("ServoAttach(");
        Serial.print(ServoNum);
        Serial.println(")");
    }
#endif

    if (ServoNum == 0)
    { // attach all (idempotent)
        for (uint8_t x = 0; x < NUM_SERVOS; x++)
        {
            if (ServoList[x].isAttached) continue;   // <-- KEY FIX (skip already attached)

            ServoList[x].sMotor.attach(ServoPins[x]);
            ServoList[x].isAttached = true;

            // Write a safe value immediately (use current if valid, else target, else center)
            int us = (int)ServoList[x].current_position;
            if (us <= 0) us = (int)ServoList[x].target_position;
            if (us <= 0) us = (int)ServoList[x].center_position;

            us = constrain(us, (int)ServoList[x].MinDegrees, (int)ServoList[x].MaxDegrees);
            ServoList[x].sMotor.writeMicroseconds(us);

            // No delay here
            // Optional debug (leave off in normal run):
            // Serial.printf("Attach servo %u pin %u us %d\n", x, ServoPins[x], us);
        }
#if DBG_ATTACH
        if (dbgAllow(2, 500)) // key 2
        {
            Serial.println("ServoAttach complete");
        }
#endif

        return;
    }

    // attach single (idempotent)
    if (ServoNum < NUM_SERVOS)
    {
        if (ServoList[ServoNum].isAttached) return;

        ServoList[ServoNum].sMotor.attach(ServoPins[ServoNum]);
        ServoList[ServoNum].isAttached = true;

        int us = (int)ServoList[ServoNum].current_position;
        if (us <= 0) us = (int)ServoList[ServoNum].target_position;
        if (us <= 0) us = (int)ServoList[ServoNum].center_position;

        us = constrain(us, (int)ServoList[ServoNum].MinDegrees, (int)ServoList[ServoNum].MaxDegrees);
        ServoList[ServoNum].sMotor.writeMicroseconds(us);
    }
}

// void ServoAttach(uint8_t ServoNum)
// {

//     if (ServoNum == 0)
//     { // attach all
//         for (uint8_t x = 0; x < NUM_SERVOS; x++)
//         {
//             ServoList[x].sMotor.attach(ServoPins[x]);

//             Serial.print(" attach debug sMotor ID: ");//debug code for servo attach
//             Serial.println(x);
//             Serial.print("Servo pin number: ");
//             Serial.println(ServoPins[x]);

//             ServoList[x].isAttached = 1;
//             ServoList[x].sMotor.writeMicroseconds(ServoList[x].target_position);
//             Serial.print("Servo target position: ");
//             Serial.println(ServoList[x].target_position);
//             delay(50);
//         }
//             Serial.println("Servo setup complete ");
//     }
//     else
//     {
//         ServoList[ServoNum].sMotor.attach(ServoPins[ServoNum]);
//             Serial.print(" attach debug ServoNum ID: ");//debug code for servo attach
//             Serial.println(ServoNum);
//             Serial.print("Servo pin number: ");
//             Serial.println(ServoPins[ServoNum]);
//         ServoList[ServoNum].isAttached = true;
//         ServoList[ServoNum].sMotor.writeMicroseconds(ServoList[ServoNum].target_position);
//     }
//        Serial.println("Servo attach complete ");
// }

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

// void SetServoPosition(int ServoID, float target_position)
// {
//     Serial.print(" SP: ");
//     Serial.print(ServoID);
//     Serial.print(" ");
//     Serial.println(target_position);
//     ServoList[ServoID].sMotor.writeMicroseconds(target_position);
// }

void SetServoPosition(int ServoID, float target_position)
{
    // Optional: very light, rate-limited debug (OFF by default)
#if DBG_MOTION
    if (dbgAllow(0, 200))  // key 0, once per 200ms max
    {
        Serial.print("SetServoPosition() last servo=");
        Serial.print(ServoID);
        Serial.print(" us=");
        Serial.println((int)target_position);
    }
#endif

    if (ServoID < 0 || ServoID >= NUM_SERVOS)
        return;

    int clampedUs = ClampPulseForServo((uint8_t)ServoID, target_position);
    ServoList[ServoID].sMotor.writeMicroseconds(clampedUs);
}






// // MotionControl.cpp
// // Functions to control Servos and other Motions
// // Comminicating to ESP32s over RS-485
// // Arranged as 4 banks of Servos

// #include <src/MotionControl.hpp>
// #include <Teensy_PWM.h>
// #include <Arduino.h>
// #include "Definitions.h"
// #include <math.h>
// #include "BottangoCore.h"
// //#include <PWMServo.h>
// #include <servo.h>
// #include <src/Events.hpp>
// #include <src/BasicCommands.h>

// uint8_t gHeadRunMode = HEADIDLE;
// uint8_t gRCBottangoMode = RC_MODE;
// uint8_t currentMode = RC_MODE;
// // uint8_t gShoulderSwitchMode = SHOULDERSWITCHLEFT;
// // bool animationModeSetupDone = false; // Flag to track if setup has run
// bool bottangoDeregistered = false; // Track whether Bottango has been deregistered

// // Variables to track if any trim button is pressed
// // bool rightElbowActive = false;
// // bool leftElbowActive = false;

// #define MOTIONFRAMERATE 25 // ms for Motor Control Loop

// unsigned long MotionMillis = millis();
// elapsedMillis gAutoHeadElapsed[6];  // Elapsed timer for updating the head
// unsigned long gAutoHeadInterval[6]; // Random movement interval of pitch roll yaw
// const int deviation = 150;          // Adjust this value to control the subtlety of movements
// const int easingSteps = 100;        // Number of steps for easing
// float easingCurve[easingSteps];
// unsigned long lastUpdateTime = 0;
// const unsigned long servoUpdateInterval = 20; // Update servos every 20ms (50Hz)

// // Smooth step easing function (can replace with easeInOut or any other function)
// float smoothStep(float t)
// {
//     return t * t * (3 - 2 * t); // Adjust formula if you want different easing effects
// }


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

// // Controlled by slave board
// // // Shoulder Servo
// // Servo servoShoulderPitch1Right;
// // Servo servoShoulderPitch2Right;
// // Servo servoShoulderPitch1Left;
// // Servo servoShoulderPitch2Left;
// // Servo servoShoulderRollRight;
// // Servo servoShoulderRollLeft;

// // // Arm Servos
// // Servo servoElbowPitchRight;
// // Servo servoElbowPitchLeft;
// // Servo servoWristPitchRight;
// // Servo servoWristPitchLeft;
// // Servo servoWristRollRight;
// // Servo servoWristRollLeft;

// // Define struct to contain the motor and parameters
// struct ServoParameters
// {
//     // uint8_t servoBank; // servo bank that servo references
//     uint8_t servoID;
//     Servo sMotor;
//     float center_position;  // center position of motor in degrees
//     float current_position; // angle in degrees
//     float target_position;  // angle in degrees
//     float speed;            // degrees per second
//     float easing;
//     float MinDegrees; // Min degree limit
//     float MaxDegrees; // Maximum degree limit
//     bool isAttached;
//     uint8_t servoBank; // when setting up multiple teensy to control motion with one teensy addr =1
//     uint32_t startTime;     // Timestamp for when movement starts (in milliseconds)
// };

// // Add new servos to ServoParameters

// ServoParameters sTorsoLRRight = {iTorsoLRRight, servoTorsoLRRight, Torso_LR_Right_MID, Torso_LR_Right_MID, Torso_LR_Right_MID, 0, 0, Torso_LR_Right_MIN, Torso_LR_Right_MAX, false, TEENSY_MAIN, 0};
// ServoParameters sTorsoLRLeft = {iTorsoLRLeft, servoTorsoLRLeft, Torso_LR_Left_MID,  Torso_LR_Left_MID,  Torso_LR_Left_MID,  0,  0,  Torso_LR_Left_MIN,   Torso_LR_Left_MAX, false, TEENSY_MAIN, 0 };
// ServoParameters sTorsoFBRight = {iTorsoFBRight, servoTorsoFBRight, Torso_FB_Right_MID, Torso_FB_Right_MID, Torso_FB_Right_MID, 0, 0, Torso_FB_Right_MIN, Torso_FB_Right_MAX, false, TEENSY_MAIN, 0};
// ServoParameters sTorsoFBLeft = {iTorsoFBLeft, servoTorsoFBLeft,  Torso_FB_Left_MID, Torso_FB_Left_MID, Torso_FB_Left_MID, 0, 0, Torso_FB_Left_MIN, Torso_FB_Left_MAX, false, TEENSY_MAIN, 0};
// ServoParameters sTorsoHipShift = {iTorsoHipShift, servoTorsoHipShift, Torso_Hip_Shift_MID, Torso_Hip_Shift_MID, Torso_Hip_Shift_MID, 0, 0, Torso_Hip_Shift_MIN, Torso_Hip_Shift_MAX, false, TEENSY_MAIN, 0};
// ServoParameters sTorsoRotate = {iTorsoRotate, servoTorsoRotate, Torso_Rotate_MID, Torso_Rotate_MID, Torso_Rotate_MID, 0, 0, Torso_Rotate_MIN, Torso_Rotate_MAX, false, TEENSY_MAIN, 0};

// // ServoParameters sShoulderPitch1Right = {iShoulderPitch1Right, servoShoulderPitch1Right, Shoulder_Pitch1_Right_MID, Shoulder_Pitch1_Right_MID, Shoulder_Pitch1_Right_MID, 0, 0, Shoulder_Pitch1_Right_MIN, Shoulder_Pitch1_Right_MAX, false, TEENSY_SLAVE1};
// // ServoParameters sShoulderPitch2Right = {iShoulderPitch2Right, servoShoulderPitch2Right, Shoulder_Pitch2_Right_MID, Shoulder_Pitch2_Right_MID, Shoulder_Pitch2_Right_MID, 0, 0, Shoulder_Pitch2_Right_MIN, Shoulder_Pitch2_Right_MAX, false, TEENSY_SLAVE1};
// // ServoParameters sShoulderPitch1Left = {iShoulderPitch1Left, servoShoulderPitch1Left, Shoulder_Pitch1_Left_MID, Shoulder_Pitch1_Left_MID, Shoulder_Pitch1_Left_MID, 0, 0, Shoulder_Pitch1_Left_MIN, Shoulder_Pitch1_Left_MAX, false, TEENSY_SLAVE1};
// // ServoParameters sShoulderPitch2Left = {iShoulderPitch2Left, servoShoulderPitch2Left, Shoulder_Pitch2_Left_MID, Shoulder_Pitch2_Left_MID, Shoulder_Pitch2_Left_MID, 0, 0, Shoulder_Pitch2_Left_MIN, Shoulder_Pitch2_Left_MAX, false, TEENSY_SLAVE1};
// // ServoParameters sShoulderRollRight = {iShoulderRollRight, servoShoulderRollRight, Shoulder_Roll_Right_MID, Shoulder_Roll_Right_MID, Shoulder_Roll_Right_MID, 0, 0, Shoulder_Roll_Right_MIN, Shoulder_Roll_Right_MAX, false, TEENSY_SLAVE1};
// // ServoParameters sShoulderRollLeft = {iShoulderRollLeft, servoShoulderRollLeft, Shoulder_Roll_Left_MID, Shoulder_Roll_Left_MID, Shoulder_Roll_Left_MID, 0, 0, Shoulder_Roll_Left_MIN, Shoulder_Roll_Left_MAX, false, TEENSY_SLAVE1};

// // ServoParameters sElbowPitchRight = {iElbowPitchRight, servoElbowPitchRight, Elbow_Pitch_Right_MID, Elbow_Pitch_Right_MID, Elbow_Pitch_Right_MID, 0, 0, Elbow_Pitch_Right_MIN, Elbow_Pitch_Right_MAX, false, TEENSY_SLAVE1};
// // ServoParameters sElbowPitchLeft = {iElbowPitchLeft, servoElbowPitchLeft, Elbow_Pitch_Left_MID, Elbow_Pitch_Left_MID, Elbow_Pitch_Left_MID, 0, 0, Elbow_Pitch_Left_MIN, Elbow_Pitch_Left_MAX, false, TEENSY_SLAVE1};
// // ServoParameters sWristPitchRight = {iWristPitchRight, servoWristPitchRight, Wrist_Pitch_Right_MID, Wrist_Pitch_Right_MID, Wrist_Pitch_Right_MID, 0, 0, Wrist_Pitch_Right_MIN, Wrist_Pitch_Right_MAX, false, TEENSY_SLAVE1};
// // ServoParameters sWristPitchLeft = {iWristPitchLeft, servoWristPitchLeft, Wrist_Pitch_Left_MID, Wrist_Pitch_Left_MID, Wrist_Pitch_Left_MID, 0, 0, Wrist_Pitch_Left_MIN, Wrist_Pitch_Left_MAX, false, TEENSY_SLAVE1};
// // ServoParameters sWristRollRight = {iWristRollRight, servoWristRollRight, Wrist_Roll_Right_MID, Wrist_Roll_Right_MID, Wrist_Roll_Right_MID, 0, 0, Wrist_Roll_Right_MIN, Wrist_Roll_Right_MAX, false, TEENSY_SLAVE1};
// // ServoParameters sWristRollLeft = {iWristRollLeft, servoWristRollLeft, Wrist_Roll_Left_MID, Wrist_Roll_Left_MID, Wrist_Roll_Left_MID, 0, 0, Wrist_Roll_Left_MIN, Wrist_Roll_Left_MAX, false, TEENSY_SLAVE1};

// ServoParameters sHeadRotate = {iHeadRotate, servoHeadRotate, Head_Rotate_MID, Head_Rotate_MID, Head_Rotate_MID, 0, 0, Head_Rotate_MIN, Head_Rotate_MAX, false, TEENSY_MAIN, 0};
// ServoParameters sHeadPitch1 = {iHeadPitch1, servoHeadPitch1, Head_Pitch1_MID, Head_Pitch1_MID,Head_Pitch1_MID, 0, 0, Head_Pitch1_MIN, Head_Pitch1_MAX, false, TEENSY_MAIN, 0};
// ServoParameters sHeadPitch2 = {iHeadPitch2, servoHeadPitch2, Head_Pitch2_MID, Head_Pitch2_MID, Head_Pitch2_MID, 0, 0, Head_Pitch2_MIN, Head_Pitch2_MAX, false, TEENSY_MAIN, 0};
// ServoParameters sHeadPitch3 = {iHeadPitch3, servoHeadPitch3, Head_Pitch3_MID, Head_Pitch3_MID, Head_Pitch3_MID, 0, 0, Head_Pitch3_MIN, Head_Pitch3_MAX, false, TEENSY_MAIN, 0};

// ServoParameters ServoList[] = {
//     sTorsoLRRight,
//     sTorsoLRLeft,
//     sTorsoFBRight,
//     sTorsoFBLeft,
//     sTorsoHipShift,
//     sTorsoRotate,

//     // sShoulderPitch1Right,
//     // sShoulderPitch2Right,
//     // sShoulderPitch1Left,
//     // sShoulderPitch2Left,
//     // sShoulderRollRight,
//     // sShoulderRollLeft,

//     // sElbowPitchRight,
//     // sElbowPitchLeft,
//     //  sWristPitchRight,
//     //  sWristPitchLeft,
//     //  sWristRollRight,
//     //   sWristRollLeft,

//     sHeadRotate,
//     sHeadPitch1,
//     sHeadPitch2,
//     sHeadPitch3,
// };

// // Total number of Servos
// const int NUM_SERVOS = sizeof(ServoList) / sizeof(ServoList[0]);

// uint8_t ServoPins[] = {
//     pinTorsoLRRight,
//     pinTorsoLRLeft,
//     pinTorsoFBRight,
//     pinTorsoFBLeft,
//     pinTorsoHipShift,
//     pinTorsoRotate,

//     // pinShoulderPitch1Right,
//     // pinShoulderPitch2Right,
//     // pinShoulderPitch1Left,
//     // pinShoulderPitch2Left,
//     // pinShoulderRollRight,
//     // pinShoulderRollLeft,

//     // pinElbowPitchRight,
//     // pinElbowPitchLeft,
//     //  pinWristPitchRight,
//     //   pinWristPitchLeft,
//     //  pinWristRollRight,
//     //  pinWristRollLeft,

//     pinHeadRotate,
//     pinHeadPitch1,
//     pinHeadPitch2,
//     pinHeadPitch3,

// };

// void SetupMotion()
// {

//     Serial.println("Setting up Servos");
//     pinMode(10, OUTPUT);
//     digitalWrite(10, HIGH); // Turn D1-D3 as outputs.
//     delay(25);

//     // Set all servos to center position

//         // TxServoPosition(ServoList[i].servoBank, ServoList[i].servoID, ServoList[i].center_position);

//     // setupEasingCurve();  // Pre-calculate easing values
//     // attach servos
//     // ServoAttach(0);
//     Serial.println("Setup Servos Complete");
// }
// // void setupEasingCurve() {
// //     for (int i = 0; i < easingSteps; i++) {
// //         float t = (float)i / (easingSteps - 1);  // Normalize `i` to [0,1] range
// //         easingCurve[i] = smoothStep(t);
// //     }
// // }
// void MotionLoop()
// {
//     float PitchAngle;
//     float RollAngle;

//     if (millis() - MotionMillis > MOTIONFRAMERATE)
//      {
//     //    toggleControlMode();
//         if(gRCBottangoMode == RC_MODE)
//         {        
//             if (!bottangoDeregistered)  // Ensure it runs only once
//             {
//               if (BottangoCore::commandStreamProvider) {
//                  BottangoCore::effectorPool.deregisterAll();
//                  }
//                bottangoDeregistered = true;
//             }
    
//           if (gHeadRunMode == HEADCONTROL)
//           {
//             // HEAD FUNCTIONS
//             PitchAngle = (ChannelData(cHEADPITCH) - SBUSCENTER) / 2;
//             RollAngle = (ChannelData(cHEADROLL) - SBUSCENTER) / 2;
//             CalculateHeadPosition(PitchAngle, RollAngle);

//             // Set Head Yaw
//             ServoList[iHeadRotate].target_position = map(ChannelData(cHEADYAW), SBUSMIN, SBUSMAX, ServoList[iHeadRotate].MinDegrees, ServoList[iHeadRotate].MaxDegrees);

//             // TORSO FUNCTIONS
//             CalculateTorsoLeanFB((ChannelData(cTORSOFB) - SBUSCENTER) / 2);
//             CalculateTorsoLeanLR((ChannelData(cTORSOLR) - SBUSCENTER) / 2);

//             // Set Torso Rotate
//             ServoList[iTorsoRotate].target_position = map(ChannelData(cTORSOROT), SBUSMIN, SBUSMAX, ServoList[iTorsoRotate].MinDegrees, ServoList[iTorsoRotate].MaxDegrees);

//             // Set Hip Shift
//             ServoList[iTorsoHipShift].target_position = map(ChannelData(cTORSOHIPSHIFT), SBUSMIN, SBUSMAX, ServoList[iTorsoHipShift].MinDegrees, ServoList[iTorsoHipShift].MaxDegrees);


//             // Update all servos
//             UpdateServos();
//         }

//         if (gHeadRunMode == HEADIDLE)
//         {
//             //  ServoDetach(0); // Detach servos
//         }
//         if (millis() - lastUpdateTime >= servoUpdateInterval)
//         {
//             lastUpdateTime = millis(); // Store the current time
        
//             if (gHeadRunMode == HEADAUTO)
//             {
//                 // Process random head movements
//                 for (uint8_t x = 0; x < 3; x++)  // Ensures 0,1,2,3
//                 {
//                     // Update each axis if enough time has passed
//                     if (gAutoHeadElapsed[x] >= gAutoHeadInterval[x])
//                     {
//                         switch (x)
//                         {
//                         case 0: // Yaw (Head Rotate)
//                             ProcessAutoHeadMovement(iHeadRotate, Head_Rotate_MID, deviation);
//                             gAutoHeadInterval[x] = random(8000, 15000);
//                             break;
//                         case 1: // Pitch1
//                             ProcessAutoHeadMovement(iHeadPitch1, Head_Pitch1_MID, deviation);
//                             gAutoHeadInterval[x] = random(3000, 18000);
//                             break;
//                         case 2: // Pitch2
//                             ProcessAutoHeadMovement(iHeadPitch2, Head_Pitch2_MID, deviation);
//                             gAutoHeadInterval[x] = random(15000, 25000);
//                             break;
//                         case 3: // Pitch3
//                             ProcessAutoHeadMovement(iHeadPitch3, Head_Pitch3_MID, deviation);
//                             gAutoHeadInterval[x] = random(15000, 25000);
//                             break;
//                         }
        
//                         // Reset elapsed time
//                         gAutoHeadElapsed[x] = 0;
//                     }
//                     else
//                     {
//                         // Increment elapsed time
//                         gAutoHeadElapsed[x] += millis() - lastUpdateTime;
//                     }
//                 }
        
//                 // Gradually move each servo toward its target position with easing
//                 for (int i = 0; i < 3; i++)
//                 {
//                     int servoIndex;
//                     if (i == 0) servoIndex = iHeadRotate;
//                     else if (i == 1) servoIndex = iHeadPitch1;
//                     else if (i == 2) servoIndex = iHeadPitch2;
//                     else servoIndex = iHeadPitch3;
        
//                     if (ServoList[servoIndex].current_position != ServoList[servoIndex].target_position)
//                     {
//                         // Compute elapsed time for easing
//                         unsigned long elapsedTime = millis() - ServoList[servoIndex].startTime;
//                         float easingProgress = (float)elapsedTime / gAutoHeadInterval[i]; 
                        
//                         // Ensure progress is within [0,1]
//                         easingProgress = constrain(easingProgress, 0.0, 1.0);
                        
//                         int easingStepIndex = (int)(easingProgress * (easingSteps - 1)); 
//                         easingStepIndex = constrain(easingStepIndex, 0, easingSteps - 1);  
        
//                         // Get eased factor from curve
//                         float easedFactor = easingCurve[easingStepIndex];
//                         float easedPosition = ServoList[servoIndex].current_position +
//                                               (ServoList[servoIndex].target_position - ServoList[servoIndex].current_position) * easedFactor;
        
//                         // Map eased position to microseconds
//                         int currentMicroseconds = map(easedPosition, 
//                                                       ServoList[servoIndex].MinDegrees, ServoList[servoIndex].MaxDegrees, 
//                                                       1000, 2000);
        
//                         // Set servo position
//                         ServoList[servoIndex].sMotor.writeMicroseconds(currentMicroseconds);
        
//                         // Update current position
//                         ServoList[servoIndex].current_position = map(currentMicroseconds, 
//                                                                      1000, 2000, 
//                                                                      ServoList[servoIndex].MinDegrees, ServoList[servoIndex].MaxDegrees);
//                     }
//                 }
//             }
//         }
//     }        


//     if (gRCBottangoMode == ANIMATION_MODE)
//     {
        
//         bottangoDeregistered = false;  // Reset flag when switching to ANIMATION_MODE
//         GeneratedCodeAnimations::AnimationControl();
//     }

// }
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
//     }
// }

// void UpdateServos()
// {
//     // if (gHeadRunMode == HEADCONTROL)
//     // {
//         for (uint8_t i = 0; i < NUM_SERVOS; i++)
//         {
//             if (abs(ServoList[i].target_position - ServoList[i].current_position) > 1)
//             {
//                 SetServoPosition(i, ServoList[i].target_position);
//                 ServoList[i].current_position = ServoList[i].target_position;
//             }
//         }
//     // }
// }

// // void StopAllMotion()
// // {
// //     ServoDetach(0);
// //     // Implement Stop Motion function logic
// // }

// void SetHeadRunModeAuto()
// {
//     gHeadRunMode = HEADAUTO;
//     Serial.println("HeadMode Auto");
// }

// void SetHeadRunModeIdle()
// {
//     gHeadRunMode = HEADIDLE;
//     Serial.println("HeadMode Idle");
//     //StopAllMotion();
// }

// void SetHeadRunModeControl()
// {
//     gHeadRunMode = HEADCONTROL;
//     Serial.println("HeadMode Control");
// }

// uint8_t GetHeadMode()
// {
//     return gHeadRunMode;
// }

// void SetRCBottangoMode_RC() {
//   gRCBottangoMode = RC_MODE;
//   if (BottangoCore::commandStreamProvider) {
//     BottangoCore::commandStreamProvider->stop();
//   }
//   Serial.println("RC MODE");
// }

// // void MotionLoop() {
// //   // ...
// //   if (gRCBottangoMode == RC_MODE) {
// //     if (!bottangoDeregistered) {
// //       if (BottangoCore::commandStreamProvider) {
// //         BottangoCore::effectorPool.deregisterAll();
// //       }
// //       bottangoDeregistered = true;
// //     }
// //     // ...
// //   }
// // }


// void SetRCBottangoMode_Animation()
// {
//     gRCBottangoMode = ANIMATION_MODE;
//     Serial.println("ANIMATION MODE");

// }

// uint8_t GetRCBottangoMode()
// {
//     return gRCBottangoMode;
// }
// // Mix the pitch and roll together to calculate the required servo positions.
// // 120° servo mix

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

// // void toggleControlMode() {
// //     if (currentMode == RC_MODE) {
// //         currentMode = ANIMATION_MODE;
// //         Serial.println("Switched to Animation Mode");
// //         // Optionally stop any playing animation when switching back to RC mode
// //         BottangoCore::commandStreamProvider.stop();
// //     } else {
// //         currentMode = RC_MODE;
// //         Serial.println("Switched to RC Mode");
// //     }
// // }
// // void triggerAnimation() {

// //     GeneratedCommandStreams::AnimationControl();
// //     {
// //     // if (!BottangoCore::commandStreamProvider.streamIsInProgress()) {
// //     //     BottangoCore::commandStreamProvider.startCommandStream(streamID, loop);
        
        

// //         // Serial.print("Started animation ID: ");
// //         // Serial.println(streamID);

// //     }
// // }
// //         if (millis() - lastUpdateTime >= servoUpdateInterval)
// //         {
// //             lastUpdateTime = millis(); // Store the current time

// //             if (gHeadRunMode == HEADAUTO)
// //             {
// //                 // ServoAttach(0);  // Ensure servos are attached

// //                 // Process random head movements
// //                 for (uint8_t x = 0; x <= 3; x++)
// //                 {
// //                     // Update each axis if enough time has passed
// //                     if (gAutoHeadElapsed[x] > gAutoHeadInterval[x])
// //                     {
// //                         switch (x)
// //                         {
// //                         case 0: // Yaw (Head Rotate)
// //                             ProcessAutoHeadMovement(iHeadRotate, Head_Rotate_MID, deviation);
// //                             gAutoHeadInterval[x] = random(8000, 15000); // Randomize next interval
// //                             break;
// //                         case 1: // Pitch1
// //                             ProcessAutoHeadMovement(iHeadPitch1, Head_Pitch1_MID, deviation);
// //                             gAutoHeadInterval[x] = random(3000, 18000); // Randomize next interval
// //                             break;
// //                         case 2: // Pitch2
// //                             ProcessAutoHeadMovement(iHeadPitch2, Head_Pitch2_MID, deviation);
// //                             gAutoHeadInterval[x] = random(15000, 25000); // Randomize next interval
// //                             break;
// //                         case 3: // Pitch3
// //                             ProcessAutoHeadMovement(iHeadPitch3, Head_Pitch3_MID, deviation);
// //                             gAutoHeadInterval[x] = random(15000, 25000); // Randomize next interval
// //                             break;
// //                         }

// //                         // Reset the timer for this axis
// //                         gAutoHeadElapsed[x] = 0;
// //                     }
// //                 }

// //                 // Gradually move each servo toward its target position with easing
// //                 for (int i = 0; i < 4; i++)
// //                 {
// //                     int servoIndex;
// //                     if (i == 0)
// //                     {
// //                         servoIndex = iHeadRotate;
// //                     }
// //                     else if (i == 1)
// //                     {
// //                         servoIndex = iHeadPitch1;
// //                     }
// //                     else if (i == 2)
// //                     {
// //                         servoIndex = iHeadPitch2;
// //                     }
// //                     else
// //                     {
// //                         servoIndex = iHeadPitch3;
// //                     }

// //                     if (ServoList[servoIndex].current_position != ServoList[servoIndex].target_position)
// //                     {
// //                         // Calculate easing progress based on elapsed time and easingSteps
// //                         float easingProgress = (float)(millis() - gAutoHeadElapsed[i]) / gAutoHeadInterval[i]; // Easing progress [0,1]
// //                         int easingStepIndex = (int)(easingProgress * (easingSteps - 1));                       // Get index in easing curve array
// //                         easingStepIndex = constrain(easingStepIndex, 0, easingSteps - 1);                      // Ensure index is in bounds

// //                         // Calculate eased position
// //                         float easedFactor = easingCurve[easingStepIndex];
// //                         float easedPosition = ServoList[servoIndex].current_position +
// //                                               (ServoList[servoIndex].target_position - ServoList[servoIndex].current_position) * easedFactor;

// //                         // Map eased position to microseconds
// //                         int currentMicroseconds = map(easedPosition, ServoList[servoIndex].MinDegrees, ServoList[servoIndex].MaxDegrees, 1000, 2000);

// //                         // Set the servo position in microseconds
// //                         ServoList[servoIndex].sMotor.writeMicroseconds(currentMicroseconds);

// //                         // Update the current position to reflect the eased value
// //                         ServoList[servoIndex].current_position = map(currentMicroseconds, 1000, 2000, ServoList[servoIndex].MinDegrees, ServoList[servoIndex].MaxDegrees);
// //                     }
// //                 }
// //             }
// //         }
// //     }
// //   }
// // ***SHOULDER AND ELBOW FUNCTIONS NOW PROCESSED ON SLAVE BOARD***
// // // void SetShoulderRunModeLeft()
// // // {
// // //     // Set the shoulder mode to left
// // //     gShoulderSwitchMode = SHOULDERSWITCHLEFT;

// // //     // Flexion Function (Shoulder Roll)
// // //     ServoList[iShoulderRollLeft].target_position = map(ChannelData(cSHFL), SBUSMIN, SBUSMAX, ServoList[iShoulderRollLeft].MinDegrees, ServoList[iShoulderRollLeft].MaxDegrees);

// // //     // Abduction Function (Shoulder Pitch 1 and 2 work in opposite directions)
// // //     int abductionValueLeft = ChannelData(cSHAB);
// // //     int targetPitch1Left = 1460;  // Default neutral for Pitch 1
// // //     int targetPitch2Left = 1130;  // Default neutral for Pitch 2

// // //     // If abduction input is active (positive direction)
// // //     if (abductionValueLeft > SBUSNEUTRAL) {
// // //         targetPitch1Left = map(abductionValueLeft, SBUSNEUTRAL, SBUSMAX, 1460, 2280);  // Abduction range for Pitch 1
// // //         targetPitch2Left = map(abductionValueLeft, SBUSNEUTRAL, SBUSMAX, 1130, 641);   // Abduction range for Pitch 2 (opposite)
// // //     }

// // //     // External Rotation Function - Both servos move together
// // //     int externalRotationValueLeft = ChannelData(cSHER);

// // //     if (externalRotationValueLeft > SBUSNEUTRAL && isExternalRotationModeLeft()) {
// // //         // If external rotation mode is active
// // //         targetPitch1Left = map(externalRotationValueLeft, SBUSNEUTRAL, SBUSMAX, 1460, 640);  // External rotation for Pitch 1
// // //         targetPitch2Left = map(externalRotationValueLeft, SBUSNEUTRAL, SBUSMAX, 1130, 640);  // External rotation for Pitch 2 (same direction)
// // //     }

// // //     // Set the target positions for Pitch 1 and Pitch 2 servos
// // //     ServoList[iShoulderPitch1Left].target_position = targetPitch1Left;
// // //     ServoList[iShoulderPitch2Left].target_position = targetPitch2Left;
// // // }

// // // // Helper function to determine if the shoulder is in external rotation mode (left)
// // // bool isExternalRotationModeLeft() 
// // // {
// // //     // Get the value from the left shoulder external rotation channel (cSHER)
// // //     int erInputLeft = ChannelData(cSHER);

// // //     // Define the "quiet" range around 992 ± 100 for the left shoulder
// // //     const int quietLowerBound = 892;
// // //     const int quietUpperBound = 1092;

// // //     // Return true if external rotation is active, meaning the input is outside the quiet range
// // //     return (erInputLeft < quietLowerBound || erInputLeft > quietUpperBound);
// // // }


// // void SetShoulderRunModeLeft()
// // {
// //     // Flexion (FL) - Shoulder Roll for Left Shoulder
// //     ServoList[iShoulderRollLeft].target_position = map(ChannelData(cSHFL), SBUSMIN, SBUSMAX, ServoList[iShoulderRollLeft].MinDegrees, ServoList[iShoulderRollLeft].MaxDegrees);

// //     // Initialize target positions to neutral
// //     int targetPitch1Left = 1460;
// //     int targetPitch2Left = 1130;

// //     // Read abduction and external rotation input values
// //     int abductionValue = ChannelData(cSHAB);
// //     int externalRotationValue = ChannelData(cSHER);

// //     // Calculate abduction target positions for Left Pitch 1 and Pitch 2
// //     if (abductionValue > SBUSNEUTRAL) {
// //         targetPitch1Left = map(abductionValue, SBUSNEUTRAL, SBUSMAX, 1460, 2280);
// //         targetPitch2Left = map(abductionValue, SBUSNEUTRAL, SBUSMAX, 1130, 641);
// //     }

// //     // Apply external rotation on top of abduction positions
// //     if (isExternalRotationMode()) {
// //         int rotationOffsetPitch1Left = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, 200);
// //         int rotationOffsetPitch2Left = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, -200);

// //         targetPitch1Left += rotationOffsetPitch1Left;
// //         targetPitch2Left += rotationOffsetPitch2Left;
// //     }

// //     // Update the target positions for Left Shoulder
// //     ServoList[iShoulderPitch1Left].target_position = targetPitch1Left;
// //     ServoList[iShoulderPitch2Left].target_position = targetPitch2Left;
// // }

// // // Helper function to determine if the shoulder is in external rotation mode (left)
// // bool isExternalRotationModeLeft() 
// // {
// //     // Get the value from the left shoulder external rotation channel (cSHER)
// //     int erInputLeft = ChannelData(cSHER);

// //     // Define the "quiet" range around 992 ± 100 for the left shoulder
// //     const int quietLowerBound = 892;
// //     const int quietUpperBound = 1092;

// //     // Return true if external rotation is active, meaning the input is outside the quiet range
// //     return (erInputLeft < quietLowerBound || erInputLeft > quietUpperBound);
// // }

// // // void SetShoulderRunModeRight() 
// // // {
// // //     gShoulderSwitchMode = SHOULDERSWITCHRIGHT;

// // //     // Shoulder Roll - Flexion Control
// // //     ServoList[iShoulderRollRight].target_position = map(ChannelData(cSHFL), SBUSMIN, SBUSMAX, ServoList[iShoulderRollRight].MinDegrees, ServoList[iShoulderRollRight].MaxDegrees);

// // //     // Abduction Function (Shoulder Pitch 1 and 2 work in opposite directions)
// // //     int abductionValueRight = ChannelData(cSHAB);
// // //     int targetPitch1Right = 1420;  // Default neutral for Pitch 1
// // //     int targetPitch2Right = 2000;  // Default neutral for Pitch 2

// // //     if (abductionValueRight > SBUSNEUTRAL) {  
// // //         // When the stick is pushed in the positive direction (abduction)
// // //         targetPitch1Right = map(abductionValueRight, SBUSNEUTRAL, SBUSMAX, 1420, 2400);  // Shoulder Pitch 1 (neutral to full abduction)
// // //         targetPitch2Right = map(abductionValueRight, SBUSNEUTRAL, SBUSMAX, 2000, 640); // Shoulder Pitch 2 (opposite)
// // //     }

// // //     // External Rotation Function (Both servos should move together in the same direction)
// // //     int externalRotationValueRight = ChannelData(cSHER);

// // //     if (externalRotationValueRight > SBUSNEUTRAL && isExternalRotationModeRight()) {  
// // //         // When the stick is pushed in the external rotation direction (992 to 1811)
// // //         targetPitch1Right = map(externalRotationValueRight, SBUSNEUTRAL, SBUSMAX, 1420, 2200);  // Pitch 1 external rotation
// // //         targetPitch2Right = map(externalRotationValueRight, SBUSNEUTRAL, SBUSMAX, 2000, 2400);  // Pitch 2 external rotation
// // //     }

// // //     // Update the target positions for Pitch 1 and Pitch 2 servos
// // //     ServoList[iShoulderPitch1Right].target_position = targetPitch1Right;
// // //     ServoList[iShoulderPitch2Right].target_position = targetPitch2Right;
    
// // // }

// // // // Helper function to determine if the shoulder is in external rotation mode
// // // bool isExternalRotationModeRight() 
// // // {
// // //     // Get the value from the external rotation channel (cSHER)
// // //     int erInput = ChannelData(cSHER);

// // //     // Define the "quiet" range around 992 ± 100
// // //     const int quietLowerBound = 892;
// // //     const int quietUpperBound = 1092;

// // //     // Return true if external rotation is active, meaning the input is outside the quiet range
// // //     return (erInput < quietLowerBound || erInput > quietUpperBound);
// // // }


// // void SetShoulderRunModeRight()
// // {
// //     // Flexion (FL) - Shoulder Roll for Right Shoulder
// //     ServoList[iShoulderRollRight].target_position = map(ChannelData(cSHFL), SBUSMIN, SBUSMAX, ServoList[iShoulderRollRight].MinDegrees, ServoList[iShoulderRollRight].MaxDegrees);

// //     // Initialize target positions to neutral
// //     int targetPitch1Right = 1420;
// //     int targetPitch2Right = 2000;

// //     // Read abduction and external rotation input values
// //     int abductionValue = ChannelData(cSHAB);
// //     int externalRotationValue = ChannelData(cSHER);

// //     // Calculate abduction target positions for Right Pitch 1 and Pitch 2
// //     if (abductionValue > SBUSNEUTRAL) {
// //         targetPitch1Right = map(abductionValue, SBUSNEUTRAL, SBUSMAX, 1420, 2600);
// //         targetPitch2Right = map(abductionValue, SBUSNEUTRAL, SBUSMAX, 2000, 640);
// //     }

// //     // Apply external rotation on top of abduction positions
// //     if (isExternalRotationMode()) {
// //         int rotationOffsetPitch1Right = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, 100);
// //         int rotationOffsetPitch2Right = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, 1200);

// //         targetPitch1Right += rotationOffsetPitch1Right;
// //         targetPitch2Right += rotationOffsetPitch2Right;
// //     }

// //     // Update the target positions for Right Shoulder
// //     ServoList[iShoulderPitch1Right].target_position = targetPitch1Right;
// //     ServoList[iShoulderPitch2Right].target_position = targetPitch2Right;
// // }


// // // Helper function to determine if the shoulder is in external rotation mode
// // bool isExternalRotationModeRight() 
// // {
// //     // Get the value from the external rotation channel (cSHER)
// //     int erInput = ChannelData(cSHER);

// //     // Define the "quiet" range around 992 ± 100
// //     const int quietLowerBound = 892;
// //     const int quietUpperBound = 1092;

// //     // Return true if external rotation is active, meaning the input is outside the quiet range
// //     return (erInput < quietLowerBound || erInput > quietUpperBound);
// // }


// // // 
// // void SetShoulderRunModeBoth()
// // {
// //     // Set flexion (FL) for both shoulders - Shoulder Roll
// //     ServoList[iShoulderRollRight].target_position = map(ChannelData(cSHFL), SBUSMIN, SBUSMAX, ServoList[iShoulderRollRight].MinDegrees, ServoList[iShoulderRollRight].MaxDegrees);
// //     ServoList[iShoulderRollLeft].target_position = map(ChannelData(cSHFL), SBUSMIN, SBUSMAX, ServoList[iShoulderRollLeft].MinDegrees, ServoList[iShoulderRollLeft].MaxDegrees);

// //     // Initialize target positions to neutral for both shoulders
// //     int targetPitch1Right = 1420;
// //     int targetPitch2Right = 2000;
// //     int targetPitch1Left = 1460;
// //     int targetPitch2Left = 1130;

// //     // Read abduction and external rotation input values
// //     int abductionValue = ChannelData(cSHAB);
// //     int externalRotationValue = ChannelData(cSHER);

// //     // Calculate abduction target positions for both shoulders
// //     if (abductionValue > SBUSNEUTRAL) {
// //         targetPitch1Left = map(abductionValue, SBUSNEUTRAL, SBUSMAX, 1460, 2280);
// //         targetPitch2Left = map(abductionValue, SBUSNEUTRAL, SBUSMAX, 1130, 641);

// //         targetPitch1Right = map(abductionValue, SBUSNEUTRAL, SBUSMAX, 1420, 640);
// //         targetPitch2Right = map(abductionValue, SBUSNEUTRAL, SBUSMAX, 2000, 2400);
// //     }

// //     // Apply external rotation on top of abduction positions for both shoulders
// //     if (isExternalRotationMode()) {
// //         int rotationOffsetPitch1Right = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, 200);
// //         int rotationOffsetPitch2Right = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, -200);
        
// //         int rotationOffsetPitch1Left = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, 200);
// //         int rotationOffsetPitch2Left = map(externalRotationValue, SBUSNEUTRAL, SBUSMAX, 0, -200);

// //         targetPitch1Right += rotationOffsetPitch1Right;
// //         targetPitch2Right += rotationOffsetPitch2Right;
        
// //         targetPitch1Left += rotationOffsetPitch1Left;
// //         targetPitch2Left += rotationOffsetPitch2Left;
// //     }

// //     // Update the target positions for both shoulders
// //     ServoList[iShoulderPitch1Right].target_position = targetPitch1Right;
// //     ServoList[iShoulderPitch2Right].target_position = targetPitch2Right;
// //     ServoList[iShoulderPitch1Left].target_position = targetPitch1Left;
// //     ServoList[iShoulderPitch2Left].target_position = targetPitch2Left;
// // }


// // // Helper function to determine if the shoulder is in external rotation mode
// // bool isExternalRotationMode() 
// // {
// //     // Get the value from the external rotation channel (cSHER)
// //     int erInput = ChannelData(cSHER);

// //     // Define the "quiet" range around 992 ± 100
// //     const int quietLowerBound = 892;
// //     const int quietUpperBound = 1092;

// //     // Return true if external rotation is active, meaning the input is outside the quiet range
// //     return (erInput < quietLowerBound || erInput > quietUpperBound);
// // }

// // uint8_t GetShoulderMode()
// // {
// //     return gShoulderSwitchMode;
// // }

// // void SetShoulderMode(uint8_t mode)
// // {
// //     gShoulderSwitchMode = mode;
// // }
// // // void TestAllServos(void)
// // // {
// // //     Serial.println("testing all servos...");
// // //     Serial.println("Set all to min");
// // //     for (int i = 0; i < NUM_SERVOS; i++)
// // //     {
// // //         delay(1000);
// // //         TxServoPosition(ServoList[i].servoBank, ServoList[i].servoID, ServoList[i].MinDegrees);
// // //     }
// // //     delay(2000);
// // //     Serial.println("Set all to max");
// // //     for (int i = 0; i < NUM_SERVOS; i++)
// // //     {
// // //         delay(1000);
// // //         TxServoPosition(ServoList[i].servoBank, ServoList[i].servoID, ServoList[i].MaxDegrees);
// // //     }
// // //     delay(2000);
// // //     Serial.println("Set all to mid");
// // //     for (int i = 0; i < NUM_SERVOS; i++)
// // //     {
// // //         delay(1000);
// // //         TxServoPosition(ServoList[i].servoBank, ServoList[i].servoID, ServoList[i].center_position);
// // //     }
// // // }

// // void elbowmotionloop()
// // {

// //     // If no trim button is actively pressed, re-center the elbows
// //     if (!rightElbowActive)
// //     {
// //         // Serial.println("Re-center right elbow");
// //         ServoList[iElbowPitchRight].target_position = Elbow_Pitch_Right_MID;
// //     }

// //     if (!leftElbowActive)
// //     {
// //         // Serial.println("Re-center left elbow");
// //         ServoList[iElbowPitchLeft].target_position = Elbow_Pitch_Left_MID;
// //     }


// // }
// // void ElbowCenter()
// // {
// //     rightElbowActive = false;
// //     leftElbowActive = false;
// //     elbowmotionloop();

// //     ServoList[iElbowPitchRight].target_position = Elbow_Pitch_Right_MID;
// //     // Serial.print(" SP: ");
// //     // Serial.print(iElbowPitchRight);
// //     // Serial.print(" ");
// //     // Serial.println(ServoList[iElbowPitchRight].target_position);
// //     ServoList[iElbowPitchRight].sMotor.writeMicroseconds(ServoList[iElbowPitchRight].target_position);

// //     ServoList[iElbowPitchLeft].target_position = Elbow_Pitch_Left_MID;
// //     // Serial.print(" SP: ");
// //     // Serial.print(iElbowPitchLeft);
// //     // Serial.print(" ");
// //     // Serial.println(ServoList[iElbowPitchLeft].target_position);
// //     ServoList[iElbowPitchLeft].sMotor.writeMicroseconds(ServoList[iElbowPitchLeft].target_position);
// // }

// // void LeftElbowPositionMin()
// // {
// //     ServoList[iElbowPitchLeft].target_position = Elbow_Pitch_Left_MIN;
// //     Serial.print(" SP: ");
// //     Serial.print(iElbowPitchLeft);
// //     Serial.print(" ");
// //     Serial.println(ServoList[iElbowPitchLeft].target_position);
// //     ServoList[iElbowPitchLeft].sMotor.writeMicroseconds(ServoList[iElbowPitchLeft].target_position);
// //     leftElbowActive = true;
// //     elbowmotionloop();
// // }

// // void LeftElbowPositionMax()
// // {
// //     ServoList[iElbowPitchLeft].target_position = Elbow_Pitch_Left_MAX;
// //     Serial.print(" SP: ");
// //     Serial.print(iElbowPitchLeft);
// //     Serial.print(" ");
// //     Serial.println(ServoList[iElbowPitchLeft].target_position);
// //     ServoList[iElbowPitchLeft].sMotor.writeMicroseconds(ServoList[iElbowPitchLeft].target_position);
// //     leftElbowActive = true;
// //     elbowmotionloop();
// // }

// // void RightElbowPositionMax()
// // {
// //     ServoList[iElbowPitchRight].target_position = Elbow_Pitch_Right_MAX;
// //     Serial.print(" SP: ");
// //     Serial.print(iElbowPitchRight);
// //     Serial.print(" ");
// //     Serial.println(ServoList[iElbowPitchRight].target_position);
// //     ServoList[iElbowPitchRight].sMotor.writeMicroseconds(ServoList[iElbowPitchRight].target_position);
// //     rightElbowActive = true;
// //     elbowmotionloop();
// // }
// // void RightElbowPositionMin()
// // {
// //     ServoList[iElbowPitchRight].target_position = Elbow_Pitch_Right_MIN;
// //     Serial.print(" SP: ");
// //     Serial.print(iElbowPitchRight);
// //     Serial.print(" ");
// //     Serial.println(ServoList[iElbowPitchRight].target_position);
// //     ServoList[iElbowPitchRight].sMotor.writeMicroseconds(ServoList[iElbowPitchRight].target_position);
// //     rightElbowActive = true;
// //     elbowmotionloop();
// // }

// void ServoAttach(uint8_t ServoNum)
// {

//     if (ServoNum == 0)
//     { // attach all
//         for (uint8_t x = 0; x < NUM_SERVOS; x++)
//         {
//             ServoList[x].sMotor.attach(ServoPins[x]);

//             Serial.print(" attach debug sMotor ID: ");//debug code for servo attach
//             Serial.println(x);
//             Serial.print("Servo pin number: ");
//             Serial.println(ServoPins[x]);

//             ServoList[x].isAttached = 1;
//             ServoList[x].sMotor.writeMicroseconds(ServoList[x].target_position);
//             Serial.print("Servo target position: ");
//             Serial.println(ServoList[x].target_position);
//             delay(50);
//         }
//             Serial.println("Servo setup complete ");
//     }
//     else
//     {
//         ServoList[ServoNum].sMotor.attach(ServoPins[ServoNum]);
//             Serial.print(" attach debug ServoNum ID: ");//debug code for servo attach
//             Serial.println(ServoNum);
//             Serial.print("Servo pin number: ");
//             Serial.println(ServoPins[ServoNum]);
//         ServoList[ServoNum].isAttached = true;
//         ServoList[ServoNum].sMotor.writeMicroseconds(ServoList[ServoNum].target_position);
//     }
//        Serial.println("Servo attach complete ");
// }

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

// void SetServoPosition(int ServoID, float target_position)
// {
//     Serial.print(" SP: ");
//     Serial.print(ServoID);
//     Serial.print(" ");
//     Serial.println(target_position);
//     ServoList[ServoID].sMotor.writeMicroseconds(target_position);
// }
