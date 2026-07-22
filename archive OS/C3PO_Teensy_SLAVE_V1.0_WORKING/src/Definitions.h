#define BOARDTYPE_T4X6 0
#define BOARDTYPE_T4CONTROL 1

// Head Run Modes
#define HEADIDLE 1
#define HEADCONTROL 2
#define HEADAUTO 3

#define SHOULDERSWITCHLEFT 1
#define SHOULDERSWITCHRIGHT 2
#define SHOULDERSWITCHBOTH 3

// Audio Defines
// Audio States
#define AUDIOVOICEOFF 0
#define AUDIOVOICEON 1
#define AUDIOMANUAL 3
#define AUDIOAUTO 4

// Voice Pitch Mode
#define PITCHOFF 0
#define PITCHHIGH 1
#define PITCHLOW 2

// SBUS defines
#define SBUSCENTER 992
#define SBUSDEADBAND 10
#define SBUSMIN 172
#define SBUSMAX 1811
#define SBUSNEUTRAL 992
#define MAXSBUSOK 20

// Servo States
#define ServoPassThrough 0
#define ServoSequences 1

// define channels
#define AUDIOSWITCH 15
#define AUDIOFOLDER 14

// #define VERBOSE  // Comment this out to disable verbose output

// -------- Define Colors ----------
// #define RED    0xFF0000
// #define GREEN  0x00FF00
// #define BLUE   0x0000FF
// #define YELLOW 0xFFFF00
// #define PINK   0xFF1088
// #define ORANGE 0xE05800
// #define WHITE  0xFFFFFF

// Less intense...

// #define RED 0x160000
// #define GREEN 0x001600
// #define BLUE 0x000016
// #define YELLOW 0x222200 // 101400
// #define PINK 0x120009
// #define ORANGE 0x100400
// #define WHITE 0x222222

#define SD_CS BUILTIN_SDCARD // CS for SD card, can use any pin

#define BUFFPIXEL 2

// Analog Input Definitions

// Servo Time out.  Time to wait to detach servos.
#define SERVOTIMEOUT 35

// Servo reference number
// Map of name to reference in array
// #define DOME 0

// Passthrough Servo Channel mapping
// Not used...
// #define DomeChannel 0
// #define UtilityArmTopChannel 10
// #define UtilityArmBotChannel 6
// #define IntArmDoorChannel 12
// #define IntArmChannel 4
// #define InterfaceChannel 9
// #define GripArmDoorChannel 7
// #define GripArmChannel 8
// #define GripChannel 5
// #define DataPanelChannel 13


// defines servo banks when using multiple teensy boards
// #define TEENSY_MAIN 1
#define TEENSY_SLAVE1 2
// #define TEENSY_SLAVE2 3

// Radio Channel Definitions = Radio channel - 1
// Set to the Radio channel desired for each function
#define cHEADPITCH 3      // Head pitch stick
#define cHEADROLL 4       // Head Roll stick
#define cHEADYAW 5        // Head Yaw stick
#define cSHFL 6           // Shoulder Flexion channel
#define cSHAB 7           // Shoulder Abduction channel
#define cSHER 8           // Shoulder External Rotate channel
#define cTORSOLR 9        // Torso lean left right
#define cTORSOROT 10      // Torso Rotate
#define cTORSOHIPSHIFT 11 // Torso Hip Shift
#define cTORSOFB 12       // Torso Front/Back Channel

#define cVOICEVOLUME 13 // Audio volume control
#define cWAVVOLUME 14   // Audio volume control

#define cRCBOTTANGOMODESW 15 // Switch to SelectRC mode or bottango animation mode for servo function
// #define cHOMEMOTORS 15  // Home Motor Switch
#define cSHOULDERMODESWITCH 16 // SWITCHES POT CONTROL TO BOTH, LEFT OR RIGHT SHOULDERS
#define cPITCHSW 17            // Switch that sets the pitch setting and enable voice audio
#define DRIVEENABLE 18         // Switch that activates the controls
#define cCONTROLMODE 20        // Switch to set the Head Mode
#define cTRIMBACKSWITCH 21     // TRIM SWITCH BANK 6 POS T2, T3 U & D, SWI AND SWJ
#define cTRIMSWITCH 22         // TRIM SWITCH BANK 8 POS T1, T4, T5, T6 L & R
#define cBUTTONSWITCH 23       // 8 pos buttons
#define cHOMEMOTORS 19      //Toggle switch for servo attach

// Switch Matrix button center points SBUS Values
// 4x4 Switch Matrix
#define ButtonOff 172
#define CButton1 377
#define CButton2 582
#define CButton3 787
#define CButton4 992
#define CButton5 1196
#define CButton6 1401
#define CButton7 1606 // pot button 1
#define CButton8 1811 // pot button 2

// Switch Matrix TRIM center points SBUS Values
// 4x4 Switch Matrix
#define trimcenter 172
#define CTrim6L 377
#define CTrim6R 582
#define CTrim5L 787
#define CTrim5R 992
#define CTrim1L 1196
#define CTrim1R 1401
#define CTrim4L 1606
#define CTrim4R 1811

// Switch Matrix TRIM 1&2 and back buttons center points SBUS Values
// 4x4 Switch Matrix
#define trimcenter2 172
#define CTrim2U 418
#define CTrim2D 664
#define CTrim3U 910
#define CTrim3D 1155
#define CSWI 1401
#define CTSWJ 1811

#define ButtonDeadband 5
#define LongPressCount 8

// Servo List Array Reference
// #define iTorsoLRRight 0
// #define iTorsoLRLeft 1
// #define iTorsoFBRight 2
// #define iTorsoFBLeft 3
// #define iTorsoHipShift 4
// #define iTorsoRotate 5

#define iShoulderPitch1Right 0
#define iShoulderPitch2Right 1
#define iShoulderPitch1Left 2
#define iShoulderPitch2Left 3
#define iShoulderRollRight 4
#define iShoulderRollLeft 5

#define iElbowPitchRight 6
#define iElbowPitchLeft 7
#define iWristPitchRight 8
#define iWristPitchLeft 9
#define iWristRollRight 10
#define iWristRollLeft 11

// #define iHeadRotate 14
// #define iHeadPitch1 15
// #define iHeadPitch2 16
// #define iHeadPitch3 17

// SERVO MIN MAX AND MID OR NEUTRAL LOCATIONS TORSO
// #define Torso_LR_Right_MIN 1025
// #define Torso_LR_Right_MAX 1925
// #define Torso_LR_Right_MID 1500
// #define Torso_LR_Left_MIN 1100
// #define Torso_LR_Left_MAX 2200
// #define Torso_LR_Left_MID 1550
// #define Torso_FB_Right_MIN 800
// #define Torso_FB_Right_MAX 2200
// #define Torso_FB_Right_MID 1500
// #define Torso_FB_Left_MIN 750
// #define Torso_FB_Left_MAX 2100
// #define Torso_FB_Left_MID 1450
// #define Torso_Hip_Shift_MIN 800
// #define Torso_Hip_Shift_MAX 2500
// #define Torso_Hip_Shift_MID 1800
// #define Torso_Rotate_MIN 1000
// #define Torso_Rotate_MAX 2000
// #define Torso_Rotate_MID 1500

// SERVO MIN MAX AND MID OR NEUTRAL LOCATIONS SHOULDER
#define Shoulder_Pitch1_Right_MIN 640
#define Shoulder_Pitch1_Right_MAX 2200
#define Shoulder_Pitch1_Right_MID 640
#define Shoulder_Pitch2_Right_MIN 1600
#define Shoulder_Pitch2_Right_MAX 2400
#define Shoulder_Pitch2_Right_MID 1600
#define Shoulder_Pitch1_Left_MIN 2280
#define Shoulder_Pitch1_Left_MAX 640
#define Shoulder_Pitch1_Left_MID 2200
#define Shoulder_Pitch2_Left_MIN 2220
#define Shoulder_Pitch2_Left_MAX 640
#define Shoulder_Pitch2_Left_MID 2220
#define Shoulder_Roll_Right_MIN 1200
#define Shoulder_Roll_Right_MAX 2500
#define Shoulder_Roll_Right_MID 1800
#define Shoulder_Roll_Left_MIN 1800
#define Shoulder_Roll_Left_MAX 1000
#define Shoulder_Roll_Left_MID 1500

// SERVO MIN MAX AND MID OR NEUTRAL LOCATIONS ARM
#define Elbow_Pitch_Right_MIN 1100
#define Elbow_Pitch_Right_MAX 2200
#define Elbow_Pitch_Right_MID 1700
#define Elbow_Pitch_Left_MIN 2250
#define Elbow_Pitch_Left_MAX 800
#define Elbow_Pitch_Left_MID 1650
#define Wrist_Pitch_Right_MIN 1100
#define Wrist_Pitch_Right_MAX 1500
#define Wrist_Pitch_Right_MID 1300
#define Wrist_Pitch_Left_MIN 1300
#define Wrist_Pitch_Left_MAX 1700
#define Wrist_Pitch_Left_MID 1500
#define Wrist_Roll_Right_MIN 700
#define Wrist_Roll_Right_MAX 2200
#define Wrist_Roll_Right_MID 1700
#define Wrist_Roll_Left_MIN 1000
#define Wrist_Roll_Left_MAX 1500
#define Wrist_Roll_Left_MID 2500

// SERVO MIN MAX AND MID OR NEUTRAL LOCATIONS HEAD
// #define Head_Rotate_MIN 1000
// #define Head_Rotate_MAX 2000
// #define Head_Rotate_MID 1500
// #define Head_Pitch1_MIN 1200
// #define Head_Pitch1_MAX 1800
// #define Head_Pitch1_MID 1500
// #define Head_Pitch2_MIN 1200
// #define Head_Pitch2_MAX 1800
// #define Head_Pitch2_MID 1500
// #define Head_Pitch3_MIN 1200
// #define Head_Pitch3_MAX 1800
// #define Head_Pitch3_MID 1500

// Servo output pin mapping
// #define pinTorsoLRRight 28  // Servo 5
// #define pinTorsoLRLeft 29   // Servo 6
// #define pinTorsoFBRight 30  // Servo 7
// #define pinTorsoFBLeft 31   // Servo 8
// #define pinTorsoHipShift 33 // Servo 10
// #define pinTorsoRotate 32   // Servo 9

#define pinShoulderPitch1Right 37 // LEDD
#define pinShoulderPitch2Right 3 // D3
#define pinShoulderPitch1Left 36 // S11
#define pinShoulderPitch2Left 39 // LEDB
#define pinShoulderRollRight 38 // LEDC
#define pinShoulderRollLeft 40 // LEDA

#define pinElbowPitchRight 1 // D1
#define pinElbowPitchLeft 2 // D2
#define pinWristPitchRight 24 // Servo 1
#define pinWristPitchLeft 25 // Servo 2
#define pinWristRollRight 26 // Servo 3
#define pinWristRollLeft 27 // Servo 4

// #define pinHeadRotate 24 // Servo 1
// #define pinHeadPitch1 25 // Servo 2
// #define pinHeadPitch2 26 // Servo 3
// #define pinHeadPitch3 27 // Servo 4


