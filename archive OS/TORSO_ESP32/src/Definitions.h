// Define the board ID for this sketch
#define BOARD_ID TORSO_ID
// #define BOARD_ID SHOULDER_ID
 //#define BOARD_ID ARM_ID
//#define BOARD_ID HEAD_ID


// Define board IDs
#define TORSO_ID 1
#define SHOULDER_ID 2
#define ARM_ID 3
#define HEAD_ID 4
#define NUM_SERVOS NUM_TORSO_SERVOS

// Initialize RS485 communication
// HardwareSerial RS485(Serial2); // Use UART2 for RS485
#define RS485 Serial2

#define BOARDTYPE_T4X6 0
#define BOARDTYPE_T4CONTROL 1

// Head Run Modes
#define HEADIDLE 1
#define HEADCONTROL 2
#define HEADAUTO 3

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
#define MAXSBUSOK 20

// Servo States
#define ServoPassThrough 0
#define ServoSequences 1

// define channels
#define AUDIOSWITCH 15
#define AUDIOFOLDER 14

#define NUMAUDIOFOLDERS 12

// number servos


// SERVO MIN MAX AND MID OR NEUTRAL LOCATIONS TORSO
#define  Torso_LR_Right_MIN 1025
#define  Torso_LR_Right_MAX 1925
#define  Torso_LR_Right_MID 1500
#define  Torso_LR_Left_MIN 1100
#define  Torso_LR_Left_MAX 2200
#define  Torso_LR_Left_MID 1550
#define  Torso_FB_Right_MIN 800
#define  Torso_FB_Right_MAX 2200
#define  Torso_FB_Right_MID 1500
#define  Torso_FB_Left_MIN 750
#define  Torso_FB_Left_MAX 2100
#define  Torso_FB_Left_MID 1450
#define  Torso_Hip_Shift_MIN 800
#define  Torso_Hip_Shift_MAX 2500 
#define  Torso_Hip_Shift_MID 1800
#define  Torso_Rotate_MIN 1000
#define  Torso_Rotate_MAX 2000
#define  Torso_Rotate_MID 1500

// SERVO MIN MAX AND MID OR NEUTRAL LOCATIONS SHOULDER
#define  Shoulder_Pitch1_Right_MIN 500
#define  Shoulder_Pitch1_Right_MAX 2500
#define  Shoulder_Pitch1_Right_MID 640
#define  Shoulder_Pitch2_Right_MIN 500
#define  Shoulder_Pitch2_Right_MAX 2500
#define  Shoulder_Pitch2_Right_MID 1600
#define  Shoulder_Pitch1_Left_MIN 500
#define  Shoulder_Pitch1_Left_MAX 2500
#define  Shoulder_Pitch1_Left_MID 2280
#define  Shoulder_Pitch2_Left_MIN 500
#define  Shoulder_Pitch2_Left_MAX 2500
#define  Shoulder_Pitch2_Left_MID 1620
#define  Shoulder_Roll_Right_MIN 500
#define  Shoulder_Roll_Right_MAX 848
#define  Shoulder_Roll_Right_MID 2100
#define  Shoulder_Roll_Left_MIN 700
#define  Shoulder_Roll_Left_MAX 2500
#define  Shoulder_Roll_Left_MID 2500

// SERVO MIN MAX AND MID OR NEUTRAL LOCATIONS ARM
#define  Elbow_Pitch_Right_MIN 800
#define  Elbow_Pitch_Right_MAX 2200
#define  Elbow_Pitch_Right_MID 800
#define  Elbow_Pitch_Left_MIN 800
#define  Elbow_Pitch_Left_MAX 2250
#define  Elbow_Pitch_Left_MID 800
#define  Wrist_Pitch_Right_MIN 1100
#define  Wrist_Pitch_Right_MAX 1500
#define  Wrist_Pitch_Right_MID 1300
#define  Wrist_Pitch_Left_MIN 1300
#define  Wrist_Pitch_Left_MAX 1700
#define  Wrist_Pitch_Left_MID 1500
#define  Wrist_Roll_Right_MIN 700
#define  Wrist_Roll_Right_MAX 2200
#define  Wrist_Roll_Right_MID 1700
#define  Wrist_Roll_Left_MIN 1000
#define  Wrist_Roll_Left_MAX 1500
#define  Wrist_Roll_Left_MID 2500

// SERVO MIN MAX AND MID OR NEUTRAL LOCATIONS HEAD
#define  Head_Rotate_MIN 1000
#define  Head_Rotate_MAX 2000
#define  Head_Rotate_MID 1500
#define  Head_Pitch1_MIN 1200 
#define  Head_Pitch1_MAX 1800
#define  Head_Pitch1_MID 1500
#define  Head_Pitch2_MIN 1200
#define  Head_Pitch2_MAX 1800
#define  Head_Pitch2_MID 1500
#define  Head_Pitch3_MIN 1200
#define  Head_Pitch3_MAX 1800
#define  Head_Pitch3_MID 1500


// -------- Define Colors ----------
// #define RED    0xFF0000
// #define GREEN  0x00FF00
// #define BLUE   0x0000FF
// #define YELLOW 0xFFFF00
// #define PINK   0xFF1088
// #define ORANGE 0xE05800
// #define WHITE  0xFFFFFF

// Less intense...

#define RED 0x160000
#define GREEN 0x001600
#define BLUE 0x000016
#define YELLOW 0x222200 // 101400
#define PINK 0x120009
#define ORANGE 0x100400
#define WHITE 0x222222

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
//#define DomeChannel 0
//#define UtilityArmTopChannel 10
//#define UtilityArmBotChannel 6
//#define IntArmDoorChannel 12
//#define IntArmChannel 4
//#define InterfaceChannel 9
//#define GripArmDoorChannel 7
//#define GripArmChannel 8
//#define GripChannel 5
//#define DataPanelChannel 13

// Radio Channel Definitions = Radio channel - 1
// Set to the Radio channel desired for each function
#define cHEADPITCH 6 // Head pitch stick
#define cHEADROLL 7  // Head Roll stick
#define cHEADYAW 8   // Head Yaw stick
#define cLIFTER 99   // Lifter Channel
#define cVISOR 9     // Visor Channel
#define cTOPRING 4   // Top Ring
#define cMIDRING 3   // Top Ring

#define cBUTTONBANK 88   // Switch to Select button Bank A or B
#define cBUTTONSWITCH 10 // 6 pos buttons
// #define cAUDIOMODE 11   //	Switch to set the Audio Mode ** not used
#define cHEADMODE 12    // Switch to set the Head Mode
#define cHOMEMOTORS 15  // Home Motor Switch
#define cVOICEVOLUME 13 // Audio volume control
#define cWAVVOLUME 14   // Audio volume control

#define DRIVEENABLE 7 // Switch that activates the controls
#define cPitchSW 17   // Switch that sets the pitch setting and enable voice audio

// Servo output pin mapping
// #define DomePin 24            // Servo 1
// #define UtilityArmTopPin 25   // Servo 2
// #define UtilityArmBotPin 26   // Servo 3
// #define IntArmDoorPin 27      // Servo 4
// #define IntArmPin 28          // Servo 5
// #define InterfacePin 29       // Servo 6
// #define GripArmDoorPin 30     // Servo 7
// #define GripArmPin 31         // Servo 8
// #define GripPin 32            // Servo 9
// #define DataPanelPin 33       // Servo 10
// #define DataPanelPin 36          // Servo 11

// Switch Matrix button center points SBUS Values
// 4x4 Switch Matrix
#define ButtonOff 172
#define CButton1 418
#define CButton2 664
#define CButton3 910
#define CButton4 1237
#define CButton5 1482
#define CButton6 1811

#define ButtonDeadband 5
#define LongPressCount 8

// 3po servo output pin mapping



// Define Servo output pin mapping
// Define servo pin mappings for the Torso board
const int TORSO_LR_RIGHT_PIN = 13;
const int TORSO_LR_LEFT_PIN = 12;
const int TORSO_FB_RIGHT_PIN = 14;
const int TORSO_FB_LEFT_PIN = 27;
const int TORSO_HIP_SHIFT_PIN = 25;
const int TORSO_ROTATE_PIN = 26;

// Servo Names for the Torso board
enum TorsoServos {
    Torso_LR_Right,
    Torso_LR_Left,
    Torso_FB_Right,
    Torso_FB_Left,
    Torso_Hip_Shift,
    Torso_Rotate,
    NUM_TORSO_SERVOS
};

// Array to hold the servo pins
const int torsoServoPins[NUM_TORSO_SERVOS] = {
    TORSO_LR_RIGHT_PIN,
    TORSO_LR_LEFT_PIN,
    TORSO_FB_RIGHT_PIN,
    TORSO_FB_LEFT_PIN,
    TORSO_HIP_SHIFT_PIN,
    TORSO_ROTATE_PIN
};

// Define servo pin mappings for the shoulder board
const int SHOULDER_PITCH1_RIGHT_PIN = 25;
const int SHOULDER_PITCH2_RIGHT_PIN = 26;
const int SHOULDER_PITCH1_LEFT_PIN = 12;
const int SHOULDER_PITCH2_LEFT_PIN = 14;
const int SHOULDER_ROLL_RIGHT_PIN = 27;
const int SHOULDER_ROLL_LEFT_PIN = 13;

// Servo Names for the Shoulder board
enum ShoulderServos {
    Shoulder_Pitch1_Right,
    Shoulder_Pitch2_Right,
    Shoulder_Pitch1_Left,
    Shoulder_Pitch2_Left,
    Shoulder_Roll_Right,
    Shoulder_Roll_Left,
    NUM_SHOULDER_SERVOS
};

// Array to hold the servo pins
const int shoulderServoPins[NUM_SHOULDER_SERVOS] = {
    SHOULDER_PITCH1_RIGHT_PIN,
    SHOULDER_PITCH2_RIGHT_PIN,
    SHOULDER_PITCH1_LEFT_PIN,
    SHOULDER_PITCH2_LEFT_PIN,
    SHOULDER_ROLL_RIGHT_PIN,
    SHOULDER_ROLL_LEFT_PIN
};

// Define servo pin mappings for the ARM board
const int ELBOW_PITCH_RIGHT_PIN = 13;
const int ELBOW_PITCH_LEFT_PIN = 27;
const int WRIST_PITCH_RIGHT_PIN = 14;
const int WRIST_PITCH_LEFT_PIN = 25;
const int WRIST_ROLL_RIGHT_PIN = 12;
const int WRIST_ROLL_LEFT_PIN = 26;

// Servo Names for the Arm board
enum ArmServos {
    Elbow_Pitch_Right,
    Elbow_Pitch_Left,
    Wrist_Pitch_Right,
    Wrist_Pitch_Left,
    Wrist_Roll_Right,
    Wrist_Roll_Left,
    NUM_ARM_SERVOS
};

// Array to hold the servo pins
const int armServoPins[NUM_ARM_SERVOS] = {
    ELBOW_PITCH_RIGHT_PIN,
    ELBOW_PITCH_LEFT_PIN,
    WRIST_PITCH_RIGHT_PIN,
    WRIST_PITCH_LEFT_PIN,
    WRIST_ROLL_RIGHT_PIN,
    WRIST_ROLL_LEFT_PIN
};

// Define servo pin mappings for the HEAD board
const int HEAD_ROTATE_PIN = 13;
const int HEAD_PITCH1_PIN = 27;
const int HEAD_PITCH2_PIN = 14;
const int HEAD_PITCH3_PIN = 12;

// Servo Names for the Head board
enum HeadServos {
    Head_Rotate,
    Head_Pitch1,
    Head_Pitch2,
    Head_Pitch3,
    NUM_HEAD_SERVOS
};

// Array to hold the servo pins
const int headServoPins[NUM_HEAD_SERVOS] = {
    HEAD_ROTATE_PIN,
    HEAD_PITCH1_PIN,
    HEAD_PITCH2_PIN,
    HEAD_PITCH3_PIN
};