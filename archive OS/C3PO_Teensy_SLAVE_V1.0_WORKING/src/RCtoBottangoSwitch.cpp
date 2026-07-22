#include <RCtoBottangoSwitch.h>
#include "Arduino.h"
// #include "BottangoCore.h"
// #include "GeneratedCommandStreams.h"
#include "definitions.h"
#include "Events.hpp"

// Define the pin for your mode switch button
const int modeSwitchPin = cRCBOTTANGOMODESW; // Adjust this pin as needed


// Control Mode Variable
enum ControlMode {
    RC_MODE,
    ANIMATION_MODE
};

ControlMode currentMode = RC_MODE;

// Variables to track button state
bool lastButtonState = HIGH; // Assume pull-up resistor
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // milliseconds

void loopSwitches() {
    // Debouncing for mode switch
    int buttonState = digitalRead(modeSwitchPin);
    if (buttonState != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        // Switch between modes on button press
        if (buttonState == LOW) {
            toggleControlMode();
        }
    }
    lastButtonState = buttonState;

    // Handle each mode
    switch (currentMode) {
        case RC_MODE:
            handleRCControl();
            break;

        // case ANIMATION_MODE:
        //     handleAnimationControl();
        //     break;
    }
}

void toggleControlMode() {
    // if (currentMode == RC_MODE) {
    //     currentMode = ANIMATION_MODE;
    //     // Serial.println("Switched to Animation Mode");
    //     // Optionally stop any playing animation when switching back to RC mode
    //     BottangoCore::commandStreamProvider.stop();
    // } else {
    //     currentMode = RC_MODE;
    //     // Serial.println("Switched to RC Mode");
    // }
}

void handleRCControl() {
    // Your existing RC control code here
    // This will run when in RC_MODE
}
  void CheckBottangoMode() {
    // Check the value of the cRCBOTTANGOMODESW switch
    switch (ChannelData(cRCBOTTANGOMODESW)) {
        // RC Mode
        case (SBUSMIN - ButtonDeadband)...(SBUSMIN + ButtonDeadband):
            if (currentMode != RC_MODE) {
                currentMode = RC_MODE;
                // Serial.println("Control Mode: RC Mode");
            }
            break;

        // Bottango Animation Mode
        case (SBUSMAX - ButtonDeadband)...(SBUSMAX + ButtonDeadband):
            if (currentMode != ANIMATION_MODE) {
                currentMode = ANIMATION_MODE;
                
                // Serial.println("Control Mode: Bottango Animation Mode");
            }
            break;

        default:
            break;
    }

}

void triggerAnimation(byte streamID, bool loop) {
    // if (!BottangoCore::commandStreamProvider.streamIsInProgress()) {
    //     BottangoCore::commandStreamProvider.startCommandStream(streamID, loop);
        
        

        // Serial.print("Started animation ID: ");
        // Serial.println(streamID);

    }
