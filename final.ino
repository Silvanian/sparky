#include "NEET_RF24.h"

#define CHANNEL 55
#define JOYSTICK_DEADZONE 5
#define NRF_CE 45
#define NRF_CSN 47

ControlInput input_vals; // holds values from controller

void setup() {
    radio.begin();
}

void loop() {
    if (radio.rxUpdate()){
        input_vals = radio.rxGetInput();
        // Do things with the input
    } else {
        // Halt the robot because we've lost connection
    }
}