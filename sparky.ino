#include "NEET_RF24.h"
#include <Adafruit_MotorShield.h>
#include <DRV8835MotorShield.h>
#include <Servo.h>
#include <Wire.h>

// RADIO //
#define CHANNEL 55
#define JOYSTICK_DEADZONE 5
#define NRF_CE 38
#define NRF_CSN 39

// SERVOS //
#define LEFT_WING_PIN 25
#define RGHT_WING_PIN 27
#define LEFT_MOTR_PIN 29
#define MIDD_MOTR_PIN 31
#define RGHT_MOTR_PIN 33
#define LEFT_WMOT_PIN 23
#define RGHT_WMOT_PIN 35


// DC MOTORS //
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* motor_scoopy = AFMS.getMotor(3);
Adafruit_DCMotor* motor_plat   = AFMS.getMotor(4);
Adafruit_DCMotor* motor_left = AFMS.getMotor(1);
Adafruit_DCMotor* motor_right = AFMS.getMotor(2);
DRV8835MotorShield motors = DRV8835MotorShield(48,46,44,42); // M1->left, M2->right

// RADIO //
ControlInput input; // holds values from controller
NEET_RF24 radio(NRF_CE, NRF_CSN, CHANNEL);

// SERVOS //
// deployed
const int left_d = 129;
const int midd_d = 155;
const int rght_d = 140;
const int wlft_d = 150;
const int wrgt_d = 180;
const int lftw_d = 134;
const int rgtw_d = 34;

// folded
const int left_f = 87;
const int midd_f = 109;
const int rght_f = 86;
const int wlft_f = 75;
const int wrgt_f = 90;
const int lftw_f = 20;
const int rgtw_f = 142;

// motors
Servo left_wing;
Servo rght_wing;
Servo wing_left;
Servo left;
Servo midd;
Servo rght;
Servo wing_rght;


void deployMCM() {
    // make sure horns are folded
    wing_left.write(wlft_f);
    left.write(left_f);
    midd.write(midd_f);
    rght.write(rght_f);
    wing_rght.write(wrgt_f);

    // deploy wings
    left_wing.write(lftw_d);
    rght_wing.write(rgtw_d);
}

void drive(int left, int right) {
    drive(motor_left, -left);
    drive(motor_right, right);
}

void drive(Adafruit_DCMotor* motor, int speed) {
    motor->setSpeed(abs(speed));
    if (speed < 0) motor->run(BACKWARD);
    else if (speed > 0) motor->run(FORWARD);
    else motor->fullOff();
}

void pushButtons(int buttons[5]) {
    left_wing.attach(LEFT_WING_PIN);
    rght_wing.attach(RGHT_WING_PIN);
    wing_left.attach(LEFT_WMOT_PIN);
    left.attach(LEFT_MOTR_PIN);
    midd.attach(MIDD_MOTR_PIN);
    rght.attach(RGHT_MOTR_PIN);
    wing_rght.attach(RGHT_WMOT_PIN);
    // for each button in the sequence, deploy the corresponding servo
    //    and fold it back up
    for (int button = 0; button < 5; button++)
    {
        Servo servo;
        int deployed;
        int folded;
        if (buttons[button] == 1)
        {
            servo = wing_left;
            deployed = wlft_d;
            folded = wlft_f;
        }
        else if (buttons[button] == 2)
        {
            servo = left;
            deployed = left_d;
            folded = left_f;
        }
        else if (buttons[button] == 3)
        {
            servo = midd;
            deployed = midd_d;
            folded = midd_f;
        }
        else if (buttons[button] == 4)
        {
            servo = rght;
            deployed = rght_d;
            folded = rght_f;
        }
        else if (buttons[button] == 5)
        {
            servo = wing_rght;
            deployed = wrgt_d;
            folded = wrgt_f;
        }

        servo.write(deployed);
        delay(500);
        servo.write(folded);
        delay(300);
  }
  left_wing.detach();
  rght_wing.detach();
  wing_left.detach();
  left.detach();
  midd.detach();
  rght.detach();
  wing_rght.detach();
}

void retractMCM() {
    // fold wings back up
    left_wing.write(lftw_f);
    rght_wing.write(rgtw_f);

    // fold horns back up
    wing_left.write(wlft_f);
    left.write(left_f);
    midd.write(midd_f);
    rght.write(rght_f);
    wing_rght.write(wrgt_f);
}


void setup() {


    Serial.begin(9600);

    AFMS.begin();
    if (!radio.begin()){
        Serial.println("Radio not started");
    } else {
        Serial.println("Radio started");
    }
}

void loop() {
    if (radio.rxUpdate()){
        input = radio.rxGetInput();
        int left, right = 0;
       
        // Don't process unless the joysticks are more than 5 away from center
        if (abs(input.j1PotY) > JOYSTICK_DEADZONE){
            left = input.j1PotY;
        }
        if (abs(input.j1PotX) > JOYSTICK_DEADZONE){
            right = input.j1PotX;
        }
        // Arcade drive
        drive(left + right * 0.4, left - right * 0.4);

        if (abs(input.j2PotX) > JOYSTICK_DEADZONE){
            drive(motor_plat, input.j2PotX);
        }
        if (abs(input.j2PotY) > JOYSTICK_DEADZONE){
            drive(motor_scoopy, input.j2PotY);
        }

        if (input.button1) {
            int buttons[] = {1, 2, 3, 4, 5};
            pushButtons(buttons);
        }

/*         if (input.button3) drive(motor_spinny, 255);
        else if (input.button4) drive(motor_spinny, -255);
        else drive(motor_spinny,0); */

        if (!input.tSwitch) retractMCM();
        else if (input.tSwitch) deployMCM();


    } else {
        // Halt the robot because we've lost connection
        drive(0,0);
        drive(motor_plat, 0);
        drive(motor_scoopy, 0);
        //drive(motor_spinny, 0);
    }
}