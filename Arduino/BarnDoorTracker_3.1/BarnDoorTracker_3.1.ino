/*
 * Author: Joshua Mayersky with lots of help from the internet
 * For the main ideas and stepper motor help: https://github.com/partofthething/startracker/blob/master/controller/motor_control.ino
 * For the button class: https://arduino.stackexchange.com/questions/48243/detecting-if-a-push-button-was-pressed-and-released
 * Uses a 28BYJ-48 stepper motor and ULN2003 driver board, Arduino Mega ADK board
 * Changelog
 * Updated 01DEC2021
 * 2.0: Implemented a watchdog/interrupt timer that calls the motor stepper function
 * 2.1: Added the button to start/stop the timer/motor, code reformatting
 * Added the button to turn the fan on/off
 * Locked the button presses until the motor is finished reversing
 * 2.2: Reworked motor stepping function to allow for one function to do both forward and reverse
 * 3.0: Adding the variable timing rate for speeding the motor up in forward motion
 * 3.1 Removing fan pin and changing to manual motor reversing
*/

//---------------Class Definitions---------------//
class Button {
    private:
        bool _state;
        uint8_t _pin;

    public:
        Button(uint8_t pin) : _pin(pin) {}

        void begin() {
            pinMode(_pin, INPUT_PULLUP);
            _state = digitalRead(_pin);
        }

        bool isPushed() {
            bool v = digitalRead(_pin);
            if (v != _state) {
                _state = v;
                if (_state) {
                    return true;
                }
            }
            return false;
        }
};

//---------------Libraries---------------//
#include <arduino-timer.h>

//---------------Variable initializations---------------//
//Constants
const int NUM_PINS = 4;
const int NUM_STEPS = 8;
const double RADS_PER_SEC = 7.292115e-05;
//const double LENGTH_CM = 29.113; // fill in with precise measured value
const double LENGTH_CM = 25;
//const double THETA0 = 0.0241218; // fill in with angle at fully closed position (radians)
const double THETA0 = 0.0559415; 
const double ROTATIONS_PER_CM = 7.8740157;  // 1/4-20 thread
const double DOUBLESTEPS_PER_ROTATION = 2048.0;
//Pins
Button motorButton(22);
Button manualMotorReverseButton(24);
//const int FAN = 44;
int allPins[NUM_PINS] = {30, 32, 34, 36};
//Modes
const int IDLING = 0;
const int FORWARD = 1;
const int STOPPING = 2;
const int REVERSING = 3;
//Variables
int motorMode = IDLING;
//int fanMode = IDLING;
auto timer = timer_create_default();
bool done = true;
bool manualReverse = false;
// from manufacturers datasheet for stepper motor
int STEPPER_SEQUENCE[NUM_STEPS][NUM_PINS] = {{1,0,0,1},
                                             {1,0,0,0},
                                             {1,1,0,0},
                                             {0,1,0,0},
                                             {0,1,1,0},
                                             {0,0,1,0},
                                             {0,0,1,1},
                                             {0,0,0,1}};
int stepDelta = 1;  // single steps while filming for smoothest operation and highest torque; 
int stepNum = 0;
double totalSeconds = 0.0;
unsigned long totalSteps = 0;
double stepInterval_s = 0.001;
int *currentStep;
volatile double next = 1000; // next time to trigger callback
//volatile double next = 0.0125; // next time to trigger callback
volatile double rightNow; // volatile keyword required when things change in callbacks
double startTime;
double endTime;
double nextOffset;


//---------------Setup Loops---------------//
void setup() {
  Serial.begin(115200);
  setup_gpio();
  motorButton.begin();
}


void setup_gpio() {
  for (int i=0;i<NUM_PINS;i++) {
     pinMode(allPins[i], OUTPUT);
  }
  //pinMode(FAN, OUTPUT);
  all_pins_off();
}


//---------------Functions---------------//
//Turn off all the pins on the ULN2003 driver board
void all_pins_off() {
  for (int i=0;i<NUM_PINS;i++) {
    digitalWrite(allPins[i], LOW); 
  }
}


//Steps the motor according to the manufacturers stepper sequence
bool motor(void *arg){
  startTime = micros(); //overflows after ~70 minutes
  nextOffset = 0.0;
  switch (motorMode){
    case IDLING:
      break;
    case FORWARD:
      stepInterval_s = 1.0/(ROTATIONS_PER_CM * ypt(totalSeconds)* 2.0 * DOUBLESTEPS_PER_ROTATION);
      stepNum %= NUM_STEPS;

      totalSeconds += stepInterval_s; // required for tangent error
      currentStep = STEPPER_SEQUENCE[stepNum];
      motorStep(currentStep);
      stepNum += stepDelta;  
      totalSteps += stepDelta;
      
      endTime = micros();
      if (endTime > startTime){
        nextOffset = endTime - startTime;
      }
      next = (stepInterval_s * 1000.0) - (nextOffset / 1000);
      timer.in(next, motor);  //Starts the timer to call the function again  // see you next time!
      return false;
    case STOPPING:
      break;
    case REVERSING:
      stepNum = NUM_STEPS-1;
      for (int i=0; i<totalSteps; i++){
        currentStep = STEPPER_SEQUENCE[stepNum];
        motorStep(currentStep);
        delay(1);
        stepNum -= stepDelta;  // 2 is double-stepping. Faster and shakier.
        if (stepNum<0) {
          stepNum += NUM_STEPS;
        } 
      }
      totalSteps = 0; //Reset the step counter
      all_pins_off(); //Turn all the pins off
      done = true;  //Set the done status to unlock the button
      timer.cancel();
      motorMode = IDLING;;
      return false;
    }
}


//Function to actually do the stepping
void motorStep(int *currentStep){
  for (int i=0; i<NUM_PINS; i++) {
    digitalWrite(allPins[i], currentStep[i]);
  }
}


//bolt insertion rate in cm/s: y'(t) 
double ypt(double ts) {
   // Note, if you run this for ~359 minutes, it goes to infinity!!
   return LENGTH_CM * RADS_PER_SEC/pow(cos(THETA0 + RADS_PER_SEC * ts),2);
}


//---------------Main Loop---------------//
void loop() {
  timer.tick();
  
  //Checking if the button to start/stop/reverse the motor is pushed
  if (motorButton.isPushed() and done == true) {
    motorMode += 1;
    manualReverse = false;
    switch (motorMode){
      case IDLING:
        break;
      case FORWARD:
        Serial.println("Motor Started");
        timer.in(next, motor);
        break;
      case STOPPING:
        Serial.println("Motor Stopped");
        timer.cancel();
        all_pins_off();
        next = 1000;
        break;
      case REVERSING:
        Serial.println("Motor Reversing");
        done = false;
        timer.in(1000, motor);
        break;
    }
  }

  //Checking if the button to manually reverse the motor is pushed
  if (manualMotorReverseButton.isPushed()) {
    //First stop the motor/reset timer/etc.
    Serial.println("Motor Stopped; now Reversing");
    timer.cancel();
    all_pins_off();
    next = 1000;
    motorMode = IDLING;
    
    manualReverse = !manualReverse;
    stepNum = NUM_STEPS-1;
  }

  if (manualReverse){
    //Begin reversing motor while the button is still pushed
    currentStep = STEPPER_SEQUENCE[stepNum];
    motorStep(currentStep);
    delay(1);
    stepNum -= stepDelta;  // 2 is double-stepping. Faster and shakier.
    if (stepNum<0) {
      stepNum += NUM_STEPS;
    }
  }
}
