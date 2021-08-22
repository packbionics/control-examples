#include "Encoder_functions.h"
#include "Sensor_Interrupt.h"
#include "motor_functions.h"

#define CALIBRATION_INV_PENDULUM

// defines pins numbers
const int stepPin = 3; 
const int dirPin = 2; 

State state;
double time;

double prevTheta;
double deltaTime;

bool firstLoop = true;
 
void setup() {

  Init_Encoders();
  
  #ifdef CALIBRATION_INV_PENDULUM
    delayMicroseconds(50000);
    setZeroSPI(ENC_0);
  #endif
  
  // Sets the two pins as Outputs
  Serial.begin(9600);
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  state.x = LEFTMOST; // Start from the left
  state.theta = 0.0;
  state.x_dot = 0.0;
  state.theta_dot=0.0;
  delay(3000);
  moveTo(&state, CENTER);
  delay(500);
  time = millis();
}
void loop() {
  if(!firstLoop) {
    deltaTime = (millis() - time) / 1000.0;
    time = millis();
    prevTheta = state.theta;
    state.theta = encInvPend;
    
    state.theta_dot = (state.theta - prevTheta) / deltaTime;
  }
  double acc = swingup(state);
  acheiveAcc(&state, acc);
  firstLoop = false;
}
