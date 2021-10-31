#include "Encoder_functions.h"
#include "Sensor_Interrupt.h"
#include "motor_functions.h"

#define CALIBRATION_INV_PENDULUM

// defines pins numbers
const int stepPin = 3; 
const int dirPin = 4; 

State state;
double time;

double prevTheta;
double prevThetaDot;
double deltaTime;
double acc;

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
  state.x = CENTER; // Start from the center
  state.theta = 0.0;
  state.x_dot = 0.0;
  state.theta_dot=0.0;
  prevThetaDot = 0.0;
  delay(1000);
  //moveTo(&state, CENTER);

  time = millis();
}
void loop() {
  Update_I2C();
  if(!firstLoop) {
    deltaTime = (millis() - time) / 1000.0;
    time = millis();
    prevTheta = state.theta;
    prevThetaDot = state.theta_dot;
    state.theta = encInvPend;
    state.theta_dot = theta_distance(state.theta, prevTheta) / deltaTime;
    state.theta_dot = (prevThetaDot + state.theta_dot) / 2;

    if ( abs(theta_distance(state.theta, M_PI)) < 0.3) {
      acc = upright_lqr(state);
    }
    else {
      acc = swingup(state);
    }
  }
  else {
    acc = 50;
  }
  //Serial.print("THETA\n");
  //Serial.println(state.theta);
  //Serial.print("\n");
  //Serial.print("THETADOT\n");
  Serial.println(state.theta_dot);
  //Serial.print("\n");
  //Serial.print("THETADIFF\n");
  //Serial.println(theta_distance(state.theta, prevTheta));
  //Serial.println(state.theta);
  //Serial.println(prevTheta);
  //Serial.print("\n");
  //Serial.print("X\n");
  //Serial.print(state.x);
  //Serial.print("\n");
  //Serial.print("XDOT\n");
  //Serial.print(state.x_dot);
  
  //Serial.print("\n");

  //Serial.print("ACCEL\n");
  //Serial.println(acc);
  //Serial.print("\n");
  //Serial.print("DT\n");
  //Serial.print("\t");
  //Serial.println(deltaTime);
  //Serial.print("\n");

  acheiveAcc(&state, acc);
  firstLoop = false;
}
