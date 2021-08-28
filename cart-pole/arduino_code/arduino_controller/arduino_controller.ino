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
  state.x = LEFTMOST; // Start from the left
  state.theta = 0.0;
  state.x_dot = 0.0;
  state.theta_dot=0.0;
  delay(1000);
  moveTo(&state, RIGHTMOST);
  moveTo(&state, CENTER);
  delay(500);
  time = millis();
}
void loop() {
  if (i2c_flag) {
    Update_I2C();
  }
  if(!firstLoop) {
    deltaTime = (millis() - time) / 1000.0;
    time = millis();
    prevTheta = state.theta;
    state.theta = encInvPend;
    state.theta_dot = (state.theta - prevTheta) / deltaTime;

    if ( abs(theta_distance(state.theta, M_PI)) < 0.4) {
      acc = upright_lqr(state);
    }
    else {
      acc = swingup(state);
    }
  }
  else {
    acc = 50;
  }
  //Serial.print("THETAS\n");
  //Serial.print(state.theta);
  //Serial.print("\n");
  //Serial.print(state.theta_dot);
  //Serial.print("\n");

  Serial.print("ACCEL\n");
  Serial.print(acc);
  Serial.print("\n");
  Serial.print("DT\n");
  Serial.print(deltaTime);
  Serial.print("\n");

  acheiveAcc(&state, acc);
  //driveMotor(&state, 0.1);
  firstLoop = false;
}
