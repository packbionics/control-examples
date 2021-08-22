#include "Encoder_functions.h"
#include "Sensor_Interrupt.h"
#include "motor_functions.h"

// defines pins numbers
const int stepPin = 3; 
const int dirPin = 2; 

State state;
 
void setup() {
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
  
}
void loop() {
  double acc = swingup(state);
  acheiveAcc(&state, acc);
}
