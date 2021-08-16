#include "Encoder_functions.h"
#include "Sensor_Interrupt.h"
#include "motor_functions.h"

// defines pins numbers
const int stepPin = 3; 
const int dirPin = 2; 
 
void setup() {
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
}
void loop() {

}
