// includes
#include <HardwareSerial.h>
//#include <SoftwareSerial.h>
#include <ODriveArduino.h>
// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////

// Below are some sample configurations.
// You can comment out the default Teensy one and uncomment the one you wish to use.
// You can of course use something different if you like
// Don't forget to also connect ODrive GND to Arduino GND.

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
 HardwareSerial& odrive_serial = Serial1;

// ODrive object
ODriveArduino odrive(odrive_serial);

// Torque Constant
float kt = 0.0868; // for R80 KV110 Motor

void setup() {
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open
  
  delay(1);

  Serial.println("ODriveArduino");
  Serial.println("Setting to Closed Loop Control...");
  int requested_state;
  requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
  if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
  Serial.println("Requested State Complete");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Setting Torque in CW Direction");
  SetTorque(-.001);
  delay(5000);
  Serial.println("Setting Torque in CCW Direction");
  SetTorque(.001);
  delay(2000);
}

void SetTorque(float t) {
  odrive.SetCurrent(0, t/kt);
}
