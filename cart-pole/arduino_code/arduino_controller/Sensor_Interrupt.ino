#include <TimerOne.h>

/**
 * Initializes Interrupt
 */
void Init_Interrupt() {
  Timer1.initialize(50000);
  Timer1.attachInterrupt(Read_Sensors, 50000);
}

/**
 * Function to update sensor global variables
 */
void Read_Sensors() {
  i2c_flag = true;
}

void Update_I2C() {
  encInvPend = bitToDegrees(getPositionSPI(ENC_0, RES14));
  i2c_flag = false;
}
