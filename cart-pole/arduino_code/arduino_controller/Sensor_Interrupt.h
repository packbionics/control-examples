volatile bool i2c_flag = false;

// Initializes the interrupt for timer 1
void Init_Interrupt(void);
// Reads and updates all sensor variables
void Read_Sensors(void);
void Update_I2C(void);
