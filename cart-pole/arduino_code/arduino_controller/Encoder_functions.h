/* Include the SPI library for the arduino boards */
#include <SPI.h>

#ifndef M_PI
#include <math.h>
#endif

/* Serial rates for UART */
#define BAUDRATE_ENCODER        115200

/* SPI commands */
#define AMT22_NOP       0x00
#define AMT22_RESET     0x60
#define AMT22_ZERO      0x70

/* Define special ascii characters */
#define NEWLINE         0x0A
#define TAB             0x09

/* We will use these define macros so we can write code once compatible with 12 or 14 bit encoders */
#define RES12           12
#define RES14           14

/* SPI pins */
#define ENC_0           2 // encoder 
#define SPI_MOSI        11
#define SPI_MISO        12
#define SPI_SCLK        13

/* Encoder Value Maximum */
#define ENC_MAX         360
#define ENC_MAX_BITS    16384

// global variables
double encInvPend;

/* Function declarations */
void Init_Encoders(void);
uint16_t getPositionSPI(uint8_t encoder, uint8_t resolution);
uint8_t spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine);
void setCSLine (uint8_t encoder, uint8_t csLine);
void setZeroSPI(uint8_t encoder);
void resetAMT22(uint8_t encoder);
double bitToDegrees(int bitCount);
