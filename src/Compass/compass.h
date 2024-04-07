#ifndef __COMPASS__H
#define __COMPASS__H

#include <stdio.h>
#include <hardware/timer.h>
#include "i2c.h"
#include <math.h>


#define PI 3.1415926F
#define COMPASS_CALIBRATION true

#define I2C_ADR_COMPASS 0x0D
#define COMPASS_X 0
#define COMPASS_Y 2
#define COMPASS_Z 4

#define COMPASS_CONTROL_A_REG 0x09
#define COMPASS_CONTROL_B_REG 0x0A
#define COMPASS_PERIOD_REG    0x0B
// #define 


// Read only status register
// bit 1 -- Data Ready Register
// bit 2 -- Overflow flag
// bit 3 -- Ski data for reading
#define COMPASS_STATUS_REG  0x06

// Number of sample to average in output:
// 0b00 - 1
// 0b01 - 2w
// 0b10 - 4
// 0b11 - 8
#define COMPASS_SAMPLING 0b10 << 5

// Measure range [Gauss]:
// 0b000 - ±0.88Ga
// 0b001 - ±1.3Ga (default)
// 0b100 - ±4.0Ga
#define COMPASS_RANGE 0b001 << 5

typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;
} COMPASS_axis_t;

extern COMPASS_axis_t COMPASS_axis;

void Compass_Init();

void Compass_readData();
void Compass_applyCalibration();

uint Compass_getAzimuth();
void Compass_calibration();



#endif