#ifndef __VL53L0X__MODULE__H
#define __VL53L0X__MODULE__H

#include <stdio.h>
#include <pico/stdio.h>
#include <hardware/gpio.h>

#include <pico/error.h>

#include "i2c.h"
#include "vl53l0x_api.h"
#include "vl53l0x_rp2040.h"

// Deny address
// 0x30 <- Compass
// 0x76 <- MBE
// 

#define ToF_CENTER_ADDRESS  0x31
#define ToF_LEFT_ADDRESS    0x32
#define ToF_RIGHT_ADDRESS   0x33

#define ToF_DEFAULT_ADDRESS 0x29

#define ToF_CENTER_PIN      12
#define ToF_LEFT_PIN        11
#define ToF_RIGHT_PIN       13



VL53L0X_Error ToF_sensorInit(VL53L0X_Dev_t *tof, uint8_t address, uint32_t pin);
int16_t ToF_measure(VL53L0X_Dev_t *tof);


#endif