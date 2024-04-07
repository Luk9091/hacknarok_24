#ifndef __MY_I2C__H
#define __MY_I2C__H

#include <stdio.h>
#include <pico/stdlib.h>

#include <hardware/gpio.h>
#include <hardware/i2c.h>

// Value in Hz
// Example: 100 * 1000 = 100kHz
#define I2C_FREQUENCY 100 * 1000

#define I2C_CHANNEL i2c1
#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19

void I2C_Init();
int I2C_scan(i2c_inst_t *channel);

// void I2C_send(uint8_t dev, uint8_t byte);
// #define I2C_write(dev, data) i2c_write_blocking(I2C_CHANNEL, dev, data, 1, false)
// uint8_t I2C_read(uint8_t dev);
// #define I2C_read(dev, data) i2c_read_blocking(I2C_CHANNEL, dev, data, 1, false)

void I2C_writeReg(uint8_t dev, uint8_t reg, uint8_t data);
uint8_t I2C_readReg(uint8_t dev, uint8_t reg);


#endif