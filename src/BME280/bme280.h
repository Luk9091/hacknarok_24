#ifndef __BME280__H
#define __BME280__H

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "i2c.h"

#define BME280_ADDR 0x76
#define REG_PARAMS_START 0xF7
#define REG_DIG_START 0x88
#define REG_HUM_DIG_START 0xA1
#define REG_CONFIG 0xF5
#define REG_CTRL_MEAS 0xF4
#define REG_RESET 0xE0

typedef struct {
    // temperature params
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;

    // pressure params
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;

    // humidity params
    uint8_t dig_h1;
    int16_t dig_h2;
    uint8_t dig_h3;
    int16_t dig_h4;
    int16_t dig_h5;
    int8_t dig_h6;

} bme280_calib_params;

void bme280_init();

void bme280_reset();

void bme280_read(uint32_t *pressure, int32_t *temp, uint32_t *humidity, bme280_calib_params *calib_params);

void bme280_read_raw(int32_t* pressure, int32_t* temp, int32_t* humidity);

void bme280_get_calib_params(bme280_calib_params* calib_params);

int32_t bme280_compensate_temperature(int32_t adc_T, bme280_calib_params *calib_params);

uint32_t bme280_compensate_pressure(int32_t adc_P, bme280_calib_params *calib_params);

uint32_t bme280_compensate_humidity(int32_t adc_H, bme280_calib_params *calib_params);

#endif // __BME280__H