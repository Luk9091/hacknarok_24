#include "bme280.h"

static int32_t t_fine;

void bme280_init() {
    // 500ms sampling time, x16 filter
    const uint8_t reg_config_val = ((0x04 << 5) | (0x05 << 2)) & 0xFC;
    uint8_t buf[2];
    buf[0] = REG_CONFIG;
    buf[1] = reg_config_val;
    i2c_write_blocking(I2C_CHANNEL, BME280_ADDR, buf, 2, false); // send register number followed by its corresponding value

    // osrs_t x1, osrs_p x4, normal mode operation
    const uint8_t reg_ctrl_meas_val = (0x01 << 5) | (0x03 << 2) | (0x03);
    buf[0] = REG_CTRL_MEAS;
    buf[1] = reg_ctrl_meas_val;
    i2c_write_blocking(I2C_CHANNEL, BME280_ADDR, buf, 2, false); // send register number followed by its corresponding value
}

void bme280_reset() {
    // reset the device with the power-on-reset procedure
    uint8_t buf[2] = { REG_RESET, 0xB6 };
    i2c_write_blocking(I2C_CHANNEL, BME280_ADDR, buf, 2, false); // send register number followed by its corresponding value
}

// collects values:
// pressure in Pa: “96386” equals 96386 Pa = 963.86 hPa
// temp in DegC: “5123” equals 51.23 DegC,
// humidity in %RH as unsigned 32 bit integer in Q22.10 format: “47445” equals 47445/1024 = 46.333 %RH
void bme280_read(uint32_t* pressure, int32_t* temp, uint32_t* humidity, bme280_calib_params* calib_params){
    int32_t adc_P, adc_T, adc_H;
    bme280_read_raw(&adc_P, &adc_T, &adc_H);
    *pressure = bme280_compensate_pressure(adc_P, calib_params); // / 100;
    *temp = bme280_compensate_temperature(adc_T, calib_params);  // / 100;
    *humidity = bme280_compensate_humidity(adc_H, calib_params); // / 1024;
}

void bme280_read_raw(int32_t* pressure, int32_t* temp, int32_t* humidity){

    uint8_t buf[8];
    uint8_t reg_start = REG_PARAMS_START;
    i2c_write_blocking(I2C_CHANNEL, BME280_ADDR, &reg_start, 1, true); // true to keep master control of bus
    i2c_read_blocking(I2C_CHANNEL, BME280_ADDR, buf, 8, false); // read in one go as register addresses auto-increment

    // temperature and pressure -> unsigned 20-bit
    // humidity -> unsigned 16-bit
    *pressure = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    *temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
    *humidity = (buf[6] << 8) | buf[7];
}

// sets values of calibrate params in given bme280_calib_params structure 
void bme280_get_calib_params(bme280_calib_params* calib_params) {

    uint8_t buf[24] = {0};
    uint8_t reg = REG_DIG_START;
    i2c_write_blocking(I2C_CHANNEL, BME280_ADDR, &reg, 1, true); // true to keep master control of bus
    i2c_read_blocking(I2C_CHANNEL, BME280_ADDR, buf, 24, false); // read in one go as register addresses auto-increment 

    calib_params->dig_t1 = (uint16_t)(buf[1] << 8) | buf[0];
    calib_params->dig_t2 = (int16_t)(buf[3] << 8) | buf[2];
    calib_params->dig_t3 = (int16_t)(buf[5] << 8) | buf[4];

    calib_params->dig_p1 = (uint16_t)(buf[7] << 8) | buf[6];
    calib_params->dig_p2 = (int16_t)(buf[9] << 8) | buf[8];
    calib_params->dig_p3 = (int16_t)(buf[11] << 8) | buf[10];
    calib_params->dig_p4 = (int16_t)(buf[13] << 8) | buf[12];
    calib_params->dig_p5 = (int16_t)(buf[15] << 8) | buf[14];
    calib_params->dig_p6 = (int16_t)(buf[17] << 8) | buf[16];
    calib_params->dig_p7 = (int16_t)(buf[19] << 8) | buf[18];
    calib_params->dig_p8 = (int16_t)(buf[21] << 8) | buf[20];
    calib_params->dig_p9 = (int16_t)(buf[23] << 8) | buf[22];

    uint8_t buf_hum[8] = {0};
    uint8_t reg_hum = REG_HUM_DIG_START;
    i2c_write_blocking(I2C_CHANNEL, BME280_ADDR, &reg_hum, 1, true); // true to keep master control of bus
    i2c_read_blocking(I2C_CHANNEL, BME280_ADDR, buf_hum, 8, false); // read in one go as register addresses auto-increment

    calib_params->dig_h1 = (uint8_t)buf_hum[0];
    calib_params->dig_h2 = (int16_t)(buf_hum[2] << 8) | buf_hum[1];
    calib_params->dig_h3 = (uint8_t)buf_hum[3];
    calib_params->dig_h4 = (int16_t)(buf_hum[4] << 4) | (buf_hum[5] & 0xF);
    calib_params->dig_h5 = (int16_t)(buf_hum[6] << 4) | (buf_hum[5] >> 4);
    calib_params->dig_h6 = (int8_t)buf_hum[7];

}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t bme280_compensate_temperature(int32_t adc_T, bme280_calib_params* calib_params){
    int32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((int32_t)calib_params->dig_t1<<1))) * ((int32_t)calib_params->dig_t2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)calib_params->dig_t1)) * ((adc_T>>4) - ((int32_t)calib_params->dig_t1))) >> 12) *
    ((int32_t)calib_params->dig_t3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
uint32_t bme280_compensate_pressure(int32_t adc_P, bme280_calib_params* calib_params){
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)calib_params->dig_p6);
    var2 = var2 + ((var1*((int32_t)calib_params->dig_p5))<<1);
    var2 = (var2>>2)+(((int32_t)calib_params->dig_p4)<<16);
    var1 = (((calib_params->dig_p3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)calib_params->dig_p2) * var1)>>1))>>18;
    var1 =((((32768+var1))*((int32_t)calib_params->dig_p1))>>15);
    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
        p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
    if (p < 0x80000000) {
        p = (p << 1) / ((uint32_t)var1);
    }
    else {
        p = (p / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)calib_params->dig_p9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
    var2 = (((int32_t)(p>>2)) * ((int32_t)calib_params->dig_p8))>>13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + calib_params->dig_p7) >> 4));
    return p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
uint32_t bme280_compensate_humidity(int32_t adc_H, bme280_calib_params* calib_params){
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib_params->dig_h4) << 20) - (((int32_t)calib_params->dig_h5) * v_x1_u32r)) +
    ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)calib_params->dig_h6)) >> 10) * (((v_x1_u32r *((int32_t)calib_params->dig_h3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
    ((int32_t)calib_params->dig_h2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib_params->dig_h1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r>>12);
}
