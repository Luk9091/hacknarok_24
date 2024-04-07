#include "compass.h"

typedef struct {
    float x;
    float y;
    float z;
} COMPASS_calibrationData_t;


COMPASS_axis_t COMPASS_axis;
COMPASS_calibrationData_t COMPASS_calibrationOffset;
COMPASS_calibrationData_t COMPASS_calibrationScales;

void Compass_clearCalibration(){
    COMPASS_calibrationOffset.x = 0.f;
    COMPASS_calibrationOffset.y = 0.f;
    COMPASS_calibrationOffset.z = 0.f;
    
    COMPASS_calibrationScales.x = 1.f;
    COMPASS_calibrationScales.y = 1.f;
    COMPASS_calibrationScales.z = 1.f;
}

void Compass_setCalibrationOffset(float x, float y, float z){
    COMPASS_calibrationOffset.x = x;
    COMPASS_calibrationOffset.y = y;
    COMPASS_calibrationOffset.z = z;
}

void Compass_setCalibrationScales(float x, float y, float z){
    COMPASS_calibrationScales.x = x;
    COMPASS_calibrationScales.y = y;
    COMPASS_calibrationScales.z = z;
}

void Compass_setCalibration(COMPASS_axis_t max, COMPASS_axis_t min){
    Compass_setCalibrationOffset(
        (min.x + max.x)/2,
        (min.y + max.y)/2,
        (min.z + max.z)/2
    );

    float x_ave_delta = (max.x - min.x)/2;
    float y_ave_delta = (max.y - min.y)/2;
    float z_ave_delta = (max.z - min.z)/2;

    float ave_delta = (x_ave_delta + y_ave_delta + z_ave_delta)/3;

    Compass_setCalibrationScales(
        ave_delta / x_ave_delta,
        ave_delta / y_ave_delta,
        ave_delta / z_ave_delta
    );
}

void Compass_Init(){
    I2C_writeReg(I2C_ADR_COMPASS, COMPASS_PERIOD_REG, 0x01);
    I2C_writeReg(I2C_ADR_COMPASS, COMPASS_CONTROL_A_REG, 0x01 | 0x0C | 0x10 | 0x00);
    I2C_writeReg(I2C_ADR_COMPASS, COMPASS_CONTROL_B_REG, 1 << 6);

    COMPASS_axis_t min = {
        .x = -500,
        .y = -4500,
        .z = 1800
    };
    COMPASS_axis_t max = {
        .x = 1850,
        .y = -1800,
        .z = 4500
    };

    Compass_setCalibration(max, min);
}

void Compass_applyCalibration(){
    COMPASS_axis.x = (COMPASS_axis.x - (int16_t)COMPASS_calibrationOffset.x) * COMPASS_calibrationScales.x;
    COMPASS_axis.y = (COMPASS_axis.y - (int16_t)COMPASS_calibrationOffset.y) * COMPASS_calibrationScales.y;
    COMPASS_axis.z = (COMPASS_axis.z - (int16_t)COMPASS_calibrationOffset.z) * COMPASS_calibrationScales.z;
}

void Compass_readData(){
    uint8_t data[2 * 3];
    i2c_write_blocking(I2C_CHANNEL, I2C_ADR_COMPASS, COMPASS_X, 1, true);
    i2c_read_blocking(I2C_CHANNEL, I2C_ADR_COMPASS, data, 6, false);
    // printf("x0: %i, x1: %i\n", data[0], data[1]);
    // printf("y0: %i, y1: %i\n", data[2], data[3]);
    // printf("z0: %i, z1: %i\n", data[4], data[5]);

    COMPASS_axis.x = (int16_t)(data[1] << 8) | data[0];
    COMPASS_axis.y = (int16_t)(data[3] << 8) | data[2];
    COMPASS_axis.z = (int16_t)(data[5] << 8) | data[4];
}

uint Compass_getAzimuth(){
    int16_t a;
    
    a = (atan2(COMPASS_axis.y, COMPASS_axis.x) * 180/PI);
    return a < 0 ? 360 + a : a;
}



void Compass_calibration(){
    Compass_clearCalibration();
    COMPASS_axis_t calibrationDataMax = {
        .x = -32767,
        .y = -32767,
        .z = -32767
    };
    COMPASS_axis_t calibrationDataMin = {
        .x = 32768,
        .y = 32768,
        .z = 32768
    };

    printf("Start calibration\n");
    uint64_t time = 10e6;
    uint64_t start = time_us_64();
    while (time_us_64() - start < time){
        Compass_readData();

        if (COMPASS_axis.x < calibrationDataMin.x) calibrationDataMin.x = COMPASS_axis.x;
        if (COMPASS_axis.x > calibrationDataMax.x) calibrationDataMax.x = COMPASS_axis.x;

        if (COMPASS_axis.y < calibrationDataMin.y) calibrationDataMin.y = COMPASS_axis.y;
        if (COMPASS_axis.y > calibrationDataMax.y) calibrationDataMax.y = COMPASS_axis.y;

        if (COMPASS_axis.z < calibrationDataMin.z) calibrationDataMin.z = COMPASS_axis.z;
        if (COMPASS_axis.z > calibrationDataMax.z) calibrationDataMax.z = COMPASS_axis.z;
    }
    Compass_setCalibration(calibrationDataMax, calibrationDataMin);
    printf("Max: %i, %i, %i\n", calibrationDataMax.x, calibrationDataMax.y, calibrationDataMax.z);
    printf("Min: %i, %i, %i\n", calibrationDataMin.x, calibrationDataMin.y, calibrationDataMin.z);

}

