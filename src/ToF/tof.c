#include "tof.h"

void ToF_sensorInit(VL53L0X_Dev_t *tof, uint8_t address, uint32_t pin){
    tof->I2cDevAddr = ToF_DEFAULT_ADDRESS;
    tof->comms_speed_khz = I2C_FREQUENCY/1000;
    tof->comms_type = 1;
    gpio_init(pin);
    gpio_set_dir(pin, 1);
    gpio_put(pin, 1);

    VL53L0X_i2c_Init(I2C_CHANNEL, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY/1000, false);
    VL53L0X_dev_init_i2c_newAddress(tof, VL53L0X_DEFAULT_MODE, address);
}


int ToF_measure(VL53L0X_Dev_t *tof){
    uint16_t data = 0;
    int error = VL53L0X_SingleRanging(tof, &data);
    if (error != VL53L0X_ERROR_NONE) data = error;
    
    return data;
}