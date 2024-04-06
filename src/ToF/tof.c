#include "tof.h"

VL53L0X_Error ToF_sensorInit(VL53L0X_Dev_t *tof, uint8_t address, uint32_t pin){
    tof->I2cDevAddr = ToF_DEFAULT_ADDRESS;
    tof->comms_speed_khz = I2C_FREQUENCY/1000;
    tof->comms_type = 1;
    gpio_init(pin);
    gpio_set_dir(pin, 1);
    // gpio_put(pin, 0);
    gpio_put(pin, 1);

    VL53L0X_i2c_Init(I2C_CHANNEL, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY/1000, false);
    return VL53L0X_dev_init_i2c_newAddress(tof, VL53L0X_DEFAULT_MODE, address);
    // return VL53L0X_device_initialise(tof, VL53L0X_DEFAULT_MODE);
}


int16_t ToF_measure(VL53L0X_Dev_t *tof){
    uint16_t data = 0;
    int16_t error = VL53L0X_SingleRanging(tof, &data);
    if (error != VL53L0X_ERROR_NONE) return error;
    
    return data;
}