#include "i2c.h"




void I2C_Init(){
    static int isInit = 0;
    if (isInit) return;
    
    i2c_init(I2C_CHANNEL, I2C_FREQUENCY);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    isInit = 1;
}

int I2C_scan(i2c_inst_t *channel){
    // I2C_Init();
    int ret = 0;
    uint8_t rxData;
    uint dev = 0;

    printf("\nI2C Scan\n");
    

    for (uint8_t address = 0; address < 128; ++address){
        ret = i2c_read_blocking(channel, address, &rxData, 1, false);

        if (ret >= 0){
            printf("Address: 0x");
            if(address < 16) printf("0");
            printf("%X\n", address);
            dev++;
        }
    }

    if(dev == 0){
        printf("Dont find any devices!\n");
        return -1;
    }
    return 0;
}

void I2C_writeReg(uint8_t dev, uint8_t reg, uint8_t data){
    uint8_t table[2] = {reg, data};
    i2c_write_blocking(I2C_CHANNEL, dev, table, 2, false);
}

uint8_t I2C_readReg(uint8_t dev, uint8_t reg){
    uint8_t data;
    i2c_write_blocking(I2C_CHANNEL, dev, &reg, 1,  true);
    if (i2c_read_blocking(I2C_CHANNEL, dev, &data, 1, false) == 0)
        return data;
    return 0;
}