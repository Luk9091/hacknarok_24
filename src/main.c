#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/stdio.h>
#include <pico/cyw43_arch.h>

#include <hardware/gpio.h>
#include <hardware/rtc.h>
#include <pico/util/datetime.h>

#include "i2c.h"
#include "vl53l0x_api.h"
#include "vl53l0x_rp2040.h"
#include "tof.h"
#include "compass.h"
#include "bme280.h"
#include "oled.h"
#include "encoder.h"


int main(){
    stdio_init_all();
    I2C_Init();
    
    if(cyw43_arch_init()){
        printf("WiFi init failed\n");
        return -1;
    }
    encoder_init();
    
    VL53L0X_Dev_t ToF_left;
    VL53L0X_Dev_t ToF_center;
    VL53L0X_Dev_t ToF_right;

    ToF_sensorInit(&ToF_left, ToF_LEFT_ADDRESS, ToF_LEFT_PIN);
    ToF_sensorInit(&ToF_center, ToF_CENTER_ADDRESS, ToF_CENTER_PIN);
    ToF_sensorInit(&ToF_right, ToF_RIGHT_ADDRESS, ToF_RIGHT_PIN);

    Compass_Init();
    // Compass_calibration();

    int32_t temp, hum, pres;
    bme280_calib_params calib;
    bme280_init();
    bme280_get_calib_params(&calib);

    OLED_init();
    
    
    // sleep_ms(5000);
    uint left, center, right;
    uint16_t azimuth = 0;

    datetime_t timeFrame = {
            .year  = 2024,
            .month = 04,
            .day   = 07,
            .dotw  = 6, 
            .hour  = 10,
            .min   = 00,
            .sec   = 00
    };
    
    rtc_init();
    rtc_set_datetime(&timeFrame);
    sleep_us(64);
    // rtc_enable_alarm();
    char time[9];
    while(1){
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, !cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN));
        // printf("Time: %lli\n", time_us_64());
        left = ToF_measure(&ToF_left);
        // printf("Left: %hi\t", measure);
        center = ToF_measure(&ToF_center);
        // printf("Center: %hi\t", center);
        right = ToF_measure(&ToF_right);
        // printf("Right: %hi", measure);
        // printf("\n");
        Compass_readData();
        Compass_applyCalibration();
        azimuth = Compass_getAzimuth();
        
        // printf("X: %4hi, Y:%4hi, Z: %4hi\n", COMPASS_axis.x, COMPASS_axis.y, COMPASS_axis.z);
        // printf("Azimuth: %i\n", azimuth);

        bme280_read(&pres, &temp, &hum, &calib);
        // printf("T: %4i, H: %4i, P: %4i\n", temp, hum, pres);
        set_time(&timeFrame);
        rtc_get_datetime(&timeFrame);
        sprintf(time, "%2i:%2i:%2i", timeFrame.hour, timeFrame.min, timeFrame.sec);
        // sprintf(time, "%2i:%2i", timeFrame.hour, timeFrame.sec);

        ssd1306_Clear();
        PrintInfo((float)(temp/10)/10, (uint8_t)(hum/1000), pres/100, time);
        PrintAngle(360-azimuth);
        PrintDistance(left, center, right);
        ssd1306_Show();

        // I2C_scan(i2c1);
        sleep_ms(500);
    }

    return 0;
}
