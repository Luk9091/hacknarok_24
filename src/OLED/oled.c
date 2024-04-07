#include "oled.h"
#include "font.h"

int compass_x[] = {
    30, 52,
    60, 52,
    30, 8,
    2, 8
};

int compass_y[] = {
    0, 8,
    32, 49,
    56, 49,
    32, 8
};
    
int angle_range[] = {
    23, 67,
    113, 157,
    203, 247,
    293, 337
};

int N_y = 0;
int N_x = 0;

int16_t temp_label_x = 84;
int16_t hud_label_x = 72;
int16_t press_label_x = 72;
int16_t time_label_x = 84;

int16_t temp_label_y = 3;
int16_t hud_label_y = 23;
int16_t press_label_y = 33;
int16_t time_label_y = 53;

int16_t temp_value_x = 72;
int16_t hud_value_x = 86;
int16_t press_value_x = 76;
int16_t time_value_x = 86-15;

int16_t temp_value_y = 13;
int16_t hud_value_y = 31;
int16_t press_value_y = 43;
int16_t time_value_y = 63;

#include <hardware/i2c.h>
#include <hardware/gpio.h>
#define OLED_I2C i2c0
#define OLED_I2C_FREQ 400e3
#define OLED_SDA 21
#define OLED_SCL 20
#define PI 3.1415926F

void OLED_init(){
    i2c_init(OLED_I2C, OLED_I2C_FREQ);
    gpio_set_function(OLED_SDA, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA);
    gpio_pull_up(OLED_SCL);
    ssd1306_Init();
}

void drawLine(int x0, int y0, int x1, int y1, int width, bool limit){
    // y0 = ax0 + b
    // y1 = ax1 + b
    // y0 - y1 = a (x0 - x1) => a = (y0 - y1) / (x0 - x1)
    // b = y0 - ax0

    double a, b;
    int x_limMax = X_LIMIT, y_limMax = X_LIMIT;
    int x_limMin = 0, y_limMin = 0;
    
    if (x0 - x1 != 0){
        a = (double)(y1 - y0) / (double)(x1 - x0);
        b = y0 - a*x0;
    } 

    if (limit){
        if (x1 >= x0){
            x_limMax = x1+1;
            x_limMin = x0;
        } else {
            x_limMax = x0+1;
            x_limMin = x1;
        }

        if (y1 >= y0)
        {
            y_limMax = y1+1;
            y_limMin = y0;
        }
        else
        {
            y_limMax = y0+1;
            y_limMin = y1;
        }
    } 
    
    
    for(int y = y_limMin; y < y_limMax; y++){
        for(int x = x_limMin; x < x_limMax; x++){
            if (x0 - x1 != 0){
                if (floor(a*x+b + width/2) >= y && floor(a*x+b - (width-1)/2) <= y)
                    ssd1306_DrawPixel(x, y);
            } else {
                if (ceil(x + width-1/2) >= x0 && floor(x - width/2) <= x0){
                    ssd1306_DrawPixel(x, y);
                }
            }
        }
    }
}

void drawCircle(int x0, int y0, int r, bool fill){
    // y0 = -y0;
    // (x - x0)^2 + (y - y0)^2 = r^2
    // y - y0 = sqrt(r^2 - (x-x0)^2)
    // y = sqrt(r^2 - (x-x0)^2) ± y0

    r = r*r;
    int a, b;
    

    for (int y = 0; y <= Y_LIMIT; y++){
        a = sqrt(r - pow(y - y0, 2));
        for (int x = 0; x <= X_LIMIT; x++){
            b = sqrt(r - pow(x - x0, 2));

            if (x == x0 + a || x == x0 - a || y == y0 + b || y == y0 - b)
                ssd1306_DrawPixel(x, y);

            if (fill && (x < x0 + a && x > x0 - a && y < y0 + b && y > y0 - b))
                ssd1306_DrawPixel(x, y);
        }
    }
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void PrintDistance(int left, int center, int right){
    int maxDist = 500;
    if (left > maxDist   || left < 0)   {left = 24;}   else { left   = map(left, 0, 500, 0, 24); }
    if (center > maxDist || center < 0) {center = 24;} else { center = map(center, 0, 500, 0, 24); }
    if (right > maxDist  || right < 0)  {right = 24;}  else { right  = map(right, 0, 500, 0, 24); }

    
    int x_l = left  * cos(45/360*2*PI) / sqrt(2);
    int x_r = right * cos(-45/360*2*PI) / sqrt(2);


    drawLine(32, 32-center, 32, 32, 1, true);
    drawLine(32 - x_l, 32 - x_l, 32, 32, 1, true);
    drawLine(32 + x_r, 32 - x_r, 32, 32, 1, true);

}

void PrintAngle(uint16_t angle){ //input needs to be added
    if(angle > angle_range[7] || angle <= angle_range[0])
        {
            N_x = compass_x[0];
            N_y = compass_y[0];           
        }
        else
        {
            for(int i = 0; i < 7; i++)
            {
                if (angle > angle_range[i] && angle <= angle_range[i+1])
                {
                    N_x = compass_x[i+1];
                    N_y = compass_y[i+1];
                    break;
                }
            }
        }
        ssd1306_Print_Char(N_x, N_y, 'N', font_8x5);
}

void PrintInfo(float temp, uint8_t hum, uint press, char *time){  //inputs needs to be added
        // float f_temp = -20.3;
        // int f_hud = 30;
        // int f_pressure = 1020;
        // char time[] = "21:37";
    
        char s_temp[32];
        char m_temp[32] = "degC";
        char s_hud[32];
        char m_hud[] = "%";
        char s_pressure[32];
        char m_press[32] = "hPa";

        sprintf(s_temp, "%.1f", temp);
        itoa(hum, s_hud, 10);
        itoa(press, s_pressure, 10);

        strcat(s_temp, m_temp);
        strcat(s_hud, m_hud);
        strcat(s_pressure, m_press);


        ssd1306_Print_Word(temp_label_x, temp_label_y, "Temp:", font_8x5);
        ssd1306_Print_Word(temp_value_x, temp_value_y, s_temp, font_8x5);

        ssd1306_Print_Word(hud_label_x, hud_label_y, "Humidity:", font_8x5);
        ssd1306_Print_Word(hud_value_x, hud_value_y, s_hud, font_8x5);

        ssd1306_Print_Word(press_label_x, press_label_y, "Pressure:", font_8x5);
        ssd1306_Print_Word(press_value_x, press_value_y, s_pressure, font_8x5);

        ssd1306_Print_Word(time_label_x, time_label_y, "Time:", font_8x5);
        ssd1306_Print_Word(time_value_x, time_value_y, time, font_8x5);

        ssd1306_DrawPixel(0,0);
        ssd1306_DrawPixel(0,63);
        ssd1306_DrawPixel(127,0);
        ssd1306_DrawPixel(127,63);

        drawCircle(32, 32, 24, false);
        drawLine(32-24, 32, 32+24, 32, 1, true);
}
