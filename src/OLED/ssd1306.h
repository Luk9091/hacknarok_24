#ifndef SSD1306_H_
#define SSD1306_H_

#include "hardware/i2c.h"
#include "stdio.h"
#include "stdlib.h"


#define I2C_ADDRESS             0x3C

#define LCDWIDTH                128
#define LCDHEIGHT               64
#define PAGES                   LCDHEIGHT/8
#define BUFFSIZE                PAGES*LCDWIDTH+1
#define FILLCONST               (u_int8_t)LCDWIDTH/100    


#define SET_CONTRAST            0x81 
#define SET_ENTIRE_ON           0xA4
#define SET_NORM_INV            0xA6
#define SET_DISP                0xAE
#define SET_DISP_OFFSET         0xD3
#define SET_COM_PIN_CFG         0xDA
#define SET_VCOM_DESEL          0xDB
#define SET_DISP_CLK_DIV        0xD5
#define SET_PRECHARGE           0xD9
#define SET_MUX_RATIO           0xA8
#define SET_DISP_START_LINE     0x40
#define SET_MEM_ADDR            0x20
#define SET_COL_ADDR            0x21
#define SET_PAGE_ADDR           0x22 
#define SET_COM_OUT_DIR         0xC0 
#define SET_SEG_REMAP           0xA0 
#define SET_CHARGE_PUMP         0x8D 



void ssd1306_Command(uint8_t cmd);

void ssd1306_Init();

void ssd1306_DrawPixel(int16_t x, int16_t y);

void ssd1306_setContrast(uint8_t val);

void ssd1306_Show();

void ssd1306_Clear();

void ssd1306_Print_Char(int16_t x, int16_t y, char c, const uint8_t *font);

void ssd1306_Print_Word(int16_t x, int16_t y, char *c, const uint8_t *font);

void ssd1306_Print_Bar_Hor(u_int16_t y, u_int8_t fill);

void ssd1306_Clear_Title();

void ssd1306_Without_Title();




#endif