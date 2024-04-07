#ifndef __OLED__H
#define __OLED__H

#include <stdio.h>
#include <stdlib.h>
#include <pico/stdio.h>
#include <math.h>
#include <string.h>
// #include "font.h"
#include "ssd1306.h"

#define Y_LIMIT 64
#define X_LIMIT 64


void OLED_init();
void drawLine(int x0, int y0, int x1, int y1, int width, bool limit);
void drawCircle(int x0, int y0, int r, bool fill);
void PrintDistance(int left, int center, int right);
void PrintAngle(uint16_t angle);
void PrintInfo(float temp, uint8_t hum, uint press, char *time);

#endif