#include "ssd1306.h" 
#include <string.h>



uint8_t buffer[BUFFSIZE]; //copy of videoRAM


void ssd1306_Command(uint8_t cmd)
{
    uint8_t command[2] = {0x00, cmd}; //address to send commands 0x00
    i2c_write_blocking(i2c0, I2C_ADDRESS, command, 2, false);
}
void ssd1306_setContrast(uint8_t val) //using for change contras
{
    ssd1306_Command(SET_CONTRAST);
    ssd1306_Command(val);
}
void ssd1306_Init() //initialize sequence
{
    ssd1306_Command(SET_DISP);  

	ssd1306_Command(SET_DISP_CLK_DIV); 
	ssd1306_Command(0x80); 
	ssd1306_Command(SET_MUX_RATIO); 
	ssd1306_Command(LCDHEIGHT-1);  
	ssd1306_Command(SET_DISP_OFFSET);  
	ssd1306_Command(0x00); 
    ssd1306_Command(SET_DISP_START_LINE);
    ssd1306_Command(SET_CHARGE_PUMP);

	ssd1306_Command(0x14);  

	ssd1306_Command(SET_SEG_REMAP | 0x01);  
	ssd1306_Command(SET_COM_OUT_DIR | 0x08);  

	ssd1306_Command(SET_COM_PIN_CFG);  
	// ssd1306_Command(0x12);  
	ssd1306_Command(128>2*64?0x02:0x12);  

	ssd1306_Command(SET_CONTRAST); 
	ssd1306_Command(0x80);  

	ssd1306_Command(SET_PRECHARGE);	
	ssd1306_Command(0x22);

	ssd1306_Command(SET_VCOM_DESEL);  
	ssd1306_Command(0x30);  
	ssd1306_Command(SET_ENTIRE_ON);	
	ssd1306_Command(SET_NORM_INV);


	ssd1306_Command(SET_DISP | 0x01);  
	ssd1306_Command(SET_MEM_ADDR);  

	ssd1306_Command(0x00);  
}
void ssd1306_DrawPixel(int16_t x, int16_t y) //function for drawing pixels
{
    buffer[x+LCDWIDTH*(y>>3)+1]|=0x1<<(y&0x07);
}
void ssd1306_Show() 
{
	ssd1306_Command(SET_COL_ADDR);
	ssd1306_Command(0);               //lower column
	ssd1306_Command(LCDWIDTH-1);      //highest columnt
	ssd1306_Command(SET_PAGE_ADDR);
	ssd1306_Command(0);               //lower page
	ssd1306_Command(7);               //highest page
	
    buffer[0] = 0x40;

	i2c_write_blocking(i2c0, I2C_ADDRESS, buffer, BUFFSIZE, false);

}
void ssd1306_Clear() //set 0 to all pixels
{
	for(size_t i = 0; i<BUFFSIZE; i++)
        buffer[i] = 0x00;
}
void ssd1306_Print_Char(int16_t x, int16_t y, char c, const uint8_t *font) //printing one char using fonts
{
	u_int8_t char_height = font[0];
	u_int8_t char_lenght = font[1];
	const u_int16_t start = ((uint64_t)(c)-font[3]+1)*char_lenght;
	
	for (size_t i = 0; i<char_lenght; i++)
	{
		buffer[x+i+LCDWIDTH*(y>>3)+1]|=font[start+i];
	}
}
void ssd1306_Print_Word(int16_t x, int16_t y, char *c,const uint8_t *font) //print word
{
	for (uint8_t i = 0; i < strlen(c); i++ )
	{

		ssd1306_Print_Char(x+6*i,y,c[i], font);

	}
}
void ssd1306_Print_Bar_Hor(u_int16_t y, u_int8_t fill) //printing bar with modifiable filling
{
	for (u_int16_t i = 0; i<LCDWIDTH; i++)
	{
		ssd1306_DrawPixel(i,y);
	}
	for (u_int16_t i = 0; i<LCDWIDTH; i++)
	{
		ssd1306_DrawPixel(i,y+10);
	}
	for (u_int16_t i = 0; i<10; i++)
	{
		ssd1306_DrawPixel(0,y+i);

	}
	for (u_int16_t i = 0; i<10; i++)
	{
		ssd1306_DrawPixel(LCDWIDTH-1,y+i);

	}
	for (u_int8_t i = 2; i < 9; i++ )
	{
		for(u_int8_t v = 1; v < fill*FILLCONST; v++ )
			ssd1306_DrawPixel(v,y+i);
	}
}
void ssd1306_Clear_Title() // clear only last page
{
	for(size_t i = BUFFSIZE-128; i<BUFFSIZE; i++)
        buffer[i] = 0x00;
}
void ssd1306_Without_Title() //clear without last page
{
	for(size_t i = 0; i<BUFFSIZE-128; i++) 
        buffer[i] = 0x00;
}


