#ifndef __LCD_FSMC_H
#define __LCD_FSMC_H

#include "stm32f1xx_hal.h"
#include "stdlib.h"
#include "stdio.h"
#include "delay.h"	

typedef struct{
	uint16_t width;				//LCD Width
	uint16_t height;			//LCD Height
	uint16_t id;				//LCD ID
	uint8_t  dir;				//LCD Direction	
	uint16_t	wramcmd;		//Write Gram Commamd
	uint16_t setxcmd;			//Set X Position 
	uint16_t setycmd;	  		//Set Y Position 
}LCD_HandleTypeDef;



typedef struct{
	vu16 LCD_REG;
	vu16 LCD_RAM;
}LCD_TypeDef;


#define LCD_BASE        ((uint32_t)(0x6C0007FE))
#define LCD             ((LCD_TypeDef *)(0x6C0007FE))

extern SRAM_HandleTypeDef TFTSRAM_Handler;

#define	LCD_LED PBout(0)

#define vertical 0
#define horizontal 1

#define LCD_Width 240
#define LCD_Height 320

//Scan_Direction
#define L2R_U2D  0 		//left to right, up to down
#define L2R_D2U  1 		//left to right, bottom to up
#define R2L_U2D  2 		//right to left, up to down
#define R2L_D2U  3 		//right to left, bottom to up

#define U2D_L2R  4 		//up to down, left to right
#define U2D_R2L  5 		//up to down, right to left
#define D2U_L2R  6 		//down to up, left to right
#define D2U_R2L  7		//down to up, right to left

//Pixel Color
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 
#define BRRED 			 0XFC07 
#define GRAY  			 0X8430 



void LCD_WR_REG(uint16_t);

void LCD_WR_DATA(uint16_t);

void LCD_WriteReg(uint16_t LCD_Reg,uint16_t LCD_RegValue);

uint16_t LCD_ReadReg(uint16_t LCD_Reg);

void LCD_WriteRAM_Prepare(void);

void LCD_WriteRAM(uint16_t RGB_Code);

void LCD_SetDirection(uint8_t Direction);

void LCD_SetWindow(uint16_t x, uint16_t y, uint16_t width, uint16_t height);

void LCD_SetCursor(uint16_t x, uint16_t y);

void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t RGB_code);

void LCD_Fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t RGB_code);

void LCD_Clear(uint16_t RGB_code);

void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t RGB_code);

void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t size, uint8_t asc2_code);

void LCD_ShowString(uint16_t x, uint16_t y, uint8_t size, char *str);

void TFT_LCD_Init(void);

#endif



