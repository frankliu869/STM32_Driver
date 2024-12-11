#ifndef __BSP_SCCB_H
#define __BSP_SCCB_H

#include "main.h"
#include "sys.h"



#define SCCB_SCL_CLK_ENABLE() 			__HAL_RCC_GPIOD_CLK_ENABLE()
#define	SCCB_SDA_CLK_ENABLE()				__HAL_RCC_GPIOG_CLK_ENABLE()

#define SCCB_SCL_PORT 	GPIOD
#define SCCB_SCL_PIN 		GPIO_PIN_3

#define SCCB_SDA_PORT 	GPIOG
#define SCCB_SDA_PIN 		GPIO_PIN_13

#define SCCB_SDA_IN() 	{GPIOG->CRH &= 0xFF0FFFFF;GPIOG->CRH |= 0x08 <<20;}
#define SCCB_SDA_OUT() 	{GPIOG->CRH &= 0xFF0FFFFF;GPIOG->CRH |= 0x03 <<20;}

#define SCCB_SCL 			PDout(3)
#define SCCB_SDA 			PGout(13)
#define SCCB_SDA_RD 	PGin(13)

#define SCCB_ID   		0X42 

void bsp_sccb_Init(void);
void bsp_sccb_Start(void);
void bsp_sccb_Stop(void);
void bsp_sccb_No_Ack(void);
uint8_t bsp_sccb_WR_Byte(uint8_t txdata);
uint8_t bsp_sccb_RD_Byte(void);
uint8_t bsp_sccb_WR_Reg(uint8_t reg, uint8_t txdata);
uint8_t bsp_sccb_RD_Reg(uint8_t reg);


#endif