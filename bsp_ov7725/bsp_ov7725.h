#ifndef __BSP_OV7725_H
#define __BSP_OV7725_H

#include "main.h"
#include "stm32f1xx_hal.h"
#include "sys.h"
#include "bsp_sccb.h"


#define CAMERA_USE_FIFO 1   /* Select FIFO ,1:USE FIFO 
																						0:DON'T USE */

#define VSYNC_DETECT_FOR_EXTI_INTERRUPT 1

#define LCD_Display_Width			240
#define LCD_Display_Height		320
																						
#define OV7725_Manufacturer		0x7FA2		
				
#define OV7670_VSYNC_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()

#define OV7670_VSYNC_PORT		GPIOA
#define OV7670_VSYNC				GPIO_PIN_8


#if CAMERA_USE_FIFO==1

#define FIFO_DATA_CLK_ENABLE() 	__HAL_RCC_GPIOC_CLK_ENABLE()
#define FIFO_RRST_CLK_ENABLE()	__HAL_RCC_GPIOG_CLK_ENABLE()
#define	FIFO_OE_CLK_ENABLE()	__HAL_RCC_GPIOG_CLK_ENABLE()
#define FIFO_WEN_CLK_ENABLE()	__HAL_RCC_GPIOB_CLK_ENABLE()
#define FIFO_RCLK_CLK_ENABLE()	__HAL_RCC_GPIOB_CLK_ENABLE()
#define FIFO_WRST_CLK_ENABLE()	__HAL_RCC_GPIOD_CLK_ENABLE()

#define FIFO_DATA_PORT				GPIOC
#define FIFO_D0						GPIO_PIN_0
#define FIFO_D1						GPIO_PIN_1
#define FIFO_D2						GPIO_PIN_2
#define FIFO_D3						GPIO_PIN_3
#define FIFO_D4						GPIO_PIN_4
#define FIFO_D5						GPIO_PIN_5
#define FIFO_D6						GPIO_PIN_6
#define FIFO_D7						GPIO_PIN_7


#define FIFO_RRST_PORT			GPIOG
#define FIFO_RRST				GPIO_PIN_14
#define FIFO_RRST_PIN			PGout(14)

#define FIFO_OE_PORT			GPIOG
#define FIFO_OE					GPIO_PIN_15
#define FIFO_OE_PIN				PGout(15)

#define FIFO_WEN_PORT  			GPIOB
#define FIFO_WEN				GPIO_PIN_3
#define FIFO_WEN_PIN			PBout(3)

#define FIFO_RCLK_PORT  		GPIOB
#define FIFO_RCLK 				GPIO_PIN_4
#define FIFO_RCLK_PIN			PBout(4)

#define FIFO_WRST_PORT 			GPIOD
#define FIFO_WRST				GPIO_PIN_6
#define FIFO_WRST_PIN			PDout(6)

#define FIFO_RD_DATA 			(GPIOC->IDR&0XFF)

#else

/* Camera control dot't use fifo */

/* Reserved */

#endif

#if VSYNC_DETECT_FOR_EXTI_INTERRUPT==1

#define OV_EXTI_CLK_ENABLE() 	__HAL_RCC_GPIOA_CLK_ENABLE()
#define OV_EXTI_PORT			GPIOA
#define OV_EXTI_PIN				GPIO_PIN_8

extern EXTI_HandleTypeDef ov_exti;
extern EXTI_ConfigTypeDef ov_exti_cfg;


uint8_t OV_EXTI_Init(void);

#endif


uint8_t bsp_ov7725_Init(void);


#endif