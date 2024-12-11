#ifndef __BSP_OV7670_H
#define __BSP_OV7670_H

#include "main.h"
#include "stm32f1xx_hal.h"
#include "sys.h"
#include "bsp_sccb.h"


#define CAMERA_USE_FIFO 1   /* Select FIFO ,1:USE FIFO 
																						0:DON'T USE */
																						
#define OV7670_VSYNC_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()

#define OV7670_VSYNC_PORT		GPIOA
#define OV7670_VSYNC				GPIO_PIN_8


#if CAMERA_USE_FIFO==1

#define FIFO_DATA_CLK_ENABLE() 	__HAL_RCC_GPIOC_CLK_ENABLE()
#define FIFO_RRST_CLK_ENABLE()	__HAL_RCC_GPIOG_CLK_ENABLE()
#define	FIFO_OE_CLK_ENABLE()		__HAL_RCC_GPIOG_CLK_ENABLE()
#define FIFO_WEN_CLK_ENABLE()		__HAL_RCC_GPIOB_CLK_ENABLE()
#define FIFO_RCLK_CLK_ENABLE()	__HAL_RCC_GPIOB_CLK_ENABLE()
#define FIFO_WRST_CLK_ENABLE()	__HAL_RCC_GPIOD_CLK_ENABLE()

#define FIFO_DATA_PORT		GPIOC
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

#define FIFO_OE_PORT			GPIOG
#define FIFO_OE					GPIO_PIN_15

#define FIFO_WEN_PORT  			GPIOB
#define FIFO_WEN				GPIO_PIN_3

#define FIFO_RCLK_PORT  		GPIOB
#define FIFO_RCLK 				GPIO_PIN_4

#define FIFO_WRST_PORT 			GPIOD
#define FIFO_WRST				GPIO_PIN_6

#else

/* Camera control dot't use fifo */

/* Reserved */

#endif


uint8_t bsp_ov7670_Init(void);


#endif