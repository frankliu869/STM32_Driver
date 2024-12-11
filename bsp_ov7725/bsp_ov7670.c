#include "bsp_ov7670.h"


static void bsp_FIFO_Init(void);


#if CAMERA_USE_FIFO==1

static void bsp_FIFO_Init(void){
	
	GPIO_InitTypeDef GPIO_Initstuct={0};
	
	FIFO_DATA_CLK_ENABLE();
	FIFO_RRST_CLK_ENABLE();
	//FIFO_OE_CLK_ENABLE(); 		/* repeat call */
	FIFO_WEN_CLK_ENABLE();
	//FIFO_RCLK_CLK_ENABLE(); 	/* repeat call */
	FIFO_WRST_CLK_ENABLE();
	
	/* FIFO DATA PORT INITIALIZE */
	
	GPIO_Initstuct.Mode=GPIO_MODE_INPUT;
	GPIO_Initstuct.Pin=FIFO_D0|FIFO_D1|FIFO_D2|FIFO_D3|
										 FIFO_D4|FIFO_D5|FIFO_D6|FIFO_D7;
	GPIO_Initstuct.Pull=GPIO_PULLUP;
	GPIO_Initstuct.Speed=GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(FIFO_DATA_PORT,&GPIO_Initstuct);
	
	/* FIFO RRST AND OE PIN */
	
	GPIO_Initstuct.Mode=GPIO_MODE_OUTPUT_PP;
	GPIO_Initstuct.Pin=FIFO_RRST|FIFO_OE;
	GPIO_Initstuct.Pull=GPIO_PULLUP;
	GPIO_Initstuct.Speed=GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(FIFO_RRST_PORT,&GPIO_Initstuct);
	
	/* FIFO OE PIN */
	
	/*
	GPIO_Initstuct.Mode=GPIO_MODE_OUTPUT_PP;
	GPIO_Initstuct.Pin=FIFO_OE|FIFO_RRST;
	GPIO_Initstuct.Pull=GPIO_PULLUP;
	GPIO_Initstuct.Speed=GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(FIFO_OE_PORT,&GPIO_Initstuct);
	*/
	
	/* FIFO WEN AND RCLK PIN */
	
	GPIO_Initstuct.Mode=GPIO_MODE_OUTPUT_PP;
	GPIO_Initstuct.Pin=FIFO_WEN|FIFO_RCLK;
	GPIO_Initstuct.Pull=GPIO_PULLUP;
	GPIO_Initstuct.Speed=GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(FIFO_WEN_PORT,&GPIO_Initstuct);
	
	/* FIFO RCLK PIN */
	
	/*
	GPIO_Initstuct.Mode=GPIO_MODE_OUTPUT_PP;
	GPIO_Initstuct.Pin=FIFO_RCLK;
	GPIO_Initstuct.Pull=GPIO_PULLUP;
	GPIO_Initstuct.Speed=GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(FIFO_RCLK_PORT,&GPIO_Initstuct);
	*/
	
	/* FIFO WRST PIN */
	
	GPIO_Initstuct.Mode=GPIO_MODE_OUTPUT_PP;
	GPIO_Initstuct.Pin=FIFO_WRST;
	GPIO_Initstuct.Pull=GPIO_PULLUP;
	GPIO_Initstuct.Speed=GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(FIFO_WRST_PORT,&GPIO_Initstuct);
}

#endif

uint8_t bsp_ov7670_Init(void){
	
	GPIO_InitTypeDef GPIO_Initstuct={0};
	
	__HAL_AFIO_REMAP_SWJ_DISABLE();
	
	/* SCCB AND FIFO INITIALIZE */
	bsp_sccb_Init();
	bsp_FIFO_Init();
	
	/* CAMERA VSYNC PIN INITIALIZE*/
	OV7670_VSYNC_CLK_ENABLE();
	
	GPIO_Initstuct.Mode=GPIO_MODE_INPUT;
	GPIO_Initstuct.Pin=OV7670_VSYNC;
	GPIO_Initstuct.Pull=GPIO_PULLUP;
	GPIO_Initstuct.Speed=GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(OV7670_VSYNC_PORT,&GPIO_Initstuct);
	
#if CAMERA_USE_FIFO==1
	
	bsp_FIFO_Init();
	
#else
	
	/* if don't use fifo , the user must to initial pin of camera data */
	
	/* Reserved */
	
#endif



	

}