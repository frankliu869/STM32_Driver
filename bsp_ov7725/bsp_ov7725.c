#include "bsp_ov7725.h"
#include "bsp_ov7725_cfg.h"
#include "sys.h"
#include "delay.h"
#include "stm32f1xx_hal.h"
#include "LCD_FSMC.h"

uint8_t frame_state=0, FPS=0;

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
	
	FIFO_WRST_PIN=1;
	FIFO_RRST_PIN=1;
	FIFO_OE_PIN=0;
	FIFO_WEN_PIN=0;
	FIFO_RCLK_PIN=0;
}

#endif

#if VSYNC_DETECT_FOR_EXTI_INTERRUPT==1

EXTI_HandleTypeDef ov_exti;
EXTI_ConfigTypeDef ov_exti_cfg;

uint8_t OV_EXTI_Init(void){
	
	GPIO_InitTypeDef GPIO_Initstuct={0};
	
	OV_EXTI_CLK_ENABLE();
	
	GPIO_Initstuct.Mode=GPIO_MODE_IT_RISING;
	GPIO_Initstuct.Pin=OV_EXTI_PIN;
	GPIO_Initstuct.Pull=GPIO_PULLDOWN;
	GPIO_Initstuct.Speed=GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(OV_EXTI_PORT,&GPIO_Initstuct);
	
	HAL_NVIC_SetPriority(EXTI9_5_IRQn,1,1);
}

void EXTI9_5_IRQHandler(void){

	HAL_GPIO_EXTI_IRQHandler(OV_EXTI_PIN);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	
	if(frame_state==0){
		FIFO_WRST_PIN=0;
		FIFO_WRST_PIN=1;
		FIFO_WEN_PIN=1;
		frame_state=1;
	}else{
		FIFO_WEN_PIN=0;
	}
	
}

#endif

uint8_t bsp_ov7725_Init(void){
	
	GPIO_InitTypeDef GPIO_Initstuct={0};
	uint32_t temp;
	
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

	if(bsp_sccb_WR_Reg(0x12,0x80)) return 1;  /* software reset */
	delay_ms(10);
	temp = (bsp_sccb_RD_Reg(0x1c)<<8) & 0xFF00;  /* read manufacturer ID! */
	temp |= bsp_sccb_RD_Reg(0x1d);
	
	if(temp != OV7725_Manufacturer){
		return 1;
	}else{
		printf("OV7725 error!\r\n %x", temp);
		temp=0;
		for(uint16_t i=0;i<sizeof(ov7725_init_reg_tb1)/2;i++){
			temp = bsp_sccb_WR_Reg(ov7725_init_reg_tb1[i][0],ov7725_init_reg_tb1[i][1]);
			if(temp == 1) return 1;
		}
	}

	return 0;

}

uint8_t bsp_ov7725_LcdDisplay(uint16_t dis_x, uint16_t dis_y){
	
	uint16_t i,k, RGB_CODE;
	
	if(frame_state==1){
		LCD_SetDirection(U2D_L2R);
		LCD_SetCursor(0,0);
		LCD_WriteRAM_Prepare();
		FIFO_RRST_PIN=0;
		FIFO_RCLK_PIN=0;
		FIFO_RCLK_PIN=1;
		FIFO_RCLK_PIN=0;
		FIFO_RRST_PIN=1;
		FIFO_RCLK_PIN=1;
		for(i=0;i<LCD_Display_Width;i++){
			for(k=0;k<LCD_Display_Height;k++){
				FIFO_RCLK_PIN=0;
				RGB_CODE = FIFO_RD_DATA;
				FIFO_RCLK_PIN=1;
				RGB_CODE <<=8;
				FIFO_RCLK_PIN=0;
				RGB_CODE |= FIFO_RD_DATA;
				FIFO_RCLK_PIN=1;
				LCD->LCD_RAM = RGB_CODE;
			}
		}
		frame_state=0;
		FPS++;
		LCD_SetDirection(L2R_U2D);
	}

}

