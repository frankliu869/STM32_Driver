#include "bsp_sccb.h"
#include "delay.h"

void bsp_sccb_Init(void){
	
	GPIO_InitTypeDef GPIO_Initstuct={0};
	
	SCCB_SCL_CLK_ENABLE();
	SCCB_SDA_CLK_ENABLE();

	GPIO_Initstuct.Mode=GPIO_MODE_OUTPUT_PP;
	GPIO_Initstuct.Pin=SCCB_SCL_PIN;
	GPIO_Initstuct.Pull=GPIO_PULLUP;
	GPIO_Initstuct.Speed=GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SCCB_SCL_PORT,&GPIO_Initstuct);
	
	GPIO_Initstuct.Mode=GPIO_MODE_OUTPUT_PP;
	GPIO_Initstuct.Pin=SCCB_SDA_PIN;
	GPIO_Initstuct.Pull=GPIO_PULLUP;
	GPIO_Initstuct.Speed=GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SCCB_SDA_PORT,&GPIO_Initstuct);

	SCCB_SCL=1;
	SCCB_SDA=1;
}

void bsp_sccb_Start(void){
	SCCB_SCL=1;
	SCCB_SDA=1;
	delay_us(50);
	SCCB_SDA=0;
	delay_us(50);
	SCCB_SCL=0;
	delay_us(50);
}

void bsp_sccb_Stop(void){
	SCCB_SCL=0;
	SCCB_SDA=0;
	delay_us(50);
	SCCB_SCL=1;
	delay_us(50);
	SCCB_SDA=1;
	delay_us(50);
}

void bsp_sccb_No_Ack(void){
	delay_us(50);
	SCCB_SCL=1;
	SCCB_SDA=1;
	delay_us(50);
	SCCB_SCL=0;
	delay_us(50);
	SCCB_SDA=0;
	delay_us(50);
}

uint8_t bsp_sccb_WR_Byte(uint8_t txdata){
	uint8_t i, res;
	SCCB_SDA_OUT();
	
	for(i=0;i<8;i++){
		SCCB_SCL=0;
		delay_us(25);
		if((txdata<<i) & 0x80) SCCB_SDA=1;
		else SCCB_SDA=0;
		delay_us(25);
		SCCB_SCL=1;
		delay_us(50);
	}
	SCCB_SCL=0;
	SCCB_SDA_IN();
	delay_us(50);
	SCCB_SCL=1;
	delay_us(50);
	if(SCCB_SDA_RD) res=1;
	else res=0;
	SCCB_SCL=0;
	delay_us(50);
	SCCB_SDA_OUT();
	return res;
}

uint8_t bsp_sccb_RD_Byte(void){
	
	uint8_t i, res=0;
	SCCB_SDA_IN();
	
	for(i=0;i<8;i++){
		SCCB_SCL=1;
		delay_us(25);
		res = res << 1;
		if(SCCB_SDA_RD) res |= 0x01;
		delay_us(25);
		SCCB_SCL=0;
		delay_us(50);
	}
	SCCB_SDA_OUT();
	return res;
}

uint8_t bsp_sccb_WR_Reg(uint8_t reg, uint8_t txdata){
	uint8_t res=0;
	delay_us(50);
	bsp_sccb_Start();
	
	if(bsp_sccb_WR_Byte(SCCB_ID)) res=1;
	delay_us(50);
	if(bsp_sccb_WR_Byte(reg)) res=1;
	delay_us(50);
	if(bsp_sccb_WR_Byte(txdata)) res=1;
	delay_us(50);
	bsp_sccb_Stop();
	
	return res;
}

uint8_t bsp_sccb_RD_Reg(uint8_t reg){
	uint8_t data; 
	delay_us(50);
	bsp_sccb_Start();
	
	bsp_sccb_WR_Byte(SCCB_ID);
	delay_us(50);
	bsp_sccb_WR_Byte(reg);
	delay_us(50);
	
	bsp_sccb_Stop();
	delay_us(50);
	bsp_sccb_Start();
	
	bsp_sccb_WR_Byte(SCCB_ID|0x01);
	delay_us(50);
	data = bsp_sccb_RD_Byte();
	
	bsp_sccb_No_Ack();
	bsp_sccb_Stop();
	
	return data;
}