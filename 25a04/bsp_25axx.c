#include "bsp_25axx.h"
#include "usart.h"


I2C_HandleTypeDef hi2c1;

void bsp_25axx_Init(void){
	
	hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000; //100KHz
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;  //50%
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; //7bit
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; 
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; //broadcast
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}	
	

void bsp_25axx_ByteWrite(uint8_t addr, uint8_t *txdata, uint8_t txsize){
	
	HAL_I2C_Mem_Write(&hi2c1,bsp_25axx_WriteAddr,addr,I2C_MEMADD_SIZE_8BIT,txdata,txsize,1000);
	HAL_Delay(5);

}

void bsp_25axx_PageWrite(uint8_t addr, uint8_t *txdata){

	int index=0;
	if(HAL_I2C_Mem_Write(&hi2c1,bsp_25axx_WriteAddr,addr,I2C_MEMADD_SIZE_8BIT,txdata,8,1000)!=HAL_OK){
		index = hi2c1.ErrorCode;
		printf("\r\n%x\r\n", index);
		MX_I2C1_Init();
	}	
	HAL_Delay(5);
}

uint8_t bsp_25axx_RandomRead(uint8_t addr){
	
	uint8_t rxdata=0;
	int index=0;
	HAL_I2C_Mem_Read(&hi2c1,bsp_25axx_ReadAddr,addr,I2C_MEMADD_SIZE_8BIT,&rxdata,1,1000);
	index = hi2c1.ErrorCode;
	printf("\r\n%x\r\n", index);
	return rxdata;

}

void bsp_25axx_SequentialRead(uint8_t addr, uint8_t *rxdata, uint8_t rxsize){
	int index=0;
	if(HAL_I2C_Mem_Read(&hi2c1,bsp_25axx_ReadAddr,addr,I2C_MEMADD_SIZE_8BIT,rxdata,rxsize,1000)!=HAL_OK){
		index = hi2c1.ErrorCode;
		printf("\r\n%x\r\n", index);
		MX_I2C1_Init();
	}		
	
}

