#include "LCD_FSMC.h"
#include "text_lib.h"



LCD_HandleTypeDef LCD_Handle;
SRAM_HandleTypeDef TFTSRAM_Handler;

uint16_t LCD_RD_DATA(void){
	vu16 ram;
	ram = LCD->LCD_RAM;
	return ram;
}

void LCD_WR_REG(uint16_t data){
	data = data;
	LCD->LCD_REG=data;

}

void LCD_WR_DATA(uint16_t data){
	data = data;
	LCD->LCD_RAM=data;

}


void LCD_WriteReg(uint16_t LCD_Reg,uint16_t LCD_RegValue){
	LCD->LCD_REG=LCD_Reg;
	LCD->LCD_RAM=LCD_RegValue;
}

uint16_t LCD_ReadReg(uint16_t LCD_Reg){
	LCD_WR_REG(LCD_Reg);
	delay_us(5);
	return LCD_RD_DATA();
}

void LCD_WriteRAM_Prepare(void){
	
	LCD->LCD_REG=LCD_Handle.wramcmd;
}

void LCD_WriteRAM(uint16_t RGB_Code){

	LCD->LCD_RAM=RGB_Code;
}

/*void LCD_Clear(){


}*/

void LCD_SetScanDirection(uint8_t ScanDirection){
	
	uint16_t CMD=0,val=0;
	
	if(LCD_Handle.id==0x9341){
		switch(ScanDirection)
		{
			case L2R_U2D:
				val|=(0<<7)|(0<<6)|(0<<5); 
				break;
			case L2R_D2U:
				val|=(1<<7)|(0<<6)|(0<<5); 
				break;
			case R2L_U2D:
				val|=(0<<7)|(1<<6)|(0<<5); 
				break;
			case R2L_D2U:
				val|=(1<<7)|(1<<6)|(0<<5); 
				break;	 
			case U2D_L2R:
				val|=(0<<7)|(0<<6)|(1<<5); 
				break;
			case U2D_R2L:
				val|=(0<<7)|(1<<6)|(1<<5); 
				break;
			case D2U_L2R:
				val|=(1<<7)|(0<<6)|(1<<5); 
				break;
			case D2U_R2L:
				val|=(1<<7)|(1<<6)|(1<<5); 
				break;	 
		}
		
		if(LCD_Handle.id==0x9341){
			CMD = 0x36;
			val |=0x08;
		}
		LCD_WriteReg(CMD,val);
	}
	
	if(LCD_Handle.id==0x9341){
		LCD_WR_REG(LCD_Handle.setxcmd);
		LCD_WR_DATA(0); LCD_WR_DATA(0);
		LCD_WR_DATA((LCD_Width-1)>>8); LCD_WR_DATA((LCD_Width-1)& 0x00FF);
		LCD_WR_REG(LCD_Handle.setycmd);
		LCD_WR_DATA(0); LCD_WR_DATA(0);
		LCD_WR_DATA((LCD_Height-1)>>8); LCD_WR_DATA((LCD_Height-1)& 0x00FF);
	
	}


}

void LCD_SetDirection(uint8_t Direction){
	
	if(Direction==vertical){
		
		LCD_Handle.width = LCD_Width;
		LCD_Handle.height = LCD_Height;
		if(LCD_Handle.id == 0x9341){
			LCD_Handle.wramcmd = 0x2C;
			LCD_Handle.setxcmd = 0x2A;
			LCD_Handle.setycmd = 0x2B;
			
		}else if(0){  //Reserved for other lcd driver
			
			
		}
	
	}else if(Direction==horizontal){
		
		LCD_Handle.width = LCD_Height;
		LCD_Handle.height = LCD_Width;
		if(LCD_Handle.id == 0x9341){
			LCD_Handle.wramcmd = 0x2C;
			LCD_Handle.setxcmd = 0x2B;
			LCD_Handle.setycmd = 0x2A;
		}else if(0){ //Reserved for other lcd driver
	
	
		}
	}
	LCD_SetScanDirection( L2R_U2D);

}


void LCD_SetWindow(uint16_t x, uint16_t y, uint16_t width, uint16_t height){
	
	if(LCD_Handle.id==0x9341){
		LCD_WR_REG(LCD_Handle.setxcmd);
		LCD_WR_DATA(x>>8); LCD_WR_DATA((x & 0x00FF));
		LCD_WR_DATA((x+LCD_Width-1)>>8); LCD_WR_DATA((x+LCD_Width-1)& 0x00FF);
		LCD_WR_REG(LCD_Handle.setycmd);
		LCD_WR_DATA(y>>8); LCD_WR_DATA((y & 0x00FF));
		LCD_WR_DATA((y+LCD_Height-1)>>8); LCD_WR_DATA((y+LCD_Height-1)& 0x00FF);
	
	}else if(0){  //Reserved for other lcd driver
	
	}
	
}

void LCD_SetCursor(uint16_t x, uint16_t y){

	LCD_SetWindow(x,y,1,1);

}

void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t RGB_code){
	LCD_SetCursor(x, y);
	LCD_WriteRAM_Prepare();
	LCD_WR_DATA(RGB_code);
}

void LCD_Fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t RGB_code){
	
	uint16_t i, k;

	for(i=y1;i<=y2;i++){
		LCD_SetCursor(x1, i);
		LCD_WriteRAM_Prepare();
		for(k=x1;k<=x2;k++) LCD_WR_DATA(RGB_code);
		delay_ms(200);
	}

}

void LCD_Clear(uint16_t RGB_code){
	
	uint32_t Total_Pixel, i;
	
	Total_Pixel = LCD_Handle.height * LCD_Handle.width;
	
	LCD_SetCursor(0, 0);
	LCD_WriteRAM_Prepare();
	for(i=0;i<Total_Pixel;i++){
		LCD_WR_DATA(RGB_code);
		
	}
}

void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t RGB_code){
	int Distance_x, Distance_y, Distance;
	int x,y,xerr=0,yerr=0;
	int inc_x, inc_y, t;
	
	Distance_x = x2-x1;
	Distance_y = y2-y1;
	
	x=x1;
	y=y1;
	
	if(Distance_x>0) inc_x=1; 
	else if(Distance_x<0) { inc_x=-1; Distance_x = -Distance_x;}
	else inc_x=0;
	
	if(Distance_y>0) inc_y=1;
	else if(Distance_y<0){ inc_y=-1; Distance_y = -Distance_y;}
	else inc_y=0;
	
	
	if(Distance_x > Distance_y) Distance = Distance_x;
	else Distance = Distance_y;
	
	for(t=0;t<=Distance+1;t++){
		
		LCD_DrawPixel(x,y,RGB_code);
		xerr += Distance_x;
		yerr += Distance_y;
		
		if(xerr>Distance){
			xerr-=Distance;
			x+=inc_x;
		}
		if(yerr>Distance){
			yerr-=Distance;
			y+=inc_y;
		}
	
	}

}

void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t size, uint8_t asc2_code){
	
	uint8_t temp, csize;
	uint16_t x0=x;
	asc2_code = asc2_code-' ';
	csize = ((size/2)/8+(((size/2)%8)?1:0))*size;
	for(uint8_t i=0;i<csize;i++){
		//if(size == 16) temp= asc2_1206[asc2_code][i];
		//else if(size == 16) temp= asc2_1608[asc2_code][i];
		if(size == 24) temp= asc2_2412[asc2_code][i];
		//else if(size == 32) temp= asc2_3216[asc2_code][i];
		for(uint8_t k=0;k<8;k++){
			if((temp<<k) & 0x80) LCD_DrawPixel(x,y,RED);
			else LCD_DrawPixel(x,y,BLACK);
			x++;
			if((x-x0)==(size/2)){
				y++;
				x=x0;
				break;
			}
					
		}
		
	}
	
}



void TFT_LCD_Init(void){
	
	FSMC_NORSRAM_TimingTypeDef Timing = {0};
  FSMC_NORSRAM_TimingTypeDef ExtTiming = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */
	__HAL_RCC_FSMC_CLK_ENABLE();
  /** Perform the SRAM4 memory initialization sequence
  */
	FSMC_NORSRAM_TimingTypeDef FSMC_ReadWriteTim;
	FSMC_NORSRAM_TimingTypeDef FSMC_WriteTim;
	TFTSRAM_Handler.Instance=FSMC_NORSRAM_DEVICE;                
	TFTSRAM_Handler.Extended=FSMC_NORSRAM_EXTENDED_DEVICE;    
    
	TFTSRAM_Handler.Init.NSBank=FSMC_NORSRAM_BANK4;     				
	TFTSRAM_Handler.Init.DataAddressMux=FSMC_DATA_ADDRESS_MUX_DISABLE; 	
	TFTSRAM_Handler.Init.MemoryType=FSMC_MEMORY_TYPE_SRAM;   			//SRAM
	TFTSRAM_Handler.Init.MemoryDataWidth=FSMC_NORSRAM_MEM_BUS_WIDTH_16; 
	TFTSRAM_Handler.Init.BurstAccessMode=FSMC_BURST_ACCESS_MODE_DISABLE; 
	TFTSRAM_Handler.Init.WaitSignalPolarity=FSMC_WAIT_SIGNAL_POLARITY_LOW;
	TFTSRAM_Handler.Init.WaitSignalActive=FSMC_WAIT_TIMING_BEFORE_WS;   
	TFTSRAM_Handler.Init.WriteOperation=FSMC_WRITE_OPERATION_ENABLE;    
	TFTSRAM_Handler.Init.WaitSignal=FSMC_WAIT_SIGNAL_DISABLE;           
	TFTSRAM_Handler.Init.ExtendedMode=FSMC_EXTENDED_MODE_ENABLE;        
	TFTSRAM_Handler.Init.AsynchronousWait=FSMC_ASYNCHRONOUS_WAIT_DISABLE;
	TFTSRAM_Handler.Init.WriteBurst=FSMC_WRITE_BURST_DISABLE;           
    

	FSMC_ReadWriteTim.AddressSetupTime=0x06;       	
	FSMC_ReadWriteTim.AddressHoldTime=0;
	FSMC_ReadWriteTim.DataSetupTime=26;				
	FSMC_ReadWriteTim.AccessMode=FSMC_ACCESS_MODE_A;

	FSMC_WriteTim.BusTurnAroundDuration=0;			
	FSMC_WriteTim.AddressSetupTime=3;          		
	FSMC_WriteTim.AddressHoldTime=0;
	FSMC_WriteTim.DataSetupTime=0x06;              	
	FSMC_WriteTim.AccessMode=FSMC_ACCESS_MODE_A;    
	HAL_SRAM_Init(&TFTSRAM_Handler,&FSMC_ReadWriteTim,&FSMC_WriteTim);	


	
	GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  
	__HAL_RCC_GPIOD_CLK_ENABLE();			
	__HAL_RCC_GPIOE_CLK_ENABLE();			
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
  /** FSMC GPIO Configuration
  PG0   ------> FSMC_A10
  PE7   ------> FSMC_D4
  PE8   ------> FSMC_D5
  PE9   ------> FSMC_D6
  PE10   ------> FSMC_D7
  PE11   ------> FSMC_D8
  PE12   ------> FSMC_D9
  PE13   ------> FSMC_D10
  PE14   ------> FSMC_D11
  PE15   ------> FSMC_D12
  PD8   ------> FSMC_D13
  PD9   ------> FSMC_D14
  PD10   ------> FSMC_D15
  PD14   ------> FSMC_D0
  PD15   ------> FSMC_D1
  PD0   ------> FSMC_D2
  PD1   ------> FSMC_D3
  PD4   ------> FSMC_NOE
  PD5   ------> FSMC_NWE
  PG12   ------> FSMC_NE4
  */
  /* GPIO_InitStruct */
	GPIO_InitStruct.Pin=GPIO_PIN_0;          	//for the backlight
	GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;  
	GPIO_InitStruct.Pull=GPIO_PULLUP;          
	GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB,&GPIO_InitStruct); 
	
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* GPIO_InitStruct */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* GPIO_InitStruct */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  /** Disconnect NADV
  */
;	delay_ms(50);
	
	//Start LCD Init !
	
	LCD_WR_REG(0x00D3);
	LCD_Handle.id = LCD_RD_DATA(); //dummy code!
	LCD_Handle.id = LCD_RD_DATA(); //code = 0x00
	LCD_Handle.id |= (LCD_RD_DATA()<<8);
	LCD_Handle.id |= LCD_RD_DATA();
	if(LCD_Handle.id != 0x9341){ 
	
	
	}
	printf("LCD is %x\r\n", LCD_Handle.id); //send LCD ID
	
	if(LCD_Handle.id == 0x9341){
		LCD_WR_REG(0xCF);  
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0xC1); 
		LCD_WR_DATA(0X30); 
		LCD_WR_REG(0xED);  
		LCD_WR_DATA(0x64); 
		LCD_WR_DATA(0x03); 
		LCD_WR_DATA(0X12); 
		LCD_WR_DATA(0X81); 
		LCD_WR_REG(0xE8);  
		LCD_WR_DATA(0x85); 
		LCD_WR_DATA(0x10); 
		LCD_WR_DATA(0x7A); 
		LCD_WR_REG(0xCB);  
		LCD_WR_DATA(0x39); 
		LCD_WR_DATA(0x2C); 
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x34); 
		LCD_WR_DATA(0x02); 
		LCD_WR_REG(0xF7);  
		LCD_WR_DATA(0x20); 
		LCD_WR_REG(0xEA);  
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x00); 
		LCD_WR_REG(0xC0);    //Power control 
		LCD_WR_DATA(0x1B);   //VRH[5:0] 
		LCD_WR_REG(0xC1);    //Power control 
		LCD_WR_DATA(0x01);   //SAP[2:0];BT[3:0] 
		LCD_WR_REG(0xC5);    //VCM control 
		LCD_WR_DATA(0x30); 	 //3F
		LCD_WR_DATA(0x30); 	 //3C
		LCD_WR_REG(0xC7);    //VCM control2 
		LCD_WR_DATA(0XB7); 
		LCD_WR_REG(0x36);    // Memory Access Control 
		LCD_WR_DATA(0x48); 
		LCD_WR_REG(0x3A);   
		LCD_WR_DATA(0x55); 
		LCD_WR_REG(0xB1);   
		LCD_WR_DATA(0x00);   
		LCD_WR_DATA(0x1A); 
		LCD_WR_REG(0xB6);    // Display Function Control 
		LCD_WR_DATA(0x0A); 
		LCD_WR_DATA(0xA2); 
		LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
		LCD_WR_DATA(0x00); 
		LCD_WR_REG(0x26);    //Gamma curve selected 
		LCD_WR_DATA(0x01); 
		LCD_WR_REG(0xE0);    //Set Gamma 
		LCD_WR_DATA(0x0F); 
		LCD_WR_DATA(0x2A); 
		LCD_WR_DATA(0x28); 
		LCD_WR_DATA(0x08); 
		LCD_WR_DATA(0x0E); 
		LCD_WR_DATA(0x08); 
		LCD_WR_DATA(0x54); 
		LCD_WR_DATA(0XA9); 
		LCD_WR_DATA(0x43); 
		LCD_WR_DATA(0x0A); 
		LCD_WR_DATA(0x0F); 
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x00); 		 
		LCD_WR_REG(0XE1);    //Set Gamma 
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x15); 
		LCD_WR_DATA(0x17); 
		LCD_WR_DATA(0x07); 
		LCD_WR_DATA(0x11); 
		LCD_WR_DATA(0x06); 
		LCD_WR_DATA(0x2B); 
		LCD_WR_DATA(0x56); 
		LCD_WR_DATA(0x3C); 
		LCD_WR_DATA(0x05); 
		LCD_WR_DATA(0x10); 
		LCD_WR_DATA(0x0F); 
		LCD_WR_DATA(0x3F); 
		LCD_WR_DATA(0x3F); 
		LCD_WR_DATA(0x0F); 
		LCD_WR_REG(0x2B); 
		LCD_WR_DATA(0x00);
		LCD_WR_DATA(0x00);
		LCD_WR_DATA(0x01);
		LCD_WR_DATA(0x3f);
		LCD_WR_REG(0x2A); 
		LCD_WR_DATA(0x00);
		LCD_WR_DATA(0x00);
		LCD_WR_DATA(0x00);
		LCD_WR_DATA(0xef);	 
		LCD_WR_REG(0x11); //Exit Sleep
		delay_ms(120);
		LCD_WR_REG(0x29); //display on	
	
	}else if(0x00){  //Reserved for other lcd driver
	
	}
	
	
	LCD_SetDirection(vertical);
	LCD_LED=1;
	LCD_Clear(BLUE);
}



