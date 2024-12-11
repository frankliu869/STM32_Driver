#include "bsp_flash_w25q.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

SPI_HandleTypeDef w25q_spi;

void flash_w25qxx_Init(void){
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	
	__HAL_RCC_GPIOE_CLK_ENABLE();

  	W25Q_SPIx_CLK_ENABLE();

  	W25Q_SPIx_GPIOx_CLK_ENABLE();

		
	GPIO_InitStruct.Pin =  W25Q_SPIx_CS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
		
		
	GPIO_InitStruct.Pin = W25Q_SPIx_SCK_PIN|W25Q_SPIx_MISI_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(W25Q_SPIx_GPIOx, &GPIO_InitStruct);

		
	GPIO_InitStruct.Pin = W25Q_SPIx_MISO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(W25Q_SPIx_GPIOx, &GPIO_InitStruct);

	W25Q_flash_CS_setHigh();
	
	w25q_spi.Instance = W25Q_SPIx;
	w25q_spi.Init.Mode = SPI_MODE_MASTER;
	w25q_spi.Init.Direction = SPI_DIRECTION_2LINES;
	w25q_spi.Init.DataSize = SPI_DATASIZE_8BIT;
	w25q_spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
	w25q_spi.Init.CLKPhase = SPI_PHASE_2EDGE;
	w25q_spi.Init.NSS = SPI_NSS_SOFT;
	w25q_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	w25q_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	w25q_spi.Init.TIMode = SPI_TIMODE_DISABLE;
	w25q_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	w25q_spi.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&w25q_spi) != HAL_OK)
	{
		Error_Handler();
	}
		__HAL_SPI_ENABLE(&w25q_spi);
}




unsigned char W25Q_flash_SendByte(unsigned char Txdata){
	int time_out_cnt = Time_out;
	uint8_t buff;
	time_out_cnt = Time_out;
	
	WRITE_REG(w25q_spi.Instance->DR,Txdata);
	
	while(__HAL_SPI_GET_FLAG(&w25q_spi,SPI_FLAG_TXE) == RESET){
		
		if((time_out_cnt--) == 0){
			flash_error_callback(Send_Error);
			break;
		}
	}
	
	while(__HAL_SPI_GET_FLAG(&w25q_spi,SPI_FLAG_RXNE)==RESET);
	
	return READ_REG(w25q_spi.Instance->DR);

}



unsigned char W25Q_flash_ReadByte(void){
	
	int time_out_cnt = Time_out;
	
	WRITE_REG(w25q_spi.Instance->DR,Dummy_cmd);
	
	while(__HAL_SPI_GET_FLAG(&w25q_spi,SPI_FLAG_TXE) == RESET){
		if((time_out_cnt--) == 0){
			
			return flash_error_callback(Read_Error);
			break;
		}
		
	}
	time_out_cnt = Time_out;
	while(__HAL_SPI_GET_FLAG(&w25q_spi,SPI_FLAG_RXNE)==RESET);
		/*if((time_out_cnt--) == 0){
			
			return flash_error_callback(Read_Error);
			break;
		}*/

	return READ_REG(w25q_spi.Instance->DR);
}


uint32_t W25Q_flash_ReadID(void){
	
	unsigned char temp[3]={0}, index=0xff;
	
	W25Q_flash_CS_setLow();
	
	W25Q_flash_SendByte(W25Q_JEDEC_ID);
	
	temp[0] = W25Q_flash_ReadByte();
	
	temp[1] = W25Q_flash_ReadByte();
	
	temp[2] = W25Q_flash_ReadByte();
	
	W25Q_flash_CS_setHigh();	

	return ((temp[0]<<16) | (temp[1]<<8) | (temp[2]));
}


void W25Q_flash_WaitBusyToEnd(void){
	
	unsigned char state=0;
	int time_out_cnt = Time_out;
	
	W25Q_flash_CS_setLow();
	
	W25Q_flash_SendByte(W25Q_Read_State_REG1);
	
	do{
		state = W25Q_flash_ReadByte();
		if((time_out_cnt--) == 0){
			flash_error_callback(EraseWrite_Error);
			break;
		}
	}while((state & W25Q_Busy_Flag) == SET);
	
	W25Q_flash_CS_setHigh();
}


void W25Q_flash_WriteENABLE(void){
	
	W25Q_flash_CS_setLow();
	
	W25Q_flash_SendByte(W25Q_Write_Enable);
	
	W25Q_flash_CS_setHigh();
}


unsigned char W25Q_flash_Read(unsigned char *Rxdata,int addr,short int data_size){
	
	int i;
	
	W25Q_flash_WriteENABLE();
	W25Q_flash_WaitBusyToEnd();
	
	W25Q_flash_CS_setLow();
	
	W25Q_flash_SendByte(W25Q_Read_Data);
	
	W25Q_flash_SendByte((addr & 0xFF0000) >>16);
	
	W25Q_flash_SendByte((addr & 0x00FF00) >>8);
	
	W25Q_flash_SendByte((addr & 0x0000FF));
	
	for(i=0;i<data_size;i++)  *(Rxdata+i) = W25Q_flash_ReadByte();
	
	W25Q_flash_CS_setHigh();
	
	return 0;
}



void W25Q_flash_Sector_Erase(uint32_t addr){
	
	W25Q_flash_WriteENABLE();
	W25Q_flash_WaitBusyToEnd();
	
	W25Q_flash_CS_setLow();
	
	W25Q_flash_SendByte(W25Q_Sector_Erase);
	
	W25Q_flash_SendByte((addr & 0xFF0000) >>16);
	
	W25Q_flash_SendByte((addr & 0x00FF00) >>8);
	
	W25Q_flash_SendByte((addr & 0x0000FF));
	
	W25Q_flash_CS_setHigh();
	
	
	W25Q_flash_WaitBusyToEnd();
	
}

unsigned char W25Q_flash_pageWrite(unsigned char *txdata ,int addr, short int data_size){
	
	int i;
	
	
	W25Q_flash_WriteENABLE();
	W25Q_flash_WaitBusyToEnd();
	
	W25Q_flash_CS_setLow();
		
	W25Q_flash_SendByte(W25Q_Page_Program);
		
	W25Q_flash_SendByte(addr>>16);
	W25Q_flash_SendByte(addr>>8);
	W25Q_flash_SendByte(addr);
	
	for(i=0;i<data_size;i++) 	W25Q_flash_SendByte(*(txdata+i));
	
	W25Q_flash_CS_setHigh();
	
	W25Q_flash_WaitBusyToEnd();
	
}

unsigned char W25Q_flash_checkWrite(unsigned char *Txdata ,int Addr, short int Data_Size){
	
	short int page_remainning_size, HowMany_page=1, pos_offset=0;
	
	page_remainning_size = 256-Addr%256;
	
	if(Data_Size > page_remainning_size){
		HowMany_page += (Data_Size - page_remainning_size)/256;
		if((Data_Size - page_remainning_size)%256) HowMany_page++;
	}else if(Data_Size<=page_remainning_size) page_remainning_size = Data_Size;
	
	while(HowMany_page > 0){
		
		W25Q_flash_pageWrite(Txdata+pos_offset,Addr+pos_offset,page_remainning_size);
		
		HowMany_page--;
		
		if(HowMany_page != 0 ){
			pos_offset += page_remainning_size;
			Data_Size = Data_Size -page_remainning_size;
			Data_Size = abs(Data_Size);
			if(Data_Size>=256) page_remainning_size=256;
			else page_remainning_size = Data_Size;
		}		
	}
	

}

unsigned char RxBuff_1[4096];
unsigned char W25Q_flash_sectorWrite(unsigned char *txdata ,int addr, short int data_size){
	
	int pos_sector, pos_offset , remaining_space,i;
	unsigned char HowMany_sector=1;
	unsigned char *RxBuff;
	RxBuff = RxBuff_1;
	pos_sector = addr/4096.0;
	pos_offset = addr%4096;
	remaining_space = 4096 - pos_offset;
	
	if(data_size>remaining_space){
		HowMany_sector += (data_size - remaining_space)/4096.0;
		if((data_size - remaining_space)%4096) HowMany_sector+=1;
	}else if(data_size<=remaining_space) remaining_space = data_size;
	
	while( HowMany_sector>0){
		
		W25Q_flash_Read(RxBuff,pos_sector*4096,4096);
		for(i=0;i<remaining_space;i++) if(RxBuff[pos_offset+i] != 0xFF) break;
		if(i<remaining_space) W25Q_flash_Sector_Erase(pos_sector * 4096);
		
		for(i=0;i<remaining_space;i++) RxBuff[i+pos_offset] = *(txdata+i);
		printf("W25Q128 �yԇ\n\r");
		W25Q_flash_checkWrite(RxBuff,pos_sector*4096,4096);

		HowMany_sector--;
		if(HowMany_sector>0){
			txdata += remaining_space;
			pos_sector++;
			pos_offset=0;
			data_size -= remaining_space;
			data_size = abs(data_size);
			if(data_size>=4096) remaining_space=4096;
			else remaining_space = data_size;
			
		}
	}
}

uint8_t bsp_flash_write(uint8_t *buff,int sector, int NumOfsector){
	int addr;
	NumOfsector = sector+NumOfsector;
	for(uint32_t i=sector;i<NumOfsector;i++){
		addr = i * 4096;
		W25Q_flash_Sector_Erase(i * 4096);
		for(uint8_t k=0;k<16;k++){
			W25Q_flash_pageWrite(buff,addr,256);
			buff+=256;
			addr+=256;
		}
	}
}



unsigned char flash_error_callback(unsigned char code){
	
	while(1);
	return 0;
}










