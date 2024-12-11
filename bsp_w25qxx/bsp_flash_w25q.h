#include "main.h"
#include "stm32f1xx_hal.h"



#ifndef __FLASH_W25QXX_H
#define __FLASH_W25QXX_H

//W25Qxx flash hardware define

#define W25Q_SPIx SPI1
#define W25Q_SPIx_CLK_ENABLE() __HAL_RCC_SPI1_CLK_ENABLE()
#define W25Q_SPIx_GPIOx_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define W25Q_SPIx_CS_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()

#define W25Q_SPIx_GPIOx GPIOA
#define W25Q_SPIx_CS_PIN GPIO_PIN_2
#define W25Q_SPIx_SCK_PIN GPIO_PIN_5
#define W25Q_SPIx_MISO_PIN GPIO_PIN_6
#define W25Q_SPIx_MISI_PIN GPIO_PIN_7

#define W25Q_flash_CS_setHigh() HAL_GPIO_WritePin(GPIOE,W25Q_SPIx_CS_PIN, GPIO_PIN_SET)
#define W25Q_flash_CS_setLow()  HAL_GPIO_WritePin(GPIOE,W25Q_SPIx_CS_PIN, GPIO_PIN_RESET)

#define W25Q_ID 0xef4018

#define Send_Error 0
#define Read_Error 1
#define EraseWrite_Error 2
#define Data_tooLarge 3
#define Time_out 65535


//W25Qxx flash command list

#define W25Q_Write_Enable 0x06
#define W25Q_Volatile_SR_Write Enable 0x50
#define W25Q_Write_Disable 0x04
#define W25Q_Read_State_REG1 0x05
#define W25Q_Read_State_REG2 0x35
#define W25Q_Read_State_REG3 0x15
#define W25Q_Write_state_REG1 0x01
#define W25Q_Write_state_REG2 0x31
#define W25Q_Write_state_REG3 0x11
#define W25Q_Chip_Erase 0xC7
#define W25Q_Erase_Program_Suspend 0x75
#define W25Q_Erase_Program_ Resume 0x7A
#define W25Q_Power_down 0xB9
#define W25Q_Global_Block_Lock 0x7E
#define W25Q_Global_Block_UnLock 0x98
#define W25Q_Enter_QPI_Mode 0x38
#define W25Q_Enable_Reset 0x66
#define W25Q_Reset_Device 0x99
#define W25Q_JEDEC_ID 0x9F
#define W25Q_Sector_Erase 0x20
#define W25Q_Read_Data 0x03
#define W25Q_Page_Program 0x02

#define W25Q_Busy_Flag 0x01
#define Dummy_cmd 0xFF


extern SPI_HandleTypeDef w25q_spi;

unsigned char flash_error_callback(unsigned char);

void flash_w25qxx_Init(void);

unsigned char W25Q_flash_SendByte(unsigned char);

unsigned char W25Q_flash_ReadByte(void);

void W25Q_flash_WaitBusyToEnd(void);

void W25Q_flash_WriteENABLE(void);

unsigned char W25Q_flash_Read(unsigned char *,int,short int);

uint32_t W25Q_flash_ReadID(void);

unsigned char W25Q_flash_pageWrite(unsigned char * ,int , short int);

unsigned char W25Q_flash_checkWrite(unsigned char *Txdata ,int Addr, short int Data_Size);

unsigned char W25Q_flash_sectorWrite(unsigned char *,int, short int);

void W25Q_flash_Sector_Erase(uint32_t);

uint8_t bsp_flash_write(uint8_t *buff,int sector, int NumOfsector);




#endif

