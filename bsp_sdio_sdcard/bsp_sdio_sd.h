#ifndef __BSP_SDIO_SD_H
#define __BSP_SDIO_SD_H

#include "stm32f1xx.h"
#include "main.h"

#define SD_time_out 0xFFFFFF

extern SD_HandleTypeDef hsd;
extern HAL_SD_CardInfoTypeDef SD_Info;
extern DMA_HandleTypeDef hdma_SD;

uint8_t bsp_SD_Init(void);

uint8_t bsp_SD_Read(uint8_t *rxbuff,uint32_t block, uint32_t cnt);

uint8_t bsp_SD_Write(uint8_t *txbuff,uint32_t block, uint32_t cnt);

#endif