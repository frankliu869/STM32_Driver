#include "bsp_sdio_sd.h"

SD_HandleTypeDef SD_hsd;
HAL_SD_CardInfoTypeDef SD_Info;
DMA_HandleTypeDef hdma_SD;

uint8_t bsp_SD_Init(void){
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	__HAL_RCC_SDIO_CLK_ENABLE();

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
	
	
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* SDIO DMA Init */
    /* SDIO Init */
  hdma_SD.Instance = DMA2_Channel4;
  hdma_SD.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_SD.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_SD.Init.MemInc = DMA_MINC_ENABLE;
  hdma_SD.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_SD.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_SD.Init.Mode = DMA_NORMAL;
	hdma_SD.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_SD) != HAL_OK)
    {
      return 1;
    }


	SD_hsd.Instance = SDIO;
  SD_hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  SD_hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  SD_hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  SD_hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  SD_hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
  SD_hsd.Init.ClockDiv = 4;
  if (HAL_SD_Init(&SD_hsd) != HAL_OK)
  {
    return 1;
  }
	
	HAL_SD_GetCardInfo(&SD_hsd,&SD_Info);
	
  if (HAL_SD_ConfigWideBusOperation(&SD_hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
  {
    return 1;
  }	
	
	__HAL_LINKDMA(&SD_hsd,hdmarx,hdma_SD);
  __HAL_LINKDMA(&SD_hsd,hdmatx,hdma_SD);

	HAL_NVIC_SetPriority(SDIO_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SDIO_IRQn);
	
	return 0;

}

uint8_t bsp_SD_Read(uint8_t *rxbuff,uint32_t block, uint32_t cnt){

	int time_out = SD_time_out;
	
	HAL_SD_ReadBlocks(&SD_hsd,rxbuff,block,cnt,time_out);
	
	while(HAL_SD_GetCardState(&SD_hsd) != HAL_SD_CARD_TRANSFER ){
	
		if((time_out--) <= 0) return 1;
	
	}
	
	 return 0;

}

uint8_t bsp_SD_Write(uint8_t *txbuff,uint32_t block, uint32_t cnt){
	
	int time_out = SD_time_out;
	
	HAL_SD_WriteBlocks(&SD_hsd,txbuff,block,cnt,time_out);
	
	while(HAL_SD_GetCardState(&SD_hsd) != HAL_SD_CARD_TRANSFER ){
	
		if((time_out--) <= 0) return 1;
	
	}
	
	 return 0;

}