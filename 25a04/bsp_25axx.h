#ifndef __BSP_25AXX_H
#define __BSP_25AXX_H

#include "stm32f1xx_hal.h"
#include "main.h"
#include "delay.h"
#include "i2c.h"

#define bsp_25axx_WriteAddr 0xA0
#define bsp_25axx_ReadAddr 0xA1


//extern I2C_HandleTypeDef hi2c1;

void bsp_25axx_Init(void);

void bsp_25axx_ByteWrite(uint8_t addr, uint8_t *txdata, uint8_t txsize);

void bsp_25axx_PageWrite(uint8_t addr, uint8_t *txdata);

uint8_t bsp_25axx_RandomRead(uint8_t addr);

void bsp_25axx_SequentialRead(uint8_t addr, uint8_t *rxdata, uint8_t rxsize);


#endif