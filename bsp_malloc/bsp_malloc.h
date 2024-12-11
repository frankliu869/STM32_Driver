#ifndef __BSP_MALLOC_H
#define __BSP_MALLOC_H

#include "stm32f1xx_hal.h"

#define Num_Of_Device 1
#define INTERNAL_RAM 1
#define EXTERNAL_RAM 0
#define Com_BlockSize 32


#define EXSRAM_BASE_ADDRESS 
#define EXSRAM_MAX_SIZE 960 *1024
#define EXSRAM_BLOCK_SIZE 32
#define EXSRAM_TABLE_ADDRESS
#define EXSRAM_NumOfTable (960 *1024)/32

typedef struct{
	uint16_t *Ram_Table[Num_Of_Device];
	uint8_t *Ram_BaseAddress[Num_Of_Device];
}Ram_State;

extern Ram_State my_ram;

void bsp_malloc_Init(void);

void *bsp_malloc(uint8_t device, uint16_t size);

uint8_t bsp_free(uint8_t device, void *p);

#endif

