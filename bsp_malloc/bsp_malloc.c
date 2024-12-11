#include "bsp_malloc.h"


__align(32) uint8_t Exram_Base[EXSRAM_MAX_SIZE] __attribute__ ((at (0X68000000)));
uint16_t Exram_StateTable[EXSRAM_NumOfTable] __attribute__ ((at (0X68000000+EXSRAM_MAX_SIZE)));


uint8_t Ram_Block_Size[Num_Of_Device] = {32};

uint16_t ram_NumOfTable[Num_Of_Device]={EXSRAM_NumOfTable};

uint8_t Init_State=0;

Ram_State my_ram={
	Exram_StateTable,
	Exram_Base,
};

void bsp_malloc_Init(void){
	for(uint8_t k=0;k<Num_Of_Device;k++){
		for(int i=0;i<EXSRAM_NumOfTable;i++) my_ram.Ram_Table[k][i]=0;
	}
	Init_State=1;
}


void *bsp_malloc(uint8_t device, uint16_t size){
	int i;
	uint16_t block_cnt=0, user_block=0;
	int *offset;
	
	if(Init_State!=1) bsp_malloc_Init();
	
	user_block = size/Ram_Block_Size[device];
	if(size%Ram_Block_Size[device]) user_block++;
	for(i=ram_NumOfTable[device]-1; i>=0 ;i--){
		if(my_ram.Ram_Table[device][i]==0) block_cnt++;
		else block_cnt=0;
		if(block_cnt == user_block){
			offset = (void *)(i * Ram_Block_Size[device] + my_ram.Ram_BaseAddress[device]);
			for(block_cnt=0;block_cnt<user_block;block_cnt++) my_ram.Ram_Table[device][i+block_cnt] =user_block;
			return offset;
		}
	}
	return (void *)0xFFFFFFFF;
}

uint8_t bsp_free(uint8_t device, void *p){
	
	uint32_t offset, block;
	offset = ((uint32_t)p - (uint32_t)my_ram.Ram_BaseAddress[device])/Ram_Block_Size[device];
	block = my_ram.Ram_Table[device][offset];
	for(int i=0;i<block;i++) my_ram.Ram_Table[device][offset+i] = 0;
	
	return 0;
}

