/**
  ******************************************************************************
  * @file    bsp_print.h
  * @author  NTUT Fryan Liu
  * @brief   Header file of print function
  ******************************************************************************
  */

#ifndef BSP_PRINT_H
#define BPS_PRINT_H 

//#include "usb_device.h" //This is for STM32 USB middleawares
#include "usart.h" //This is for STM32 USB middleawares
#include <stdio.h>
#include <string.h>

int _write(int file, char *ptr, int len)
{
    //while(CDC_Transmit_FS((uint8_t *)ptr, len) == USBD_BUSY);
    //HAL_UART_Transmit_DMA(&huart2, tx_data, sizeof(tx_data)-1);
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 0xFFFF);
    return len;
}

// #ifdef __GNUC__  
// #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)  
// #else  
// #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE* f)  
// #endif 
   
// #ifdef __cplusplus  
// extern "C" {  
// #endif
   
// PUTCHAR_PROTOTYPE  
// {  
//     HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
//     //CDC_Transmit_FS((uint8_t *)&ch, 1);
//     return (ch);   
// }  
   
// #ifdef __cplusplus  
// }  
//#endif 



#endif