/**
  ******************************************************************************
  * @file    bsp_delay.h
  * @author  NTUT Fryan Liu
  * @brief   Header file of clock function
  ******************************************************************************
  */

 #ifndef BSP_CLOCK_H
 #define BSP_CLOCK_H

/* This is for STM32 HAL library */
#include "stm32f1xx_hal.h"
/* ----------------------------- */

float Clock_GetTimeStampUs(void);

 #endif