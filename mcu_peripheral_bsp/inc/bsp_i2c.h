/**
  ******************************************************************************
  * @file    mpu6050.h
  * @author  NTUT MCA Lab Fryan Liu
  * @brief   Header file of I2C driver 
  ******************************************************************************
  */

#ifndef BSP_I2C_H
#define BSP_I2C_H

/* This is for STM32 HAL library */
#include "stm32f1xx_hal.h"  
#include "i2c.h"
/* ----------------------------- */

typedef enum
{
    I2C_OK = 0,
    I2C_ERROR = 1,
    I2C_TIMEOUT = 2
} I2C_StatusTypeDef;

/* Definition of i2c id */
#define I2C1_ID 1
#define I2C2_ID 2
#define I2C3_ID 3
/* End of definition */

#define I2C_TIMEOUT 1000 //ms

I2C_StatusTypeDef I2C_Init(uint8_t i2c_controller_num);
I2C_StatusTypeDef I2C_Write(uint8_t i2c_controller_num, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t size);
I2C_StatusTypeDef I2C_Read(uint8_t i2c_controller_num, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t size);

#endif