/**
  ******************************************************************************
  * @file    mpu6050.h
  * @author  NTUT Fryan Liu
  * @brief   Header file of I2C driver 
  ******************************************************************************
  */

 #include "bsp_i2c.h"


I2C_StatusTypeDef I2C_Init(uint8_t i2c_controller_num)
{
    switch (i2c_controller_num)
    {
        case I2C1_ID :
            MX_I2C1_Init();
            break;
        default:
            return I2C_ERROR;
    }

    return I2C_OK;
}

I2C_StatusTypeDef I2C_Write(uint8_t i2c_controller_num, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t size)
{
    uint8_t tx_data[size+1];
    tx_data[0] = reg_addr;
    for (int i = 0; i < size; i++)
    {
        tx_data[i+1] = data[i];
    }

    switch (i2c_controller_num)
    {
        case I2C1_ID :
            if(HAL_I2C_Master_Transmit(&hi2c1, dev_addr, tx_data, size+1, I2C_TIMEOUT) != HAL_OK)
            {
                return I2C_ERROR;
            }
            break;
        default:
            return I2C_ERROR;
    }

    return I2C_OK;
}


I2C_StatusTypeDef I2C_Read(uint8_t i2c_controller_num, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t size)
{   
    switch (i2c_controller_num)
    {
        case I2C1_ID :
            if(HAL_I2C_Master_Transmit(&hi2c1, dev_addr, &reg_addr, 1, I2C_TIMEOUT) != HAL_OK)
            {
                return I2C_ERROR;
            }

            if(HAL_I2C_Master_Receive(&hi2c1, dev_addr, data, size, I2C_TIMEOUT) != HAL_OK)
            {
                return I2C_ERROR;
            }
            break;
        default:
            return I2C_ERROR;
    }

    return I2C_OK;
}