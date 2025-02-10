/**
  ******************************************************************************
  * @file    mpu6050.h
  * @author  NTUT Fryan Liu
  * @brief   Header file of MPU6050 driver  
  ******************************************************************************
  */

#include "mpu6050.h"

static MPU6050_StatusTypeDef MPU6050_Reset(void)
{
  uint8_t tx_data = 0, rx_data = 0;
  int i;
  if(I2C_Write(MPU6050_I2C_ID, MPU6050_ADDRESS, REG_PWR_MGMT_1, &tx_data, 1) != I2C_OK)
  {
    return MPU6050_I2C_WRITE_ERROR;
  }
  else
  {
    delay_ms(100);
    for(i = 0; i < 10; i++)
    {
      rx_data = 0;
      if(I2C_Read(MPU6050_I2C_ID, MPU6050_ADDRESS, REG_PWR_MGMT_1, &rx_data, 1) != I2C_OK)
      {
        return MPU6050_I2C_READ_ERROR;
      }
      else
      {
         if((rx_data & 0x80) == 0)
         {
           break;
         }
         else
         {
           delay_ms(100);
         }
      }

      if(i == 10)
      {
        return MPU6050_TIMEOUT;
      }
    }
  }

  return MPU6050_OK;
}

static MPU6050_StatusTypeDef MPU6050_SelfTest(IMU_Data_Frame *imu_data_frame)
{
  uint8_t tx_data = 0;
  uint8_t raw_data[4];
  uint8_t self_test_data[6];
  float factory_trim[6];
  int i = 0;

  tx_data = 0xF0;
  if(I2C_Write(MPU6050_I2C_ID, MPU6050_ADDRESS, REG_ACCEL_CONFIG, &tx_data, 1) != I2C_OK)
  {
    return MPU6050_I2C_WRITE_ERROR;
  }
  
  tx_data = 0xE0;
  if(I2C_Write(MPU6050_I2C_ID, MPU6050_ADDRESS, REG_GYRO_CONFIG, &tx_data, 1) != I2C_OK)
  {
    return MPU6050_I2C_WRITE_ERROR;
  }
  delay_ms(250);

  for(i = 0; i < 4; i++)
  {
    if(I2C_Read(MPU6050_I2C_ID, MPU6050_ADDRESS, REG_SELF_TEST_X + i, &raw_data[i], 1) != I2C_OK)
    {
      return MPU6050_I2C_READ_ERROR;
    }
  }

  self_test_data[0] = (raw_data[0] >> 3) | (raw_data[3] & 0x30) >> 4;
  self_test_data[1] = (raw_data[1] >> 3) | (raw_data[3] & 0x0C) >> 2;
  self_test_data[2] = (raw_data[2] >> 3) | (raw_data[3] & 0x03) >> 0;
  self_test_data[3] = raw_data[0] & 0x1F;
  self_test_data[4] = raw_data[1] & 0x1F;
  self_test_data[5] = raw_data[2] & 0x1F;

  factory_trim[0] = (4096.0 * 0.34) * (pow((0.92 / 0.34), ((self_test_data[0] - 1.0) / 30.0)));
  factory_trim[1] = (4096.0 * 0.34) * (pow((0.92 / 0.34), ((self_test_data[1] - 1.0) / 30.0)));
  factory_trim[2] = (4096.0 * 0.34) * (pow((0.92 / 0.34), ((self_test_data[2] - 1.0) / 30.0)));
  factory_trim[3] = (25.0 * 131.0) * (pow(1.046, (self_test_data[3] - 1.0)));
  factory_trim[4] = (-25.0 * 131.0) * (pow(1.046, (self_test_data[4] - 1.0)));
  factory_trim[5] = (25.0 * 131.0) * (pow(1.046, (self_test_data[5] - 1.0)));

  for(i = 0; i < 3; i++)
  {
    imu_data_frame->self_test_accel[i] = 100.0 + 100.0 * (self_test_data[i] - factory_trim[i]) / factory_trim[i];
  }

  for(i = 3; i < 6; i++)
  {
    imu_data_frame->self_test_gyro[i] = 100.0 + 100.0 * (self_test_data[i] - factory_trim[i]) / factory_trim[i];
  }

  return MPU6050_OK;
}

static MPU6050_StatusTypeDef MPU6050_Calibrate(IMU_Data_Frame *imu_data_frame)
{
  const uint8_t packet_count = 200;
  uint8_t tx_data = 0;

  int i = 0;
  uint8_t raw_data[6];
  int32_t accel_reg_bias[3] = {0, 0, 0}, gyro_reg_bias[3] = {0, 0, 0};
  for(i=0; i<packet_count; i++)
  {
    
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    while(MPU6050_GetDataStatus() != MPU6050_DATA_READY){}

    if(I2C_Read(MPU6050_I2C_ID, MPU6050_ADDRESS, REG_ACCEL_XOUT_H, raw_data, 6) != I2C_OK)
    {
      return MPU6050_I2C_READ_ERROR;
    }

    accel_temp[0] = (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);
    accel_temp[1] = (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
    accel_temp[2] = (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

    if(I2C_Read(MPU6050_I2C_ID, MPU6050_ADDRESS, REG_GYRO_XOUT_H, raw_data, 6) != I2C_OK)
    {
      return MPU6050_I2C_READ_ERROR;
    }

    gyro_temp[0] = (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);
    gyro_temp[1] = (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
    gyro_temp[2] = (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

    accel_reg_bias[0] += (int32_t)accel_temp[0];
    accel_reg_bias[1] += (int32_t)accel_temp[1];
    accel_reg_bias[2] += (int32_t)accel_temp[2];
    gyro_reg_bias[0] += (int32_t)gyro_temp[0];
    gyro_reg_bias[1] += (int32_t)gyro_temp[1];
    gyro_reg_bias[2] += (int32_t)gyro_temp[2];
  }
  
  accel_reg_bias[0] /= (int32_t)packet_count;
  accel_reg_bias[1] /= (int32_t)packet_count;
  accel_reg_bias[2] /= (int32_t)packet_count;
  gyro_reg_bias[0] /= (int32_t)packet_count;
  gyro_reg_bias[1] /= (int32_t)packet_count;
  gyro_reg_bias[2] /= (int32_t)packet_count;

  imu_data_frame->accel_bias[0] = (float)accel_reg_bias[0] * imu_data_frame->accel_res;
  imu_data_frame->accel_bias[1] = (float)accel_reg_bias[1] * imu_data_frame->accel_res;
  imu_data_frame->accel_bias[2] = (float)accel_reg_bias[2] * imu_data_frame->accel_res;

  if(imu_data_frame->accel_bias[2] > 0)
  {
    imu_data_frame->accel_bias[2] -= 1;
  }
  else
  {
    imu_data_frame->accel_bias[2] += 1;
  }

  imu_data_frame->gyro_bias[0] = (float)gyro_reg_bias[0] * imu_data_frame->gyro_res;
  imu_data_frame->gyro_bias[1] = (float)gyro_reg_bias[1] * imu_data_frame->gyro_res;
  imu_data_frame->gyro_bias[2] = (float)gyro_reg_bias[2] * imu_data_frame->gyro_res;
  
  return MPU6050_OK;
}

MPU6050_StatusTypeDef MPU6050_Init(IMU_Data_Frame *imu_data_frame)
{
  uint8_t tx_data = 0, rx_data = 0;
  int i = 0;
  float PI = 3.14159265358979323846f;
  float gyro_meas_error = PI * (60.0f / 180.0f);
  float gyro_meas_drift = PI * (1.0f / 180.0f);

  imu_data_frame->current_timestamp = 0;
  imu_data_frame->last_timestamp = 0;
  imu_data_frame->filter_beta = sqrt(3.0f / 4.0f) * gyro_meas_error;
  imu_data_frame->filter_zeta = sqrt(3.0f / 4.0f) * gyro_meas_drift;

  imu_data_frame->accel_data[0] = 0;
  imu_data_frame->accel_data[1] = 0;
  imu_data_frame->accel_data[2] = 0;
  imu_data_frame->accel_bias[0] = 0;
  imu_data_frame->accel_bias[1] = 0;
  imu_data_frame->accel_bias[2] = 0;
  imu_data_frame->accel_res = 2;
  imu_data_frame->gyro_data[0] = 0;
  imu_data_frame->gyro_data[1] = 0;
  imu_data_frame->gyro_data[2] = 0;
  imu_data_frame->gyro_bias[0] = 0;
  imu_data_frame->gyro_bias[1] = 0;
  imu_data_frame->gyro_bias[2] = 0;
  imu_data_frame->gyro_res = 250;
  imu_data_frame->quat[0] = 1;
  imu_data_frame->quat[1] = 0;
  imu_data_frame->quat[2] = 0;
  imu_data_frame->quat[3] = 0;

  if(I2C_Read(I2C1_ID, MPU6050_ADDRESS, REG_WHO_AM_I, &rx_data, sizeof(rx_data)) != I2C_OK)
  {
    printf("Read MPU6050 error !\n");
    return MPU6050_I2C_READ_ERROR;
  }
  else
  {
    rx_data = rx_data & 0x7D;
    printf("The Who i am register value : %x\n", rx_data);
  }

  if(MPU6050_SelfTest(imu_data_frame) != MPU6050_OK)
  {
    return MPU6050_ERROR;
  }
  else
  {
    if(imu_data_frame->self_test_accel[0] > 1 || imu_data_frame->self_test_accel[1] > 1 || imu_data_frame->self_test_accel[2] > 1)
    {
      return MPU6050_ERROR;
    }
    if(imu_data_frame->self_test_gyro[0] > 1 || imu_data_frame->self_test_gyro[1] > 1 || imu_data_frame->self_test_gyro[2] > 1)
    {
      return MPU6050_ERROR;
    }
  }

  if(MPU6050_Reset() != MPU6050_OK)
  {
    return MPU6050_ERROR;
  }

  tx_data = 0x01;
  if(I2C_Write(MPU6050_I2C_ID, MPU6050_ADDRESS, REG_PWR_MGMT_1, &tx_data, 1) != I2C_OK)
  {
    return MPU6050_I2C_WRITE_ERROR;
  }
  
  tx_data = 0x01;
  if(I2C_Write(MPU6050_I2C_ID, MPU6050_ADDRESS, REG_CONFIG, &tx_data, 1) != I2C_OK)
  {
    return MPU6050_I2C_WRITE_ERROR;
  }
  
  tx_data = 0x04;
  if(I2C_Write(MPU6050_I2C_ID, MPU6050_ADDRESS, REG_SMPLRT_DIV, &tx_data, 1) != I2C_OK)
  {
    return MPU6050_I2C_WRITE_ERROR;
  }
  
  
  for(i = 0; i < MPU6050_ACCEL_SCALE; i++){imu_data_frame->accel_res *= 2;}
  imu_data_frame->accel_res = imu_data_frame->accel_res / 32768.0;
  tx_data = 0x00 | (MPU6050_ACCEL_SCALE << 3);
  if(I2C_Write(MPU6050_I2C_ID, MPU6050_ADDRESS, REG_GYRO_CONFIG, &tx_data, 1) != I2C_OK)
  {
    return MPU6050_I2C_WRITE_ERROR;
  }
  
  for(i = 0; i < MPU6050_GYRO_SCALE; i++){imu_data_frame->gyro_res *= 2;}
  imu_data_frame->gyro_res = imu_data_frame->gyro_res / 32768.0;
  tx_data = 0x00 | (MPU6050_GYRO_SCALE << 3);
  if(I2C_Write(MPU6050_I2C_ID, MPU6050_ADDRESS, REG_ACCEL_CONFIG, &tx_data, 1) != I2C_OK)
  {
    return MPU6050_I2C_WRITE_ERROR;
  }
  
  tx_data = 0x22;
  if(I2C_Write(MPU6050_I2C_ID, MPU6050_ADDRESS, REG_INT_PIN_CFG, &tx_data, 1) != I2C_OK)
  {
    return MPU6050_I2C_WRITE_ERROR;
  }
  
  tx_data = 0x01;
  if(I2C_Write(MPU6050_I2C_ID, MPU6050_ADDRESS, REG_INT_ENABLE, &tx_data, 1) != I2C_OK)
  {
    return MPU6050_I2C_WRITE_ERROR;
  }

  if(MPU6050_Calibrate(imu_data_frame) != MPU6050_OK)
  {
    return MPU6050_ERROR;
  }
  
  return MPU6050_OK;
}

MPU6050_StatusTypeDef MPU6050_ReadAccel(IMU_Data_Frame *imu_data_frame)
{
  uint8_t raw_data[6];
  if(I2C_Read(MPU6050_I2C_ID, MPU6050_ADDRESS, REG_ACCEL_XOUT_H, raw_data, 6) != I2C_OK)
  {
    return MPU6050_I2C_READ_ERROR;
  }
  
  imu_data_frame->accel_data[0] = (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]) * imu_data_frame->accel_res;
  imu_data_frame->accel_data[1] = (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]) * imu_data_frame->accel_res;
  imu_data_frame->accel_data[2] = (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]) * imu_data_frame->accel_res;
  imu_data_frame->accel_data[0] -= imu_data_frame->accel_bias[0];
  imu_data_frame->accel_data[1] -= imu_data_frame->accel_bias[1];
  imu_data_frame->accel_data[2] -= imu_data_frame->accel_bias[2];
  return MPU6050_OK;
}

MPU6050_StatusTypeDef MPU6050_ReadGyro(IMU_Data_Frame *imu_data_frame)
{
  uint8_t raw_data[6];
  if(I2C_Read(MPU6050_I2C_ID, MPU6050_ADDRESS, REG_GYRO_XOUT_H, raw_data, 6) != I2C_OK)
  {
    return MPU6050_I2C_READ_ERROR;
  }
  
  imu_data_frame->gyro_data[0] = (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]) * imu_data_frame->gyro_res;
  imu_data_frame->gyro_data[1] = (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]) * imu_data_frame->gyro_res;
  imu_data_frame->gyro_data[2] = (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]) * imu_data_frame->gyro_res;
  imu_data_frame->gyro_data[0] -= imu_data_frame->gyro_bias[0];
  imu_data_frame->gyro_data[1] -= imu_data_frame->gyro_bias[1];
  imu_data_frame->gyro_data[2] -= imu_data_frame->gyro_bias[2];
  return MPU6050_OK;
  
}

MPU6050_StatusTypeDef MPU6050_UpdataTimestamp(IMU_Data_Frame *imu_data_frame)
{
  imu_data_frame->last_timestamp = imu_data_frame->current_timestamp;
  imu_data_frame->current_timestamp = HAL_GetTick();
  if(imu_data_frame->current_timestamp < imu_data_frame->last_timestamp)
  {
    imu_data_frame->delta_t = imu_data_frame->current_timestamp + ((uint32_t)4294967295 - imu_data_frame->last_timestamp);
  }
  else
  {
    imu_data_frame->delta_t = imu_data_frame->current_timestamp - imu_data_frame->last_timestamp;
  }

  imu_data_frame->delta_t /= 1000;

  return MPU6050_OK;
}

MPU6050_StatusTypeDef MPU6050_UpdateOrientation(IMU_Data_Frame *imu_data_frame)
{
  float q1 = imu_data_frame->quat[0], q2 = imu_data_frame->quat[1], q3 = imu_data_frame->quat[2], q4 = imu_data_frame->quat[3];         // short name local variable for readability
  float ax = imu_data_frame->accel_data[0], ay = imu_data_frame->accel_data[1], az = imu_data_frame->accel_data[2];         // short name local variable for readability
  float gx = imu_data_frame->gyro_data[0], gy = imu_data_frame->gyro_data[1], gz = imu_data_frame->gyro_data[2];         // short name local variable for readability
  float beta = imu_data_frame->filter_beta;                                
  float zeta = imu_data_frame->filter_zeta;      
  float deltat = imu_data_frame->delta_t;                         
  float norm;                                               // vector norm
  float f1, f2, f3;                                         // objective funcyion elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;  // gyro bias error
  gx = gx * M_PI / 180.0;
  gy = gy * M_PI / 180.0;
  gz = gz * M_PI / 180.0;
  // Auxiliary variables to avoid repeated arithmetic
  float _halfq1 = 0.5f * q1;
  float _halfq2 = 0.5f * q2;
  float _halfq3 = 0.5f * q3;
  float _halfq4 = 0.5f * q4;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  //            float _2q1q3 = 2.0f * q1 * q3;
  //            float _2q3q4 = 2.0f * q3 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return MPU6050_OK; // handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Compute the objective function and Jacobian
  f1 = _2q2 * q4 - _2q1 * q3 - ax;
  f2 = _2q1 * q2 + _2q3 * q4 - ay;
  f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
  J_11or24 = _2q3;
  J_12or23 = _2q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;

  // Compute the gradient (matrix multiplication)
  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
  hatDot4 = J_14or21 * f1 + J_11or24 * f2;

  // Normalize the gradient
  norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
  hatDot1 /= norm;
  hatDot2 /= norm;
  hatDot3 /= norm;
  hatDot4 /= norm;

  // Compute estimated gyroscope biases
  gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
  gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
  gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

  // Compute and remove gyroscope biases
  // gbiasx += gerrx * deltat * zeta;
  // gbiasy += gerry * deltat * zeta;
  // gbiasz += gerrz * deltat * zeta;
  //           gx -= gbiasx;
  //           gy -= gbiasy;
  //           gz -= gbiasz;

  // Compute the quaternion derivative
  qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
  qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
  qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
  qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

  // Compute then integrate estimated quaternion derivative
  q1 += (qDot1 -(beta * hatDot1)) * deltat;
  q2 += (qDot2 -(beta * hatDot2)) * deltat;
  q3 += (qDot3 -(beta * hatDot3)) * deltat;
  q4 += (qDot4 -(beta * hatDot4)) * deltat;

  // Normalize the quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f/norm;
  imu_data_frame->quat[0] = q1 * norm;
  imu_data_frame->quat[1] = q2 * norm;
  imu_data_frame->quat[2] = q3 * norm;
  imu_data_frame->quat[3] = q4 * norm;
            
}

MPU6050_StatusTypeDef MPU6050_ShowDataFrame(IMU_Data_Frame *imu_data_frame)
{
  // printf("Current Timestamp:            %f\n", imu_data_frame->current_timestamp);
  // printf("Last Timestamp:               %f\n", imu_data_frame->last_timestamp);
  printf("Dealt t:                      %f\n", imu_data_frame->delta_t);
  printf("Accel Data x, y, z:           %f, %f, %f\n", imu_data_frame->accel_data[0], imu_data_frame->accel_data[1], imu_data_frame->accel_data[2]);
  // printf("Accel Bias x, y, z:           %f, %f, %f\n", imu_data_frame->accel_bias[0], imu_data_frame->accel_bias[1], imu_data_frame->accel_bias[2]);
  // printf("Accel Self-test Data x, y, z: %f, %f, %f\n", imu_data_frame->self_test_accel[0], imu_data_frame->self_test_accel[1], imu_data_frame->self_test_accel[2]);
  // printf("Accel Resolution:             %f\n", imu_data_frame->accel_res);
  printf("Gyro Data x, y, z:            %f, %f, %f\n", imu_data_frame->gyro_data[0], imu_data_frame->gyro_data[1], imu_data_frame->gyro_data[2]);
  // printf("Gyro Bias x, y, z:            %f, %f, %f\n", imu_data_frame->gyro_bias[0], imu_data_frame->gyro_bias[1], imu_data_frame->gyro_bias[2]);
  // printf("Gyro Self-test Data x, y, z:  %f, %f, %f\n", imu_data_frame->self_test_gyro[0], imu_data_frame->self_test_gyro[1], imu_data_frame->self_test_gyro[2]);
  // printf("Gyro Resolution:              %f\n", imu_data_frame->gyro_res);
  printf("quat w, x, y, z:              %f, %f, %f, %f\n", imu_data_frame->quat[0], imu_data_frame->quat[1], imu_data_frame->quat[2], imu_data_frame->quat[3]);
  // printf("Madgwick Filter Beta:         %f\n", imu_data_frame->filter_beta);

  return MPU6050_OK;
}

MPU6050_StatusTypeDef MPU6050_TransmitDataFrame(IMU_Data_Frame *imu_data_frame)
{
  printf("time %d\ndelta%f\naccel%f %f %f\ngyro %f %f %f\nquat %f %f %f %f\n", imu_data_frame->current_timestamp, imu_data_frame->delta_t,
                         imu_data_frame->accel_data[0], imu_data_frame->accel_data[1], imu_data_frame->accel_data[2],
                         imu_data_frame->gyro_data[0], imu_data_frame->gyro_data[1], imu_data_frame->gyro_data[2],
                         imu_data_frame->quat[0], imu_data_frame->quat[1], imu_data_frame->quat[2], imu_data_frame->quat[3]);
  return MPU6050_OK;
}

MPU6050_StatusTypeDef MPU6050_GetDataStatus(void)
{
  uint8_t rx_data;
  if(I2C_Read(MPU6050_I2C_ID, MPU6050_ADDRESS, REG_INT_STATUS, &rx_data, 1) != I2C_OK)
  {
    return MPU6050_I2C_READ_ERROR;
  }
  else
  {
    rx_data = rx_data & 0x01;
    if(rx_data)
    {
      return MPU6050_DATA_READY;
    }
    else
    {
      return MPU6050_DATA_UNREADY;
    }
  }
  
  return MPU6050_DATA_UNREADY;
}

