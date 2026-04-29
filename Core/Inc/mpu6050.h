#ifndef MPU_MPU6050_H
#define MPU_MPU6050_H

#include "stm32f7xx_hal.h"

#define MPU_DEV_ADDR_0 (0x68 << 1)
#define MPU_DEV_ADDR_1 (0x69 << 1)

typedef struct
{
    I2C_HandleTypeDef   *hi2c;
    uint8_t             dev_addr;
} MPU_6050_HandleTypeDef;

typedef struct
{
    float accel_x, accel_y, accel_z;    /*!< in G */
    float temperature;                  /*!< in Celsius degrees */
    float gyro_x, gyro_y, gyro_z;       /*!< in degrees per second */
} MPU_6050_Data_TypeDef;

HAL_StatusTypeDef MPU_6050_Init(MPU_6050_HandleTypeDef *hmpu, I2C_HandleTypeDef *hi2c, uint8_t dev_addr);
HAL_StatusTypeDef MPU_6050_Read(const MPU_6050_HandleTypeDef *hmpu, MPU_6050_Data_TypeDef *hdata);

#endif //MPU_MPU6050_H