/**
  ******************************************************************************
  * @file    mpu6050.h
  * @author  Jakub Marszalek
  * @brief   Header for mpu6050.c file.
  *          This file contains the common defines and structures of the
  *          MPU-6050 6-axis IMU sensor driver (accelerometer + gyroscope).
  *          Communication is performed over I2C using STM32 HAL.
  ******************************************************************************
  * @attention
  *
  * This software is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MPU_MPU6050_H
#define MPU_MPU6050_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Exported constants --------------------------------------------------------*/
#define MPU_DEV_ADDR_0 (0x68 << 1)  /*!< I2C device address when AD0 pin is LOW  */
#define MPU_DEV_ADDR_1 (0x69 << 1)  /*!< I2C device address when AD0 pin is HIGH */

/** @defgroup MPU6050_Accel_Range Accelerometer full-scale range selection
  * @brief    Values to pass as accel_afs in MPU_6050_Init() and MPU_6050_SetAccelAfs().
  *           Correspond to AFS_SEL bits [4:3] of the ACCEL_CONFIG register (0x1C).
  * @{
  */
#define ACCEL_AFS_2    (0x00 << 3)  /*!< ±2g  — sensitivity: 16384 LSB/g */
#define ACCEL_AFS_4    (0x01 << 3)  /*!< ±4g  — sensitivity:  8192 LSB/g */
#define ACCEL_AFS_8    (0x02 << 3)  /*!< ±8g  — sensitivity:  4096 LSB/g */
#define ACCEL_AFS_16   (0x03 << 3)  /*!< ±16g — sensitivity:  2048 LSB/g */
/** @} */

/** @defgroup MPU6050_Gyro_Range Gyroscope full-scale range selection
  * @brief    Values to pass as gyro_fs in MPU_6050_Init() and MPU_6050_SetGyroFs().
  *           Correspond to FS_SEL bits [4:3] of the GYRO_CONFIG register (0x1B).
  * @{
  */
#define GYRO_FS_250    (0x00 << 3)  /*!< ±250  °/s — sensitivity: 131.0 LSB/°/s */
#define GYRO_FS_500    (0x01 << 3)  /*!< ±500  °/s — sensitivity:  65.5 LSB/°/s */
#define GYRO_FS_1000   (0x02 << 3)  /*!< ±1000 °/s — sensitivity:  32.8 LSB/°/s */
#define GYRO_FS_2000   (0x03 << 3)  /*!< ±2000 °/s — sensitivity:  16.4 LSB/°/s */
/** @} */

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  MPU-6050 handle structure definition.
  * @note   Must be initialized with MPU_6050_Init() before use.
  *         Stores I2C context, sensor configuration, and calibration offsets.
  *         Offsets are set to 0.0 by MPU_6050_Init() and updated by MPU_6050_Calibrate().
  */
typedef struct
{
    I2C_HandleTypeDef *hi2c;            /*!< Pointer to the I2C handle used for communication          */
    uint8_t           dev_addr;         /*!< I2C device address (MPU_DEV_ADDR_0 or MPU_DEV_ADDR_1)     */
    float             accel_sens;       /*!< Accelerometer sensitivity in LSB/g (set by range config)  */
    float             gyro_sens;        /*!< Gyroscope sensitivity in LSB/°/s (set by range config)    */
    float             accel_offset_x;   /*!< Accelerometer X-axis offset in g (set by calibration)     */
    float             accel_offset_y;   /*!< Accelerometer Y-axis offset in g (set by calibration)     */
    float             accel_offset_z;   /*!< Accelerometer Z-axis offset in g (set by calibration)     */
    float             gyro_offset_x;    /*!< Gyroscope X-axis offset in °/s (set by calibration)       */
    float             gyro_offset_y;    /*!< Gyroscope Y-axis offset in °/s (set by calibration)       */
    float             gyro_offset_z;    /*!< Gyroscope Z-axis offset in °/s (set by calibration)       */
} MPU_6050_HandleTypeDef;

/**
  * @brief  MPU-6050 sensor data structure.
  * @note   Populated by MPU_6050_Read(). All values are calibrated (offsets applied).
  */
typedef struct
{
    float accel_x;  /*!< X-axis acceleration in g                */
    float accel_y;  /*!< Y-axis acceleration in g                */
    float accel_z;  /*!< Z-axis acceleration in g                */
    float temp;     /*!< Internal temperature in degrees Celsius */
    float gyro_x;   /*!< X-axis angular velocity in degrees/s    */
    float gyro_y;   /*!< Y-axis angular velocity in degrees/s    */
    float gyro_z;   /*!< Z-axis angular velocity in degrees/s    */
} MPU_6050_Data_TypeDef;

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef MPU_6050_Init(
        MPU_6050_HandleTypeDef *hmpu,
        I2C_HandleTypeDef *hi2c,
        uint8_t dev_addr,
        uint8_t accel_afs,
        uint8_t gyro_fs
    );
HAL_StatusTypeDef MPU_6050_SetAccelAfs(MPU_6050_HandleTypeDef *hmpu, uint8_t accel_afs);
HAL_StatusTypeDef MPU_6050_SetGyroFs(MPU_6050_HandleTypeDef *hmpu, uint8_t gyro_fs);
HAL_StatusTypeDef MPU_6050_Read(const MPU_6050_HandleTypeDef *hmpu, MPU_6050_Data_TypeDef *hdata);
HAL_StatusTypeDef MPU_6050_Calibrate(MPU_6050_HandleTypeDef *hmpu, uint16_t samples);

#endif /* MPU_MPU6050_H */