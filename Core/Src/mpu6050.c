#include "mpu6050.h"

#define PWR_MGMT_1_ADDR   (0x6B)

#define ACCEL_CONFIG_ADDR (0x1C)
#define ACCEL_AFS_2       (0x00 << 3)
#define ACCEL_AFS_4       (0x01 << 3)
#define ACCEL_AFS_8       (0x02 << 3)
#define ACCEL_AFS_16      (0x03 << 3)
#define ACCEL_AFS_2_SENS  (16384.0f)
#define ACCEL_AFS_4_SENS  (8192.0f)
#define ACCEL_AFS_8_SENS  (4096.0f)
#define ACCEL_AFS_16_SENS (2048.0f)

#define TEMP_DIVISOR      (340.0f)
#define TEMP_OFFSET       (36.53f)

#define GYRO_CONFIG_ADDR  (0x1B)
#define GYRO_FS_250       (0x00 << 3)
#define GYRO_FS_500       (0x01 << 3)
#define GYRO_FS_1000      (0x02 << 3)
#define GYRO_FS_2000      (0x03 << 3)
#define GYRO_FS_250_SENS  (131.0f)
#define GYRO_FS_500_SENS  (65.5f)
#define GYRO_FS_1000_SENS (32.8f)
#define GYRO_FS_2000_SENS (16.4f)

#define DATA_START_ADDR   (0x3B)

static int16_t raw_to_int16(const uint8_t raw_hsb, const uint8_t raw_lsb)
{
    return (int16_t)(raw_hsb << 8 | raw_lsb);
}

static float calculate_accel(const int16_t accel, const float sens)
{
    return (float)accel / sens;
}

static float calculate_temp(const int16_t temp)
{
    return (float)temp / TEMP_DIVISOR + TEMP_OFFSET;
}

static float calculate_ang_vel(const int16_t ang_vel, const float sens)
{
    return (float)ang_vel / sens;
}

HAL_StatusTypeDef MPU_6050_Init(MPU_6050_HandleTypeDef *hmpu, I2C_HandleTypeDef *hi2c, const uint8_t dev_addr)
{
    // Assign I2C handle and device address to sensor's handle structure
    hmpu->hi2c = hi2c;
    hmpu->dev_addr = dev_addr;
    HAL_StatusTypeDef ret = HAL_OK;

    // Check if device is ready
    ret = HAL_I2C_IsDeviceReady(hmpu->hi2c, hmpu->dev_addr, 10, 20);
    if (ret != HAL_OK) return ret;

    // Wake the sensor up by writing 0 to PWR_MGMT_1 address
    uint8_t byte = 0x00;
    ret = HAL_I2C_Mem_Write(
                hmpu->hi2c,
                hmpu->dev_addr,
                PWR_MGMT_1_ADDR,
                I2C_MEMADD_SIZE_8BIT,
                &byte,
                1,
                20
            );
    if (ret != HAL_OK) return ret;

    // Configure measuring ranges
    byte = GYRO_FS_250;
    ret = HAL_I2C_Mem_Write(
                hmpu->hi2c,
                hmpu->dev_addr,
                GYRO_CONFIG_ADDR,
                I2C_MEMADD_SIZE_8BIT,
                &byte,
                1,
                20
            );
    if (ret != HAL_OK) return ret;

    byte = ACCEL_AFS_2;
    ret = HAL_I2C_Mem_Write(
                hmpu->hi2c,
                hmpu->dev_addr,
                ACCEL_CONFIG_ADDR,
                I2C_MEMADD_SIZE_8BIT,
                &byte,
                1,
                20
            );

    return ret;
}

HAL_StatusTypeDef MPU_6050_Read(const MPU_6050_HandleTypeDef *hmpu, MPU_6050_Data_TypeDef *hdata)
{
    uint8_t raw_data[14];
    HAL_StatusTypeDef ret = HAL_OK;

    ret = HAL_I2C_Mem_Read(
            hmpu->hi2c,
            hmpu->dev_addr,
            DATA_START_ADDR,
            I2C_MEMADD_SIZE_8BIT,
            raw_data,
            14,
            20
        );
    if (ret != HAL_OK) return ret;

    hdata->accel_x     = calculate_accel(raw_to_int16(raw_data[0],  raw_data[1]), ACCEL_AFS_2_SENS);
    hdata->accel_y     = calculate_accel(raw_to_int16(raw_data[2],  raw_data[3]), ACCEL_AFS_2_SENS);
    hdata->accel_z     = calculate_accel(raw_to_int16(raw_data[4],  raw_data[5]), ACCEL_AFS_2_SENS);
    hdata->temperature = calculate_temp(raw_to_int16(raw_data[6],  raw_data[7]));
    hdata->gyro_x      = calculate_ang_vel(raw_to_int16(raw_data[8],  raw_data[9]), GYRO_FS_250_SENS);
    hdata->gyro_y      = calculate_ang_vel(raw_to_int16(raw_data[10],  raw_data[11]), GYRO_FS_250_SENS);
    hdata->gyro_z      = calculate_ang_vel(raw_to_int16(raw_data[12],  raw_data[13]), GYRO_FS_250_SENS);

    return ret;
}