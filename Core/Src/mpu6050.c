/**
  ******************************************************************************
  * @file    mpu6050.c
  * @author  Jakub Marszalek
  * @brief   MPU-6050 6-axis IMU sensor driver.
  *          Implements I2C communication with the MPU-6050 accelerometer and
  *          gyroscope. Provides initialization, range configuration, data
  *          reading, and offset-based calibration.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"
#include "math.h"

/* Private define ------------------------------------------------------------*/
#define PWR_MGMT_1_ADDR   (0x6B)        /*!< Power Management 1 register address      */
#define SLEEP_BIT         (1 << 6)      /*!< SLEEP bit (bit 6) of PWR_MGMT_1 register */

#define ACCEL_CONFIG_ADDR (0x1C)        /*!< Accelerometer configuration register     */
#define ACCEL_AFS_2_SENS  (16384.0f)    /*!< Sensitivity for ±2g range  in LSB/g      */
#define ACCEL_AFS_4_SENS  (8192.0f)     /*!< Sensitivity for ±4g range  in LSB/g      */
#define ACCEL_AFS_8_SENS  (4096.0f)     /*!< Sensitivity for ±8g range  in LSB/g      */
#define ACCEL_AFS_16_SENS (2048.0f)     /*!< Sensitivity for ±16g range in LSB/g      */

#define TEMP_DIVISOR      (340.0f)      /*!< Temperature conversion divisor           */
#define TEMP_OFFSET       (36.53f)      /*!< Temperature conversion offset            */

#define GYRO_CONFIG_ADDR  (0x1B)        /*!< Gyroscope configuration register         */
#define GYRO_FS_250_SENS  (131.0f)      /*!< Sensitivity for ±250  °/s in LSB/°/s     */
#define GYRO_FS_500_SENS  (65.5f)       /*!< Sensitivity for ±500  °/s in LSB/°/s     */
#define GYRO_FS_1000_SENS (32.8f)       /*!< Sensitivity for ±1000 °/s in LSB/°/s     */
#define GYRO_FS_2000_SENS (16.4f)       /*!< Sensitivity for ±2000 °/s in LSB/°/s     */

#define DATA_START_ADDR   (0x3B)        /*!< First data register (ACCEL_XOUT_H)       */

#define MPU_INIT_TRIALS   (10)          /*!< Number of trails for I2C ready check     */
#define MPU_I2C_TIMEOUT   (30)          /*!< I2C communication timeout in ms          */

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Combines two raw bytes into a signed 16-bit integer.
  * @param  raw_hsb  High byte of the 16-bit value.
  * @param  raw_lsb  Low byte of the 16-bit value.
  * @retval int16_t  Combined signed 16-bit value.
  */
static int16_t raw_to_int16(const uint8_t raw_hsb, const uint8_t raw_lsb)
{
    return (int16_t)(raw_hsb << 8 | raw_lsb);
}

/**
  * @brief  Reads one or more registers from the MPU-6050 over I2C.
  * @param  hi2c      Pointer to the I2C handle.
  * @param  dev_addr  I2C device address.
  * @param  reg_addr  Starting register address to read from.
  * @param  size      Number of bytes to read.
  * @param  reg       Pointer to buffer where read data will be stored.
  * @retval HAL_StatusTypeDef  HAL_OK on success, HAL_ERROR on invalid args or I2C failure.
  */
static HAL_StatusTypeDef mpu_read_register(
    I2C_HandleTypeDef *hi2c,
    const uint8_t dev_addr,
    const uint8_t reg_addr,
    const uint8_t size,
    uint8_t *reg
) {
    if (hi2c == NULL || reg == NULL || size == 0) return HAL_ERROR;

    const HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        hi2c,
        dev_addr,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        reg,
        size,
        MPU_I2C_TIMEOUT
    );
    return ret;
}

/**
  * @brief  Writes one or more registers to the MPU-6050 over I2C.
  * @param  hi2c      Pointer to the I2C handle.
  * @param  dev_addr  I2C device address.
  * @param  reg_addr  Starting register address to write to.
  * @param  size      Number of bytes to write.
  * @param  value     Pointer to buffer containing data to write.
  * @retval HAL_StatusTypeDef  HAL_OK on success, HAL_ERROR on invalid args or I2C failure.
  */
static HAL_StatusTypeDef mpu_write_register(
    I2C_HandleTypeDef *hi2c,
    const uint8_t dev_addr,
    const uint8_t reg_addr,
    const uint8_t size,
    const uint8_t *value
) {
    if (hi2c == NULL || value == NULL || size == 0) return HAL_ERROR;

    const HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(
        hi2c,
        dev_addr,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t*)value,
        size,
        MPU_I2C_TIMEOUT
    );
    return ret;
}

/**
  * @brief  Converts a raw accelerometer value to g units.
  * @param  accel  Raw signed 16-bit accelerometer reading.
  * @param  sens   Accelerometer sensitivity in LSB/g.
  * @retval float  Acceleration in g.
  */
static float calculate_accel(const int16_t accel, const float sens)
{
    return (float)accel / sens;
}

/**
  * @brief  Converts a raw temperature register value to degrees Celsius.
  * @note   Formula from MPU-6050 datasheet: Temp[°C] = raw / 340 + 36.53.
  *         This reflects the internal temperature, not ambient temperature.
  * @param  temp   Raw signed 16-bit temperature reading.
  * @retval float  Temperature in degrees Celsius.
  */
static float calculate_temp(const int16_t temp)
{
    return (float)temp / TEMP_DIVISOR + TEMP_OFFSET;
}

/**
  * @brief  Converts a raw gyroscope value to degrees per second.
  * @param  ang_vel  Raw signed 16-bit gyroscope reading.
  * @param  sens     Gyroscope sensitivity in LSB/°/s.
  * @retval float    Angular velocity in degrees per second.
  */
static float calculate_ang_vel(const int16_t ang_vel, const float sens)
{
    return (float)ang_vel / sens;
}

/**
  * @brief  Updates an angles structure with values calculated from accelerometer measurements.
  * @param  hangles Pointer to a MPU_6050_AnglesTypeDef structure.
  * @param  hdata   Pointer to a MPU_6050_DataTypeDef structure.
  * @note   This is an internal helper. Not exposed in the public API.
  *         Uses tri-axis formula for constant sensitivity over 360° of rotation.
  */
static void MPU_6050_UpdateAccelAngles(MPU_6050_AnglesTypeDef *hangles, const MPU_6050_DataTypeDef *hdata)
{
    const float acc_x = hdata->accel_x;
    const float acc_y = hdata->accel_y;
    const float acc_z = hdata->accel_z;

    const float norm_x = sqrtf(acc_x * acc_x + acc_z * acc_z);
    const float norm_y = sqrtf(acc_y * acc_y + acc_z * acc_z);
    if (norm_x < 1e-6f || norm_y < 1e-6f) return;

    hangles->accel_angle_x = atan2f(acc_y, sqrtf(acc_x * acc_x + acc_z * acc_z)) * (180.0f / (float)M_PI);
    hangles->accel_angle_y = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * (180.0f / (float)M_PI);
}

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Initializes the MPU-6050 sensor and configures measuring ranges.
  * @note   Assigns the I2C handle and device address to the sensor handle,
  *         resets all calibration offsets to 0.0, verifies device presence,
  *         wakes the sensor from sleep mode, and configures accelerometer
  *         and gyroscope full-scale ranges.
  * @param  hmpu             Pointer to a MPU_6050_HandleTypeDef structure.
  * @param  hi2c             Pointer to the I2C handle used for communication.
  * @param  dev_addr         I2C device address (MPU_DEV_ADDR_0 or MPU_DEV_ADDR_1).
  * @param  accel_afs        Accelerometer full-scale range (ACCEL_AFS_2..ACCEL_AFS_16).
  * @param  gyro_fs          Gyroscope full-scale range (GYRO_FS_250..GYRO_FS_2000).
  * @retval HAL_StatusTypeDef  HAL_OK on success, HAL_ERROR if device not found
  *                            or invalid range specified.
  */
HAL_StatusTypeDef MPU_6050_Init(
    MPU_6050_HandleTypeDef *hmpu,
    I2C_HandleTypeDef *hi2c,
    const uint8_t dev_addr,
    const uint8_t accel_afs,
    const uint8_t gyro_fs
) {
    if (hmpu == NULL || hi2c == NULL) return HAL_ERROR;

    HAL_StatusTypeDef ret = HAL_OK;

    hmpu->hi2c           = hi2c;
    hmpu->dev_addr       = dev_addr;
    hmpu->accel_offset_x = 0.0f;
    hmpu->accel_offset_y = 0.0f;
    hmpu->accel_offset_z = 0.0f;
    hmpu->gyro_offset_x  = 0.0f;
    hmpu->gyro_offset_y  = 0.0f;
    hmpu->gyro_offset_z  = 0.0f;

    ret = HAL_I2C_IsDeviceReady(hmpu->hi2c, hmpu->dev_addr, MPU_INIT_TRIALS, MPU_I2C_TIMEOUT);
    if (ret != HAL_OK) return ret;

    const uint8_t byte = 0x00;
    ret = mpu_write_register(hmpu->hi2c, hmpu->dev_addr, PWR_MGMT_1_ADDR, 1, &byte);
    if (ret != HAL_OK) return ret;

    HAL_Delay(20);

    ret = MPU_6050_SetAccelAfs(hmpu, accel_afs);
    if (ret != HAL_OK) return ret;

    ret = MPU_6050_SetGyroFs(hmpu, gyro_fs);
    return ret;
}

/**
  * @brief  Configures the accelerometer full-scale range.
  * @note   Updates the accel_sens field in the handle to match the selected range.
  *         Writes the AFS_SEL bits [4:3] of the ACCEL_CONFIG register (0x1C).
  * @param  hmpu       Pointer to a MPU_6050_HandleTypeDef structure.
  * @param  accel_afs  Accelerometer full-scale range (ACCEL_AFS_2..ACCEL_AFS_16).
  * @retval HAL_StatusTypeDef  HAL_OK on success, HAL_ERROR if invalid range.
  */
HAL_StatusTypeDef MPU_6050_SetAccelAfs(MPU_6050_HandleTypeDef *hmpu, const uint8_t accel_afs)
{
    HAL_StatusTypeDef ret = HAL_OK;

    const uint8_t afs = accel_afs;
    switch (afs) {
        case ACCEL_AFS_2:
            hmpu->accel_sens = ACCEL_AFS_2_SENS;
            break;
        case ACCEL_AFS_4:
            hmpu->accel_sens = ACCEL_AFS_4_SENS;
            break;
        case ACCEL_AFS_8:
            hmpu->accel_sens = ACCEL_AFS_8_SENS;
            break;
        case ACCEL_AFS_16:
            hmpu->accel_sens = ACCEL_AFS_16_SENS;
            break;
        default:
            return HAL_ERROR;
    }

    ret = mpu_write_register(hmpu->hi2c, hmpu->dev_addr, ACCEL_CONFIG_ADDR, 1, &afs);
    return ret;
}

/**
  * @brief  Configures the gyroscope full-scale range.
  * @note   Updates the gyro_sens field in the handle to match the selected range.
  *         Writes the FS_SEL bits [4:3] of the GYRO_CONFIG register (0x1B).
  * @param  hmpu     Pointer to a MPU_6050_HandleTypeDef structure.
  * @param  gyro_fs  Gyroscope full-scale range (GYRO_FS_250..GYRO_FS_2000).
  * @retval HAL_StatusTypeDef  HAL_OK on success, HAL_ERROR if invalid range.
  */
HAL_StatusTypeDef MPU_6050_SetGyroFs(MPU_6050_HandleTypeDef *hmpu, const uint8_t gyro_fs)
{
    HAL_StatusTypeDef ret = HAL_OK;

    const uint8_t fs = gyro_fs;
    switch (fs) {
        case GYRO_FS_250:
            hmpu->gyro_sens = GYRO_FS_250_SENS;
            break;
        case GYRO_FS_500:
            hmpu->gyro_sens = GYRO_FS_500_SENS;
            break;
        case GYRO_FS_1000:
            hmpu->gyro_sens = GYRO_FS_1000_SENS;
            break;
        case GYRO_FS_2000:
            hmpu->gyro_sens = GYRO_FS_2000_SENS;
            break;
        default:
            return HAL_ERROR;
    }

    ret = mpu_write_register(hmpu->hi2c, hmpu->dev_addr, GYRO_CONFIG_ADDR, 1, &fs);
    return ret;
}

/**
  * @brief  Puts the MPU-6050 sensor into sleep mode.
  * @note   Sets the SLEEP bit (bit 6) of the PWR_MGMT_1 register (0x6B).
  *         In sleep mode the sensor stops updating data registers.
  *         I2C communication remains active.
  * @param  hmpu  Pointer to a MPU_6050_HandleTypeDef structure.
  * @retval HAL_StatusTypeDef  HAL_OK on success, HAL_ERROR on I2C failure.
  */
HAL_StatusTypeDef MPU_6050_Sleep(const MPU_6050_HandleTypeDef *hmpu)
{
    HAL_StatusTypeDef ret = HAL_OK;
    uint8_t reg = 0;

    ret = mpu_read_register(hmpu->hi2c, hmpu->dev_addr, PWR_MGMT_1_ADDR, 1, &reg);
    if (ret != HAL_OK) return ret;

    reg |= SLEEP_BIT;

    ret = mpu_write_register(hmpu->hi2c, hmpu->dev_addr, PWR_MGMT_1_ADDR, 1, &reg);
    return ret;
}

/**
  * @brief  Wakes the MPU-6050 sensor from sleep mode.
  * @note   Clears the SLEEP bit (bit 6) of the PWR_MGMT_1 register (0x6B).
  *         After waking, wait at least 20 ms before reading data registers.
  * @param  hmpu  Pointer to a MPU_6050_HandleTypeDef structure.
  * @retval HAL_StatusTypeDef  HAL_OK on success, HAL_ERROR on I2C failure.
  */
HAL_StatusTypeDef MPU_6050_Wake(const MPU_6050_HandleTypeDef *hmpu)
{
    HAL_StatusTypeDef ret = HAL_OK;
    uint8_t reg = 0;

    ret = mpu_read_register(hmpu->hi2c, hmpu->dev_addr, PWR_MGMT_1_ADDR, 1, &reg);
    if (ret != HAL_OK) return ret;

    reg &= ~(SLEEP_BIT);

    ret = mpu_write_register(hmpu->hi2c, hmpu->dev_addr, PWR_MGMT_1_ADDR, 1, &reg);
    return ret;
}

/**
  * @brief  Reads all sensor data from the MPU-6050 in a single I2C transaction.
  * @note   Reads 14 consecutive registers starting at ACCEL_XOUT_H (0x3B):
  *           bytes [0..5]  — accelerometer X, Y, Z (2 bytes each)
  *           bytes [6..7]  — temperature
  *           bytes [8..13] — gyroscope X, Y, Z (2 bytes each)
  *         Calibration offsets stored in the handle are subtracted from each reading.
  * @param  hmpu   Pointer to a MPU_6050_HandleTypeDef structure.
  * @param  hdata  Pointer to a MPU_6050_DataTypeDef structure to be populated.
  * @retval HAL_StatusTypeDef  HAL_OK on success, HAL_ERROR on I2C failure.
  */
HAL_StatusTypeDef MPU_6050_Read(const MPU_6050_HandleTypeDef *hmpu, MPU_6050_DataTypeDef *hdata)
{
    uint8_t raw_data[14];
    HAL_StatusTypeDef ret = HAL_OK;

    ret = mpu_read_register(hmpu->hi2c, hmpu->dev_addr, DATA_START_ADDR, 14, raw_data);
    if (ret != HAL_OK) return ret;

    hdata->accel_x = calculate_accel(raw_to_int16(raw_data[0], raw_data[1]), hmpu->accel_sens) - hmpu->accel_offset_x;
    hdata->accel_y = calculate_accel(raw_to_int16(raw_data[2], raw_data[3]), hmpu->accel_sens) - hmpu->accel_offset_y;
    hdata->accel_z = calculate_accel(raw_to_int16(raw_data[4], raw_data[5]), hmpu->accel_sens) - hmpu->accel_offset_z;
    hdata->temp = calculate_temp(raw_to_int16(raw_data[6], raw_data[7]));
    hdata->gyro_x = calculate_ang_vel(raw_to_int16(raw_data[8], raw_data[9]), hmpu->gyro_sens) - hmpu->gyro_offset_x;
    hdata->gyro_y = calculate_ang_vel(raw_to_int16(raw_data[10], raw_data[11]), hmpu->gyro_sens) - hmpu->gyro_offset_y;
    hdata->gyro_z = calculate_ang_vel(raw_to_int16(raw_data[12], raw_data[13]), hmpu->gyro_sens) - hmpu->gyro_offset_z;

    return ret;
}

/**
  * @brief  Calibrates the sensor by computing and storing axis offsets.
  * @note   The sensor must be held stationary and level (flat) during calibration.
  *         Collects the specified number of samples and computes the mean for each axis.
  *         Gyroscope offsets target 0 °/s on all axes.
  *         Accelerometer offsets target 0g on X and Y, and 1g on Z (gravity compensation).
  *         Computed offsets are stored in the handle and automatically applied by MPU_6050_Read().
  * @param  hmpu     Pointer to a MPU_6050_HandleTypeDef structure.
  * @param  samples  Number of samples to average (must be > 0, recommended: 500).
  * @retval HAL_StatusTypeDef  HAL_OK on success, HAL_ERROR if samples == 0.
  */
HAL_StatusTypeDef MPU_6050_Calibrate(MPU_6050_HandleTypeDef *hmpu, const uint16_t samples)
{
    if (samples == 0) return HAL_ERROR;

    MPU_6050_DataTypeDef data;
    double accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    double gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;

    for (uint16_t i = 0; i < samples; i++) {
        MPU_6050_Read(hmpu, &data);

        accel_sum_x += data.accel_x;
        accel_sum_y += data.accel_y;
        accel_sum_z += data.accel_z;
        gyro_sum_x  += data.gyro_x;
        gyro_sum_y  += data.gyro_y;
        gyro_sum_z  += data.gyro_z;
    }

    hmpu->accel_offset_x = (float)(accel_sum_x / samples);
    hmpu->accel_offset_y = (float)(accel_sum_y / samples);
    hmpu->accel_offset_z = (float)(accel_sum_z / samples) - 1.0f; /* Compensate for 1g gravity */
    hmpu->gyro_offset_x  = (float)(gyro_sum_x / samples);
    hmpu->gyro_offset_y  = (float)(gyro_sum_y / samples);
    hmpu->gyro_offset_z  = (float)(gyro_sum_z / samples);

    return HAL_OK;
}

/**
  * @brief  Resets all values in an angles structure to 0.0f.
  * @param  hangles  Pointer to a MPU_6050_AnglesTypeDef structure.
  */
void MPU_6050_ResetAngles(MPU_6050_AnglesTypeDef *hangles)
{
    hangles->accel_angle_x  = 0.0f;
    hangles->accel_angle_y  = 0.0f;
    hangles->filter_angle_x = 0.0f;
    hangles->filter_angle_y = 0.0f;
    hangles->kalman_angle_x = 0.0f;
    hangles->kalman_angle_y = 0.0f;
}

/**
  * @brief  Sets the gain parameter for complementary filter.
  * @param  hmpu              Pointer to a MPU_6050_HandleTypeDef structure.
  * @param  filter_gain       Float number from 0.00 .. 1.00 range (recommended: 0.98).
  * @retval HAL_StatusTypeDef HAL_OK on success, HAL_ERROR if gain is out of range.
  */
HAL_StatusTypeDef MPU_6050_SetCompFilterGain(MPU_6050_HandleTypeDef *hmpu, const float filter_gain)
{
    if (filter_gain < 0.0f || filter_gain > 1.0f) return HAL_ERROR;
    hmpu->comp_filter_gain = filter_gain;
    return HAL_OK;
}

/**
  * @brief  Updates an angles structure with values calculated from complementary filter formula.
  * @param  hmpu     Pointer to a MPU_6050_HandleTypeDef structure.
  * @param  hangles  Pointer to a MPU_6050_AnglesTypeDef structure.
  * @param  hdata    Pointer to a MPU_6050_DataTypeDef structure.
  * @param  dt       Time period between samples in seconds.
  */
void MPU_6050_UpdateCompFilterAngles(
    const MPU_6050_HandleTypeDef *hmpu,
    MPU_6050_AnglesTypeDef *hangles,
    const MPU_6050_DataTypeDef *hdata,
    const float dt
) {
    if (dt <= 0.0f) return;
    const float gain = hmpu->comp_filter_gain;

    MPU_6050_UpdateAccelAngles(hangles, hdata);
    hangles->filter_angle_x =
        gain * (hangles->filter_angle_x + hdata->gyro_x * dt) + (1.0f - gain) * hangles->accel_angle_x;
    hangles->filter_angle_y =
        gain * (hangles->filter_angle_y + hdata->gyro_y * dt) + (1.0f - gain) * hangles->accel_angle_y;
}

/**
  * @brief  Initializes a MPU_6050_KalmanFilterTypeDef structure with noise parameters.
  * @param  hkalman  Pointer to a MPU_6050_KalmanFilter_TypeDef structure.
  * @param  q        Process noise covariance — models gyroscope drift (recommended: 0.001).
  * @param  r        Measurement noise covariance — models accelerometer noise (recommended: 0.03).
  * @note   Both q and r must be greater than 0. Higher q trusts the accelerometer more;
  *         higher r trusts the gyroscope more.
  * @retval HAL_StatusTypeDef  HAL_OK on success, HAL_ERROR if q or r are less or equal to 0.
  */
HAL_StatusTypeDef MPU_6050_InitKalmanFilter(MPU_6050_KalmanFilterTypeDef *hkalman, const float q, const float r)
{
    if (q <= 0.0f || r <= 0.0f) return HAL_ERROR;

    hkalman->angle_x = 0.0f;
    hkalman->angle_y = 0.0f;
    hkalman->p_x     = 1.0f;
    hkalman->p_y     = 1.0f;
    hkalman->q       = q;
    hkalman->r       = r;

    return HAL_OK;
}

/**
  * @brief  Updates an angles structure with values calculated from Kalman filter formulas.
  * @param  hdata    Pointer to a MPU_6050_DataTypeDef structure.
  * @param  hkalman  Pointer to a MPU_6050_KalmanFilterTypeDef structure.
  * @param  hangles  Pointer to a MPU_6050_AnglesTypeDef structure.
  * @param  dt       Time period between samples in seconds.
  * @note   Implements a discrete Kalman filter in two steps:
  *         Prediction: angle += gyro * dt; P += Q
  *         Correction: K = P / (P + R); angle += K * (accel_angle - angle); P = (1-K) * P
  */
void MPU_6050_UpdateKalmanFilterAngles(
    const MPU_6050_DataTypeDef *hdata,
    MPU_6050_KalmanFilterTypeDef *hkalman,
    MPU_6050_AnglesTypeDef *hangles,
    const float dt
) {
    if (dt <= 0.0f) return;
    MPU_6050_UpdateAccelAngles(hangles, hdata);

    const float pred_angle_x = hkalman->angle_x + hdata->gyro_x * dt;
    const float pred_angle_y = hkalman->angle_y + hdata->gyro_y * dt;
    const float pred_p_x     = hkalman->p_x + hkalman->q;
    const float pred_p_y     = hkalman->p_y + hkalman->q;

    const float k_x     = pred_p_x / (pred_p_x + hkalman->r);
    const float angle_x = pred_angle_x + k_x * (hangles->accel_angle_x - pred_angle_x);
    const float p_x     = (1.0f - k_x) * pred_p_x;

    const float k_y     = pred_p_y / (pred_p_y + hkalman->r);
    const float angle_y = pred_angle_y + k_y * (hangles->accel_angle_y - pred_angle_y);
    const float p_y     = (1.0f - k_y) * pred_p_y;

    hangles->kalman_angle_x = angle_x;
    hkalman->angle_x        = angle_x;
    hkalman->p_x            = p_x < 0.0f ? 0.0f : p_x;

    hangles->kalman_angle_y = angle_y;
    hkalman->angle_y        = angle_y;
    hkalman->p_y            = p_y < 0.0f ? 0.0f : p_y;
}