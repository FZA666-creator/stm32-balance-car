#include "imu.h"
#include "pitch_kalman.h"
#include "STM32_I2C.h"
#include <math.h>
#include <stddef.h>

#define MPU6050_ADDR             0x68U
#define MPU6050_RA_SMPLRT_DIV    0x19U
#define MPU6050_RA_CONFIG        0x1AU
#define MPU6050_RA_GYRO_CONFIG   0x1BU
#define MPU6050_RA_ACCEL_CONFIG  0x1CU
#define MPU6050_RA_INT_ENABLE    0x38U
#define MPU6050_RA_ACCEL_XOUT_H  0x3BU
#define MPU6050_RA_PWR_MGMT_1    0x6BU
#define MPU6050_RA_PWR_MGMT_2    0x6CU
#define MPU6050_RA_WHO_AM_I      0x75U

#define IMU_RAD_TO_DEG           57.2957795f
#define IMU_GYRO_2000DPS_SENS    16.4f

#ifndef IMU_PITCH_SIGN
#define IMU_PITCH_SIGN           (-1.0f)
#endif

static PitchKalman_t pitch_filter;
static uint8_t imu_ready;

/**
 * @brief 将 MPU6050 连续两个 8 位寄存器合成为有符号 16 位数据。
 *
 * MPU6050 的加速度和陀螺仪原始数据都是高字节在前、低字节在后，
 * 这里用于把寄存器读出的 high/low 合成 int16_t。
 */
static int16_t Imu_CombineInt16(uint8_t high, uint8_t low)
{
  return (int16_t)((uint16_t)((uint16_t)high << 8) | (uint16_t)low);
}

/**
 * @brief 向 MPU6050 指定寄存器写入 1 字节数据。
 *
 * 底层调用软件 I2C 写函数，用于配置采样率、量程、滤波器和中断。
 */
static uint8_t Imu_WriteReg(uint8_t reg, uint8_t value)
{
  return (uint8_t)(i2cWrite(MPU6050_ADDR, reg, value) ? 1U : 0U);
}

/**
 * @brief 从 MPU6050 指定寄存器读取 1 字节数据。
 *
 * 主要用于读取 WHO_AM_I，确认 MPU6050 是否在线。
 */
static uint8_t Imu_ReadReg(uint8_t reg, uint8_t *value)
{
  return (uint8_t)(i2cRead(MPU6050_ADDR, reg, 1U, value) ? 1U : 0U);
}

/**
 * @brief 初始化 MPU6050。
 *
 * 完成软件 I2C 初始化、总线恢复、设备 ID 检查、复位传感器，
 * 并配置低通滤波、采样率、陀螺仪量程、加速度量程和数据就绪中断。
 */
uint8_t Imu_Init(void)
{
  uint8_t who_am_i = 0U;

  imu_ready = 0U;
  Imu_ResetFilter();
  i2cInit();
  i2cUnstick();
  HAL_Delay(100U);

  if (Imu_ReadReg(MPU6050_RA_WHO_AM_I, &who_am_i) == 0U)
  {
    return 0U;
  }
  if ((who_am_i != 0x68U) && (who_am_i != 0x70U))
  {
    return 0U;
  }

  if (Imu_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x80U) == 0U)
  {
    return 0U;
  }
  HAL_Delay(100U);

  /* DLPF=3, 1 kHz/(4+1)=200 Hz, gyro=+-2000 dps, accel=+-2 g。 */
  if ((Imu_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x01U) == 0U) ||
      (Imu_WriteReg(MPU6050_RA_PWR_MGMT_2, 0x00U) == 0U) ||
      (Imu_WriteReg(MPU6050_RA_CONFIG, 0x03U) == 0U) ||
      (Imu_WriteReg(MPU6050_RA_SMPLRT_DIV, 0x04U) == 0U) ||
      (Imu_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18U) == 0U) ||
      (Imu_WriteReg(MPU6050_RA_ACCEL_CONFIG, 0x00U) == 0U) ||
      (Imu_WriteReg(MPU6050_RA_INT_ENABLE, 0x01U) == 0U))
  {
    return 0U;
  }

  HAL_Delay(50U);
  imu_ready = 1U;
  return 1U;
}

/**
 * @brief 复位 Pitch 卡尔曼滤波器。
 *
 * 当系统重新启动、IMU 异常或需要重新建立姿态估计时调用。
 */
void Imu_ResetFilter(void)
{
  PitchKalman_Reset(&pitch_filter);
}

/**
 * @brief 读取 MPU6050 原始加速度和陀螺仪数据。
 *
 * 从 ACCEL_XOUT_H 开始一次性读取 14 字节，
 * 取出三轴加速度和三轴陀螺仪原始 ADC 值。
 */
uint8_t Imu_ReadRaw(ImuRaw_t *raw)
{
  uint8_t buffer[14];

  if (raw == NULL)
  {
    return 0U;
  }
  if (i2cRead(MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H,
              sizeof(buffer), buffer) == false)
  {
    return 0U;
  }

  raw->accel_x = Imu_CombineInt16(buffer[0], buffer[1]);
  raw->accel_y = Imu_CombineInt16(buffer[2], buffer[3]);
  raw->accel_z = Imu_CombineInt16(buffer[4], buffer[5]);
  raw->gyro_x = Imu_CombineInt16(buffer[8], buffer[9]);
  raw->gyro_y = Imu_CombineInt16(buffer[10], buffer[11]);
  raw->gyro_z = Imu_CombineInt16(buffer[12], buffer[13]);
  return 1U;
}

/**
 * @brief 每 5ms 更新一次姿态角。
 *
 * 先读取 MPU6050 原始数据，再由加速度计算静态 Pitch 角，
 * 由陀螺仪计算 Pitch 角速度，最后通过卡尔曼滤波融合得到
 * 平衡控制使用的 pitch_deg 和 gyro_pitch_dps。
 */
uint8_t Imu_UpdateKalman5ms(BalanceSample_t *sample)
{
  ImuRaw_t raw;
  float accel_pitch_deg;
  float gyro_pitch_dps;
  float corrected_gyro_dps;

  if (sample == NULL)
  {
    return 0U;
  }

  sample->timestamp_ms = HAL_GetTick();
  sample->sample_dt_ms = IMU_SAMPLE_PERIOD_MS;
  sample->valid = 0U;
  if ((imu_ready == 0U) || (Imu_ReadRaw(&raw) == 0U))
  {
    sample->pitch_deg = 0.0f;
    sample->gyro_pitch_dps = 0.0f;
    sample->gyro_yaw_dps = 0.0f;
    sample->raw.accel_x = 0;
    sample->raw.accel_y = 0;
    sample->raw.accel_z = 0;
    sample->raw.gyro_x = 0;
    sample->raw.gyro_y = 0;
    sample->raw.gyro_z = 0;
    return 0U;
  }

  accel_pitch_deg = (float)(atan2((double)raw.accel_y,
                                   (double)raw.accel_z) *
                            (double)IMU_RAD_TO_DEG);
  gyro_pitch_dps = (float)raw.gyro_x / IMU_GYRO_2000DPS_SENS;
  accel_pitch_deg *= IMU_PITCH_SIGN;
  gyro_pitch_dps *= IMU_PITCH_SIGN;

  sample->pitch_deg = PitchKalman_Update(
    &pitch_filter, accel_pitch_deg, gyro_pitch_dps, IMU_SAMPLE_PERIOD_S,
    IMU_KALMAN_Q_ANGLE, IMU_KALMAN_Q_BIAS, IMU_KALMAN_R_ANGLE,
    &corrected_gyro_dps);
  sample->gyro_pitch_dps = corrected_gyro_dps;
  sample->gyro_yaw_dps = ((float)raw.gyro_z / IMU_GYRO_2000DPS_SENS) *
                         IMU_YAW_SIGN;
  sample->raw = raw;
  sample->timestamp_ms = HAL_GetTick();
  sample->valid = 1U;
  return 1U;
}
