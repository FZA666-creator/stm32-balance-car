#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "balance_types.h"

#define IMU_SAMPLE_PERIOD_MS        5U
#define IMU_SAMPLE_PERIOD_S         0.005f

/* 与 WHEELTEC 示例一致的二状态角度/陀螺零偏 Kalman 参数。 */
#define IMU_KALMAN_Q_ANGLE          0.001f
#define IMU_KALMAN_Q_BIAS           0.003f
#define IMU_KALMAN_R_ANGLE          0.5f
#ifndef IMU_YAW_SIGN
#define IMU_YAW_SIGN                1.0f
#endif

uint8_t Imu_Init(void);
void Imu_ResetFilter(void);
uint8_t Imu_ReadRaw(ImuRaw_t *raw);
uint8_t Imu_UpdateKalman5ms(BalanceSample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */
