#ifndef __BALANCE_TYPES_H
#define __BALANCE_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
} ImuRaw_t;

typedef struct
{
  uint32_t timestamp_ms;
  uint32_t data_ready_sequence;
  float pitch_deg;
  float gyro_pitch_dps;
  float gyro_yaw_dps;
  uint16_t sample_dt_ms;
  ImuRaw_t raw;
  uint8_t valid;
} BalanceSample_t;

typedef enum
{
  BALANCE_STATE_STOPPED = 0,
  BALANCE_STATE_RUNNING = 1,
  BALANCE_STATE_IMU_ERROR = 2,
  BALANCE_STATE_TILT = 3,
  BALANCE_STATE_TIMEOUT = 4,
  BALANCE_STATE_OVERRUN = 5
} BalanceState_t;

#ifdef __cplusplus
}
#endif

#endif /* __BALANCE_TYPES_H */
