#ifndef __PITCH_KALMAN_H
#define __PITCH_KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  float angle_deg;
  float gyro_bias_dps;
  float p00;
  float p01;
  float p10;
  float p11;
  uint8_t initialized;
} PitchKalman_t;

void PitchKalman_Reset(PitchKalman_t *filter);
float PitchKalman_Update(PitchKalman_t *filter,
                         float accel_angle_deg,
                         float gyro_dps,
                         float dt_s,
                         float q_angle,
                         float q_bias,
                         float r_angle,
                         float *corrected_gyro_dps);

#ifdef __cplusplus
}
#endif

#endif /* __PITCH_KALMAN_H */
