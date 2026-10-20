#include "motor_control.h"

/**
 * @brief 将控制器请求的逻辑 PWM 映射到实际电机 PWM。
 *
 * 平衡车电机存在机械摩擦和驱动死区，小 PWM 可能完全带不动轮子。
 * 本函数先保留正负方向，再把超过小死带的逻辑 PWM 映射到
 * “死区补偿值 ~ 最大 PWM”之间，使小车在小角度修正时也能响应。
 *
 * @param requested_pwm 控制器计算出的原始 PWM 请求，带正负号。
 * @param deadzone_counts 实测电机启动死区补偿值。
 * @return 补偿后的实际 PWM 请求，仍带正负号。
 */
int16_t Motor_MapPWMCommand(int16_t requested_pwm, int16_t deadzone_counts)
{
  int32_t magnitude = requested_pwm;
#if MOTOR_PWM_DEADZONE_ENABLE == 1U
  int32_t mapped_magnitude;
  int32_t logical_span;
  int32_t output_span;
  int32_t sign = 1;
#endif

  if (magnitude > MOTOR_PWM_LIMIT_COUNTS)
  {
    magnitude = MOTOR_PWM_LIMIT_COUNTS;
  }
  else if (magnitude < -MOTOR_PWM_LIMIT_COUNTS)
  {
    magnitude = -MOTOR_PWM_LIMIT_COUNTS;
  }

#if MOTOR_PWM_DEADZONE_ENABLE == 0U
  (void)deadzone_counts;
  return (int16_t)magnitude;
#else
  if (magnitude < 0)
  {
    magnitude = -magnitude;
    sign = -1;
  }
  if (magnitude <= MOTOR_PWM_COMMAND_DEADBAND_COUNTS)
  {
    return 0;
  }

  if (deadzone_counts < 0)
  {
    deadzone_counts = 0;
  }
  else if (deadzone_counts >= MOTOR_PWM_LIMIT_COUNTS)
  {
    deadzone_counts = MOTOR_PWM_LIMIT_COUNTS - 1;
  }

  logical_span = MOTOR_PWM_LIMIT_COUNTS -
                 MOTOR_PWM_COMMAND_DEADBAND_COUNTS;
  output_span = MOTOR_PWM_LIMIT_COUNTS - deadzone_counts;
  mapped_magnitude = deadzone_counts +
    (((magnitude - MOTOR_PWM_COMMAND_DEADBAND_COUNTS) * output_span +
      (logical_span / 2)) / logical_span);
  if (mapped_magnitude > MOTOR_PWM_LIMIT_COUNTS)
  {
    mapped_magnitude = MOTOR_PWM_LIMIT_COUNTS;
  }

  return (int16_t)(sign * mapped_magnitude);
#endif
}
