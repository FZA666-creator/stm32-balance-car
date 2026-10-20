#include "balance_control.h"
#include "motor_control.h"
#include <stddef.h>
#include <string.h>

#if BALANCE_ENABLE_VELOCITY_LOOP
/**
 * @brief 对浮点数做正负限幅。
 *
 * 速度积分项会长期累加，如果不限制，车子被推一下后积分可能越积越大，
 * 导致松手后还一直冲。这个函数用于把积分限制在安全范围内。
 */
static float Balance_ClampFloat(float value, float limit)
{
  if (value > limit)
  {
    return limit;
  }
  if (value < -limit)
  {
    return -limit;
  }
  return value;
}
#endif

/**
 * @brief 将浮点控制量四舍五入成整数 PWM。
 *
 * PID 计算得到的是 float，但最终定时器 PWM 需要整数计数值，
 * 所以这里把正负浮点数统一转换成 int32_t。
 */
static int32_t Balance_RoundFloat(float value)
{
  if (value >= 0.0f)
  {
    return (int32_t)(value + 0.5f);
  }
  return (int32_t)(value - 0.5f);
}

/**
 * @brief 对最终电机 PWM 做总限幅。
 *
 * 防止角度环、速度环和转向量叠加后超过定时器允许的最大 PWM，
 * 保护电机和驱动电路。
 */
static int16_t Balance_ClampTotalPwm(int32_t value)
{
  if (value > MOTOR_PWM_LIMIT_COUNTS)
  {
    return MOTOR_PWM_LIMIT_COUNTS;
  }
  if (value < -MOTOR_PWM_LIMIT_COUNTS)
  {
    return -MOTOR_PWM_LIMIT_COUNTS;
  }
  return (int16_t)value;
}

/**
 * @brief 判断小车是否已经倾倒。
 *
 * 当 Pitch 角超过设定阈值时，认为小车已经倒地或接近倒地，
 * 控制器会停止输出，避免电机继续高速转动。
 */
static uint8_t Balance_IsTilted(float angle)
{
  return (uint8_t)((angle > BALANCE_TILT_SHUTDOWN_DEG) ||
                   (angle < -BALANCE_TILT_SHUTDOWN_DEG));
}

/**
 * @brief 构造一个停止状态的控制输出。
 *
 * 出现 IMU 错误、倒地保护、数据超时等情况时使用，
 * 所有 PWM 和内部状态清零，只保留故障状态码。
 */
static BalanceOutput_t Balance_StoppedOutput(BalanceState_t state)
{
  BalanceOutput_t output;

  (void)memset(&output, 0, sizeof(output));
  output.state = state;
  return output;
}

/**
 * @brief 初始化平衡控制器。
 *
 * 当前主要是清空速度滤波值和速度积分值，让控制器从干净状态启动。
 */
void BalanceControl_Init(BalanceController_t *controller)
{
  BalanceControl_Reset(controller);
}

/**
 * @brief 复位平衡控制器内部状态。
 *
 * 倒地、IMU 异常、关闭速度环或重新启动时调用，
 * 目的是清空速度滤波和积分，避免旧数据影响下一次平衡。
 */
void BalanceControl_Reset(BalanceController_t *controller)
{
  if (controller == NULL)
  {
    return;
  }
  controller->speed_filtered = 0.0f;
  controller->speed_integral = 0.0f;
}

/**
 * @brief 平衡小车核心控制函数。
 *
 * 输入当前姿态、左右编码器增量、蓝牙目标速度和蓝牙目标转向量；
 * 内部先计算角度环 PD，再计算速度环 PI，最后叠加转向差速，
 * 输出左右电机的逻辑 PWM。
 *
 * 控制关系：
 * - 角度环 PD：让小车站起来，Pitch 偏差越大恢复力越大；
 * - 速度环 PI：抑制长期前后漂移，让小车尽量停在原地或按目标速度运动；
 * - 转向量 target_turn：作为左右轮差速，实现蓝牙左转/右转。
 */
BalanceOutput_t BalanceControl_Update(BalanceController_t *controller,
                                      const BalanceSample_t *sample,
                                      int16_t encoder_left,
                                      int16_t encoder_right,
                                      int16_t target_speed,
                                      int16_t target_turn)
{
  BalanceOutput_t output;
  float angle_error;
  float balance_pwm;
  float velocity_pwm = 0.0f;
  int32_t total_pwm;

  if ((controller == NULL) || (sample == NULL) || (sample->valid == 0U))
  {
    BalanceControl_Reset(controller);
    return Balance_StoppedOutput(BALANCE_STATE_IMU_ERROR);
  }

  if (Balance_IsTilted(sample->pitch_deg) != 0U)
  {
    BalanceControl_Reset(controller);
    return Balance_StoppedOutput(BALANCE_STATE_TILT);
  }

  (void)memset(&output, 0, sizeof(output));
  angle_error = sample->pitch_deg - BALANCE_MIDDLE_ANGLE_DEG;
  balance_pwm = (BALANCE_KP * angle_error) +
                (BALANCE_KD * sample->gyro_pitch_dps);

#if BALANCE_ENABLE_VELOCITY_LOOP
  {
    float speed = (float)encoder_left + (float)encoder_right;
    float speed_error = speed - (float)target_speed;

    /*
     * 正逻辑 PWM 必须对应正编码器增量。正速度产生正 velocity_pwm，
     * 使车轮继续追向运动方向并建立反向车身倾角，这是示例的等效串级符号。
     */
    controller->speed_filtered =
      (BALANCE_ENCODER_FILTER_KEEP * controller->speed_filtered) +
      (BALANCE_ENCODER_FILTER_NEW * speed_error);
    controller->speed_integral = Balance_ClampFloat(
      controller->speed_integral + controller->speed_filtered,
      BALANCE_VELOCITY_INTEGRAL_LIMIT);
    velocity_pwm = (BALANCE_VELOCITY_KP * controller->speed_filtered) +
                   (BALANCE_VELOCITY_KI * controller->speed_integral);
  }
#else
  (void)encoder_left;
  (void)encoder_right;
  (void)target_speed;
  BalanceControl_Reset(controller);
#endif

  output.balance_pwm = Balance_RoundFloat(balance_pwm);
  output.velocity_pwm = Balance_RoundFloat(velocity_pwm);
  output.requested_pwm = output.balance_pwm + output.velocity_pwm;
  output.speed_filtered = controller->speed_filtered;
  output.speed_integral = controller->speed_integral;
  output.turn_pwm = target_turn;
  total_pwm = output.requested_pwm;
  output.left_pwm = Balance_ClampTotalPwm(total_pwm - (int32_t)target_turn);
  output.right_pwm = Balance_ClampTotalPwm(total_pwm + (int32_t)target_turn);
  output.state = BALANCE_STATE_RUNNING;
  return output;
}
