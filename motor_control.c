#include "motor_control.h"
#include "tim.h"

#define MOTOR_PI_F 3.14159265358979323846f
#define MOTOR_MM_PER_COUNT \
  ((MOTOR_PI_F * MOTOR_WHEEL_DIAMETER_MM) / MOTOR_ENCODER_COUNTS_PER_REV)
#define MOTOR_MM_S_PER_COUNT \
  (MOTOR_MM_PER_COUNT / MOTOR_ENCODER_SAMPLE_PERIOD_S)

static MotorTelemetry_t motor_telemetry;
static uint16_t left_previous_count;
static uint16_t right_previous_count;

/**
 * @brief 将 PWM 限制在允许范围内。
 *
 * 控制算法可能给出较大的正负 PWM，这里统一限制到
 * MOTOR_PWM_LIMIT_COUNTS 以内，避免超过定时器和驱动能力。
 */
static int16_t Motor_ClampPWM(int32_t value)
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
 * @brief 对左右电机 PWM 做比例修正。
 *
 * 两个电机实际性能可能不同，通过 numerator/denominator 对某一侧输出
 * 做比例缩放，用来补偿左右轮速度不一致的问题。
 */
static int16_t Motor_ApplyScale(int16_t value, int32_t numerator, int32_t denominator)
{
  int32_t scaled;

  if (denominator == 0)
  {
    denominator = 1;
  }
  scaled = ((int32_t)value * numerator) / denominator;
  return Motor_ClampPWM(scaled);
}

/**
 * @brief 将 PWM 幅值转换为定时器比较值。
 *
 * 本工程 PWM 计数方式是比较值越小，占空比越大，所以用
 * MOTOR_PWM_PERIOD_COUNTS - magnitude 得到实际写入 CCR 的值。
 */
static uint32_t Motor_PWMCompareFromMagnitude(int32_t magnitude)
{
  if (magnitude < 0)
  {
    magnitude = -magnitude;
  }
  if (magnitude > MOTOR_PWM_LIMIT_COUNTS)
  {
    magnitude = MOTOR_PWM_LIMIT_COUNTS;
  }
  return (uint32_t)(MOTOR_PWM_PERIOD_COUNTS - magnitude);
}

/**
 * @brief 初始化电机 PWM 和编码器。
 *
 * 启动 TIM3 的四路 PWM 作为左右电机正反转控制；
 * 同时启动 TIM4、TIM8 编码器模式，用来读取左右轮脉冲增量。
 */
void Motor_Init(void)
{
  motor_telemetry.left_delta = 0;
  motor_telemetry.right_delta = 0;
  motor_telemetry.left_total = 0;
  motor_telemetry.right_total = 0;
  motor_telemetry.left_raw_mm_s = 0.0f;
  motor_telemetry.right_raw_mm_s = 0.0f;
  motor_telemetry.left_filtered_mm_s = 0.0f;
  motor_telemetry.right_filtered_mm_s = 0.0f;
  motor_telemetry.left_requested_pwm = 0;
  motor_telemetry.right_requested_pwm = 0;
  motor_telemetry.left_pwm = 0;
  motor_telemetry.right_pwm = 0;

  Motor_Brake();
  if ((HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK))
  {
    Error_Handler();
  }

  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  __HAL_TIM_SET_COUNTER(&htim8, 0U);
  if ((HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK) ||
      (HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL) != HAL_OK))
  {
    Error_Handler();
  }
  left_previous_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  right_previous_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);
}

/**
 * @brief 电机刹车/停止输出。
 *
 * 将四路 PWM 都置为停止状态，并清空记录的请求 PWM 和实际 PWM。
 * 倒地保护、IMU 错误、初始化前都会调用它。
 */
void Motor_Brake(void)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MOTOR_PWM_PERIOD_COUNTS);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MOTOR_PWM_PERIOD_COUNTS);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MOTOR_PWM_PERIOD_COUNTS);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MOTOR_PWM_PERIOD_COUNTS);
  motor_telemetry.left_requested_pwm = 0;
  motor_telemetry.right_requested_pwm = 0;
  motor_telemetry.left_pwm = 0;
  motor_telemetry.right_pwm = 0;
}

/**
 * @brief 设置左右电机 PWM。
 *
 * 输入的是控制器计算出的左右轮逻辑 PWM。
 * 函数内部依次完成方向修正、左右电机比例修正、死区补偿、
 * 定时器比较值换算，最后写入 TIM3 的四个 PWM 通道。
 */
void Motor_SetPWM(int16_t left, int16_t right)
{
  uint32_t left_compare;
  uint32_t right_compare;

  left = Motor_ClampPWM((int32_t)left * MOTOR_LEFT_COMMAND_SIGN);
  right = Motor_ClampPWM((int32_t)right * MOTOR_RIGHT_COMMAND_SIGN);
  left = Motor_ApplyScale(left, MOTOR_LEFT_PWM_NUM, MOTOR_LEFT_PWM_DEN);
  right = Motor_ApplyScale(right, MOTOR_RIGHT_PWM_NUM, MOTOR_RIGHT_PWM_DEN);
  motor_telemetry.left_requested_pwm = left;
  motor_telemetry.right_requested_pwm = right;
  left = Motor_MapPWMCommand(left, MOTOR_LEFT_DEADZONE_COUNTS);
  right = Motor_MapPWMCommand(right, MOTOR_RIGHT_DEADZONE_COUNTS);
  left_compare = Motor_PWMCompareFromMagnitude(left);
  right_compare = Motor_PWMCompareFromMagnitude(right);

  if (left > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MOTOR_PWM_PERIOD_COUNTS);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, left_compare);
  }
  else if (left < 0)
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_compare);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MOTOR_PWM_PERIOD_COUNTS);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MOTOR_PWM_PERIOD_COUNTS);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MOTOR_PWM_PERIOD_COUNTS);
  }

  if (right > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, right_compare);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MOTOR_PWM_PERIOD_COUNTS);
  }
  else if (right < 0)
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MOTOR_PWM_PERIOD_COUNTS);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, right_compare);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MOTOR_PWM_PERIOD_COUNTS);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MOTOR_PWM_PERIOD_COUNTS);
  }

  motor_telemetry.left_pwm = left;
  motor_telemetry.right_pwm = right;
}

/**
 * @brief 每 5ms 更新一次编码器测速数据。
 *
 * 读取 TIM4/TIM8 当前计数，与上一次计数相减得到左右轮脉冲增量；
 * 再根据轮径和单圈脉冲数换算成 mm/s，并做一阶低通滤波。
 */
void Encoder_Update5ms(void)
{
  uint16_t left_current = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  uint16_t right_current = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);
  int16_t left_delta = (int16_t)(left_current - left_previous_count);
  int16_t right_delta = (int16_t)(right_current - right_previous_count);

  left_previous_count = left_current;
  right_previous_count = right_current;
  left_delta = (int16_t)(left_delta * MOTOR_LEFT_ENCODER_SIGN);
  right_delta = (int16_t)(right_delta * MOTOR_RIGHT_ENCODER_SIGN);

  motor_telemetry.left_delta = left_delta;
  motor_telemetry.right_delta = right_delta;
  motor_telemetry.left_total += left_delta;
  motor_telemetry.right_total += right_delta;
  motor_telemetry.left_raw_mm_s = (float)left_delta * MOTOR_MM_S_PER_COUNT;
  motor_telemetry.right_raw_mm_s = (float)right_delta * MOTOR_MM_S_PER_COUNT;
  motor_telemetry.left_filtered_mm_s += MOTOR_SPEED_FILTER_ALPHA *
    (motor_telemetry.left_raw_mm_s - motor_telemetry.left_filtered_mm_s);
  motor_telemetry.right_filtered_mm_s += MOTOR_SPEED_FILTER_ALPHA *
    (motor_telemetry.right_raw_mm_s - motor_telemetry.right_filtered_mm_s);
}

/**
 * @brief 获取当前电机和编码器遥测数据。
 *
 * 返回值包含左右编码器增量、累计脉冲、原始速度、滤波速度、
 * 以及最近一次请求/实际输出的 PWM，供控制任务和串口打印使用。
 */
const MotorTelemetry_t *Motor_GetTelemetry(void)
{
  return &motor_telemetry;
}
