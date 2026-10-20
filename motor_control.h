#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* TIM3 ARR=7199，因此一个完整 PWM 周期共有 7200 个计数。 */
#define MOTOR_PWM_PERIOD_COUNTS            7200

/* 最大驱动输出，只用于总输出保护，不能拿来调低速响应。 */
#define MOTOR_PWM_LIMIT_COUNTS             6900
#define MOTOR_CONTROL_PERIOD_MS            10U
#define MOTOR_ENCODER_SAMPLE_PERIOD_MS     5U
#define MOTOR_ENCODER_SAMPLE_PERIOD_S      0.005f

/* PWM 死区补偿总开关：1U 开启，0U 关闭，便于补偿前后对比。 */
#ifndef MOTOR_PWM_DEADZONE_ENABLE
#define MOTOR_PWM_DEADZONE_ENABLE          1U
#endif

/*
 * 控制器逻辑零区，单位为 PWM 计数。
 * 绝对值不超过此值时输出 0，用于过滤姿态噪声造成的小幅反复换向。
 * 它不是电机死区；响应迟钝时减小，零点发响时增大。
 */
#ifndef MOTOR_PWM_COMMAND_DEADBAND_COUNTS
#define MOTOR_PWM_COMMAND_DEADBAND_COUNTS  80
#endif

/*
 * 左右电机可靠起转所需的最小实际 PWM，单位为 PWM 计数。
 * 已有扫描结果为 360；仍不起转时每次增加 20~40，启动冲击大时减小。
 */
#ifndef MOTOR_LEFT_DEADZONE_COUNTS
#define MOTOR_LEFT_DEADZONE_COUNTS         360
#endif

#ifndef MOTOR_RIGHT_DEADZONE_COUNTS
#define MOTOR_RIGHT_DEADZONE_COUNTS        360
#endif

#if ((MOTOR_PWM_DEADZONE_ENABLE != 0U) && \
     (MOTOR_PWM_DEADZONE_ENABLE != 1U))
#error "MOTOR_PWM_DEADZONE_ENABLE must be 0U or 1U"
#endif

#if ((MOTOR_PWM_COMMAND_DEADBAND_COUNTS < 0) || \
     (MOTOR_PWM_COMMAND_DEADBAND_COUNTS >= MOTOR_PWM_LIMIT_COUNTS))
#error "MOTOR_PWM_COMMAND_DEADBAND_COUNTS must be in [0, PWM limit)"
#endif

#if ((MOTOR_LEFT_DEADZONE_COUNTS < 0) || \
     (MOTOR_LEFT_DEADZONE_COUNTS >= MOTOR_PWM_LIMIT_COUNTS))
#error "MOTOR_LEFT_DEADZONE_COUNTS must be in [0, PWM limit)"
#endif

#if ((MOTOR_RIGHT_DEADZONE_COUNTS < 0) || \
     (MOTOR_RIGHT_DEADZONE_COUNTS >= MOTOR_PWM_LIMIT_COUNTS))
#error "MOTOR_RIGHT_DEADZONE_COUNTS must be in [0, PWM limit)"
#endif

#ifndef MOTOR_ENCODER_COUNTS_PER_REV
#define MOTOR_ENCODER_COUNTS_PER_REV       1560.0f
#endif

#ifndef MOTOR_WHEEL_DIAMETER_MM
#define MOTOR_WHEEL_DIAMETER_MM            70.0f
#endif

#define MOTOR_SPEED_FILTER_ALPHA           0.2f

/* 正逻辑 PWM 必须得到左右同为正的编码器增量；只在这里修正硬件方向。 */
#ifndef MOTOR_LEFT_ENCODER_SIGN
#define MOTOR_LEFT_ENCODER_SIGN            1
#endif

#ifndef MOTOR_RIGHT_ENCODER_SIGN
#define MOTOR_RIGHT_ENCODER_SIGN           -1
#endif

#ifndef MOTOR_LEFT_COMMAND_SIGN
#define MOTOR_LEFT_COMMAND_SIGN            1
#endif

#ifndef MOTOR_RIGHT_COMMAND_SIGN
#define MOTOR_RIGHT_COMMAND_SIGN           1
#endif

/* 左右轮差异微调，默认均为 1.000。 */
#ifndef MOTOR_LEFT_PWM_NUM
#define MOTOR_LEFT_PWM_NUM                 1000
#endif
#ifndef MOTOR_LEFT_PWM_DEN
#define MOTOR_LEFT_PWM_DEN                 1000
#endif
#ifndef MOTOR_RIGHT_PWM_NUM
#define MOTOR_RIGHT_PWM_NUM                1000
#endif
#ifndef MOTOR_RIGHT_PWM_DEN
#define MOTOR_RIGHT_PWM_DEN                1000
#endif

typedef struct
{
  int16_t left_delta;
  int16_t right_delta;
  int32_t left_total;
  int32_t right_total;
  float left_raw_mm_s;
  float right_raw_mm_s;
  float left_filtered_mm_s;
  float right_filtered_mm_s;
  int16_t left_requested_pwm;  /* 方向和 trim 修正后的逻辑请求值。 */
  int16_t right_requested_pwm;
  int16_t left_pwm;            /* 死区映射后实际写入 TIM3 的有效 PWM。 */
  int16_t right_pwm;
} MotorTelemetry_t;

int16_t Motor_MapPWMCommand(int16_t requested_pwm, int16_t deadzone_counts);
void Motor_Init(void);
void Motor_Brake(void);
void Motor_SetPWM(int16_t left, int16_t right);
void Encoder_Update5ms(void);
const MotorTelemetry_t *Motor_GetTelemetry(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */
