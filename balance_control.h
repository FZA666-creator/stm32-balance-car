#ifndef __BALANCE_CONTROL_H
#define __BALANCE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "balance_types.h"

/*
 * 控制模式选择入口，修改后重新编译并烧录：
 *   BALANCE_MODE_ANGLE_PD_ONLY：只运行角度 PD，速度滤波和积分保持清零。
 *   BALANCE_MODE_WHEELTEC_CASCADE：速度 PI 直接输出 PWM，与角度 PD PWM 相加。
 *
 * WHEELTEC 示例把理论串级公式进行了代数化简，因此速度环不再输出目标角，
 * 但它仍然承担外环“消除轮速、把车推回原位”的作用。
 */
#define BALANCE_MODE_ANGLE_PD_ONLY       0U
#define BALANCE_MODE_WHEELTEC_CASCADE    1U

#ifndef BALANCE_CONTROL_MODE
#define BALANCE_CONTROL_MODE             BALANCE_MODE_WHEELTEC_CASCADE
#endif

#if BALANCE_CONTROL_MODE == BALANCE_MODE_ANGLE_PD_ONLY
#define BALANCE_ENABLE_VELOCITY_LOOP     0U
#define BALANCE_CONTROL_MODE_TEXT        "ANGLE_PD_ONLY"
#elif BALANCE_CONTROL_MODE == BALANCE_MODE_WHEELTEC_CASCADE
#define BALANCE_ENABLE_VELOCITY_LOOP     1U
#define BALANCE_CONTROL_MODE_TEXT        "WHEELTEC_CASCADE_EQUIV"
#else
#error "Unsupported BALANCE_CONTROL_MODE"
#endif

/* 角度环调参入口。Kp 单位约为 PWM/度，Kd 单位为 PWM/(度/秒)。 */
#ifndef BALANCE_KP
#define BALANCE_KP                       270.0f
#endif

/* 18.04 = WHEELTEC 原始陀螺参数 1.10 * MPU6050 的 16.4 LSB/(deg/s)。 */
#ifndef BALANCE_KD
#define BALANCE_KD                       18.04f
#endif

/* 机械平衡中值角。完成角度环测试后建议每次只调整 0.1 度。 */
#ifndef BALANCE_MIDDLE_ANGLE_DEG
#define BALANCE_MIDDLE_ANGLE_DEG         0.0f
#endif

/*
 * 速度环调参入口。这里的输出单位是 PWM，不再是目标角度：
 * velocity_pwm = Kp * filtered_speed + Ki * speed_integral。
 */
#ifndef BALANCE_VELOCITY_KP
#define BALANCE_VELOCITY_KP              4.0f
#endif

#ifndef BALANCE_VELOCITY_KI
#define BALANCE_VELOCITY_KI              0.02f
#endif

/* WHEELTEC 速度低通：filtered = 0.84 * old + 0.16 * current。 */
#ifndef BALANCE_ENCODER_FILTER_KEEP
#define BALANCE_ENCODER_FILTER_KEEP      0.84f
#endif

#ifndef BALANCE_ENCODER_FILTER_NEW
#define BALANCE_ENCODER_FILTER_NEW       0.16f
#endif

/* 速度积分累计限幅；停机、故障、倾倒或关闭速度环时立即清零。 */
#ifndef BALANCE_VELOCITY_INTEGRAL_LIMIT
#define BALANCE_VELOCITY_INTEGRAL_LIMIT  380000.0f
#endif

/* 安全保护参数。PB9 超过 15 ms 无数据或倾角超过 40 度时立即停机。 */
#ifndef BALANCE_TILT_SHUTDOWN_DEG
#define BALANCE_TILT_SHUTDOWN_DEG        40.0f
#endif

#ifndef BALANCE_SAMPLE_TIMEOUT_MS
#define BALANCE_SAMPLE_TIMEOUT_MS        15U
#endif

typedef struct
{
  float speed_filtered;       /* 左右编码器 5 ms 增量之和的低通值。 */
  float speed_integral;       /* 低通速度累计值，用于速度积分项。 */
} BalanceController_t;

typedef struct
{
  int32_t balance_pwm;        /* 角度 PD 分量，尚未做总 PWM 限幅。 */
  int32_t velocity_pwm;       /* 速度 PI 分量，直接以 PWM 为单位。 */
  int32_t requested_pwm;      /* 两分量相加后的原始请求。 */
  float speed_filtered;       /* 速度低通值，便于确认反馈方向。 */
  float speed_integral;       /* 速度积分累计值，便于检查积分饱和。 */
  int16_t turn_pwm;           /* 当前原地站立版本固定为 0。 */
  int16_t left_pwm;           /* 总限幅后的左电机逻辑请求。 */
  int16_t right_pwm;          /* 总限幅后的右电机逻辑请求。 */
  BalanceState_t state;
} BalanceOutput_t;

void BalanceControl_Init(BalanceController_t *controller);
void BalanceControl_Reset(BalanceController_t *controller);
BalanceOutput_t BalanceControl_Update(BalanceController_t *controller,
                                      const BalanceSample_t *sample,
                                      int16_t encoder_left,
                                      int16_t encoder_right,
                                      int16_t target_speed,
                                      int16_t target_turn);

#ifdef __cplusplus
}
#endif

#endif /* __BALANCE_CONTROL_H */
