#ifndef __OLED_STATUS_H
#define __OLED_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "balance_control.h"
#include "line_follower.h"
#include "motor_control.h"
#include <stdint.h>

/* 初始化 PC14/PB5/PB4/PB3 四线软件 SPI OLED（无 CS）。 */
void OledStatus_Init(void);

/* 建议每 200~300 ms 刷新，避免显示传输占用高优先级控制时间。 */
void OledStatus_Update(const BalanceSample_t *sample,
                       const MotorTelemetry_t *motor,
                       const BalanceOutput_t *output,
                       const LineFollowerOutput_t *line_output);

uint8_t OledStatus_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_STATUS_H */
