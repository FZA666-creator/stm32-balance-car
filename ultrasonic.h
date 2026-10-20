#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include <stdint.h>

/* 初始化 HC-SR04 的 DWT 微秒计时器。 */
void Ultrasonic_Init(void);
/* 低优先级任务函数：约 60 ms 测量一次，内部带超时，不阻塞平衡任务。 */
void Ultrasonic_Task(void *argument);
/* 返回最近一次有效距离，单位 mm；0 表示当前无有效回波。 */
uint16_t Ultrasonic_GetDistanceMm(void);
/* 连续测量状态：1=有有效回波，0=无效/超时。 */
uint8_t Ultrasonic_IsValid(void);
/* 每完成一次测量递增，用于控制任务识别“新样本”，避免重复使用旧回波。 */
uint32_t Ultrasonic_GetSequence(void);

#endif
