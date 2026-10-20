/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I2C_SCL_Pin GPIO_PIN_14
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_15
#define I2C_SDA_GPIO_Port GPIOB
#define INT_Pin GPIO_PIN_9
#define INT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* 四路数字灰度传感器，GPIO 低电平表示检测到黑线。 */
#define LINE_LEFT_OUTER_Pin GPIO_PIN_8
#define LINE_LEFT_OUTER_GPIO_Port GPIOB
#define LINE_LEFT_INNER_Pin GPIO_PIN_9
#define LINE_LEFT_INNER_GPIO_Port GPIOC
#define LINE_RIGHT_INNER_Pin GPIO_PIN_4
#define LINE_RIGHT_INNER_GPIO_Port GPIOC
#define LINE_RIGHT_OUTER_Pin GPIO_PIN_8
#define LINE_RIGHT_OUTER_GPIO_Port GPIOC

/* 六针无 CS OLED 的四线软件 SPI 引脚。 */
#define OLED_SCL_Pin GPIO_PIN_14
#define OLED_SCL_GPIO_Port GPIOC
#define OLED_SDA_Pin GPIO_PIN_5
#define OLED_SDA_GPIO_Port GPIOB
#define OLED_RES_Pin GPIO_PIN_4
#define OLED_RES_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_3
#define OLED_DC_GPIO_Port GPIOB

/* HC-SR04 超声波：Echo 接 PA1，Trig 接 PC15。 */
#define ULTRASONIC_ECHO_Pin GPIO_PIN_1
#define ULTRASONIC_ECHO_GPIO_Port GPIOA
#define ULTRASONIC_TRIG_Pin GPIO_PIN_15
#define ULTRASONIC_TRIG_GPIO_Port GPIOC

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
