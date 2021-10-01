/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32g0xx_hal.h"

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
#define VOLTAGE_PROBE_5V_Pin GPIO_PIN_1
#define VOLTAGE_PROBE_5V_GPIO_Port GPIOA
#define CHASIS_T_1_Pin GPIO_PIN_2
#define CHASIS_T_1_GPIO_Port GPIOA
#define CHASIS_T_2_Pin GPIO_PIN_3
#define CHASIS_T_2_GPIO_Port GPIOA
#define BOARD_TEMP_PROBE1_Pin GPIO_PIN_4
#define BOARD_TEMP_PROBE1_GPIO_Port GPIOA
#define BOARD_TEMP_PROBE2_Pin GPIO_PIN_5
#define BOARD_TEMP_PROBE2_GPIO_Port GPIOA
#define POWER_SWITCH_Pin GPIO_PIN_6
#define POWER_SWITCH_GPIO_Port GPIOA
#define VBAT_1_Pin GPIO_PIN_7
#define VBAT_1_GPIO_Port GPIOA
#define VBAT_2_Pin GPIO_PIN_0
#define VBAT_2_GPIO_Port GPIOB
#define VBAT_3_Pin GPIO_PIN_1
#define VBAT_3_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_2
#define LED_B_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_8
#define LED_R_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_6
#define LED_G_GPIO_Port GPIOC
#define ENABLE_SENSORS_Pin GPIO_PIN_15
#define ENABLE_SENSORS_GPIO_Port GPIOA
#define ENABLE_STEPPER_MOTORS_Pin GPIO_PIN_3
#define ENABLE_STEPPER_MOTORS_GPIO_Port GPIOB
#define ENABLE_TOFS_Pin GPIO_PIN_4
#define ENABLE_TOFS_GPIO_Port GPIOB
#define PWM_FAN_Pin GPIO_PIN_5
#define PWM_FAN_GPIO_Port GPIOB
#define ENABLE_RAIL_12V_Pin GPIO_PIN_7
#define ENABLE_RAIL_12V_GPIO_Port GPIOB
#define ENABLE_RAIL_5V_Pin GPIO_PIN_8
#define ENABLE_RAIL_5V_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
