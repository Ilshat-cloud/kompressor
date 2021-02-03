/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TEMPERATURE_1_Pin GPIO_PIN_0
#define TEMPERATURE_1_GPIO_Port GPIOA
#define PWM_LED_Pin GPIO_PIN_2
#define PWM_LED_GPIO_Port GPIOA
#define Btn_Pin GPIO_PIN_5
#define Btn_GPIO_Port GPIOA
#define ENCODER_1_Pin GPIO_PIN_6
#define ENCODER_1_GPIO_Port GPIOA
#define ENCODER_2_Pin GPIO_PIN_7
#define ENCODER_2_GPIO_Port GPIOA
#define SPI2_Pin GPIO_PIN_15
#define SPI2_GPIO_Port GPIOB
#define PID_PWM_OUT_Pin GPIO_PIN_15
#define PID_PWM_OUT_GPIO_Port GPIOA
#define BREIZ_IN_EXTI_Pin GPIO_PIN_3
#define BREIZ_IN_EXTI_GPIO_Port GPIOB
#define BREIZ_OUT_Pin GPIO_PIN_4
#define BREIZ_OUT_GPIO_Port GPIOB
#define RELAY_HIST_Pin GPIO_PIN_5
#define RELAY_HIST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
//page 16- 8000-8800

  static const int   User_Page_Adress[]={
  0x0800FC00,0x0800FC04,0x0800FC08,0x0800FC0C,0x0800FC10,0x0800FC14,0x0800FC18,0x0800FC1C,
  0x0800FC20,0x0800FC24,0x0800FC28,0x0800FC2C,0x0800FC30,0x0800FC34,0x0800FC38,0x0800FC3C,
  0x0800FC40,0x0800FC44,0x0800FC48,0x0800FC4C,0x0800FC50,0x0800FC54,0x0800FC58,0x0800FC5C,
  0x0800FC60,0x0800FC64};
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
