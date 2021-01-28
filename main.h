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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Voltage_Pin GPIO_PIN_0
#define Voltage_GPIO_Port GPIOA
#define T1_Pin GPIO_PIN_1
#define T1_GPIO_Port GPIOA
#define T2_Pin GPIO_PIN_2
#define T2_GPIO_Port GPIOA
#define Pressure_Pin GPIO_PIN_3
#define Pressure_GPIO_Port GPIOA
#define Current_A_Pin GPIO_PIN_4
#define Current_A_GPIO_Port GPIOA
#define Current_B_Pin GPIO_PIN_5
#define Current_B_GPIO_Port GPIOA
#define Current_C_Pin GPIO_PIN_6
#define Current_C_GPIO_Port GPIOA
#define Phase_A_Pin GPIO_PIN_0
#define Phase_A_GPIO_Port GPIOB
#define Phase_B_Pin GPIO_PIN_1
#define Phase_B_GPIO_Port GPIOB
#define Phase_C_Pin GPIO_PIN_2
#define Phase_C_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define Start_solenoid_Pin GPIO_PIN_3
#define Start_solenoid_GPIO_Port GPIOB
#define ENCODER_1_Pin GPIO_PIN_4
#define ENCODER_1_GPIO_Port GPIOB
#define ENCODER_2_Pin GPIO_PIN_5
#define ENCODER_2_GPIO_Port GPIOB
#define TIM4_Ch1_BTN_Pin GPIO_PIN_6
#define TIM4_Ch1_BTN_GPIO_Port GPIOB
#define START_Pin GPIO_PIN_7
#define START_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/