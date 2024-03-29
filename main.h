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
#define Phase_C_Pin GPIO_PIN_7
#define Phase_C_GPIO_Port GPIOA
#define Phase_A_Pin GPIO_PIN_0
#define Phase_A_GPIO_Port GPIOB
#define Phase_B_Pin GPIO_PIN_1
#define Phase_B_GPIO_Port GPIOB
#define Stop_btn_Pin GPIO_PIN_10
#define Stop_btn_GPIO_Port GPIOB
#define On_LED_Pin GPIO_PIN_11
#define On_LED_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define W5500_RST_Pin GPIO_PIN_8
#define W5500_RST_GPIO_Port GPIOA
#define W5500_INT_Pin GPIO_PIN_9
#define W5500_INT_GPIO_Port GPIOA
#define Alarm_Pin GPIO_PIN_15
#define Alarm_GPIO_Port GPIOA
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
#define  simulation 0
#define dreamtime 20
  static const int   User_Page_Adress[]={
  0x0800FC00,0x0800FC04,0x0800FC08,0x0800FC0C,0x0800FC10,0x0800FC14,0x0800FC18,0x0800FC1C,
  0x0800FC20,0x0800FC24,0x0800FC28,0x0800FC2C,0x0800FC30,0x0800FC34,0x0800FC38,0x0800FC3C,
  0x0800FC40,0x0800FC44,0x0800FC48,0x0800FC4C,0x0800FC50,0x0800FC54,0x0800FC58,0x0800FC5C,
  0x0800FC60,0x0800FC64};
  static const uint16_t NTC_10k[116]={
    3437,3407,3376,3344,3311,3278,
    3243,3208,3172,3135,3098,3060,3021,2981,
    2941,2901,2860,2818,2776,2733,2690,2647,
    2604,2560,2516,2472,2428,2383,2339,2295,
    2251,2206,2162,2119,2075,2032,1989,1946,
    1904,1862,1820,1779,1739,1699,1659,1620,
    1582,1544,1507,1470,1434,1399,1364,1330,
    1297,1264,1232,1201,1170,1140,1110,1082,
    1053,1026,999,973,947,922,898,874,
    851,829,807,785,764,744,724,705,
    686,668,650,633,616,600,584,569,
    554,539,525,511,498,485,472,460,
    448,436,425,414,403,393,383,373,
    364,355,346,337,328,320,312,304,
    297,289,282,275,269,262};               //-5 to 110

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
