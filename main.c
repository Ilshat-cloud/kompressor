/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wh1602.h"
#include "MAX31855.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint32_t flash_read(uint32_t address) {
	return (*(__IO uint32_t*) address);
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch2;

/* Definitions for main_task */
osThreadId_t main_taskHandle;
const osThreadAttr_t main_task_attributes = {
  .name = "main_task",
  .priority = (osPriority_t) osPriorityLow5,
  .stack_size = 256 * 4
};
/* Definitions for PID_task */
osThreadId_t PID_taskHandle;
const osThreadAttr_t PID_task_attributes = {
  .name = "PID_task",
  .priority = (osPriority_t) osPriorityLow6,
  .stack_size = 128 * 4
};
/* Definitions for Screen_task */
osThreadId_t Screen_taskHandle;
const osThreadAttr_t Screen_task_attributes = {
  .name = "Screen_task",
  .priority = (osPriority_t) osPriorityLow7,
  .stack_size = 128 * 4
};
/* Definitions for Flash_task */
osThreadId_t Flash_taskHandle;
const osThreadAttr_t Flash_task_attributes = {
  .name = "Flash_task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */

//--------my global variables----------//
MAX31855_Sensor termocoupe;
void Breizinheim(void);
uint16_t EncoderVal,brightness=50,target1,worktime=0;
uint8_t i,P1=4,I1=2,D1=1,X1,x_1=0,dead_zone1=0,sensor=0,iter,y1,direct=0,hysteresys=2,start=0,autostart=1,flag=0, curPoint=0,mistake=0,flash_i;
int16_t errorsold1[256];
int16_t  Error1,Temp_ADC=0,MAX_EU=1000,MIN_EU=0,Temp_start=0;
int32_t regD1=0,regP1=0,regI1=0,PID1;
uint32_t button;
FLASH_EraseInitTypeDef Erase;
int16_t t_cur[2]={0,0};
typedef struct
{
  uint8_t Sec;
  uint8_t Minuts;  
  uint16_t  target;
} points;

points point[20];
uint8_t point_num=0;
//--------------------------------------//

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM4_Init(void);
void main_func(void *argument);
void PID(void *argument);
void Screen(void *argument);
void Flash(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    //-------------------------Flash---------------------//

    if (flash_read(User_Page_Adress[0])!=0xFFFFFFFF)
     {
          MAX_EU=flash_read(User_Page_Adress[0]);
          MIN_EU=flash_read(User_Page_Adress[0])>>16;
          direct=flash_read(User_Page_Adress[1]);
          sensor=flash_read(User_Page_Adress[1])>>8;
          brightness=flash_read(User_Page_Adress[1])>>16;
          hysteresys=flash_read(User_Page_Adress[2]);
          P1=flash_read(User_Page_Adress[2])>>8;  
          I1=flash_read(User_Page_Adress[2])>>16;
          D1=flash_read(User_Page_Adress[2])>>24; 
          autostart=flash_read(User_Page_Adress[3]);
          point_num=flash_read(User_Page_Adress[3])>>8;  
          for (flash_i=0;flash_i<20;flash_i++)
            {
              point[flash_i].Minuts=flash_read(User_Page_Adress[flash_i+4]);
              point[flash_i].Sec=flash_read(User_Page_Adress[flash_i+4])>>8;
              point[flash_i].target=flash_read(User_Page_Adress[flash_i+4])>>16;  
            }
     }

    //====================================================//

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  MX_RTC_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of main_task */
  main_taskHandle = osThreadNew(main_func, NULL, &main_task_attributes);

  /* creation of PID_task */
  PID_taskHandle = osThreadNew(PID, NULL, &PID_task_attributes);

  /* creation of Screen_task */
  Screen_taskHandle = osThreadNew(Screen, NULL, &Screen_task_attributes);

  /* creation of Flash_task */
  Flash_taskHandle = osThreadNew(Flash, NULL, &Flash_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_SECOND;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 1;
  DateToUpdate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2048;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 65535;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|SPI2_Pin|BREIZ_OUT_Pin|RELAY_HIST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : Btn_Pin */
  GPIO_InitStruct.Pin = Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 SPI2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|SPI2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BREIZ_IN_EXTI_Pin */
  GPIO_InitStruct.Pin = BREIZ_IN_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BREIZ_IN_EXTI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BREIZ_OUT_Pin RELAY_HIST_Pin */
  GPIO_InitStruct.Pin = BREIZ_OUT_Pin|RELAY_HIST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//------------------------OUT-------------------------
void Breizinheim(void) 
{
  i++;
  uint8_t x;
  x=i*x_1/100;
    if (x==y1)
    {   //off
      HAL_GPIO_WritePin(BREIZ_OUT_GPIO_Port,BREIZ_OUT_Pin, GPIO_PIN_RESET);
    }
    else 
    {   //on
      HAL_GPIO_WritePin(BREIZ_OUT_GPIO_Port,BREIZ_OUT_Pin, GPIO_PIN_SET);
    }
  y1=x;
  if (i>=100)
  {
    i=0; y1=0; x_1=X1;
  }
}
void EXTI3_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
    Breizinheim();
}
//---------------------------------------------------------------//
/* USER CODE END 4 */

/* USER CODE BEGIN Header_main_func */
/**
  * @brief  Function implementing the main_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_main_func */
void main_func(void *argument)
{
  /* USER CODE BEGIN 5 */
  
  volatile uint16_t dma[2];

  uint8_t SPI_Buf[4],last_sec=0;
  uint16_t ramp_time;
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);  //pid
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);  //led
  HAL_TIM_IC_Start_DMA(&htim4,TIM_CHANNEL_2,&button,1);
  //HAL_TIM_IC_Start(&htim4,TIM_CHANNEL_2);
  //HAL_TIM_IC_Start(&htim4,TIM_CHANNEL_1);
  /* Infinite loop */
  InitializeLCD();
  ClearLCDScreen();
  start=(autostart==1)?1:0;
  flag=start;
  RTC_TimeTypeDef CurTime = {0};
  target1=point[0].target;  
  __HAL_TIM_SET_COUNTER(&htim3,point[curPoint].target);
  for(;;)
  {
    //-------------SPI ADC IDWG Encoder------------//
    HAL_IWDG_Refresh(&hiwdg);
    HAL_RTC_GetTime(&hrtc, &CurTime, RTC_FORMAT_BIN);
    switch (sensor)
    {
    case 1:
      HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&dma,2); 
      osDelay(10);
      t_cur[0]=dma[0];
      t_cur[1]=dma[1];//linearize here
      Temp_ADC=(Temp_ADC+t_cur[0])/2;
      if ((Temp_ADC>MAX_EU)||(Temp_ADC<MIN_EU))
      {
        mistake=1;
      }else{
        mistake=0;
      }
    break;  
    case 0:
      HAL_GPIO_WritePin(SPI2_GPIO_Port,SPI2_Pin,GPIO_PIN_RESET);
      if (HAL_SPI_Receive(&hspi2,SPI_Buf,4,200)==HAL_OK)
      {
        HAL_GPIO_WritePin(SPI2_GPIO_Port,SPI2_Pin,GPIO_PIN_SET);
        termocoupe.Buf=SPI_Buf;
        MAX31855_convert_buf(&termocoupe);
        Temp_ADC=(int16_t)(Temp_ADC+termocoupe.t_termocoupe)/2;
        if ((termocoupe.OC) ||(termocoupe.SCG)||(termocoupe.SCV)||(Temp_ADC>MAX_EU)||(Temp_ADC<MIN_EU))
        {
          mistake=1;
        }else{
          mistake=0;
        }
      }else{mistake=1;}
      break;
    case 2:
      HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&dma,2); 
      osDelay(10);
      t_cur[0]=dma[0];
      t_cur[1]=dma[1];//linearize here
      Temp_ADC=(Temp_ADC+ t_cur[0]+t_cur[1])/3;

      if ((Temp_ADC>MAX_EU)||(Temp_ADC<MIN_EU))
      {
        mistake=1;
      }else{
        mistake=0;
      }
    break;
    case 3:
      HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&dma,2); 
      osDelay(10);
      t_cur[0]=dma[0];
      t_cur[1]=dma[1];//linearize here
      if (t_cur[0]>=t_cur[1])
      {
        Temp_ADC=(Temp_ADC+ t_cur[0])/2;
      }else{
        Temp_ADC=(Temp_ADC+ t_cur[1])/2;
      }

      if ((Temp_ADC>MAX_EU)||(Temp_ADC<MIN_EU))
      {
        mistake=1;
      }else{
        mistake=0;
      }
    break;
    case 4:
      HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&dma,2); 
      osDelay(10);
      t_cur[0]=dma[0];
      t_cur[1]=dma[1];//linearize here
      if (t_cur[0]<=t_cur[1])
      {
        Temp_ADC=(Temp_ADC+ t_cur[0])/2;
      }else{
        Temp_ADC=(Temp_ADC+ t_cur[1])/2;
      }

      if ((Temp_ADC>MAX_EU)||(Temp_ADC<MIN_EU))
      {
        mistake=1;
      }else{
        mistake=0;
      }
    break;
    
    }  
    EncoderVal=__HAL_TIM_GET_COUNTER(&htim3);
    TIM2->CCR3=brightness*10;
    //----------------------------------------------//     
    
   //button=TIM4->CCR2-TIM4->CCR1; 
    
    //-------------------------------startuem+hyst-----------------------//   
    if (start)
    {
      
      if (mistake)
      {
        TIM2->CCR1=(direct==0)?0:1000;
        NVIC_DisableIRQ(EXTI3_IRQn);
        HAL_GPIO_WritePin(BREIZ_OUT_GPIO_Port,BREIZ_OUT_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RELAY_HIST_GPIO_Port,RELAY_HIST_Pin, GPIO_PIN_RESET);
        
      }else{

        NVIC_EnableIRQ(EXTI3_IRQn);
        TIM2->CCR1=PID1/10;
        
        if (Temp_ADC>=target1+hysteresys)
        {
          HAL_GPIO_WritePin(RELAY_HIST_GPIO_Port,RELAY_HIST_Pin, GPIO_PIN_RESET);
        }
        else if (Temp_ADC<=target1-hysteresys)
        {
          HAL_GPIO_WritePin(RELAY_HIST_GPIO_Port,RELAY_HIST_Pin, GPIO_PIN_SET);
        }
        
      }
      
      if (last_sec!=CurTime.Seconds)
        {
          last_sec=CurTime.Seconds;
          worktime++;
          if (curPoint%2==1)
            {
              target1=Temp_start+(point[curPoint].target-Temp_start)*worktime/ramp_time;                 
            }
        }
      if ((point_num>0)&&(worktime>=(point[curPoint].Minuts*60+point[curPoint].Sec)))
        {
          worktime=0;
          curPoint++;
          if (curPoint<=point_num){
            if (curPoint%2==0)
            {
              __HAL_TIM_SET_COUNTER(&htim3,point[curPoint].target);
              target1=point[curPoint].target;
            }else {
              Temp_start=Temp_ADC;
              ramp_time=point[curPoint].Minuts*60+point[curPoint].Sec;
            }
          }
          else{
            start=0;
            flag=0;
            curPoint=0;
          }
        
        }
        
        
    }else{
       TIM2->CCR1=0;
       NVIC_DisableIRQ(EXTI3_IRQn);
       HAL_GPIO_WritePin(BREIZ_OUT_GPIO_Port,BREIZ_OUT_Pin, GPIO_PIN_RESET);
       HAL_GPIO_WritePin(RELAY_HIST_GPIO_Port,RELAY_HIST_Pin, GPIO_PIN_RESET);  
    }
    //------------------------------------------------------------//
    
    
   
    osDelay(200);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_PID */
/**
* @brief Function implementing the PID_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PID */
void PID(void *argument)
{
  /* USER CODE BEGIN PID */
  /* Infinite loop */
  for(;;)
  {
    osDelay(300);
//-------------------pid1------------------------------------------//   
      Error1=target1-Temp_ADC;   
      regI1=0;
      for (iter=0; iter<I1; iter++)  
      {
        regI1=regI1+errorsold1[iter];
      } 
      regP1=Error1*P1;  
      regI1 =(I1>0)? (regI1 + Error1):0; 
      regD1=(Error1-errorsold1[0])*D1; 
      if (regD1<(-10000))
      {
        regD1=(-10000);
      }
      if (regD1>(10000))
      {
        regD1=(10000);
      }
      if (regI1<(-10000))
      {
        regI1=(-10000);
      }
      if (regI1>(10000))
      {
        regI1=(10000);
      }
      for (iter=254; iter>0; iter--)
      {
      errorsold1[iter+1]=errorsold1[iter];
      } 
      errorsold1[0]=Error1;
      if ((Error1>=dead_zone1)||(dead_zone1==0)||(Error1<=(dead_zone1*(-1))) )  
      {
        PID1=regP1+regI1+regD1; 
        PID1=(direct==0)?PID1:PID1*(-1);
        if (PID1<0)
        {
          X1=0;
        }
        if (PID1>=10000)
        {
          X1=100;
          PID1=10000;
        }
        if((PID1>=0)&&(PID1<10000))
        {
          X1=PID1/100; //koeff moshnosti
        }
      }
//---------------------------------------------------------//    
    
  }
  /* USER CODE END PID */
}

/* USER CODE BEGIN Header_Screen */
/**
* @brief Function implementing the Screen_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Screen */
void Screen(void *argument)
{
  /* USER CODE BEGIN Screen */
    char R[16];
    osDelay(500);
    uint16_t worktime_old=0,worktime_hour;
    uint8_t choise=0,point_n=0,clear;
    
    
  /* Infinite loop */
  for(;;)
  {
    switch(flag)
    {
      //------------------------------startyem--------------------------------//
      case 0:
        Cursor(0,0);
        PrintStr("C");
        SendByte(0xBF,1);  //t
        PrintStr("ap");
        SendByte(0xBF,1);
        PrintStr("ye");
        SendByte(0xBC,1);   //m            
        PrintStr("? Kop-");
        SendByte(0xE0,1);   //D
        PrintStr("a");

        Cursor(1,0);
        SendByte(0xA9,1);  //Y
        SendByte(0xE3,1);  //d
        PrintStr("ep");
        SendByte(0xB6,1);  //j
        PrintStr(".-Hac");
        SendByte(0xBF,1);  //t
        PrintStr("po");
        SendByte(0xB9,1);  //i'
        SendByte(0xBA,1);  //k
        SendByte(0xB8,1);  //i
        
        if (button>1000) 
          {
            ClearLCDScreen();
            flag=2;
            __HAL_TIM_SET_COUNTER(&htim3,P1);
            choise=0;
          } else if (button>100)
          {
            ClearLCDScreen();
            __HAL_TIM_SET_COUNTER(&htim3,point[curPoint].target);
            flag=1;
            start=1;
			worktime_old=0;
          }
      break; 
      //===========================================================//
      
      
      //-------------------------------main-----------------------------//
      case 1:
        if (clear){ClearLCDScreen();clear=0;}
        Cursor(0,0);
        SendByte(0xA4,1);  //Z
        PrintStr("a");
        SendByte(0xE3,1);  //d
        PrintStr("a");
        SendByte(0xBD,1);  //n
        SendByte(0xB8,1);  //i
        PrintStr("e: ");
        sprintf(R,"%04d",target1);
        PrintStr(R);        
        SendByte(0xEF,1);  //0
        PrintStr("C");
        if (sensor<2)
        {
          Cursor(1,0);
          PrintStr("T:");  
          sprintf(R,"%04d",Temp_ADC);
          PrintStr(R);
        } else{
          if (worktime<worktime_old+3){
            Cursor(1,0);
            PrintStr("1:");  
            sprintf(R,"%04d",t_cur[0]);
            PrintStr(R);
          }else if (worktime<worktime_old+6){
            Cursor(1,0);
            PrintStr("2:");  
            sprintf(R,"%04d",t_cur[1]);
            PrintStr(R);
          }else{
            Cursor(1,0);
            PrintStr("T:");  
            sprintf(R,"%04d",Temp_ADC);
            PrintStr(R);
          }
        }
        if (!mistake)
        {
          PrintStr(" ");
        }else
        {
          SendByte(0xD5,1);  //x
        }
        
        if ((target1!=EncoderVal)&&(curPoint%2==0))
          {
            point[curPoint].target=EncoderVal%1200;
            target1=point[curPoint].target;
          }
        if (worktime<worktime_old+5)
          {
            Cursor(1,7);
            PrintStr("P:"); 
            sprintf(R,"%03d",X1);
            PrintStr(R);
            PrintStr("%");
            sprintf(R,"%02d",curPoint);
            PrintStr(R);
            SendByte(0xBF,1);  //t
          } else 
          {
            Cursor(1,7);
            worktime_hour=worktime/3600;
            worktime_hour=(worktime_hour>999)?999:worktime_hour;
            sprintf(R,"%03d",worktime/3600);
            PrintStr(R);
            PrintStr(":");
            sprintf(R,"%02d",(worktime%3600)/60);
            PrintStr(R);
            PrintStr(":");            
            sprintf(R,"%02d",worktime%60);
            PrintStr(R);        
            if (worktime>worktime_old+9)
            {
              clear=1;
              worktime_old=worktime;
            } 
          }
        if (button>1000) 
          {
            ClearLCDScreen();
            flag=2;
            start=0;
            __HAL_TIM_SET_COUNTER(&htim3,P1);
            choise=0;
          }         

      break; 
      //===========================================================//
      
      
      //-------------------------------set1-----------------------------// 
      case 2:
        Cursor(0,0);
        PrintStr("P");
        sprintf(R,"%03d",P1);
        PrintStr(R);
        PrintStr(" I");
        sprintf(R,"%03d",I1);
        PrintStr(R);
        PrintStr(" D");
        sprintf(R,"%03d",D1);
        PrintStr(R);
        Cursor(1,0);
        SendByte(0xA1,1);  //G          
        SendByte(0xB8,1);  //i  
        PrintStr("c");
        SendByte(0xBF,1);  //t        
        PrintStr(":");
        sprintf(R,"%03d",hysteresys);
        PrintStr(R);        
        SendByte(0xEF,1);  //0
        PrintStr("C Dir-");  
        sprintf(R,"%01d",direct);
        PrintStr(R); 
        switch (choise)
        {
          case 0:
            Cursor(0,3);
            P1=EncoderVal%256;
            break;
          case 1:
            Cursor(0,8);
            I1=EncoderVal%256;
            break;
          case 2:
            Cursor(0,13);
            D1=EncoderVal%256;
            break;
          case 3:
            Cursor(1,7);
            hysteresys=EncoderVal%256;
            break;    
          case 4:
            Cursor(1,15);
            direct=EncoderVal%2;
            break;           
        }
        if (button>1000) 
          {
            ClearLCDScreen();
            flag=3;
            start=0;
            choise=0;
            __HAL_TIM_SET_COUNTER(&htim3,sensor);
          } else if (button>100) 
          {
            choise=(choise>3)?0:choise+1;
            switch (choise)
              {
                case 0:
                  __HAL_TIM_SET_COUNTER(&htim3,P1);
                  break;
                case 1:
                  __HAL_TIM_SET_COUNTER(&htim3,I1);
                  break;
                case 2:
                  __HAL_TIM_SET_COUNTER(&htim3,D1);
                  break;
                case 3:
                  __HAL_TIM_SET_COUNTER(&htim3,hysteresys);
                  break;    
                case 4:
                  __HAL_TIM_SET_COUNTER(&htim3,direct);
                  break;           
              }
          }         
      break;     
      //===========================================================//
      
      
      //-------------------------------set2-----------------------------//       
      
      case 3:
        Cursor(0,0);
        SendByte(0xE0,1);   //D
        PrintStr("a");        
        SendByte(0xBF,1);  //t      
        SendByte(0xC0,1);  //4  
        SendByte(0xB8,1);  //i 
        SendByte(0xBA,1);  //k
        PrintStr("-");
        if (sensor==0)
          {
            SendByte(0xBF,1);  //t 
            PrintStr("ep");
            SendByte(0xBC,1);   //m
            PrintStr("o");
            SendByte(0xBE,1);   //n-p
            PrintStr("apa");
          }else if(sensor==1){
            PrintStr("co");
            SendByte(0xBE,1);   //n-p
            PrintStr("po");
            SendByte(0xBF,1);  //t
            SendByte(0xB8,1);  //i
            SendByte(0xB3,1);  //v
            SendByte(0xBB,1);  //l
          }else if(sensor==2){   
            PrintStr("cpe");
            SendByte(0xE3,1);  //d
            PrintStr(".");
            SendByte(0xB8,1);  //i 
            SendByte(0xB7,1);  //z
            PrintStr("2x");
          }else if(sensor==3){   
            PrintStr("Ma");
            SendByte(0xBA,1);  //k
            PrintStr("c");
            PrintStr(".");
            SendByte(0xB8,1);  //i 
            SendByte(0xB7,1);  //z
            PrintStr("2x");
           }else if(sensor==4){   
            PrintStr("M");
            SendByte(0xB8,1);  //i
            SendByte(0xBD,1);   //n-N
            PrintStr(" .");
            SendByte(0xB8,1);  //i 
            SendByte(0xB7,1);  //z
            PrintStr("2x");  
        }
            
        Cursor(1,0);     
        PrintStr("Ma");
        SendByte(0xBA,1);  //k
        PrintStr("c");
        sprintf(R,"%04d",MAX_EU);
        PrintStr(R);
        PrintStr(" M");
        SendByte(0xB8,1);  //i
        SendByte(0xBD,1);   //n-p
        sprintf(R,"%04d",MIN_EU);
        PrintStr(R);
        
        switch (choise)
        {
          case 0:
            Cursor(0,7);
            sensor=EncoderVal%5;
            break;
          case 1:
            Cursor(1,4);
            MAX_EU=(EncoderVal%1300)-100;
            break;
          case 2:
            Cursor(1,12);
            MIN_EU=(EncoderVal%1300)-100;
            break;
        }
        
        if (button>1000) 
          {
            ClearLCDScreen();
            flag=4;
            start=0;
            choise=0;
            __HAL_TIM_SET_COUNTER(&htim3,brightness);
          } else if (button>100) 
          {
             choise=(choise>1)?0:choise+1;
             switch (choise)
              {
                case 0:
                  __HAL_TIM_SET_COUNTER(&htim3,sensor);
                  break;
                case 1:
                  __HAL_TIM_SET_COUNTER(&htim3,MAX_EU+100);
                  break;
                case 2:
                  __HAL_TIM_SET_COUNTER(&htim3,MIN_EU+100);
                  break;
              }
          } 
      break; 
      //===========================================================//
      
      
      //-------------------------------set3-----------------------------//       
      case 4:
        Cursor(0,0);
        SendByte(0xB1,1);  //9I
        PrintStr("p");
        SendByte(0xBA,1);  //k
        PrintStr("-");
        sprintf(R,"%03d",brightness);
        PrintStr(R);
        PrintStr("% To");
        SendByte(0xC0,1);  //4
        PrintStr("e");
        SendByte(0xBA,1);  //k
        sprintf(R,"%02d",point_num);
        PrintStr(R);
        Cursor(1,0);
        PrintStr("A");
        SendByte(0xB3,1);  //v
        SendByte(0xBF,1);  //t
        PrintStr("o");
        SendByte(0xB7,1);  //z
        PrintStr("a");
        SendByte(0xBE,1);   //n-p
        PrintStr("yc");
        SendByte(0xBA,1);  //k
        PrintStr("-");
        if (autostart)
          {
            SendByte(0xE3,1);  //d
            PrintStr("a ");
          }else {
            SendByte(0xBD,1); //n
            PrintStr("e");
            SendByte(0xBF,1);  //t
          }
        switch (choise)
        {
          case 0:
            Cursor(0,6);
            brightness=EncoderVal%101;
            break;
          case 1:
            Cursor(0,15);
            point_num=EncoderVal%20;
            break;
          case 2:
            Cursor(1,13);
            autostart=EncoderVal%2;
            break;
        }
        if (button>1000) 
          {
            ClearLCDScreen();
            flag=5;
            start=0;
            choise=0;
            __HAL_TIM_SET_COUNTER(&htim3,point_n);
          } else if (button>100) 
          {
            choise=(choise>1)?0:choise+1;
            switch (choise)
              {
                case 0:
                  __HAL_TIM_SET_COUNTER(&htim3,brightness);
                  break;
                case 1:
                  __HAL_TIM_SET_COUNTER(&htim3,point_num);
                  break;
                case 2:
                  __HAL_TIM_SET_COUNTER(&htim3,autostart);
                  break;
              }
          } 
      break; 
      //===========================================================//
      
      
      //-------------------------------set4-----------------------------// 
      case 5:
      
        Cursor(0,0);
        PrintStr("To");
        SendByte(0xC0,1);  //4
        SendByte(0xBA,1);  //k
        PrintStr("a");
        sprintf(R,"%02d",point_n);
        PrintStr(R);
        PrintStr(" Te");
        SendByte(0xBC,1);   //m
        SendByte(0xBE,1);   //n-p
        sprintf(R,"%04d",point[point_n].target);
        PrintStr(R);
        Cursor(1,0);
        PrintStr("M");
        SendByte(0xB8,1);  //i
        SendByte(0xBD,1);  //n
        PrintStr(":");
        sprintf(R,"%03d",point[point_n].Minuts);
        PrintStr(R);
        PrintStr(" Ce");
        SendByte(0xBA,1);  //k
        PrintStr(":");
        sprintf(R,"%03d",point[point_n].Sec);
        PrintStr(R);
        
        switch (choise)
        {
          case 0:
            Cursor(0,6);
            point_n=EncoderVal%(point_num+1);
            break;
          case 1:
            Cursor(0,15);
            point[point_n].target=EncoderVal%1200;
            break;
          case 2:
            Cursor(1,6);
            point[point_n].Minuts=EncoderVal%256;
            break;
          case 3:
            Cursor(1,14);
            point[point_n].Sec=EncoderVal%256;
            break;
        }
    
        if (button>1000) 
          {
            flag=6;
            start=0;
            choise=0;
            __HAL_TIM_SET_COUNTER(&htim3,0);
            ClearLCDScreen();
          } else if (button>100) 
          {
            choise=(choise>2)?0:choise+1;
            switch (choise)
              {
                case 0:
                  __HAL_TIM_SET_COUNTER(&htim3,point_n);
                  break;
                case 1:
                  __HAL_TIM_SET_COUNTER(&htim3,point[point_n].target);
                  break;
                case 2:
                  __HAL_TIM_SET_COUNTER(&htim3,point[point_n].Minuts);
                  break;
                case 3:
                  __HAL_TIM_SET_COUNTER(&htim3,point[point_n].Sec);
                  break;
              }
          } 
      
      break; 
      //===========================================================//
      
      
      //-------------------------------set5-----------------------------// 
      case 6:
        Cursor(0,0);
        PrintStr("Coxpa");
        SendByte(0xBD,1);  //n
        SendByte(0xB8,1);  //i
        SendByte(0xBF,1);  //t
        SendByte(0xC4,1);  //b
        PrintStr(" ");
        SendByte(0xBD,1);  //n
        PrintStr("ac");
        SendByte(0xBF,1);  //t
        PrintStr("p.");
        Cursor(1,0);
        PrintStr("  ");
        SendByte(0xE0,1);  //D
        PrintStr("a      He");
        SendByte(0xBF,1);  //t
        switch (choise)
        {
          case 0:
            Cursor(1,4);
            break;
          case 1:
            Cursor(1,11);
            break;
        }
        if (button>1000) 
          {
            if (!choise){
              flag=0;
              start=0;
              curPoint=0;
              choise=0;
              NVIC_DisableIRQ(EXTI3_IRQn);
              osThreadResume(Flash_taskHandle);
            }else{
              flag=0;
              start=0;
              curPoint=0;
              choise=0;
              
              //-------------------------Flash---------------------//
              if (flash_read(User_Page_Adress[0])!=0xFFFFFFFF)
               {
                    MAX_EU=flash_read(User_Page_Adress[0]);
                    MIN_EU=flash_read(User_Page_Adress[0])>>16;
                    direct=flash_read(User_Page_Adress[1]);
                    sensor=flash_read(User_Page_Adress[1])>>8;
                    brightness=flash_read(User_Page_Adress[1])>>16;
                    hysteresys=flash_read(User_Page_Adress[2]);
                    P1=flash_read(User_Page_Adress[2])>>8;  
                    I1=flash_read(User_Page_Adress[2])>>16;
                    D1=flash_read(User_Page_Adress[2])>>24; 
                    autostart=flash_read(User_Page_Adress[3]);
                    point_num=flash_read(User_Page_Adress[3])>>8;  
                    for (flash_i=0;flash_i<20;flash_i++)
                      {
                        point[flash_i].Minuts=flash_read(User_Page_Adress[flash_i+4]);
                        point[flash_i].Sec=flash_read(User_Page_Adress[flash_i+4])>>8;
                        point[flash_i].target=flash_read(User_Page_Adress[flash_i+4])>>16;  
                      }
               }

              //====================================================//
            }
          } else if (button>100) 
          {
            choise=(choise>1)?0:choise+1;
          } 
      
      break; 
    }
    button=0;
    osDelay(350);    
  
  }
  /* USER CODE END Screen */
}

/* USER CODE BEGIN Header_Flash */
/**
* @brief Function implementing the Flash_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Flash */
void Flash(void *argument)
{
  /* USER CODE BEGIN Flash */
  uint32_t flash_ret;
  /* Infinite loop */
  for(;;)
  {
 
      osThreadSuspend(Flash_taskHandle);
 
      HAL_FLASH_Unlock();
      Erase.TypeErase=FLASH_TYPEERASE_PAGES;
      Erase.PageAddress=User_Page_Adress[0];
      Erase.NbPages=1;
      HAL_FLASHEx_Erase(&Erase,&flash_ret);
      if (flash_ret==0xFFFFFFFF)
      {
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress[0],(MAX_EU&0x0000FFFF)|((MIN_EU<<16)&0xFFFF0000));
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress[1],(direct&0x000000FF)|((sensor<<8)&0x0000FF00)|((brightness<<16)&0xFFFF0000));
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress[2],(hysteresys&0x000000FF)|((P1<<8)&0x0000FF00)|((I1<<16)&0x00FF0000)|((D1<<24)&0xFF000000));
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress[3],(autostart&0x000000FF)|((point_num<<8)&0x0000FF00));
       for (flash_i=0;flash_i<20;flash_i++)
            {
              HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress[flash_i+4],(point[flash_i].Minuts&0x000000FF)|((point[flash_i].Sec<<8)&0x0000FF00)|((point[flash_i].target<<16)&0xFFFF0000)); 
            }
 
      
      HAL_FLASH_Lock();
      flash_ret=0;
      }
    
  }
  /* USER CODE END Flash */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
