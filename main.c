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
#include "sh1106.h"
#include "task.h"
//#include "socket.h"
//#include "mqtt_interface.h"
//#include "MQTTClient.h"


//#include "w5500.h"
//#include "wizchip_conf.h"



//#include "socket.c"
//#define HTTP_SOCKET     0
//#define TCP_SOCKET       0
//#define PORT_TCPS		    5000
//#define DATA_BUF_SIZE   2048
//uint8_t gDATABUF[DATA_BUF_SIZE];
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
  .stack_size = 128 * 4
};
/* Definitions for Screen_task */
osThreadId_t Screen_taskHandle;
const osThreadAttr_t Screen_task_attributes = {
  .name = "Screen_task",
  .priority = (osPriority_t) osPriorityLow7,
  .stack_size = 128 * 4
};
/* Definitions for MQTT_task */
osThreadId_t MQTT_taskHandle;
const osThreadAttr_t MQTT_task_attributes = {
  .name = "MQTT_task",
  .priority = (osPriority_t) osPriorityLow6,
  .stack_size = 256 * 4
};
/* Definitions for MonitorTask */
osThreadId_t MonitorTaskHandle;
const osThreadAttr_t MonitorTask_attributes = {
  .name = "MonitorTask",
  .priority = (osPriority_t) osPriorityLow4,
  .stack_size = 64 * 4
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
void main_func(void *argument);
void Screen(void *argument);
void MQTT(void *argument);
void monitor(void *argument);

/* USER CODE BEGIN PFP */
void Flash_save(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//--------my global variables----------//
uint16_t EncoderVal, EncoderValOld, power=0,voltage,voltage_LL=0,voltage_HH=0,Time_b=0,Time_c=0,Reciver_capacyty=0,current_a,current_b,current_c,Time_Pmin_Pmax_old,Time_Pmin_Pmax=0;
int16_t Flow_current=0,Flow_nominal=0, P_display, P_max_display,P_min_display,P_HH_display,P_LL_display,total_flow_lit_min=0,counter_flow_lit_min=0;
uint8_t P=0,error=0, error_first=0,P_max,P_min,P_HH,P_LL,Current_diference,T1_max,T2_max,P_ini=0,cosFI, dreamenable=0,dream=0, alarm_display=0,
current=0,phase_control,phase_a_work=0,phase_b_work=0,phase_c_work=0,short_circut_current,dP_time,power_nom,dP_error,flag=0,iter=0,koef_Pres=0,koef_U=0, save=0;
int8_t T1,T2;
RTC_TimeTypeDef CurTime = {0};
RTC_DateTypeDef CurDate={0};
uint32_t button=0,worktime,flash_ret;
FLASH_EraseInitTypeDef Erase;
//========================================//

//-----------------------------------------------------------------------------------------temp!!!!!---------------------------------------------------------------//
/*
wiz_NetInfo gWIZNETINFO = { .mac = {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},
                            .ip = {192, 168, 0, 3},
                            .sn = {255, 255, 255, 0},
                            .gw = {192, 168, 0, 1},
                            .dns = {0, 0, 0, 0},
                            .dhcp = NETINFO_STATIC };
  uint8_t targetIP[4] = {192, 168, 0, 1};   //brocker IP adress

  
  
  
void W5500_Select(void)
{
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
}

void W5500_Unselect(void) 
{
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

void W5500_ReadBuff(uint8_t* buff, uint16_t len) 
{
    HAL_SPI_Receive(&hspi2, buff, len, 1000);
}

void W5500_WriteBuff(uint8_t* buff, uint16_t len) 
{
    HAL_SPI_Transmit(&hspi2, buff, len, 1000);
}

uint8_t W5500_ReadByte(void) 
{
    uint8_t byte;
    W5500_ReadBuff(&byte, sizeof(byte));
    return byte;
}

void W5500_WriteByte(uint8_t byte) 
{
    W5500_WriteBuff(&byte, sizeof(byte));
}

struct opts_struct
{
	char* clientid;
	int nodelimiter;
	char* delimiter;
	enum QoS qos;
	char* username;
	char* password;
	char* host;
	int port;
	int showtopics;
} opts ={ (char*)"stdout-subscriber", 0, (char*)"\n", QOS0, NULL, NULL, targetIP, 1883, 0 };


// @brief messageArrived callback function
void messageArrived(MessageData* md)
{
	MQTTMessage* message = md->message;

}
*/
//-----------------------------------------------------------------------------------------!---------------------------------------------------------------//





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_IWDG_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  
  //-------------------------Flash---------------------//
  if (flash_read(User_Page_Adress[0])!=0xFFFFFFFF)
     {
          Reciver_capacyty=flash_read(User_Page_Adress[0]);
          Time_Pmin_Pmax_old=flash_read(User_Page_Adress[0])>>16;
          Flow_nominal=flash_read(User_Page_Adress[1]);
          P_max=flash_read(User_Page_Adress[1])>>16;
          P_min=flash_read(User_Page_Adress[1])>>24;
          P_HH=flash_read(User_Page_Adress[2]);
          P_LL=flash_read(User_Page_Adress[2])>>8;  
          Current_diference=flash_read(User_Page_Adress[2])>>16;
          T1_max=flash_read(User_Page_Adress[2])>>24; 
          T2_max=flash_read(User_Page_Adress[3]);
          cosFI=flash_read(User_Page_Adress[3])>>8;  
          phase_control=flash_read(User_Page_Adress[3])>>16; 
          short_circut_current=flash_read(User_Page_Adress[3])>>24; 
          dP_time=flash_read(User_Page_Adress[4]);
          power_nom=flash_read(User_Page_Adress[4])>>8;
          dP_error=flash_read(User_Page_Adress[4])>>16;
          worktime=flash_read(User_Page_Adress[5]);
          voltage_LL=flash_read(User_Page_Adress[6]);
          voltage_HH=flash_read(User_Page_Adress[6])>>16;
          koef_Pres=flash_read(User_Page_Adress[7])>>16;
          koef_U=flash_read(User_Page_Adress[7])>>24;
          
     }
  //====================================================//
  

    
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

  /* creation of Screen_task */
  Screen_taskHandle = osThreadNew(Screen, NULL, &Screen_task_attributes);

  /* creation of MQTT_task */
 // MQTT_taskHandle = osThreadNew(MQTT, NULL, &MQTT_task_attributes);

  /* creation of MonitorTask */
  MonitorTaskHandle = osThreadNew(monitor, NULL, &MonitorTask_attributes);

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
  hadc1.Init.NbrOfConversion = 7;
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
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
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
  hi2c1.Init.OwnAddress1 = 82;
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
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 3;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2047;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 7;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 7;
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
  htim4.Init.Prescaler = 65534;
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
  HAL_GPIO_WritePin(GPIOB, On_LED_Pin|SPI2_CS_Pin|Start_solenoid_Pin|START_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, W5500_RST_Pin|Alarm_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Phase_C_Pin */
  GPIO_InitStruct.Pin = Phase_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Phase_C_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Phase_A_Pin Phase_B_Pin */
  GPIO_InitStruct.Pin = Phase_A_Pin|Phase_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Stop_btn_Pin */
  GPIO_InitStruct.Pin = Stop_btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Stop_btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : On_LED_Pin */
  GPIO_InitStruct.Pin = On_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(On_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : W5500_RST_Pin Alarm_Pin */
  GPIO_InitStruct.Pin = W5500_RST_Pin|Alarm_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : W5500_INT_Pin */
  GPIO_InitStruct.Pin = W5500_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(W5500_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Start_solenoid_Pin START_Pin */
  GPIO_InitStruct.Pin = Start_solenoid_Pin|START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//---------------------------FLASH--------------------//
void Flash_save(void)
{
          taskENTER_CRITICAL();
          HAL_FLASH_Unlock();
          Erase.TypeErase=FLASH_TYPEERASE_PAGES;
          Erase.PageAddress=User_Page_Adress[0];
          Erase.NbPages=1;
          HAL_FLASHEx_Erase(&Erase,&flash_ret);
          if (flash_ret==0xFFFFFFFF)
          {
          HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress[0],(Reciver_capacyty&0x0000FFFF)|((Time_Pmin_Pmax_old<<16)&0xFFFF0000));
          HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress[1],(Flow_nominal&0x0000FFFF)|((P_max<<16)&0x00FF0000)|((P_min<<24)&0xFF000000));
          HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress[2],(P_HH&0x000000FF)|((P_LL<<8)&0x0000FF00)|((Current_diference<<16)&0x00FF0000)|((T1_max<<24)&0xFF000000));
          HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress[3],(T2_max&0x000000FF)|((cosFI<<8)&0x0000FF00)|((phase_control<<16)&0x00FF0000)|((short_circut_current<<24)&0xFF000000));
          HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress[4],(dP_time&0x000000FF)|((power_nom<<8)&0x0000FF00)|((dP_error<<16)&0x00FF0000)); 
          HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress[5],worktime);    
          HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress[6],(voltage_LL&0x0000FFFF)|((voltage_HH<<16)&0xFFFF0000));
          HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress[7],(((koef_Pres<<16)&0x00FF0000)|((koef_U<<24)&0xFF000000)));         
          HAL_FLASH_Lock();
          flash_ret=0;
          }     
          taskEXIT_CRITICAL();

     //====================================================//
	
	
}


//---------------------------exti--------------------//

void EXTI0_IRQHandler(void)
{
    __HAL_TIM_SET_COUNTER(&htim2,0);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
    phase_a_work=1;
}
void EXTI1_IRQHandler(void)
{
    Time_b=TIM2->CNT;
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
    phase_b_work=1;
}
void EXTI9_5_IRQHandler(void)
{
    Time_c=TIM2->CNT;
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
    phase_c_work=1;
}

//====================================================//
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
  
  
  volatile uint16_t dma[7];
  volatile uint8_t P_array[7];
  uint8_t secound_old=99,minute_old=0,P_old=0,P_time_old,furst_run=0,P_ini=1,time_10s=0,P_old_10s=0,strobe=0,NTC_t,day_old,sim_P_dec=2,run=0;
  uint16_t start_time=0,secound_increment=0,secound_inc_old=0;
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  HAL_TIM_IC_Start_DMA(&htim4,TIM_CHANNEL_2,&button,1);
  HAL_TIM_Base_Start(&htim2);
  HAL_GPIO_WritePin(Start_solenoid_GPIO_Port,Start_solenoid_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(START_GPIO_Port,START_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(On_LED_GPIO_Port,On_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Alarm_GPIO_Port,Alarm_Pin, GPIO_PIN_RESET);
  osDelay(500);
  HAL_ADC_Stop(&hadc1);
  HAL_ADC_Stop_DMA(&hadc1);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&dma,7);
  osDelay(50);
  //-------------------simulation init-------------------------------//
  if (simulation==1)
  {
   dma[0]=0;  
   dma[1]=0;  
   dma[2]=0;     
   dma[3]=0;  
   dma[4]=0;  
   dma[5]=0;   
   dma[6]=0;
  }
  //-----------------------------------------------------------------//
  P=dma[3]/32; //128==12.8at==3v
  P_array[0]=P;
  P_array[1]=P;
  P_array[2]=P;
  P_array[3]=P;
  P_array[4]=P;
  P_array[5]=P;
  P_array[6]=P;
  P=P*koef_Pres/100;
  HAL_RTC_GetDate(&hrtc, &CurDate, RTC_FORMAT_BIN);
  day_old=CurDate.Date;
  
  /* Infinite loop */
  
  for(;;)
  {
    //-------------phases and delay------------//
    if (phase_control!=0){
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn); 
    phase_a_work=0;
    phase_b_work=0;
    phase_c_work=0;
    osDelay(42);
    NVIC_DisableIRQ(EXTI0_IRQn);
    NVIC_DisableIRQ(EXTI1_IRQn);
    NVIC_DisableIRQ(EXTI9_5_IRQn);}
    else{
    phase_a_work=0;
    phase_b_work=0;
    phase_c_work=0;
    NVIC_DisableIRQ(EXTI0_IRQn);
    NVIC_DisableIRQ(EXTI1_IRQn);
    NVIC_DisableIRQ(EXTI9_5_IRQn);
    }

    //-------------ADC IDWG Encoder screendream------------//
    HAL_IWDG_Refresh(&hiwdg);
    HAL_RTC_GetTime(&hrtc, &CurTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &CurDate, RTC_FORMAT_BIN);
    HAL_ADC_Stop(&hadc1);
    HAL_ADC_Stop_DMA(&hadc1);
    
    //----------------------simulation------------------------//
    if (simulation==1)
    {
        //dma[0]=(4096>(dma[0]+sim_v_inc))?(dma[0]+sim_v_inc):dma[0];  //voltage  increse
        // dma[1]=(4096>(dma[1]+sim_t1_inc))?(dma[1]+sim_t1_inc):dma[1];  //t1   increse
        //dma[2]=(4096>(dma[2]+sim_t2_inc))?(dma[2]+sim_t2_inc):dma[2];  //t2  increse
         dma[3]=(dma[3]>sim_P_dec)?(dma[3]-sim_P_dec):dma[3];  //pressure decrese
        // dma[4]=0;  //current a
        // dma[5]=0;  //current b
        // dma[6]=0;  //current c    
    }else{
      HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&dma,7);
    }
    
    
    EncoderVal=__HAL_TIM_GET_COUNTER(&htim3);
    if (EncoderVal!=EncoderValOld){
      dream=0;
      secound_increment=secound_inc_old;}
    EncoderValOld=EncoderVal;
    osDelay(10);
    voltage=dma[0]*koef_U/(1131.37);       //real_voltage 511v==4095==3v  8*sqrt(2)
    voltage_LL&=0x1FF;
    voltage_HH&=0x1FF;
    for(iter=0;iter<6;iter++)
      {
       P_array[iter]= P_array[iter+1];  
      }
     P_array[6]=dma[3]/32;
    P=(P_array[6]+P_array[5]+P_array[4]+P_array[3]+P_array[2]+P_array[1]+P_array[0])/7;                  //128==12.8at==3v
    P=P*koef_Pres/100;
    current_a=(current_a+dma[4])/2;
    current_b=(current_b+dma[5])/2;
    current_c=(current_c+dma[6])/2;  //5mA==1 20.48A==4096==3v
    NTC_t=0;
    while (NTC_10k[NTC_t]>dma[1])
        {
          NTC_t++;
        }
      NTC_t=(NTC_t==0)?1:NTC_t;
      if ((NTC_10k[NTC_t-1]-dma[1])>(dma[1]-NTC_10k[NTC_t]))
        {
          T1=NTC_t-5;
        }else
        {
          T1=NTC_t-6;
        }
    NTC_t=0;
    while (NTC_10k[NTC_t]>dma[2])
        {
          NTC_t++;
        }
      NTC_t=(NTC_t==0)?1:NTC_t;
      if ((NTC_10k[NTC_t-1]-dma[2])>(dma[2]-NTC_10k[NTC_t]))
        {
          T2=NTC_t-5;
        }else
        {
          T2=NTC_t-6;
        }
    
    
    
    
    //=========================================//
    
    //----------------one secound strobe-----------------//
    if ((secound_old!=CurTime.Seconds)&&(!strobe))
    {
      strobe=1;
      secound_old=CurTime.Seconds;
      secound_increment++;
    }  
    //=========================================// 
    
    
    //----------------dream-----------------//
    if (!dreamenable){secound_inc_old=secound_increment;}
    if (secound_increment>secound_inc_old+dreamtime)
    {
      dream=1;
    }
    secound_inc_old=(secound_inc_old>secound_increment)?secound_increment:secound_inc_old;
    
    //=========================================// 
    
      //-------------------calculations power, phase quantity,difference between phase------------------------//
    if(phase_a_work&&phase_b_work&&phase_c_work)
      {
        power=3*voltage*cosFI*(current_a+current_b+current_c)/(3*200*100);   //current*1000/5=current/200 A cosFI(0,5..1)=0.5==50, 1==100
        if (Current_diference>0){
          if (current_a>((current_b+current_c+Current_diference)/2)){error=10;}                //255=2.55 A*100 1==0,01A
          if (current_a<((current_b+current_c-Current_diference)/2)){error=11;}
          if (current_b>((current_a+current_c+Current_diference)/2)){error=12;} 
          if (current_b<((current_a+current_c-Current_diference)/2)){error=13;} 
          if (current_c>((current_a+current_b+Current_diference)/2)){error=14;} 
          if (current_c<((current_a+current_b-Current_diference)/2)){error=15;} 
        }
    }else if ((phase_a_work==1)&&((phase_control==1)||(phase_control==2)))
      {
        if (!phase_b_work)
        {
          error=19;  
        }
        if (!phase_c_work)
        {
          error=20;  
        }
        
    }else if ((phase_a_work==0)&&(phase_control!=0))
      { 
        error=18;
    }else if (phase_control==0||phase_control==3)
      { 
        power=voltage*cosFI*(current_a)/(200*100);
      }
      //=========================================//
    

    
    
    //-------------------calculations short current, temperature, pressure sensor,stop_btn, Voltage up or down------------------------//
    if ((short_circut_current)&&(short_circut_current<current_a/20)){error=7;}                                                      //255=25.5A*10, 4096/200
    if ((short_circut_current)&&(short_circut_current<current_b/20)){error=8;}
    if ((short_circut_current)&&(short_circut_current<current_c/20)){error=9;}
    if ((T1_max)&&(T1>T1_max)){error=2;}
    if ((T2_max)&&(T2>T2_max)){error=3;}
    if ((P_HH)&&(P_HH<P)){error=5;}
    if ((P_LL)&&(P_LL>P)){error=6;}
    if (HAL_GPIO_ReadPin(Stop_btn_GPIO_Port,Stop_btn_Pin)==GPIO_PIN_SET){error=21;}
    if ((voltage>voltage_HH)&&(voltage_HH)){error=23;}
    if ((voltage<voltage_LL)&&(voltage_LL)){error=22;}
    //=========================================//
    
    //-------------------phase control-------------------//
    if ((phase_control==1)&&(Time_b>Time_c)){error=1;}
    if ((phase_control==2)&&(Time_b<Time_c)){error=1;}
    //==================================================//
    if (day_old!=CurDate.Date)
    {
      save=1;
      day_old=CurDate.Date;
    }    
    //----------------------------startuem----------------------------------//
    if ((!error)&&(P<P_min))
      {
        //----------------------------furst run----------------------------------//
        if (worktime==0)
          {
            furst_run=1;
          }
        //----------------------------------------------------------------------//
        
        //----------------------------run----------------------------------/
        HAL_GPIO_WritePin(On_LED_GPIO_Port,On_LED_Pin, GPIO_PIN_SET);
        run=1;
        //-------------------------------------------------------------------//

      }
    else if((error)||(P>P_max))
      {
        if (furst_run){
           furst_run=0;
           start_time=(start_time>3)?start_time:4;
           Flow_nominal=((P-P_ini)*Reciver_capacyty)*6/(start_time-3);
           Flow_nominal=((Flow_nominal<9999)&&(Flow_nominal>0))?Flow_nominal:0;
           Time_Pmin_Pmax_old=start_time;
          }
        if (run)
        {
          HAL_GPIO_WritePin(On_LED_GPIO_Port,On_LED_Pin, GPIO_PIN_RESET);
        }
        run=0;
        P_time_old=0;
        start_time=0;
        
        HAL_GPIO_WritePin(START_GPIO_Port,START_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Start_solenoid_GPIO_Port,Start_solenoid_Pin, GPIO_PIN_RESET);
      }
    
    
    if (run)
    {       
        if (strobe)
            {
              if (start_time>0)
              {
                HAL_GPIO_WritePin(START_GPIO_Port,START_Pin, GPIO_PIN_SET);
              }
              //----------------------------solenoid----------------------------------//
              if (start_time<4)
              {
                P_ini=P;
                P_old=P; 
                P_time_old=0;
                HAL_GPIO_WritePin(Start_solenoid_GPIO_Port,Start_solenoid_Pin, GPIO_PIN_SET);
              }else{
                HAL_GPIO_WritePin(Start_solenoid_GPIO_Port,Start_solenoid_Pin, GPIO_PIN_RESET);
              } 
              //-----------------------------------------------------------------------//
              start_time++;
              worktime++;
              time_10s++;  
              Time_Pmin_Pmax=start_time;
              if (start_time>5)
                {
                  if ((power_nom*40>power)&&power_nom){error=17;}  //wrong load
                  if (P<(P_old+3)&&P>(P_old-3)){
                    P_time_old++;
                    if ((P_time_old>dP_time)&&dP_time){error=16;}  //pressure not change
                  }else{
                    P_time_old=0;
                    P_old=P; 
                  }              
                }

          }
        //----------------------simulation------------------------//
        if (simulation==1)
        {
            //dma[0]=(4096>(dma[0]+sim_v_inc))?(dma[0]+sim_v_inc):dma[0];  //voltage  increse
            // dma[1]=(4096>(dma[1]+sim_t1_inc))?(dma[1]+sim_t1_inc):dma[1];  //t1   increse
            //dma[2]=(4096>(dma[2]+sim_t2_inc))?(dma[2]+sim_t2_inc):dma[2];  //t2  increse
             dma[3]=(4096>(dma[3]+sim_P_dec*6))?(dma[3]+sim_P_dec*6):dma[3];  //pressure decrese
            // dma[4]=0;  //current a
            // dma[5]=0;  //current b
            // dma[6]=0;  //current c    
        }        
    
    }
      

    //==================================================//
    if (strobe)
        {
            if ((P_old_10s>P)&&(dP_error)&&(P_old_10s-P)>dP_error){error=4;}  //depressurization
            if (error)
              {
                HAL_GPIO_WritePin(On_LED_GPIO_Port,On_LED_Pin, GPIO_PIN_RESET);
                if (alarm_display>5)
                  {
                    if (alarm_display%2)
                      {
                        if(0x01&(error>>((alarm_display-7)/2)))
                          {
                            HAL_GPIO_WritePin(Alarm_GPIO_Port,Alarm_Pin, GPIO_PIN_SET);
                            
                          }else{
                            HAL_GPIO_WritePin(On_LED_GPIO_Port,On_LED_Pin, GPIO_PIN_SET);
                          }  
                      }else{
                        HAL_GPIO_WritePin(Alarm_GPIO_Port,Alarm_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(On_LED_GPIO_Port,On_LED_Pin, GPIO_PIN_RESET);
                      }
                  }else{
                    HAL_GPIO_WritePin(Alarm_GPIO_Port,Alarm_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(On_LED_GPIO_Port,On_LED_Pin, GPIO_PIN_RESET);
                  }
                alarm_display++;
                alarm_display=(alarm_display>16)?0:alarm_display;
              }else{
                  
                  HAL_GPIO_WritePin(Alarm_GPIO_Port,Alarm_Pin, GPIO_PIN_RESET);
              }
            /*
                  {
                    HAL_GPIO_TogglePin(Alarm_GPIO_Port,Alarm_Pin);								
                  }else{
                    HAL_GPIO_WritePin(Alarm_GPIO_Port,Alarm_Pin, GPIO_PIN_RESET);
                  }*/
        }    
    //----------------------------flow 6s----------------------------------//
    if ((CurTime.Seconds%6==5)&&(strobe))
     {
       if (start_time>0){
        Flow_current=(P-P_old_10s)*Reciver_capacyty-Flow_nominal;
       }else{
        Flow_current=(P-P_old_10s)*Reciver_capacyty;
       }
       counter_flow_lit_min=counter_flow_lit_min+Flow_current;
       P_old_10s=P;
     }
    //=======================================================================================//
    
    //---------------------------minute strobe---------------------------------------------//
    if (CurTime.Minutes!=minute_old)
    {
      total_flow_lit_min=counter_flow_lit_min/(-10);
      counter_flow_lit_min=0;
    
    }
    
    //======================================================================================//
    strobe=0;
    if ((error!=0)&&(error_first==0))
    {
      error_first=error;
    }
    osDelay(100);
  }
  /* USER CODE END 5 */
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
  uint8_t choise=0,worktimeclear=0,alredydream=0;
   //----------Horse---------------
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();
  startScreen();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(32,16); 
  startScreen();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(64,32); 
  startScreen();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(96,16); 
  startScreen();
  ssd1306_Fill(Black);
  //------------------------------------
  
  osDelay(500);
  
  /* Infinite loop */
  for(;;)
  {
    
    switch(flag)
    {
    case 0:
        //------------------------------startyem--------------------------------//
                dreamenable=1;
		ssd1306_SetCursor(0,0);
		ssd1306_WriteString("Flow nom:",Font_7x10,White);
		sprintf(R,"%04d",Flow_nominal);
		ssd1306_WriteString(R,Font_7x10,White);
		ssd1306_WriteString("L/min",Font_7x10,White);
		ssd1306_SetCursor(0,11);
		ssd1306_WriteString("Ia:",Font_7x10,White);
		sprintf(R,"%02d%c%01d",current_a/200,'.',(current_a/20)%10);
		ssd1306_WriteString(R,Font_7x10,White);
		ssd1306_WriteString("A  T1:",Font_7x10,White);
		sprintf(R,"%03d",T1);
		ssd1306_WriteString(R,Font_7x10,White);
		ssd1306_WriteString("C",Font_7x10,White);
		ssd1306_WriteChar(0x7F,Font_7x10,White);
		ssd1306_SetCursor(0,22);
		ssd1306_WriteString("Ib:",Font_7x10,White);
		sprintf(R,"%02d%c%01d",current_b/200,'.',(current_b/20)%10);
		ssd1306_WriteString(R,Font_7x10,White);
		ssd1306_WriteString("A  T2:",Font_7x10,White);
		sprintf(R,"%03d",T2);
		ssd1306_WriteString(R,Font_7x10,White);
		ssd1306_WriteString("C",Font_7x10,White);
		ssd1306_WriteChar(0x7F,Font_7x10,White);
		ssd1306_SetCursor(0,33);
		ssd1306_WriteString("Ic:",Font_7x10,White);
		sprintf(R,"%02d%c%01d",current_c/200,'.',(current_c/20)%10);
		ssd1306_WriteString(R,Font_7x10,White);
		ssd1306_WriteString("A Work",Font_7x10,White);
		sprintf(R,"%04u%c",worktime/3600,'h');
		ssd1306_WriteString(R,Font_7x10,White);
		ssd1306_SetCursor(0,43);
		ssd1306_WriteString("Ton  ",Font_7x10,White);
		sprintf(R,"%04d",Time_Pmin_Pmax);
		ssd1306_WriteString(R,Font_7x10,White);
		ssd1306_WriteString("s V:",Font_7x10,White);
		sprintf(R,"%03d",voltage);
		ssd1306_WriteString(R,Font_7x10,White);
		ssd1306_WriteString(" V",Font_7x10,White);
		ssd1306_SetCursor(0,53);
		ssd1306_WriteString("TNom ",Font_7x10,White);
		sprintf(R,"%04d%c",Time_Pmin_Pmax_old,'s');
		ssd1306_WriteString(R,Font_7x10,White);
		
        if (error!=0)
          {
            flag=5;//5
            break;
          }
        if (button>1000) 
          {
            flag=3;
            __HAL_TIM_SET_COUNTER(&htim3,P_max);
            dream=0;
          } else if (button>100)
          {
            flag=1;
            dream=0;
          }
        break;
    case 1:
        //------------------------------prodoljaem--------------------------------//
                dreamenable=0;
                P_display=P-10;
		ssd1306_SetCursor(0,7);
		ssd1306_WriteString("Pressure:",Font_7x10,White);
		ssd1306_SetCursor(63,0);
                if (P==0){
                  ssd1306_WriteString("-1.",Font_11x18,White);
                  sprintf(R,"%01d",P);
                }else if (P<10)
                {
                  ssd1306_WriteString("-0.",Font_11x18,White);
                  sprintf(R,"%01d",(10-P));
                }else{
                  sprintf(R,"%02d%c%01d",P_display/10,'.',P_display%10);}
		ssd1306_WriteString(R,Font_11x18,White);
		ssd1306_SetCursor(107,7);
		ssd1306_WriteString("kg",Font_7x10,White);
		//ssd1306_Draw_dot_colum_line(0,20);
		ssd1306_SetCursor(0,29);
		ssd1306_WriteString("Flow:",Font_7x10,White);
		ssd1306_SetCursor(35,22);
		sprintf(R,"%05d",Flow_current);
		ssd1306_WriteString(R,Font_11x18,White);
		ssd1306_SetCursor(90,29);
		ssd1306_WriteString("L/min",Font_7x10,White);
		//ssd1306_Draw_dot_colum_line(0,43);
		ssd1306_SetCursor(0,52);
		ssd1306_WriteString("Power:",Font_7x10,White);
		ssd1306_SetCursor(63,45);
		sprintf(R,"%02d%c%01d",power/1000,'.',(power/100)%10);
		ssd1306_WriteString(R,Font_11x18,White);
		ssd1306_SetCursor(107,52);
		ssd1306_WriteString("kW",Font_7x10,White);
        if (error!=0)
          {
            flag=5; 
            break;
          }
        if (button>1000) 
          {
            flag=3;
            __HAL_TIM_SET_COUNTER(&htim3,P_max);
          } else if (button>100)
          {
            flag=2;
          }
        break;
    case 2:
        //------------------------------do sih por--------------------------------//
	dreamenable=0;
        if (error!=0)
          {
            flag=5; 
            //break;
          }
        if (button>1000) 
          {
            flag=3;
            __HAL_TIM_SET_COUNTER(&htim3,P_max);
          } else if (button>100)
          {
            flag=0;
          }       
        break;    
    case 3:
        //------------------------------settings page1--------------------------------//
        dreamenable=0;
        ssd1306_SetCursor(0,0);
                P_max_display=P_max-10;
		ssd1306_WriteString("Pmax:",Font_7x10,White);
                
                if (P_max_display==-10){
                  ssd1306_WriteString("-1.",Font_7x10,White);
                  sprintf(R,"%01d",P_max);
                }
                else if (P_max_display<0)
                {
                  ssd1306_WriteString("-0.",Font_7x10,White);
                  sprintf(R,"%01d",(10-P_max));
                }else{
                  sprintf(R,"%02d%c%01d",P_max_display/10,'.',P_max_display%10);}
		if (choise==0)
			{
				ssd1306_WriteString(R,Font_7x10,CurTime.Seconds%2);		
			}else{
				ssd1306_WriteString(R,Font_7x10,White);
			}		
		ssd1306_WriteString(" kg/cm",Font_7x10,White);
                
		ssd1306_WriteChar(0x80,Font_7x10,White);
		ssd1306_SetCursor(0,10);
                P_min_display=P_min-10;
		ssd1306_WriteString("Pmin:",Font_7x10,White);
                if (P_min_display==-10){
                  ssd1306_WriteString("-1.",Font_7x10,White);
                  sprintf(R,"%01d",P_min);
                }
                else if (P_min_display<0)
                {
                  ssd1306_WriteString("-0.",Font_7x10,White);
                  sprintf(R,"%01d",(10-P_min));
                }else{
                  sprintf(R,"%02d%c%01d",P_min_display/10,'.',P_min_display%10);}
		if (choise==1)
			{
				ssd1306_WriteString(R,Font_7x10,CurTime.Seconds%2);		
			}else{
				ssd1306_WriteString(R,Font_7x10,White);
			}	
                
		ssd1306_WriteString(" kg/cm",Font_7x10,White);
                
		ssd1306_WriteChar(0x80,Font_7x10,White);
		ssd1306_SetCursor(0,21);
                P_LL_display=P_LL-10;
		ssd1306_WriteString("P<LL:",Font_7x10,White);
		if (P_LL_display==-10){
                  ssd1306_WriteString("-1.",Font_7x10,White);
                  sprintf(R,"%01d",P_LL);
                }
                else if (P_LL_display<0)
                {
                  ssd1306_WriteString("-0.",Font_7x10,White);
                  sprintf(R,"%01d",(10-P_LL));
                }else{
                  sprintf(R,"%02d%c%01d",P_LL_display/10,'.',P_LL_display%10);
                }
		if (choise==2)
			{
				ssd1306_WriteString(R,Font_7x10,CurTime.Seconds%2);		
			}else{
				ssd1306_WriteString(R,Font_7x10,White);
			}			
                ssd1306_WriteString(" kg/cm",Font_7x10,White);
                
		ssd1306_WriteChar(0x80,Font_7x10,White);
		ssd1306_SetCursor(0,32);
		P_HH_display=P_HH-10;
		ssd1306_WriteString("P<HH:",Font_7x10,White);
		if (P_HH_display==-10){
                  ssd1306_WriteString("-1.",Font_7x10,White);
                  sprintf(R,"%01d",P_HH);
                }
                else if (P_HH_display<0)
                {
                  ssd1306_WriteString("-0.",Font_7x10,White);
                  sprintf(R,"%01d",(10-P_HH));
                }else{
                  sprintf(R,"%02d%c%01d",P_HH_display/10,'.',P_HH_display%10);
                }
                if (choise==3)
			{
				ssd1306_WriteString(R,Font_7x10,CurTime.Seconds%2);		
			}else{
				ssd1306_WriteString(R,Font_7x10,White);
			}	
		ssd1306_WriteString(" kg/cm",Font_7x10,White);
		ssd1306_WriteChar(0x80,Font_7x10,White);
		ssd1306_SetCursor(0,43);
		ssd1306_WriteString("dPt :",Font_7x10,White);
		sprintf(R,"%02d",dP_time);
		if (choise==4)
			{
				ssd1306_WriteString(R,Font_7x10,CurTime.Seconds%2);		
			}else{
				ssd1306_WriteString(R,Font_7x10,White);
			}	
		ssd1306_WriteString("c dPErr",Font_7x10,White);
		sprintf(R,"%02d%c%01d",dP_error/10,'.',dP_error%10);
		if (choise==5)
			{
				ssd1306_WriteString(R,Font_7x10,CurTime.Seconds%2);		
			}else{
				ssd1306_WriteString(R,Font_7x10,White);
			}	
		
		//ssd1306_Draw_dot_colum_line(0,53);
		ssd1306_SetCursor(0,53);
		ssd1306_WriteString("ReciverCap:",Font_7x10,White);
		sprintf(R,"%03d",Reciver_capacyty);
		if (choise==6)
			{
				ssd1306_WriteString(R,Font_7x10,CurTime.Seconds%2);		
			}else{
				ssd1306_WriteString(R,Font_7x10,White);
			}	
		ssd1306_WriteString(" lit",Font_7x10,White);
		
		switch (choise)
			{
			  case 0:
				P_max=EncoderVal%151;
				break;
			  case 1:
				P_min=EncoderVal%151;
				break;
			  case 2:
				P_LL=EncoderVal%151;
				break;
			  case 3:
				P_HH=EncoderVal%151;
				break;
			  case 4:
				dP_time=EncoderVal%100;
				break;
			  case 5:
				dP_error=EncoderVal%128;
				break;
			  case 6:
				Reciver_capacyty=EncoderVal%1000;
				break;				
			}
		
        if (button>1000) 
          {
            flag=4;
            choise=0;
            __HAL_TIM_SET_COUNTER(&htim3,phase_control);
          } else if (button>100){
            choise=(choise>=6)?0:choise+1;
			switch (choise)
              {
                case 0:
                  __HAL_TIM_SET_COUNTER(&htim3,P_max);
                  break;
                case 1:
                  __HAL_TIM_SET_COUNTER(&htim3,P_min);
                  break;
                case 2:
                  __HAL_TIM_SET_COUNTER(&htim3,P_LL);
                  break;
                case 3:
                  __HAL_TIM_SET_COUNTER(&htim3,P_HH);
                  break;
				case 4:
                  __HAL_TIM_SET_COUNTER(&htim3,dP_time);
                  break;
                case 5:
                  __HAL_TIM_SET_COUNTER(&htim3,dP_error);
                  break;
                case 6:
                  __HAL_TIM_SET_COUNTER(&htim3,Reciver_capacyty);
                  break;
              }
          }       
        break;    
    case 4:
        //------------------------------settings page2--------------------------------// 
                dreamenable=0;
		ssd1306_SetCursor(0,0);
		ssd1306_WriteString("Phase",Font_7x10,White);
		if (choise==0)
		{
			switch(phase_control){
                                case 0:
					ssd1306_WriteString("---",Font_7x10,CurTime.Seconds%2);
				break;
				case 1:
					ssd1306_WriteString("ABC",Font_7x10,CurTime.Seconds%2);
				break;
				case 2:
					ssd1306_WriteString("CBA",Font_7x10,CurTime.Seconds%2);
				break;
                                case 3:
					ssd1306_WriteString("-A-",Font_7x10,CurTime.Seconds%2);
				break;

			}
		}else{
			switch(phase_control){
                                case 0:
					ssd1306_WriteString("---",Font_7x10,White);
				break;
				case 1:
					ssd1306_WriteString("ABC",Font_7x10,White);
				break;
				case 2:
					ssd1306_WriteString("CBA",Font_7x10,White);
				break;
                                case 3:
					ssd1306_WriteString("-A-",Font_7x10,White);
				break;
			}
			
		}

		ssd1306_WriteString(" CosFI",Font_7x10,White);
		if (choise==1)
			{
				sprintf(R,"%01d%c%02d",cosFI/100,'.',cosFI%100);
				ssd1306_WriteString(R,Font_7x10,CurTime.Seconds%2);
			}else{
				sprintf(R,"%01d%c%02d",cosFI/100,'.',cosFI%100);
				ssd1306_WriteString(R,Font_7x10,White);
			}
		
		
		
		ssd1306_SetCursor(0,10);
		ssd1306_WriteString("dI :",Font_7x10,White);
		if (choise==2)
			{
				sprintf(R,"%01d%c%02d",Current_diference/100,'.',Current_diference%100);
				ssd1306_WriteString(R,Font_7x10,CurTime.Seconds%2);
			}else{
				sprintf(R,"%01d%c%02d",Current_diference/100,'.',Current_diference%100);
				ssd1306_WriteString(R,Font_7x10,White);
			}
		
		ssd1306_WriteString("A T2:",Font_7x10,White);
		if (choise==3)
			{
				sprintf(R,"%03d",T2_max);
				ssd1306_WriteString(R,Font_7x10,CurTime.Seconds%2);
			}else{
				sprintf(R,"%03d",T2_max);
				ssd1306_WriteString(R,Font_7x10,White);
			}		
		ssd1306_WriteString("C",Font_7x10,White);
		ssd1306_WriteChar(0x7F,Font_7x10,White);
		ssd1306_SetCursor(0,21);
		ssd1306_WriteString("IK3:",Font_7x10,White);
		if (choise==4)
			{
				sprintf(R,"%02d%c%01d",short_circut_current/10,'.',short_circut_current%10);
				ssd1306_WriteString(R,Font_7x10,CurTime.Seconds%2);
			}else{
				sprintf(R,"%02d%c%01d",short_circut_current/10,'.',short_circut_current%10);
				ssd1306_WriteString(R,Font_7x10,White);
			}			

		ssd1306_WriteString("A T1:",Font_7x10,White);
		if (choise==5)
			{
				sprintf(R,"%03d",T1_max);
				ssd1306_WriteString(R,Font_7x10,CurTime.Seconds%2);
			}else{
				sprintf(R,"%03d",T1_max);
				ssd1306_WriteString(R,Font_7x10,White);
			}				
		ssd1306_WriteString("C",Font_7x10,White);
		ssd1306_WriteChar(0x7F,Font_7x10,White);
		ssd1306_SetCursor(0,32);
		ssd1306_WriteString("PowerNom:",Font_7x10,White);
		if (choise==6)
			{
				sprintf(R,"%01d%c%02d",power_nom/25,'.',power_nom*4%100);
				ssd1306_WriteString(R,Font_7x10,CurTime.Seconds%2);
			}else{
				sprintf(R,"%01d%c%02d",power_nom/25,'.',power_nom*4%100);
				ssd1306_WriteString(R,Font_7x10,White);
			}			
		ssd1306_WriteString("kW",Font_7x10,White);

		ssd1306_SetCursor(0,43);
		ssd1306_WriteString("WorkTimeClear-",Font_7x10,White);
		if (choise==7)
			{
				if (worktimeclear)
					{
						ssd1306_WriteString("YES",Font_7x10,CurTime.Seconds%2);
					}else{
						ssd1306_WriteString("NOT",Font_7x10,CurTime.Seconds%2);
					}
			}else{
				if (worktimeclear)
					{
						ssd1306_WriteString("YES",Font_7x10,White);
					}else{
						ssd1306_WriteString("NOT",Font_7x10,White);
					}
			}	
                
		ssd1306_SetCursor(0,53);
		ssd1306_WriteString("V_LL:",Font_7x10,White);
		if (choise==8)
			{
				sprintf(R,"%03d ",voltage_LL);
				ssd1306_WriteString(R,Font_7x10,CurTime.Seconds%2);
			}else{
                                sprintf(R,"%03d ",voltage_LL);
				ssd1306_WriteString(R,Font_7x10,White);
			}
		ssd1306_SetCursor(63,53);
		ssd1306_WriteString("V_HH:",Font_7x10,White);
		if (choise==9)
			{
				sprintf(R,"%03d ",voltage_HH);
				ssd1306_WriteString(R,Font_7x10,CurTime.Seconds%2);
			}else{
                                sprintf(R,"%03d ",voltage_HH);
				ssd1306_WriteString(R,Font_7x10,White);
			}			

		switch (choise)
			{
			  case 0:
				phase_control=EncoderVal%4;
				break;
			  case 1:
				cosFI=EncoderVal%101;
				break;
			  case 2:
				Current_diference=EncoderVal%256;
				break;
			  case 3:
				T2_max=EncoderVal%105;
				break;
			  case 4:
				short_circut_current=EncoderVal%256;
				break;
			  case 5:
				T1_max=EncoderVal%105;
				break;
			  case 6:
				power_nom=EncoderVal%256;
				break;				
			  case 7:
				worktimeclear=EncoderVal%2;
				break;	
                          case 8:
				voltage_LL=EncoderVal%512;
				break;	
                          case 9:
				voltage_HH=EncoderVal%512;
				break;	
			}
			
          if (button>1000) 
          {
            flag=6;
            choise=0;      
            __HAL_TIM_SET_COUNTER(&htim3,koef_U);
          } else if (button>100)
          {
            choise=(choise>=9)?0:choise+1;
              switch (choise)
              {
                case 0:
                  __HAL_TIM_SET_COUNTER(&htim3,phase_control);
                  break;
                case 1:
                  __HAL_TIM_SET_COUNTER(&htim3,cosFI);
                  break;
                case 2:
                  __HAL_TIM_SET_COUNTER(&htim3,Current_diference);
                  break;
                case 3:
                  __HAL_TIM_SET_COUNTER(&htim3,T2_max);
                  break;
		case 4:
                  __HAL_TIM_SET_COUNTER(&htim3,short_circut_current);
                  break;
                case 5:
                  __HAL_TIM_SET_COUNTER(&htim3,T1_max);
                  break;
                case 6:
                  __HAL_TIM_SET_COUNTER(&htim3,power_nom);
                  break;
                case 7:
                  __HAL_TIM_SET_COUNTER(&htim3,worktimeclear);
                  break;
                case 8:
                  __HAL_TIM_SET_COUNTER(&htim3,voltage_LL);
                  break;
                case 9:
                  __HAL_TIM_SET_COUNTER(&htim3,voltage_HH);
                  break;                  
              }
          } 

        break;
    case 5:
        //------------------------------vse its over--------------------------------//
                dreamenable=1;
		ssd1306_SetCursor(0,7);
		ssd1306_WriteString("Pressure:",Font_7x10,White);
		ssd1306_SetCursor(63,0);
                P_display=P-10;
                if (P==0){
                  ssd1306_WriteString("-1.",Font_11x18,White);
                  sprintf(R,"%01d",P);
                }
                else if (P<10)
                {
                  ssd1306_WriteString("-0.",Font_11x18,White);
                  sprintf(R,"%01d",(10-P));
                }else{
                sprintf(R,"%02d%c%01d",P_display/10,'.',P_display%10);}
		ssd1306_WriteString(R,Font_11x18,White);
		ssd1306_SetCursor(107,7);
		ssd1306_WriteString("kg",Font_7x10,White);
		ssd1306_SetCursor(35,18);
		ssd1306_WriteString("Error!!!",Font_7x10,White);
		switch(error_first)
		{
			case 1:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("    Wrong phase",Font_7x10,White);  //18 max
			ssd1306_SetCursor(0,40);
			ssd1306_WriteString("    rotation",Font_7x10,White);  //18 max
			break;
			case 2:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Overheating T1>max",Font_7x10,White);  //18 max

			break;
			case 3:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Overheating T2>max",Font_7x10,White);  //18 max
			break;
			case 4:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("depressurization",Font_7x10,White);  //18 max		
			ssd1306_SetCursor(0,40);
			ssd1306_WriteString("dP>dP_max for 6sec",Font_7x10,White);  //18 max			
			break;
			case 5:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("wrong sensor",Font_7x10,White);  //18 max		
			ssd1306_SetCursor(0,40);
			ssd1306_WriteString("P>P_HH",Font_7x10,White);  //18 max							
			break;
			case 6:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("wrong sensor",Font_7x10,White);  //18 max		
			ssd1306_SetCursor(0,40);
			ssd1306_WriteString("P<P_LL",Font_7x10,White);  //18 max			
			break;
			case 7:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Short circuit",Font_7x10,White);  //18 max		
			ssd1306_SetCursor(0,40);
			ssd1306_WriteString("Phase A",Font_7x10,White);  //18 max		
			break;
			case 8:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Short circuit",Font_7x10,White);  //18 max		
			ssd1306_SetCursor(0,40);
			ssd1306_WriteString("Phase B",Font_7x10,White);  //18 max				
			break;
			case 9:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Short circuit",Font_7x10,White);  //18 max		
			ssd1306_SetCursor(0,40);
			ssd1306_WriteString("Phase C",Font_7x10,White);  //18 max				
			break;
			case 10:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Current difference",Font_7x10,White);  //18 max		
			ssd1306_SetCursor(0,40);
			ssd1306_WriteString("Phase A>BC",Font_7x10,White);  //18 max					
			break;
			case 11:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Current difference",Font_7x10,White);  //18 max		
			ssd1306_SetCursor(0,40);
			ssd1306_WriteString("Phase A<BC",Font_7x10,White);  //18 max			
			break;
			case 12:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Current difference",Font_7x10,White);  //18 max		
			ssd1306_SetCursor(0,40);
			ssd1306_WriteString("Phase B>CA",Font_7x10,White);  //18 max			
			break;
			case 13:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Current difference",Font_7x10,White);  //18 max		
			ssd1306_SetCursor(0,40);
			ssd1306_WriteString("Phase B<CA",Font_7x10,White);  //18 max				
			break;
			case 14:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Current difference",Font_7x10,White);  //18 max		
			ssd1306_SetCursor(0,40);
			ssd1306_WriteString("Phase C>AB",Font_7x10,White);  //18 max				
			break;
			case 15:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Current difference",Font_7x10,White);  //18 max		
			ssd1306_SetCursor(0,40);
			ssd1306_WriteString("Phase C>AB",Font_7x10,White);  //18 max				
			break;
			case 16:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Pressure NOT",Font_7x10,White);  //18 max		
			ssd1306_SetCursor(0,40);
			ssd1306_WriteString("change",Font_7x10,White);  //18 max			
			break;
			case 17:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Dry running",Font_7x10,White);  //18 max		
			ssd1306_SetCursor(0,40);
			ssd1306_WriteString("Power<PowerNominal",Font_7x10,White);  //18 max				
			break;
			case 18:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Phase A",Font_7x10,White);  //18 max		
			ssd1306_SetCursor(0,40);
			ssd1306_WriteString("open circuit",Font_7x10,White);  //18 max				
			break;
			case 19:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Phase B",Font_7x10,White);  //18 max		
			ssd1306_SetCursor(0,40);
			ssd1306_WriteString("open circuit",Font_7x10,White);  //18 max					
			break;
			case 20:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Phase C",Font_7x10,White);  //18 max		
			ssd1306_SetCursor(0,40);
			ssd1306_WriteString("open circuit",Font_7x10,White);  //18 max					
			break;
                        case 21:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Button STOP",Font_7x10,White);  //18 max					
			break;
                        case 22:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Voltage < V_LL",Font_7x10,White);  //18 max					
			break;
                        case 23:
			ssd1306_SetCursor(0,28);
			ssd1306_WriteString("Voltage > V_HH",Font_7x10,White);  //18 max					
			break;
			
		}

		
        if (button>1000) 
          {
            flag=3;
            choise=0;
            __HAL_TIM_SET_COUNTER(&htim3,P_max);
             dream=0;
          } else if (button>100)
          {
            error=0;  
            flag=0;
            choise=0;
            error_first=0;
             dream=0;
          }  
      break; 
      case 6:
        //------------------------------settings page3--------------------------------//      
                dreamenable=0;
		ssd1306_SetCursor(0,0);


		ssd1306_WriteString("koef_U:",Font_7x10,White);
		if (choise==0)
			{
				sprintf(R,"%01d%c%02d",koef_U/100,'.',koef_U%100);
				ssd1306_WriteString(R,Font_7x10,CurTime.Seconds%2);
			}else{
				sprintf(R,"%01d%c%02d",koef_U/100,'.',koef_U%100);
				ssd1306_WriteString(R,Font_7x10,White);
			}
		ssd1306_SetCursor(0,11);		
		ssd1306_WriteString("U:",Font_7x10,White);
		sprintf(R,"%03d",voltage);		
                ssd1306_WriteString(R,Font_7x10,White);
		ssd1306_SetCursor(0,30);
		ssd1306_WriteString("koef_Pres:",Font_7x10,White);
		if (choise==1)
			{
				sprintf(R,"%01d%c%02d",koef_Pres/100,'.',koef_Pres%100);
				ssd1306_WriteString(R,Font_7x10,CurTime.Seconds%2);
			}else{
				sprintf(R,"%01d%c%02d",koef_Pres/100,'.',koef_Pres%100);
				ssd1306_WriteString(R,Font_7x10,White);
			}
		ssd1306_SetCursor(0,41);
                P_display=P-10;
		ssd1306_WriteString("P:",Font_7x10,White);
                if (P==0){
                  ssd1306_WriteString("-1.",Font_7x10,White);
                  sprintf(R,"%01d",P);
                }
                else if (P<10)
                {
                  ssd1306_WriteString("-0.",Font_7x10,White);
                  sprintf(R,"%01d",(10-P));
                }else{
                  sprintf(R,"%02d%c%01d",P_display/10,'.',P_display%10);}
                
                ssd1306_WriteString(R,Font_7x10,White);
		switch (choise)
			{
			  case 0:
				koef_U=EncoderVal;
				break;
			  case 1:
				koef_Pres=EncoderVal;
				break;
			}
			
          if (button>1000) 
          {
            flag=0;
            choise=0;
            if (worktimeclear){worktime=0;worktimeclear=0;}
            save=1;	           
            error=0;  
            error_first=0;
          } else if (button>100)
          {
            choise=(choise>=1)?0:choise+1;
              switch (choise)
              {
                case 0:
                  __HAL_TIM_SET_COUNTER(&htim3,koef_U);
                  break;
                case 1:
                  __HAL_TIM_SET_COUNTER(&htim3,koef_Pres);
                  break;                
              }
          } 

        break;
      
    }
    if (dreamenable&&dream){                                                                           //dream
      if (!alredydream){
      ssd1306_Fill(Black); 
      ssd1306_UpdateScreen();}
      alredydream=1;
    }else{
      ssd1306_UpdateScreen();
      ssd1306_Fill(Black); 
      alredydream=0;
    }
    
    button=0;
    if (save==1)
    {
      Flash_save();
      save=0;
    }
    osDelay(200);
  }
  /* USER CODE END Screen */
}

/* USER CODE BEGIN Header_MQTT */
/**
* @brief Function implementing the MQTT_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MQTT */
void MQTT(void *argument)
{
  /* USER CODE BEGIN MQTT */
  
  HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_RESET);
  osDelay(1);
  HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);
  osDelay(1000);
  /*
  uint8_t buf[100];
  uint32_t rc,mes_id;
  
  
  uint8_t stat;
  uint8_t reqnr;
  uint8_t rIP[4];
  char Message[128];
  
  reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
  reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
  reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);
	
  uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
  wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);
  wizchip_setnetinfo(&gWIZNETINFO);
  ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);
  osDelay(1000);
   */
 // Network n;
 // MQTTClient c;
 // n.my_socket = 0;

  
 // NewNetwork(&n,TCP_SOCKET);  //, TCP_SOCKET
 // ConnectNetwork(&n, targetIP, 1883);
//  MQTTClientInit(&c, &n, 1000, buf, 100, gDATABUF, DATA_BUF_SIZE);

//connecting 
//  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
//  data.willFlag = 0;
// data.MQTTVersion = 3;
//  data.clientID.cstring = (char*)"w5500-client";
//  data.username.cstring = "";
//  data.password.cstring = "";
  
//  data.keepAliveInterval = 60;
//  data.cleansession = 1;
 
//  rc = MQTTConnect(&c, &data);
  //subscribing
//  rc = MQTTSubscribe(&c, "YS/#", QOS0, messageArrived);
  
  
  
  
  

  /* Infinite loop */
  for(;;)
  {
   /* stat = socket(HTTP_SOCKET, Sn_MR_TCP, 80, 0);
    stat = listen(HTTP_SOCKET);
    
    while(getSn_SR(HTTP_SOCKET) == SOCK_LISTEN)
    {
	osDelay(2);
    }
    stat = getSn_SR(HTTP_SOCKET);
    
    getsockopt(HTTP_SOCKET, SO_DESTIP, rIP);
    sprintf(Message, "CH 232 - %d /n", reqnr);
    send(0, (uint8_t*)Message, strlen(Message));
    sprintf(Message, "error %d", error);
    send(0, (uint8_t*)Message, strlen(Message));   
    disconnect(HTTP_SOCKET);
    close(HTTP_SOCKET);
    reqnr++;
    */
    /*
        MQTTMessage pubMessage;
        pubMessage.qos = QOS0;
        pubMessage.id = mes_id++;
        pubMessage.payloadlen = 13;
        pubMessage.payload = "sasai_lalka";
        MQTTPublish(&c, "/w5500_stm32_client", &pubMessage);

	
	MQTTYield(&c, 1000);
    */
    osDelay(1000);
  }
  /* USER CODE END MQTT */
}

/* USER CODE BEGIN Header_monitor */
/**
* @brief Function implementing the MonitorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_monitor */
void monitor(void *argument)
{
  /* USER CODE BEGIN monitor */
  
  
  /* Infinite loop */
  for(;;)
  {
   // MilliTimer_Handler();
    osDelay(10000);
    /*
    UBaseType_t task_count = uxTaskGetNumberOfTasks();
    uint32_t _total_runtime,zalupa,konya;
    TaskStatus_t _buffer[6];
     if (task_count <= 6)
      {
        task_count = uxTaskGetSystemState(_buffer, task_count, &_total_runtime);
        zalupa=xPortGetFreeHeapSize();
        konya=xPortGetMinimumEverFreeHeapSize();
        if (zalupa==konya)
          {
              vTaskDelete(NULL);
          }else{
              vTaskDelete(NULL);
          }
      }
    */
    
    
  }
  /* USER CODE END monitor */
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
