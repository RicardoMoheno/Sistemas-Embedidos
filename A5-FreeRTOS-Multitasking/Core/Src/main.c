/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim2;

/* Definitions for breakTask */
osThreadId_t breakTaskHandle;
const osThreadAttr_t breakTask_attributes = {
  .name = "breakTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for throttleTask */
osThreadId_t throttleTaskHandle;
const osThreadAttr_t throttleTask_attributes = {
  .name = "throttleTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for gearSelectTask */
osThreadId_t gearSelectTaskHandle;
const osThreadAttr_t gearSelectTask_attributes = {
  .name = "gearSelectTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for dataProcTask */
osThreadId_t dataProcTaskHandle;
const osThreadAttr_t dataProcTask_attributes = {
  .name = "dataProcTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for outputDispTask */
osThreadId_t outputDispTaskHandle;
const osThreadAttr_t outputDispTask_attributes = {
  .name = "outputDispTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for breakQueue */
osMessageQueueId_t breakQueueHandle;
const osMessageQueueAttr_t breakQueue_attributes = {
  .name = "breakQueue"
};
/* Definitions for throttleQueue */
osMessageQueueId_t throttleQueueHandle;
const osMessageQueueAttr_t throttleQueue_attributes = {
  .name = "throttleQueue"
};
/* Definitions for gearQueue */
osMessageQueueId_t gearQueueHandle;
const osMessageQueueAttr_t gearQueue_attributes = {
  .name = "gearQueue"
};
/* Definitions for speedQueue */
osMessageQueueId_t speedQueueHandle;
const osMessageQueueAttr_t speedQueue_attributes = {
  .name = "speedQueue"
};
/* USER CODE BEGIN PV */
char LCDdir;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
void startBreakInputTask(void *argument);
void startThrottleInputTask(void *argument);
void startGearSelectTask(void *argument);
void startDataProcessTask(void *argument);
void startOutputDispTask(void *argument);

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
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

  /* Create the queue(s) */
  /* creation of breakQueue */
  breakQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &breakQueue_attributes);

  /* creation of throttleQueue */
  throttleQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &throttleQueue_attributes);

  /* creation of gearQueue */
  gearQueueHandle = osMessageQueueNew (16, sizeof(char), &gearQueue_attributes);

  /* creation of speedQueue */
  speedQueueHandle = osMessageQueueNew (16, sizeof(uint32_t), &speedQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of breakTask */
  breakTaskHandle = osThreadNew(startBreakInputTask, NULL, &breakTask_attributes);

  /* creation of throttleTask */
  throttleTaskHandle = osThreadNew(startThrottleInputTask, NULL, &throttleTask_attributes);

  /* creation of gearSelectTask */
  gearSelectTaskHandle = osThreadNew(startGearSelectTask, NULL, &gearSelectTask_attributes);

  /* creation of dataProcTask */
  dataProcTaskHandle = osThreadNew(startDataProcessTask, NULL, &dataProcTask_attributes);

  /* creation of outputDispTask */
  outputDispTaskHandle = osThreadNew(startOutputDispTask, NULL, &outputDispTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, dispRS_Pin|dispRW_Pin|dispEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, dispD7_Pin|dispD0_Pin|dispD1_Pin|dispD2_Pin
                          |dispD3_Pin|dispD4_Pin|disoD5_Pin|dispD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : dispRS_Pin dispRW_Pin dispEN_Pin */
  GPIO_InitStruct.Pin = dispRS_Pin|dispRW_Pin|dispEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : gearSelectB1_Pin gearSelectB0_Pin */
  GPIO_InitStruct.Pin = gearSelectB1_Pin|gearSelectB0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : dispD7_Pin dispD0_Pin dispD1_Pin dispD2_Pin
                           dispD3_Pin dispD4_Pin disoD5_Pin dispD6_Pin */
  GPIO_InitStruct.Pin = dispD7_Pin|dispD0_Pin|dispD1_Pin|dispD2_Pin
                          |dispD3_Pin|dispD4_Pin|disoD5_Pin|dispD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void LCD_init(){
	HAL_GPIO_WritePin(controlPort,dispRS_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(controlPort,dispRW_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(controlPort,dispEN_Pin,GPIO_PIN_RESET);
    LCD_cmd(setLCD_8bits_2Lines_smallFont);
    osDelay(250);
    LCD_cmd(DISPLAY_ON_C_OFF_B_OFF);
    osDelay(250);
    LCD_cmd(Clear_DISP);
    osDelay(250);
    set_LCD_dir('R');
    osDelay(250);
}
void LCD_cmd(char cx) {
	HAL_GPIO_WritePin(controlPort,dispRS_Pin,GPIO_PIN_RESET); // select IR register
	HAL_GPIO_WritePin(controlPort,dispRW_Pin,GPIO_PIN_RESET); // set Write mode
    HAL_GPIO_WritePin(controlPort,dispEN_Pin,GPIO_PIN_SET);    // set to clock data
    __NOP();
    dataPort -> ODR = (0b00000000000|cx) << 3;   // send out command
    __NOP();
    HAL_GPIO_WritePin(controlPort,dispEN_Pin,GPIO_PIN_RESET);  // complete external write cycle
}
void send_to_LCD(char xy){
	HAL_GPIO_WritePin(controlPort,dispRS_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(controlPort,dispRW_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(controlPort,dispEN_Pin,GPIO_PIN_SET);
    __NOP();
	dataPort -> ODR = (0b00000000000|xy) << 3;
    __NOP();
    HAL_GPIO_WritePin(controlPort,dispEN_Pin,GPIO_PIN_RESET);
    osDelay(10);
}
void set_LCD_dir(char d){
    if(d == 'L'){
        LCD_cmd(EMS_DEC_S);
        LCDdir = 'L';
    } else if (d == 'R'){
        LCD_cmd(EMS_INC_S);
        LCDdir = 'R';
    }
}
void set_cursor_pos(char row,char col){
    unsigned char Pos = 0;
    switch (row){
        case 0: {
            Pos = 0x80|col;
            LCD_cmd(Pos);
        } break;
        case 1: {
            Pos = 0xC0|col;
            LCD_cmd(Pos);
        } break;
        default: __NOP(); break;
    }
    osDelay(10);
}
void move_cursor_n_times(int n, char dir){
    if (dir == 'R'){
        for (int i = 0; i <n ;i++){
        LCD_cmd(CURSOR_MOVE_RIGHT);
        }
    } else if (dir == 'L'){
        for (int i = 0; i <n ;i++){
        LCD_cmd(CURSOR_MOVE_LEFT);
        }
    }
}
void write_string_LCD(const char *s){
    while(*s)
        send_to_LCD(*s++);
    osDelay(10);
}
void write_int_LCD(int x){
    unsigned char UnitDigit  = 0;
    unsigned char TenthDigit = 0;
    unsigned char HundrethDigit = 0;
    HundrethDigit = x/100;
    TenthDigit = (x - HundrethDigit*100)/10;
    UnitDigit = x - TenthDigit*10 - HundrethDigit*100;
    if(LCDdir == 'R'){
        send_to_LCD(HundrethDigit+'0');
        send_to_LCD(TenthDigit+'0');
        send_to_LCD(UnitDigit+'0');
    } else if(LCDdir == 'L'){
        send_to_LCD(UnitDigit+'0');
        send_to_LCD(TenthDigit+'0');
        send_to_LCD(HundrethDigit+'0');
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_startBreakInputTask */
/**
  * @brief  Function implementing the breakTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startBreakInputTask */
void startBreakInputTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	HAL_ADCEx_Calibration_Start(&hadc1);
  /* Infinite loop */
  for(;;)
  {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,300);
	int breakMeasured = HAL_ADC_GetValue(&hadc1);
	int breakPercentage = breakMeasured*(100.0/4081.0);
	osMessageQueuePut(breakQueueHandle, &breakPercentage, 0, 200);
    osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startThrottleInputTask */
/**
* @brief Function implementing the throttleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startThrottleInputTask */
void startThrottleInputTask(void *argument)
{
  /* USER CODE BEGIN startThrottleInputTask */
	HAL_ADCEx_Calibration_Start(&hadc2);
  /* Infinite loop */
  for(;;)
  {
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2,300);
	int throttleMeasured = HAL_ADC_GetValue(&hadc2);
	int throttlePercentage = throttleMeasured*(100.0/4081.0);
	osMessageQueuePut(throttleQueueHandle, &throttlePercentage, 0, 200);
	osDelay(10);
  }
  /* USER CODE END startThrottleInputTask */
}

/* USER CODE BEGIN Header_startGearSelectTask */
/**
* @brief Function implementing the gearSelectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startGearSelectTask */
void startGearSelectTask(void *argument)
{
  /* USER CODE BEGIN startGearSelectTask */
	char N = 'N';
	char gear1 = '1';
	char gear2 = '2';
	int bit0,bit1;
  /* Infinite loop */
  for(;;)
  {
	bit0 = HAL_GPIO_ReadPin(GPIOB,gearSelectB0_Pin);
	bit1 = HAL_GPIO_ReadPin(GPIOB,gearSelectB1_Pin);
	if (bit0 == 1){
		osMessageQueuePut(gearQueueHandle,&N,0,200);
	} else if (bit1 == 1){
		osMessageQueuePut(gearQueueHandle,&gear2,0,200);
	} else osMessageQueuePut(gearQueueHandle,&gear1,0,200);
    osDelay(50);
  }
  /* USER CODE END startGearSelectTask */
}

/* USER CODE BEGIN Header_startDataProcessTask */
/**
* @brief Function implementing the dataProcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startDataProcessTask */
void startDataProcessTask(void *argument)
{
  /* USER CODE BEGIN startDataProcessTask */
	int breakPercentageReceived = 0;
	int throttlePercentageReceived = 0;
	char requestedGear = 'N';
	int desiredSpeed = 0;
	int maxSpeed;
	int acceleration;
  /* Infinite loop */
  for(;;)
  {
	int breakPercentageSum = 0;
	int throttlePercentageSum = 0;
	for (int i = 0; i < 16; i++){
		osMessageQueueGet(breakQueueHandle,&breakPercentageReceived,NULL,250);
		osMessageQueueGet(throttleQueueHandle,&throttlePercentageReceived,NULL,250);
		breakPercentageSum = breakPercentageSum + breakPercentageReceived;
		throttlePercentageSum = throttlePercentageSum + throttlePercentageReceived;
	}
	int avgBreakPercentage = breakPercentageSum/16;
	int avgThrottlePercentage = throttlePercentageSum/16;
	osMessageQueueGet(gearQueueHandle,&requestedGear,NULL,250);
	if (requestedGear == 'N'){
		desiredSpeed = desiredSpeed - 400;
		if (desiredSpeed < 0) desiredSpeed = 0;
	} else if (requestedGear == '1'){
		maxSpeed = 80000;
		if (avgBreakPercentage == 0 && avgThrottlePercentage > 0){
			acceleration = 1600*(avgThrottlePercentage/100.0);
			desiredSpeed = desiredSpeed + acceleration;
		} else if (avgBreakPercentage > 0 && avgThrottlePercentage == 0){
			acceleration = -3200*(avgBreakPercentage/100.0);
			desiredSpeed = desiredSpeed + acceleration;
		} else if (avgBreakPercentage > 0 && avgThrottlePercentage > 0){
			acceleration = 0;
			desiredSpeed = desiredSpeed + acceleration;
		}
		if (desiredSpeed > maxSpeed) desiredSpeed = maxSpeed;
		if (desiredSpeed < 0) desiredSpeed = 0;
	} else if (requestedGear == '2'){
		maxSpeed = 200000;
		if (avgBreakPercentage == 0 && avgThrottlePercentage > 0){
			acceleration = 2400*(avgThrottlePercentage/100.0);
			desiredSpeed = desiredSpeed + acceleration;
		} else if (avgBreakPercentage > 0 && avgThrottlePercentage == 0){
			acceleration = -3200*(avgBreakPercentage/100.0);
			desiredSpeed = desiredSpeed + acceleration;
		} else if (avgBreakPercentage > 0 && avgThrottlePercentage > 0){
			acceleration = 0;
			desiredSpeed = desiredSpeed + acceleration;
		}
		if (desiredSpeed > maxSpeed) desiredSpeed = maxSpeed;
		if (desiredSpeed < 0) desiredSpeed = 0;
	}
	osMessageQueuePut(speedQueueHandle,&desiredSpeed,0,200);
    osDelay(100);
  }
  /* USER CODE END startDataProcessTask */
}

/* USER CODE BEGIN Header_startOutputDispTask */
/**
* @brief Function implementing the outputDispTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startOutputDispTask */
void startOutputDispTask(void *argument)
{
  /* USER CODE BEGIN startOutputDispTask */
	LCD_init();
	set_cursor_pos(0,0);
	write_string_LCD("Desired speed:");
	set_cursor_pos(1,11);
	write_string_LCD("km/hr");
	set_cursor_pos(1,7);
	uint32_t desiredSpeedReceived = 0;
	uint32_t avgDesiredSpeed;
  /* Infinite loop */
  for(;;)
  {
	uint32_t sumDesiredSpeed = 0;
	for (int i = 0; i < 16 ; i++){
		osMessageQueueGet(speedQueueHandle,&desiredSpeedReceived,NULL,250);
		sumDesiredSpeed = sumDesiredSpeed + desiredSpeedReceived;
	}
	avgDesiredSpeed = sumDesiredSpeed/16;
	uint16_t dutyCycle = avgDesiredSpeed*(65535.0/200000.0);
	int desiredSpeedInKM = avgDesiredSpeed/1000;
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, dutyCycle);
	write_int_LCD(desiredSpeedInKM);
	set_cursor_pos(1,7);
    osDelay(500);
  }
  /* USER CODE END startOutputDispTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
