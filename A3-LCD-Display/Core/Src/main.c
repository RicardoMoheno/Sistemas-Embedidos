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

/* USER CODE BEGIN PV */
char PacMan_frame1[] = {0x00,0x00,0x0E,0x1B,0x1C,0x18,0x1F,0x0E};
char PacMan_frame2[] = {0x00,0x00,0x0E,0x1B,0x1F,0x1C,0x1F,0x0E};
char PacMan_frame3[] = {0x00,0x00,0x0E,0x1B,0x1F,0x1F,0x1F,0x0E};
char LCDdir;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
  /* USER CODE BEGIN 2 */
  LCD_init();

  create_custom_char(0,PacMan_frame1);
  create_custom_char(1,PacMan_frame2);
  create_custom_char(2,PacMan_frame3);
  set_cursor_pos(0,0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  set_cursor_pos(0,5);
	  write_string_LCD("HELLO");
	  set_cursor_pos(1,3);
	  write_string_LCD("Mr.PacMan");
	  HAL_Delay(500);
	  for(int i=0;i<2;i++){
		  set_cursor_pos(i,0);
		  for(int j=0;j<=15;j++){
			  disp_custom_char(0);
			  HAL_Delay(157);
			  set_cursor_pos(i,j);
			  send_to_LCD(' ');
			  disp_custom_char(2);
			  HAL_Delay(157);
			  set_cursor_pos(i,j+1);
		  }
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void LCD_init(){
	HAL_GPIO_WritePin(controlPort,RS_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(controlPort,RW_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(controlPort,E_Pin,GPIO_PIN_RESET);
    LCD_cmd(setLCD_8bits_2Lines_smallFont);
    HAL_Delay(250);
    LCD_cmd(DISPLAY_ON_C_OFF_B_OFF);
    HAL_Delay(250);
    LCD_cmd(Clear_DISP);
    HAL_Delay(250);
    set_LCD_dir('R');
    HAL_Delay(250);
}
void LCD_cmd(char cx) {
	HAL_GPIO_WritePin(controlPort,RS_Pin,GPIO_PIN_RESET); // select IR register
	HAL_GPIO_WritePin(controlPort,RW_Pin,GPIO_PIN_RESET); // set Write mode
    HAL_GPIO_WritePin(controlPort,E_Pin,GPIO_PIN_SET);    // set to clock data
    __NOP();
    dataPort -> ODR = cx;   // send out command
    __NOP();
    HAL_GPIO_WritePin(controlPort,E_Pin,GPIO_PIN_RESET);  // complete external write cycle
}
void LCD_read(char ADD){
	HAL_GPIO_WritePin(controlPort,RS_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(controlPort,RW_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(controlPort,E_Pin,GPIO_PIN_SET);
	__NOP();
	dataPort -> IDR = ADD;
	__NOP();
	HAL_GPIO_WritePin(controlPort,E_Pin,GPIO_PIN_RESET);
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
void send_to_LCD(char xy){
	HAL_GPIO_WritePin(controlPort,RS_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(controlPort,RW_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(controlPort,E_Pin,GPIO_PIN_SET);
	dataPort -> ODR = xy;
    __NOP();
    __NOP();
    HAL_GPIO_WritePin(controlPort,E_Pin,GPIO_PIN_RESET);
    HAL_Delay(10);
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
    HAL_Delay(10);
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
    HAL_Delay(10);
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
void create_custom_char(uint8_t loc,char* data){
	switch (loc){
		case 0:LCD_cmd(NEW_CHARACTER);break;
		case 1:LCD_cmd(NEW_CHARACTER+8);break;
		case 2:LCD_cmd(NEW_CHARACTER+16);break;
		case 3:LCD_cmd(NEW_CHARACTER+24);break;
		case 4:LCD_cmd(NEW_CHARACTER+32);break;
		case 5:LCD_cmd(NEW_CHARACTER+40);break;
		case 6:LCD_cmd(NEW_CHARACTER+48);break;
		case 7:LCD_cmd(NEW_CHARACTER+56);break;
		default : break;
	}
	for (int i=0;i<8;i++){
		send_to_LCD(data[i]);
	}

}
void disp_custom_char(uint8_t loc){
	switch (loc){
		case 0:send_to_LCD(0);break;
		case 1:send_to_LCD(1);break;
		case 2:send_to_LCD(2);break;
		case 3:send_to_LCD(3);break;
		case 4:send_to_LCD(4);break;
		case 5:send_to_LCD(5);break;
		case 6:send_to_LCD(6);break;
		case 7:send_to_LCD(7);break;
		default : break;
	}
}
/* USER CODE END 4 */

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
