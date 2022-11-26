/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
void LCD_init (void);
void LCD_cmd (char);
void set_LCD_dir (char);
void send_to_LCD (char);
void set_cursor_pos(char,char);
void move_cursor_n_times(int,char);
void write_string_LCD (const char*);
void write_int_LCD (int x);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define dispRS_Pin GPIO_PIN_0
#define dispRS_GPIO_Port GPIOA
#define dispRW_Pin GPIO_PIN_1
#define dispRW_GPIO_Port GPIOA
#define dispEN_Pin GPIO_PIN_2
#define dispEN_GPIO_Port GPIOA
#define throttlePot_Pin GPIO_PIN_3
#define throttlePot_GPIO_Port GPIOA
#define breakPot_Pin GPIO_PIN_4
#define breakPot_GPIO_Port GPIOA
#define gearSelectB1_Pin GPIO_PIN_1
#define gearSelectB1_GPIO_Port GPIOB
#define dispD7_Pin GPIO_PIN_10
#define dispD7_GPIO_Port GPIOB
#define gearSelectB0_Pin GPIO_PIN_11
#define gearSelectB0_GPIO_Port GPIOB
#define speedLED_Pin GPIO_PIN_15
#define speedLED_GPIO_Port GPIOA
#define dispD0_Pin GPIO_PIN_3
#define dispD0_GPIO_Port GPIOB
#define dispD1_Pin GPIO_PIN_4
#define dispD1_GPIO_Port GPIOB
#define dispD2_Pin GPIO_PIN_5
#define dispD2_GPIO_Port GPIOB
#define dispD3_Pin GPIO_PIN_6
#define dispD3_GPIO_Port GPIOB
#define dispD4_Pin GPIO_PIN_7
#define dispD4_GPIO_Port GPIOB
#define disoD5_Pin GPIO_PIN_8
#define disoD5_GPIO_Port GPIOB
#define dispD6_Pin GPIO_PIN_9
#define dispD6_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define dataPort GPIOB
#define controlPort GPIOA
/* Function set
   RS R/W DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
  	0 	0 	0 	0 	1 	DL 	N 	F 	? 	?
  	(DL) data length				DL=1: 8 bits, 	DL=0: 4 bits
 	(N)  number of display lines  	N=1:  2 lines, 	 N=0: 1 line
 	(F)	 character font  			F=1:  5x10 dots, F=0: 5x8 dots
 */
#define setLCD_8bits_2Lines_smallFont  0b00111000
#define setLCD_8bits_2Lines_bigFont    0b00111100
/* Display ON/OFF
   RS R/W DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
    0 	0 	0 	0 	0 	0 	1 	D 	C 	B
 	Sets entire Display on/off				(D).
 	Cursor on/off 							(C).
 	Blinking of cursor position character 	(B).
 */
#define  DISPLAY_ALL_ON				0b00001111
#define  DISPLAY_ON_C_OFF_B_OFF		0b00001100
#define  DISPLAY_ON_C_ON_B_OFF      0b00001110
#define  DISPLAY_ON_C_OFF_B_ON      0b00001101
#define  DISPLAY_ALL_OFF			0b00001000
/* Entry mode set
   Sets cursor move direction and specifies display shift.
   These operations are performed during data write and read.
   RS R/W DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
    0 	0 	0 	0 	0 	0 	0 	1  I/D 	S
    I/D=1	: Increment
    I/D=0	: Decrement
    S=1	: Accompanies display shift
 */
#define	 EMS_INC_S			0b00000110
#define  EMS_DEC_S          0b00000100
/* Cursor or display shift
   Moves cursor and shifts display without changing DDRAM contents.
   RS R/W DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
	0 	0 	0 	0 	0 	1  S/C R/L 	? 	?
	S/C=1: Display shift			S/C=0: Cursor move
	R/L=1: Shift to the right		R/L=0: Shift to the left
 */
#define	DISP_MOVE_RIGHT		0b00011100
#define	DISP_MOVE_LEFT		0b00011000
#define	CURSOR_MOVE_RIGHT	0b00010100
#define	CURSOR_MOVE_LEFT	0b00010000
/* Set CGRAM
   RS R/W DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
    0 	0 	0   1 ACG ACG ACG ACG ACG ACG
 */
#define NEW_CHARACTER 0b01000000
/* Set	DDRAM
   RS R/W DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
    0 	0 	1  ADD ADD ADD ADD ADD ADD ADD
 */
#define LCD_LINE0 0x80
#define LCD_LINE1 0xC0
// Instrucciones básicas
#define Return_Home 0b00000010
#define Clear_DISP  0b00000001
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
