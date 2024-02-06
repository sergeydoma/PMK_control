/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#ifndef _timeLanC
#define _timeLanC		20		// цикл опроса по i2c 4  платы
#endif

#ifndef _timeDelay
#define _timeDeley			10 // Задержка для завершения операции вычисления сопротивления
#endif

#ifndef _timePause
#define _timePause		20 // задержка на установку режима после подачи/снятия напряжения смещения
#endif

#ifndef _timeMeasure
#define _timeMeasure				120 //80 //120 // задержка на измерение сопротивления
#endif

#ifndef _timeMeasureVolt
#define _timeMeasureVolt		70 // задержка на измерение напряжения
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIG0_Pin GPIO_PIN_14
#define DIG0_GPIO_Port GPIOC
#define DIG1_Pin GPIO_PIN_15
#define DIG1_GPIO_Port GPIOC
#define SEG0_Pin GPIO_PIN_0
#define SEG0_GPIO_Port GPIOC
#define SEG1_Pin GPIO_PIN_1
#define SEG1_GPIO_Port GPIOC
#define SEG2_Pin GPIO_PIN_2
#define SEG2_GPIO_Port GPIOC
#define SEG3_Pin GPIO_PIN_3
#define SEG3_GPIO_Port GPIOC
#define Bipolar_Pin GPIO_PIN_0
#define Bipolar_GPIO_Port GPIOA
#define A1_Pin GPIO_PIN_1
#define A1_GPIO_Port GPIOA
#define A2_Pin GPIO_PIN_2
#define A2_GPIO_Port GPIOA
#define A3_Pin GPIO_PIN_3
#define A3_GPIO_Port GPIOA
#define A4_Pin GPIO_PIN_4
#define A4_GPIO_Port GPIOA
#define A5_Pin GPIO_PIN_5
#define A5_GPIO_Port GPIOA
#define A6_Pin GPIO_PIN_6
#define A6_GPIO_Port GPIOA
#define A7_Pin GPIO_PIN_7
#define A7_GPIO_Port GPIOA
#define SEG4_Pin GPIO_PIN_4
#define SEG4_GPIO_Port GPIOC
#define SEG5_Pin GPIO_PIN_5
#define SEG5_GPIO_Port GPIOC
#define EN_HV_Pin GPIO_PIN_15
#define EN_HV_GPIO_Port GPIOB
#define SEG6_Pin GPIO_PIN_6
#define SEG6_GPIO_Port GPIOC
#define SEG7_Pin GPIO_PIN_7
#define SEG7_GPIO_Port GPIOC
#define SW_HV_Pin GPIO_PIN_8
#define SW_HV_GPIO_Port GPIOC
#define HV_POL_Pin GPIO_PIN_8
#define HV_POL_GPIO_Port GPIOA
#define HV_OUT_Pin GPIO_PIN_9
#define HV_OUT_GPIO_Port GPIOA
#define LED_HV_GRN_Pin GPIO_PIN_10
#define LED_HV_GRN_GPIO_Port GPIOC
#define LED_HV_RED_Pin GPIO_PIN_11
#define LED_HV_RED_GPIO_Port GPIOC
#define P0_LED1_Pin GPIO_PIN_4
#define P0_LED1_GPIO_Port GPIOB
#define P0_LED2_Pin GPIO_PIN_5
#define P0_LED2_GPIO_Port GPIOB
#define P1_LED1_Pin GPIO_PIN_6
#define P1_LED1_GPIO_Port GPIOB
#define P1_LED2_Pin GPIO_PIN_7
#define P1_LED2_GPIO_Port GPIOB
#define MDI_G_Pin GPIO_PIN_8
#define MDI_G_GPIO_Port GPIOB
#define MDI_Y_Pin GPIO_PIN_9
#define MDI_Y_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void us_delay(uint16_t d);
void HEX_digit(int digit, uint16_t pin);
_Bool timeDel(uint16_t time);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
//gin
