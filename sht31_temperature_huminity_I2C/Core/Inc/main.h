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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// EL TERMINAL PINLERI.....................................................................
//#define  	BUTTON_1_PIN				GPIO_Pin_11  // PB8

#define  	DSP_CLK_PIN			  	    GPIO_PIN_6  // PB13
#define  	DSP_CLK_HI		    	    HAL_GPIO_WritePin(GPIOB,DSP_CLK_PIN,GPIO_PIN_SET)
#define  	DSP_CLK_LO			  	    HAL_GPIO_WritePin(GPIOB,DSP_CLK_PIN,GPIO_PIN_RESET)

#define  	DSP_SI_PIN			  	    GPIO_PIN_14  // PB14
#define  	DSP_SI_HI		      	    HAL_GPIO_WritePin(GPIOB,DSP_SI_PIN,GPIO_PIN_SET)
#define  	DSP_SI_LO			        HAL_GPIO_WritePin(GPIOB,DSP_SI_PIN,GPIO_PIN_RESET)

#define  	DSP_STB_PIN 			    GPIO_PIN_15  // PB15
#define  	DSP_STB_HI		    	    HAL_GPIO_WritePin(GPIOB,DSP_STB_PIN,GPIO_PIN_SET)
#define  	DSP_STB_LO			  	    HAL_GPIO_WritePin(GPIOB,DSP_STB_PIN,GPIO_PIN_RESET)
/* USER CODE END Includes */
//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
