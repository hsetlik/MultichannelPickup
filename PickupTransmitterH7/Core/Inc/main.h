/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define LORA_SCK_Pin GPIO_PIN_2
#define LORA_SCK_GPIO_Port GPIOE
#define LORA_NRST_Pin GPIO_PIN_3
#define LORA_NRST_GPIO_Port GPIOE
#define LORA_CS_Pin GPIO_PIN_4
#define LORA_CS_GPIO_Port GPIOE
#define LORA_MISO_Pin GPIO_PIN_5
#define LORA_MISO_GPIO_Port GPIOE
#define LORA_MOSI_Pin GPIO_PIN_6
#define LORA_MOSI_GPIO_Port GPIOE
#define LORA_BUSY_Pin GPIO_PIN_0
#define LORA_BUSY_GPIO_Port GPIOE
#define LORA_IRQ_Pin GPIO_PIN_1
#define LORA_IRQ_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

// for global access to the communication handles
extern I2S_HandleTypeDef hi2s1;
extern I2S_HandleTypeDef hi2s2;
extern I2S_HandleTypeDef hi2s3;
extern SPI_HandleTypeDef hspi4;
#define RADIO_SPI hspi4

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
