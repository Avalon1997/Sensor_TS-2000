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
extern uint8_t USART1_Tsimen_ERROR[3];
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

// uint16_t ModBus_CRC16( uint8_t *pdata, int len);
// void memv(uint8_t *pdata,uint8_t *ppdata,int st1,int st2,int length);
void WaitandClear(void);
void GetSpecData(uint8_t *specdata,int a);
// void Config_WaitTime(void);
void STM32_Init(void);
void SPEC_Init(void);
void Coefficient_Init(void);
uint8_t STM32FLASH_WriteWave(uint8_t *writedata,uint16_t length);
uint8_t STM32FLASH_WriteEnergy(uint16_t *writedata,uint16_t length);
void Judge_SpecStatus(void);
// void Calibrate_Wavelength(void);
// void Send_Double(double *pr, int a);
// double Convert_CharToDouble(uint8_t *pr,int a,int b);
// void Send_Double(double *pr,uint8_t a);
// void Receive_Double(uint8_t *pr,int a);
// void Receive_Float(uint8_t *pr,int a);
// void Send_Float(float *pr,int a);
void UnderCoefficient(uint8_t *xenon1,uint8_t *xenon2, uint8_t *xenon3,int dark,int ref,int sam);
void UnderCoefficient_WithoutLightpath(uint8_t *xenon1,uint8_t *xenon2, uint8_t *xenon3,int dark,int ref,int sam);
void delay_us(__IO uint32_t delay);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define CPU_FREQUENCY_MHZ 72
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
