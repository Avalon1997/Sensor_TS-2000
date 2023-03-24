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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "S_IIC.h"
#include "SHT30.h"
#include "Flash.h"
#include "Tool.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

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

//----- Miscellaneous variable definitions BEGIN
uint8_t WHILEA = 0, 
        A = 0, 
        FLASHSTATUS = 4;
uint8_t CRC_DATA[8], 
        CHECKDATA[8] = {0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

int i = 0;
uint16_t CRC16 = 0;
         
float DarkData[1024] = {0}, 
      RefData[1024] = {0}, 
      SamData[1024] = {0}, 
      ABS[1024] = {0},
      Optical_path = 5.0;
double Wavelength = 0;
//----- Miscellaneous variable definitions END

//----- Steering engine angle parameter & Version BEGIN
int PWM_DARK = 920, 
    PWM_REFERENCE = 1150, 
    PWM_SAMPLE = 730;
uint8_t static USART1_Tsimen_Version[]       = "TS-2000-000005/V1.1.2";    
//----- Steering engine angle parameter & Version END


/* ----- Master computer communication command (19 instructions) ----- */

// ----- status check array BEGIN
uint8_t static USART1_Tsimen_Reset[]             = {0x01,0x01,0x00,0x00,0x00,0x00,0x0A,0x3C};      // 1 Global reset
uint8_t static USART1_Tsimen_Ver[]               = {0x01,0x02,0x00,0x00,0x00,0x00,0x0A,0x78};      // 2 Read the sensor version
uint8_t static USART1_Tsimen_CheckInt[]          = {0x01,0x04,0x00,0x00,0x00,0x00,0x0A,0xF0};      // 4 Check the current integration time of the spectrometer
uint8_t static USART1_Tsimen_CheckAve[]          = {0x01,0x06,0x00,0x00,0x00,0x00,0xCA,0x89};      // 6 Check the current average times of the spectrometer
uint8_t static USART1_Tsimen_CheckWLC[]          = {0x01,0x0E,0x00,0x00,0x00,0x00,0x0B,0x2C};      // 14 Read the wavelength coefficient
uint8_t static USART1_Tsimen_CheckENC[]          = {0x01,0x10,0x00,0x00,0x00,0x00,0x09,0xC0};      // 16 Read the energy cofficient
uint8_t static USART1_Tsimen_CheckOD[]           = {0x01,0x12,0x00,0x00,0x00,0x00,0xC9,0xB9};      // 18 Read the Optical path
uint8_t static USART1_Tsimen_GlobalReset[]       = {0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0x4A,0x14};      // 99 Global Reset
// ----- status check array END

// ----- configuration and measurement array BEGIN
uint8_t static USART1_Tsimen_Int[]               = {0x01,0x03,0x00,0x00,0x01,0xF4,0xDD,0x45};      // 3 Set the spectrometer integration time (example)
uint8_t static USART1_Tsimen_Ave[]               = {0x01,0x05,0x00,0x32,0x00,0x00,0x05,0x6C};      // 5 Set the spectrometer average Times (example)
uint8_t static USART1_Tsimen_DarSignal[]         = {0x01,0x07,0x00,0x00,0x00,0x00,0x0A,0xB4};      // 7 Read the spectrometer data under dark current conditions
uint8_t static USART1_Tsimen_RefSignal[]         = {0x01,0x08,0x00,0x00,0x00,0x00,0x0B,0xE0};      // 8 Read the spectrometer data under reference signal conditions
uint8_t static USART1_Tsimen_SamSignal[]         = {0x01,0x09,0x00,0x00,0x00,0x00,0xCB,0xDD};      // 9 Read the spectrometer data under sample signal conditions
uint8_t static USART1_Tsimen_FullData[]          = {0x01,0x0A,0x00,0x00,0x00,0x00,0xCB,0x99};      // 10 One-click acquisition of dark, parametric and sample spectrometer data
uint8_t static USART1_Tsimen_TempData[]          = {0x01,0x0B,0x00,0x00,0x00,0x00,0x0B,0xA4};      // 11 Read temperature and humidity data
uint8_t static USART1_Tsimen_WL[]                = {0x01,0x0C,0x00,0x00,0x00,0x00,0xCB,0x11};      // 12 Read the calibrated wavelength
uint8_t static USART1_Tsimen_SetWLC[]            = { 0x01,0x0D,
                                                     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                     0x3D,0xB1,0x7F,0x1C,0x7E,0x71,0xE7,0x98,
                                                     0xBE,0x6D,0x29,0x79,0xFF,0xA7,0x63,0x0F,
                                                     0x3E,0xF3,0xAA,0x03,0x46,0xF2,0x1A,0x6E,
                                                     0x3F,0xE5,0x6F,0x47,0x42,0xCC,0x1F,0x27,
                                                     0x40,0x66,0xBA,0xE8,0x7E,0x6E,0x46,0x1C,
                                                     0x60,0xDD};                                   // 13 Set the wavelength coefficient
uint8_t static USART1_Tsimen_SetENC[]           = {0x01,0x0F};                                     // 15 Set the energy cofficient
uint8_t static USART1_Tsimen_UnderCoefficient[] = {0x01,0x11,0x00,0x00,0x00,0x00,0xC9,0xFD};       // 17 Read the spectrometer data under the coefficient and read the ABS
uint8_t static USART1_Tsimen_SetOD[]            = {0x01,0x13,0x00,0x00,0x00,0x05,0x0A,0x44};       // 19 Set the Optical path
uint8_t static USART1_Tsimen_WithoutLightpath[] = {0x01,0x14,0x00,0x00,0x00,0x00,0xC9,0x31};       // 17 Read the spectrometer data under the coefficient and read the ABS
// ----- configuration and measurement array END

// ----- feedback array BEGIN
uint8_t static USART1_Tsimen_OK[]               = {0x01,0x52,0x49};                                // OK code
uint8_t        USART1_Tsimen_ERROR[]            = {0x01,0x46,0x41};                                // ERROR code
uint8_t static USART1_Tsimen_SpecERROR[]        = {0x01,0x53,0x50,0x45,0x43,0x45,0x52};            // Spectrometer has some problems, maybe it is broken
uint8_t static USART1_Tsimen_CRCERROR[]         = {0x01,0x43,0x52,0x43,0x45,0x52};                 // CRC ERROR
uint8_t static USART1_Tsimen_SpecOK[]           = {0x01,0x53,0x50,0x45,0x43,0x4F,0x4B};            // Handle with the Spectrometer
uint8_t static USART1_Tsimen_AA[]               = {0x01,0xAA,0x55,0xBB,0x44,0xCC,0x33,0xDD,0x22};
uint8_t static USART1_Tsimen_DD[]               = {0xDD,0xDD,0xAA,0xAA};
uint8_t static USART1_Tsimen_Stemp[]            = {0x34,0x34,0x2e,0x33,0x34};
uint8_t static master = 0x01;
// ----- feedback array END



//----- Spectrometer communication command (11)

uint8_t static USART2_Spec_Reset[]       = {0x52,0xBD,0x3E};                               // Spec reset
uint8_t static USART2_Spec_Int[]         = {0x69,0x00,0x00,0x01,0xF4,0x1E,0x78};           // Set spec int time
uint8_t static USART2_Spec_Int_Default[] = {0x69,0x00,0x00,0x01,0xF4,0x1E,0x78};           // Default spec int time
uint8_t static USART2_Spec_CheckInt[]    = {0x3F,0x69,0x6E,0xD0};                          // Check spec int time
uint8_t static USART2_Spec_Ave[]         = {0x41,0x00,0x14,0xDB,0x21};                     // Set spec average
uint8_t static USART2_Spec_Ave_Default[] = {0x41,0x00,0x32,0x01,0xA0};                     // Default spec average
uint8_t static USART2_Spec_CheckAve[]    = {0x3F,0x41,0x70,0xD0};                          // Check spec average
uint8_t static USART2_Spec_XenonOff[]    = {0x31,0x00,0x20,0x14};                          // Turn the xenon off
// uint8_t static USART2_Spec_XenonOnCon[]  = {0x31,0x01,0xE0,0xD5};                          // Turn the xenon on under continuous mode
uint8_t static USART2_Spec_XenonOnOne[]  = {0x31,0x81,0x40,0xD4};                          // Turn the xenon on under signal mode
uint8_t static USART2_Spec_Data[]        = {0x53,0x7D,0xFF};                               // Read spec data
uint8_t static USART2_Spec_OK[]          = {0x06,0x42,0x3F};                               // OK code
uint8_t static USART2_Spec_ERROR[]       = {0x15,0x8F,0x7E};                               // ERROR code
uint8_t static USART2_Spec_FormData[]    = {0x06,0xAA,0x55,0xBB,0x44,0xCC,0x33,0xDD,0x22}; // Spectrometer Form Data


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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

  // ----- Clear the array cache
  memset(CRC_DATA,0,sizeof(CRC_DATA));
  memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
  memset(DATA_CACHE2,0,RX2BUFFERSIZE);
  memset(Energy_Coefficient,0,sizeof(Energy_Coefficient));
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // ------ System Init
  Coefficient_Init();
  STM32_Init();
  SPEC_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    //----- Working LED
    HAL_GPIO_TogglePin(GPIOB,LED_Pin);
    HAL_Delay(50);

    /*--------------------------------------------------Main Part-------------------------------------------------------*/

    /*--------------------------------------------------Config Part-------------------------------------------------------*/
    //----- command one: Global reset -----
    if (memcmp(DATA_CACHE1,USART1_Tsimen_Reset,6) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        PWM_PulseWidth(PWM_DARK);
        HAL_UART_Transmit(&huart2,USART2_Spec_Reset,sizeof(USART2_Spec_Reset),0xFFFF);
        HAL_Delay(1500);
        HAL_UART_Transmit(&huart2,USART2_Spec_Int_Default,sizeof(USART2_Spec_Int_Default),0xFFFF);
        HAL_Delay(30);
        HAL_UART_Transmit(&huart2,USART2_Spec_Ave_Default,sizeof(USART2_Spec_Ave_Default),0xFFFF);
        HAL_Delay(30);
        // Config_WaitTime();
        WaitandClear();
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }

    //----- command two: Read the sensor Version -----
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_Ver,6) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0; 
        HAL_UART_Transmit(&huart1,&master,1,0xFFFF);
        HAL_UART_Transmit(&huart1,USART1_Tsimen_Version,21,0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
      else 
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }

    //----- command three: Set the spectrometer integration time -----
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_Int,2) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        memv(USART2_Spec_Int,DATA_CACHE1,2,3,4);
        CRC16 = ModBus_CRC16(USART2_Spec_Int,5);
        USART2_Spec_Int[6] = CRC16&0xFF;
        USART2_Spec_Int[5] = (CRC16>>8)&0xFF;
        CRC16 = 0;
        HAL_UART_Transmit(&huart2,USART2_Spec_Int,sizeof(USART2_Spec_Int),0xFFFF);
        HAL_Delay(30);
        WaitandClear();
        // Config_WaitTime();
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }

    //----- command four: Check the spectrometer integration time -----
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_CheckInt,6) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        HAL_UART_Transmit(&huart2,USART2_Spec_CheckInt,sizeof(USART2_Spec_CheckInt),0xFFFF);
        HAL_Delay(30);
        memv(CHECKDATA,DATA_CACHE2,2,2,4);
        HAL_UART_Transmit(&huart1,CHECKDATA,5,0xFFFF);
        USART_RX2_LENDEMO = 0;
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
        memset(DATA_CACHE2,0,RX2BUFFERSIZE);
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }

    //----- command five: Set the spectrometer average Times -----
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_Ave,2) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        memv(USART2_Spec_Ave,DATA_CACHE1,2,5,2);
        CRC16 = ModBus_CRC16(USART2_Spec_Ave,3);
        USART2_Spec_Ave[4] = CRC16&0xFF;
        USART2_Spec_Ave[3] = (CRC16>>8)&0xFF;
        CRC16 = 0;
        HAL_UART_Transmit(&huart2,USART2_Spec_Ave,sizeof(USART2_Spec_Ave),0xFFFF);
        HAL_Delay(30);
        WaitandClear();
        // Config_WaitTime();
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }

    //----- command six: Check the current average times of the spectrometer -----
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_CheckAve,6) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        HAL_UART_Transmit(&huart2,USART2_Spec_CheckAve,sizeof(USART2_Spec_CheckAve),0xFFFF);
        HAL_Delay(30);
        memv(CHECKDATA,DATA_CACHE2,2,2,2);
        HAL_UART_Transmit(&huart1,CHECKDATA,3,0xFFFF);
        USART_RX2_LENDEMO = 0;
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
        memset(DATA_CACHE2,0,RX2BUFFERSIZE);
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }

    //----- command twelve: Read the wavelenth -----
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_WL,6) == RESET)
    { 
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;

        HAL_UART_Transmit(&huart1,USART1_Tsimen_AA,sizeof(USART1_Tsimen_AA),0xFFFF);
        //Counter the wavelenth and Send them to the master
        for (Dou_i=1;Dou_i<1025;Dou_i++)
        {
          Wavelength = 
          Wave_Coefficient[0]*(long long int)Dou_i*(long long int)Dou_i*(long long int)Dou_i*(long long int)Dou_i*(long long int)Dou_i+
          Wave_Coefficient[1]*(long long int)Dou_i*(long long int)Dou_i*(long long int)Dou_i*(long long int)Dou_i+
          Wave_Coefficient[2]*(long long int)Dou_i*(long long int)Dou_i*(long long int)Dou_i+
          Wave_Coefficient[3]*(long long int)Dou_i*(long long int)Dou_i+
          Wave_Coefficient[4]*(long long int)Dou_i+
          Wave_Coefficient[5];
          DoubleBuf[7] = *((uint8_t*)&Wavelength);
          DoubleBuf[6] = *((uint8_t*)&Wavelength+1);
          DoubleBuf[5] = *((uint8_t*)&Wavelength+2);
          DoubleBuf[4] = *((uint8_t*)&Wavelength+3);
          DoubleBuf[3] = *((uint8_t*)&Wavelength+4);
          DoubleBuf[2] = *((uint8_t*)&Wavelength+5);
          DoubleBuf[1] = *((uint8_t*)&Wavelength+6);
          DoubleBuf[0] = *((uint8_t*)&Wavelength+7);
          HAL_UART_Transmit(&huart1,DoubleBuf,sizeof(DoubleBuf),0xFFFF);
          // printf("%lf\r\n",Wavelength);
        }
        HAL_UART_Transmit(&huart1,USART1_Tsimen_DD,sizeof(USART1_Tsimen_DD),0xFFFF);

        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }

    //----- command thirteen: Set the wavelength cofficient -----
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_SetWLC,2) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,50);
      if (DATA_CACHE1[51]==(uint8_t)CRC16&0xFF && DATA_CACHE1[50]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        // Receive_Double(DATA_CACHE1,6);
        HAL_UART_DMAStop(&huart1);
        __HAL_UART_DISABLE_IT(&huart1,UART_IT_IDLE);
        HAL_UART_DMAStop(&huart2);
        __HAL_UART_DISABLE_IT(&huart2,UART_IT_IDLE);
        FLASHSTATUS = STM32FLASH_WriteWave(DATA_CACHE1,48);
        __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
        HAL_UART_Receive_DMA(&huart1,USART_RX1_BUFFER,RX1BUFFERSIZE);
        __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
        HAL_UART_Receive_DMA(&huart2,USART_RX2_BUFFER,RX2BUFFERSIZE);
        if (FLASHSTATUS == 0)
        {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_OK,sizeof(USART1_Tsimen_OK),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
        FLASHSTATUS = 4;
        }
        else if (FLASHSTATUS == 1)
        {
          HAL_UART_Transmit(&huart1,USART1_Tsimen_ERROR,sizeof(USART1_Tsimen_ERROR),0xFFFF);
          memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
          FLASHSTATUS = 4;
        }
      }

      else
      {
        HAL_UART_Transmit(&huart1,DATA_CACHE1,52,0xFFFF);
        HAL_Delay(500);
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }

    // ----- command fouteen: Read the wavelength cofficient -----
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_CheckWLC,6) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        HAL_UART_Transmit(&huart1,&master,1,0xFFFF);
        Send_Double(Wave_Coefficient,6);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }

    // ----- command fifteen: Set the energy cofficient -----
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_SetENC,2) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,4100);
      if (DATA_CACHE1[4101]==(uint8_t)CRC16&0xFF && DATA_CACHE1[4100]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        // Receive_Float(DATA_CACHE1,DATA_CACHE1[3]);
        // HAL_TIM_Base_Stop_IT(&htim3);
        HAL_UART_DMAStop(&huart1);
        __HAL_UART_DISABLE_IT(&huart1,UART_IT_IDLE);
        HAL_UART_DMAStop(&huart2);
        __HAL_UART_DISABLE_IT(&huart2,UART_IT_IDLE);
        FLASHSTATUS = STM32FLASH_WriteEnergy((uint16_t*)DATA_CACHE1,48);
        __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
        HAL_UART_Receive_DMA(&huart1,USART_RX1_BUFFER,RX1BUFFERSIZE);
        __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
        HAL_UART_Receive_DMA(&huart2,USART_RX2_BUFFER,RX2BUFFERSIZE);
        if (FLASHSTATUS == 0)
        {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_OK,sizeof(USART1_Tsimen_OK),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
        FLASHSTATUS = 4;
        }
        else if (FLASHSTATUS == 1)
        {
          HAL_UART_Transmit(&huart1,USART1_Tsimen_ERROR,sizeof(USART1_Tsimen_ERROR),0xFFFF);
          memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
          FLASHSTATUS = 4;
        }
        // HAL_TIM_Base_Start_IT(&htim3);
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }

    // ----- command sixteen: Read the energy cofficient -----
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_CheckENC,6) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        HAL_UART_Transmit(&huart1,USART1_Tsimen_AA,sizeof(USART1_Tsimen_AA),0xFFFF);
        Send_Float(Energy_Coefficient,2048);
        HAL_UART_Transmit(&huart1,USART1_Tsimen_DD,sizeof(USART1_Tsimen_DD),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }

    // ----- command eitghteen: Read the Optical path
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_CheckOD,6) == RESET )
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        HAL_UART_Transmit(&huart1,&master,1,0xFFFF);
        Send_Float(&Optical_path,1);

        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }

    // ----- command nineteen: Set the Optical path
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_SetOD,2) == RESET )
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        // printf("optical path: %d \r\n",DATA_CACHE1[5]);
        STM32FLASH_Write(STM32_FLASH_OPTICALPATH,(uint16_t*)&DATA_CACHE1[2],2);
        memset(DATA_CACHE2,0,sizeof(DATA_CACHE2));
        STM32FLASH_Read(STM32_FLASH_OPTICALPATH,(uint16_t*)DATA_CACHE2,2);
        FloatBuf[0] = DATA_CACHE2[3];
        FloatBuf[1] = DATA_CACHE2[2];
        FloatBuf[2] = DATA_CACHE2[1];
        FloatBuf[3] = DATA_CACHE2[0];
        Optical_path = *((float*)&FloatBuf[0]);
        // printf("%f",Optical_path);
        
        HAL_UART_Transmit(&huart1,USART1_Tsimen_OK,sizeof(USART1_Tsimen_OK),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }

    /*--------------------------------------------------Data Part-------------------------------------------------------*/
    //----- command seven: Read the spectrometer data under dark current conditions -----
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_DarSignal,6) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        // HAL_UART_DMAStop(&huart1);
        // __HAL_UART_DISABLE_IT(&huart1,UART_IT_IDLE);
        GetSpecData(USART2_Spec_XenonOff,PWM_DARK);
        // __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
        // HAL_UART_Receive_DMA(&huart1,USART_RX1_BUFFER,RX1BUFFERSIZE);
      }
      
      else 
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }
   
    //----- command eight: Read the spectrometer data under reference signal conditions -----
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_RefSignal,6) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        // HAL_UART_DMAStop(&huart1);
        // __HAL_UART_DISABLE_IT(&huart1,UART_IT_IDLE);
        GetSpecData(USART2_Spec_XenonOnOne,PWM_REFERENCE);
        // __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
        // HAL_UART_Receive_DMA(&huart1,USART_RX1_BUFFER,RX1BUFFERSIZE);
      }
      
      else 
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }

    //----- command nine: Read the spectrometer data under sample signal conditions -----
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_SamSignal,6) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        // HAL_UART_DMAStop(&huart1);
        // __HAL_UART_DISABLE_IT(&huart1,UART_IT_IDLE);
        GetSpecData(USART2_Spec_XenonOnOne,PWM_SAMPLE);
        // __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
        // HAL_UART_Receive_DMA(&huart1,USART_RX1_BUFFER,RX1BUFFERSIZE);
      }
      
      else 
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }

    //----- command ten: One-click acquisition of dark, parametric and sample spectrometer data -----
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_FullData,6) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        // HAL_UART_DMAStop(&huart1);
        // __HAL_UART_DISABLE_IT(&huart1,UART_IT_IDLE);
        GetSpecData(USART2_Spec_XenonOff,PWM_DARK);
        HAL_Delay(50);
        GetSpecData(USART2_Spec_XenonOnOne,PWM_REFERENCE);
        HAL_Delay(50);
        GetSpecData(USART2_Spec_XenonOnOne,PWM_SAMPLE);
        // PWM_PulseWidth(PWM_DARK);
        // __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
        // HAL_UART_Receive_DMA(&huart1,USART_RX1_BUFFER,RX1BUFFERSIZE);
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }

    //----- command eleven: Read temperature and humidity data -----
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_TempData,6) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        // HAL_UART_DMAStop(&huart1);
        // __HAL_UART_DISABLE_IT(&huart1,UART_IT_IDLE);
        // HAL_UART_DMAStop(&huart2);
        // __HAL_UART_DISABLE_IT(&huart2,UART_IT_IDLE);
        HAL_UART_Transmit(&huart1,&master,1,0xFFFF);
        // HAL_UART_Transmit(&huart1,USART1_Tsimen_Stemp,sizeof(USART1_Tsimen_Stemp),0x2000);
        // HAL_UART_Transmit(&huart1,USART1_Tsimen_Stemp,sizeof(USART1_Tsimen_Stemp),0x2000);
        SHT30_ReadData(0x44);
        HAL_UART_Transmit(&huart1,USART1_Tsimen_Stemp,sizeof(USART1_Tsimen_Stemp),0x2000);
        // __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
        // HAL_UART_Receive_DMA(&huart1,USART_RX1_BUFFER,RX1BUFFERSIZE);
        // __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
        // HAL_UART_Receive_DMA(&huart2,USART_RX2_BUFFER,RX2BUFFERSIZE);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }

    // ----- command seventeen: Read the whole spectrometer data under coeffiecient and the ABS data
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_UnderCoefficient,6) == RESET )
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        HAL_UART_Transmit(&huart1,USART1_Tsimen_AA,sizeof(USART1_Tsimen_AA),0xFFFF);
        UnderCoefficient(USART2_Spec_XenonOff,USART2_Spec_XenonOnOne,USART2_Spec_XenonOnOne,PWM_DARK,PWM_REFERENCE,PWM_SAMPLE);
        HAL_UART_Transmit(&huart1,USART1_Tsimen_DD,sizeof(USART1_Tsimen_DD),0xFFFF);
        // PWM_PulseWidth(PWM_DARK);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }
    }

    // ----- command twenty: Read the whole spectrometer data under coeffiecient and the ABS data (without lightpath)
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_WithoutLightpath,6) == RESET )
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        HAL_UART_Transmit(&huart1,USART1_Tsimen_AA,sizeof(USART1_Tsimen_AA),0xFFFF);
        UnderCoefficient_WithoutLightpath(USART2_Spec_XenonOff,USART2_Spec_XenonOnOne,USART2_Spec_XenonOnOne,PWM_DARK,PWM_REFERENCE,PWM_SAMPLE);
        HAL_UART_Transmit(&huart1,USART1_Tsimen_DD,sizeof(USART1_Tsimen_DD),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
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
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ---------------------------------------------------------------------- Init Function -------------------------------------------------------------------------- */
/**
 * @brief Init the STM32 System
 * 
 */
void STM32_Init(void)
{
  /*--------------------------------------------------Init the usart DMA-------------------------------------------------------*/
  HAL_UART_Receive_DMA(&huart1,USART_RX1_BUFFER,sizeof(DATA_CACHE1));
  HAL_UART_Receive_DMA(&huart2,USART_RX2_BUFFER,RX2BUFFERSIZE);
  SHT30_ReadData(0x44);
  SHT30_ReadData(0x44);

  /*--------------------------------------------------Init the PWM-------------------------------------------------------*/
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,PWM_DARK);
  // HAL_TIM_Base_Start_IT(&htim3);

  
}

/**
 * @brief Init the Spectrometer
 * 
 */
void SPEC_Init(void)
{
  HAL_UART_Transmit(&huart2,USART2_Spec_Reset,sizeof(USART2_Spec_Reset),0xFFFF);
  HAL_Delay(1500);
  Judge_SpecStatus();
  HAL_UART_Transmit(&huart2,USART2_Spec_Int_Default,sizeof(USART2_Spec_Int_Default),0xFFFF);
  HAL_Delay(30);
  HAL_UART_Transmit(&huart2,USART2_Spec_Ave_Default,sizeof(USART2_Spec_Ave_Default),0xFFFF);
  HAL_Delay(30);
  // Config_WaitTime();
}

/**
 * @brief 
 * 
 */
void Coefficient_Init(void)
{
  // ----- Read the Wave Coefficient
  STM32FLASH_Read(STM32_FLASH_WASAVEADDR,(uint16_t*)DATA_CACHE1,24);
  //Read the Wave Coefficient In Flash
  for (i=0;i<6;i++)
  {
    DoubleBuf[0] = DATA_CACHE1[7+i*8];
    DoubleBuf[1] = DATA_CACHE1[6+i*8];
    DoubleBuf[2] = DATA_CACHE1[5+i*8];
    DoubleBuf[3] = DATA_CACHE1[4+i*8];
    DoubleBuf[4] = DATA_CACHE1[3+i*8];
    DoubleBuf[5] = DATA_CACHE1[2+i*8];
    DoubleBuf[6] = DATA_CACHE1[1+i*8];
    DoubleBuf[7] = DATA_CACHE1[i*8];
    Wave_Coefficient[i] = *((double*)&DoubleBuf[0]);
    // printf("%.15lf %d\r\n",Wave_Coefficient[i],i);
  }
  // ----- Read the Wave Coefficient

  // ----- Read the Energy Coefficient
  STM32FLASH_Read(STM32_FLASH_ENSAVEADDR1,(uint16_t*)DATA_CACHE2,1024);
  for (Ene_i=0;Ene_i<512;Ene_i++)
  {
    FloatBuf[0] = DATA_CACHE2[3+Ene_i*4];
    FloatBuf[1] = DATA_CACHE2[2+Ene_i*4];
    FloatBuf[2] = DATA_CACHE2[1+Ene_i*4];
    FloatBuf[3] = DATA_CACHE2[Ene_i*4];
    Energy_Coefficient[Ene_i*2] = *((float*)&FloatBuf[0]);
    // printf("%.6f %d\r\n",Energy_Coefficient[Ene_i*2],Ene_i*2);
  }
  STM32FLASH_Read(STM32_FLASH_ENSAVEADDR2,(uint16_t*)DATA_CACHE2,1024);
  for (Ene_i=512;Ene_i<1024;Ene_i++)
  {
    FloatBuf[0] = DATA_CACHE2[3+(Ene_i-512)*4];
    FloatBuf[1] = DATA_CACHE2[2+(Ene_i-512)*4];
    FloatBuf[2] = DATA_CACHE2[1+(Ene_i-512)*4];
    FloatBuf[3] = DATA_CACHE2[(Ene_i-512)*4];
    Energy_Coefficient[Ene_i*2] = *((float*)&FloatBuf[0]);
    // printf("%.6f %d\r\n",Energy_Coefficient[Ene_i*2],Ene_i*2);
  }
  STM32FLASH_Read(STM32_FLASH_ENSAVEADDR3,(uint16_t*)DATA_CACHE2,1024);
  for (Ene_i=0;Ene_i<512;Ene_i++)
  {
    FloatBuf[0] = DATA_CACHE2[3+Ene_i*4];
    FloatBuf[1] = DATA_CACHE2[2+Ene_i*4];
    FloatBuf[2] = DATA_CACHE2[1+Ene_i*4];
    FloatBuf[3] = DATA_CACHE2[Ene_i*4];
    Energy_Coefficient[Ene_i*2+1] = *((float*)&FloatBuf[0]);
    // printf("%.6f %d\r\n",Energy_Coefficient[Ene_i*2+1],Ene_i*2+1);
  }
  STM32FLASH_Read(STM32_FLASH_ENSAVEADDR4,(uint16_t*)DATA_CACHE2,1024);
  for (Ene_i=512;Ene_i<1024;Ene_i++)
  {
    FloatBuf[0] = DATA_CACHE2[3+(Ene_i-512)*4];
    FloatBuf[1] = DATA_CACHE2[2+(Ene_i-512)*4];
    FloatBuf[2] = DATA_CACHE2[1+(Ene_i-512)*4];
    FloatBuf[3] = DATA_CACHE2[(Ene_i-512)*4];
    Energy_Coefficient[Ene_i*2+1] = *((float*)&FloatBuf[0]);
    // printf("%.6f %d\r\n",Energy_Coefficient[Ene_i*2+1],Ene_i*2+1);
  }
  // ----- Read the Energy Coefficient

  // ----- Read the Optical Path
  STM32FLASH_Read(STM32_FLASH_OPTICALPATH,(uint16_t*)DATA_CACHE2,2);
  FloatBuf[0] = DATA_CACHE2[3];
  FloatBuf[1] = DATA_CACHE2[2];
  FloatBuf[2] = DATA_CACHE2[1];
  FloatBuf[3] = DATA_CACHE2[0];
  Optical_path = *((float*)&FloatBuf[0]);
  // ----- Read the Optical Path

    // for ( i = 0; i < 6; i++)
    // {
    //   printf("%.15lf %d\r\n",Wave_Coefficient[i],i+1);
    // }

    // for ( i = 0; i < 2048; i++)
    // {
    //   printf("%.6f %d\r\n",Energy_Coefficient[i],i+1);
    // }
    
}
/* ---------------------------------------------------------------------- Init Function -------------------------------------------------------------------------- */


/**
 * @brief Systick delay
 * 
 * @param delay 
 */
void delay_us(__IO uint32_t delay)
{
    int last, curr, val;
    int temp;

    while (delay != 0)
    {
        temp = delay > 900 ? 900 : delay;
        last = SysTick->VAL;
        curr = last - CPU_FREQUENCY_MHZ * temp;
        if (curr >= 0)
        {
            do
            {
                val = SysTick->VAL;
            }
            while ((val < last) && (val >= curr));
        }
        else
        {
            curr += CPU_FREQUENCY_MHZ * 1000;
            do
            {
                val = SysTick->VAL;
            }
            while ((val <= last) || (val > curr));
        }
        delay -= temp;
    }
}

/**
 * @brief Wait and clear the cache data
 * 
 */
void  WaitandClear(void)
{
        while (WHILEA < 10)
        {
          if (memcmp(DATA_CACHE2,USART2_Spec_OK,3) ==  RESET)
          {
            HAL_UART_Transmit(&huart1,USART1_Tsimen_OK,sizeof(USART1_Tsimen_OK),0xFFFF);
            break;
          }
          else if (memcmp(DATA_CACHE2,USART2_Spec_ERROR,3) == RESET)
          {
            HAL_UART_Transmit(&huart1,USART1_Tsimen_ERROR,sizeof(USART1_Tsimen_ERROR),0xFFFF);
            break;
          }
          WHILEA++;
        }

        if (WHILEA == 10)
        {
          HAL_UART_Transmit(&huart1,USART1_Tsimen_SpecERROR,sizeof(USART1_Tsimen_SpecERROR),0xFFFF);
        }
        
        WHILEA = 0;
        USART_RX2_LENDEMO = 0;
        memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
        memset(DATA_CACHE2,0,RX2BUFFERSIZE);
}

/**
 * @brief Get the Spec Data object
 * 
 * @param specdata control the xenon's status
 * @param a set the pulse width of the PWM_timer2
 */
void GetSpecData(uint8_t *specdata,int a)
{
  HAL_UART_Transmit(&huart2,specdata,sizeof(specdata),0xFFFF);
  HAL_Delay(30);

  if (memcmp(DATA_CACHE2,USART2_Spec_OK,sizeof(USART2_Spec_OK)) == RESET)
  {
    PWM_PulseWidth(a);
    memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
    memset(DATA_CACHE2,0,RX2BUFFERSIZE);
    // HAL_UART_DMAStop(&huart2);
    // __HAL_UART_DISABLE_IT(&huart2,UART_IT_IDLE);
    HAL_Delay(500);

    HAL_UART_Transmit(&huart2,USART2_Spec_Data,sizeof(USART2_Spec_Data),0xFFFF);
    // HAL_UART_DMAStop(&huart1);
    while (memcmp(DATA_CACHE2,USART2_Spec_FormData,9) != RESET)
    {
      HAL_Delay(20);
      if (memcmp(DATA_CACHE1,USART1_Tsimen_GlobalReset,sizeof(USART1_Tsimen_GlobalReset)) == RESET)
      {
        CRC16 = ModBus_CRC16(DATA_CACHE1,6);
        if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
        {
          CRC16 = 0;
          // HAL_UART_Transmit(&huart1,USART1_Tsimen_OK,sizeof(USART1_Tsimen_OK),0xFFFF);
          // break;
          printf("Will restart the MCU in 1s ...");
          HAL_Delay(1000);
          __set_PRIMASK(1);
          HAL_NVIC_SystemReset();
        }
      }
    }
    HAL_Delay(300);
    HAL_UART_Transmit(&huart1,DATA_CACHE2,2063,0xFFFF);

    //----- clear variables
    USART_RX2_LENDEMO = 0;
    memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
    memset(DATA_CACHE2,0,RX2BUFFERSIZE);
    // __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
    // __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
    // HAL_UART_Receive_DMA(&huart1,USART_RX1_BUFFER,RX1BUFFERSIZE);
    HAL_Delay(20);
    // HAL_UART_Receive_DMA(&huart2,DATA_CACHE2,RX2BUFFERSIZE);
  }
  
  else if (memcmp(DATA_CACHE2,USART2_Spec_ERROR,sizeof(USART2_Spec_ERROR)) == RESET)
  {
    HAL_UART_Transmit(&huart1,USART1_Tsimen_ERROR,sizeof(USART1_Tsimen_ERROR),0xFFFF);
    memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
    memset(DATA_CACHE2,0,RX2BUFFERSIZE);
  }
  else 
  {
    HAL_UART_Transmit(&huart1,USART1_Tsimen_SpecERROR,sizeof(USART1_Tsimen_SpecERROR),0xFFFF);
    memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
  }
}

/**
 * @brief STM32Flash Write Wave Data 
 * @param writedata 
 * @param length 
 * @return uint8_t 
 */
uint8_t STM32FLASH_WriteWave(uint8_t *writedata,uint16_t length)
{
  if (DATA_CACHE1[1] == 0x0D)
  {
    STM32FLASH_Write(STM32_FLASH_WASAVEADDR,(uint16_t*)&writedata[2],24);
    STM32FLASH_Read(STM32_FLASH_WASAVEADDR,(uint16_t*)DATA_CACHE2,24);
    for (i=0;i<6;i++)
    {
      DoubleBuf[0] = DATA_CACHE2[7+i*8];
      DoubleBuf[1] = DATA_CACHE2[6+i*8];
      DoubleBuf[2] = DATA_CACHE2[5+i*8];
      DoubleBuf[3] = DATA_CACHE2[4+i*8];
      DoubleBuf[4] = DATA_CACHE2[3+i*8];
      DoubleBuf[5] = DATA_CACHE2[2+i*8];
      DoubleBuf[6] = DATA_CACHE2[1+i*8];
      DoubleBuf[7] = DATA_CACHE2[i*8];
      Wave_Coefficient[i] = *((double*)&DoubleBuf[0]);
      // printf("%.15lf\r\n",Wave_Coefficient[i]);
    }
    return 0;
  }
  else
    return 1;
}

/**
 * @brief STM32Flash Write Energy Data 
 * 
 * @param writedata 
 * @param length 
 * @return uint8_t 
 */
uint8_t STM32FLASH_WriteEnergy(uint16_t *writedata,uint16_t length)
{
  if (DATA_CACHE1[3] == 0x01)
  {
    STM32FLASH_Write(STM32_FLASH_ENSAVEADDR1,&writedata[2],1024);
    STM32FLASH_Read(STM32_FLASH_ENSAVEADDR1,(uint16_t*)DATA_CACHE2,1024);
      for (Ene_i=0;Ene_i<512;Ene_i++)
      {
        FloatBuf[0] = DATA_CACHE2[3+Ene_i*4];
        FloatBuf[1] = DATA_CACHE2[2+Ene_i*4];
        FloatBuf[2] = DATA_CACHE2[1+Ene_i*4];
        FloatBuf[3] = DATA_CACHE2[Ene_i*4];
        Energy_Coefficient[Ene_i*2] = *((float*)&FloatBuf[0]);
        // printf("%.6f %d\r\n",Energy_Coefficient[Ene_i*2],Ene_i*2);
      }
    STM32FLASH_Write(STM32_FLASH_ENSAVEADDR2,&writedata[2+1024],1024);
    STM32FLASH_Read(STM32_FLASH_ENSAVEADDR2,(uint16_t*)DATA_CACHE2,1024);
      for (Ene_i=512;Ene_i<1024;Ene_i++)
      {
        FloatBuf[0] = DATA_CACHE2[3+(Ene_i-512)*4];
        FloatBuf[1] = DATA_CACHE2[2+(Ene_i-512)*4];
        FloatBuf[2] = DATA_CACHE2[1+(Ene_i-512)*4];
        FloatBuf[3] = DATA_CACHE2[(Ene_i-512)*4];
        Energy_Coefficient[Ene_i*2] = *((float*)&FloatBuf[0]);
        // printf("%.6f %d\r\n",Energy_Coefficient[Ene_i*2],Ene_i*2);
      }
    return 0;
  }
  else if (DATA_CACHE1[3] == 0x02)
  {
    STM32FLASH_Write(STM32_FLASH_ENSAVEADDR3,&writedata[2],1024);
    STM32FLASH_Read(STM32_FLASH_ENSAVEADDR3,(uint16_t*)DATA_CACHE2,1024);
      for (Ene_i=0;Ene_i<512;Ene_i++)
      {
        FloatBuf[0] = DATA_CACHE2[3+Ene_i*4];
        FloatBuf[1] = DATA_CACHE2[2+Ene_i*4];
        FloatBuf[2] = DATA_CACHE2[1+Ene_i*4];
        FloatBuf[3] = DATA_CACHE2[Ene_i*4];
        Energy_Coefficient[Ene_i*2+1] = *((float*)&FloatBuf[0]);
        // printf("%.6f %d\r\n",Energy_Coefficient[Ene_i*2+1],Ene_i*2+1);
      }
    STM32FLASH_Write(STM32_FLASH_ENSAVEADDR4,&writedata[2+1024],1024);
    STM32FLASH_Read(STM32_FLASH_ENSAVEADDR4,(uint16_t*)DATA_CACHE2,1024);
      for (Ene_i=512;Ene_i<1024;Ene_i++)
      {
        FloatBuf[0] = DATA_CACHE2[3+(Ene_i-512)*4];
        FloatBuf[1] = DATA_CACHE2[2+(Ene_i-512)*4];
        FloatBuf[2] = DATA_CACHE2[1+(Ene_i-512)*4];
        FloatBuf[3] = DATA_CACHE2[(Ene_i-512)*4];
        Energy_Coefficient[Ene_i*2+1] = *((float*)&FloatBuf[0]);
        // printf("%.6f %d\r\n",Energy_Coefficient[Ene_i*2+1],Ene_i*2+1);
      }
    return 0;
  }
  else
    return 1;
}

/**
 * @brief To judge the spec status, if the spec is incorrect, send message to the master PC
 * 
 */
void Judge_SpecStatus(void)
{
  A = 20 ; 
  while (A!=1)
  {
    if (memcmp(DATA_CACHE2,USART2_Spec_OK,sizeof(USART2_Spec_OK)) == RESET)
    {
      HAL_UART_Transmit(&huart1,USART1_Tsimen_SpecOK,sizeof(USART1_Tsimen_SpecOK),0xFFFF);
      USART_RX2_LENDEMO = 0;
      memset(DATA_CACHE2,0,10);
      break;
    }
    A-- ;
  }

  if (A == 1)
  {
    HAL_UART_Transmit(&huart1,USART1_Tsimen_SpecERROR,sizeof(USART1_Tsimen_SpecERROR),0xFFFF);
  }
  A = 0 ; 
}



/**
 * @brief Read Everything
 * 
 * @param xenon1 
 * @param xenon2 
 * @param xenon3 
 * @param dark 
 * @param ref 
 * @param sam 
 */
void UnderCoefficient(uint8_t *xenon1,uint8_t *xenon2, uint8_t *xenon3,int dark,int ref,int sam)
{
	// ----- Under Dark Signal -----
	HAL_UART_Transmit(&huart2,xenon1,sizeof(xenon1),0xFFFF);
	HAL_Delay(30);
  if (memcmp(DATA_CACHE2,USART2_Spec_OK,sizeof(USART2_Spec_OK)) == RESET)
  {
    PWM_PulseWidth(dark);
    memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
    memset(DATA_CACHE2,0,RX2BUFFERSIZE);
    HAL_Delay(400);
    HAL_UART_Transmit(&huart2,USART2_Spec_Data,sizeof(USART2_Spec_Data),0xFFFF);
    while (memcmp(DATA_CACHE2,USART2_Spec_FormData,9) != RESET)
    {
      HAL_Delay(20);
      if (memcmp(DATA_CACHE1,USART1_Tsimen_GlobalReset,sizeof(USART1_Tsimen_GlobalReset)) == RESET)
      {
        CRC16 = ModBus_CRC16(DATA_CACHE1,6);
        if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
        {
          CRC16 = 0;
          // HAL_UART_Transmit(&huart1,USART1_Tsimen_OK,sizeof(USART1_Tsimen_OK),0xFFFF);
          // break;
          printf("Will restart the MCU in 1s ...");
          HAL_Delay(1000);
          __set_PRIMASK(1);
          HAL_NVIC_SystemReset();
        }
      }
    }
    HAL_Delay(300);
    for (i=0;i<1024;i++)
    {
      DarkData[i] = ((DATA_CACHE2[i*2+9]<<8) | DATA_CACHE2[i*2+10]);
      // printf("Dark%d:%f\r\n",i,DarkData[i]);
    }
    Send_Float(DarkData,1024);
    // printf("OK\r\n");
    USART_RX2_LENDEMO = 0;
    memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
    memset(DATA_CACHE2,0,RX2BUFFERSIZE);
  }

  // ----- Under Reference Signal -----
	HAL_UART_Transmit(&huart2,xenon2,sizeof(xenon2),0xFFFF);
	HAL_Delay(30);
  if (memcmp(DATA_CACHE2,USART2_Spec_OK,sizeof(USART2_Spec_OK)) == RESET)
  {
    PWM_PulseWidth(ref);
    memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
    memset(DATA_CACHE2,0,RX2BUFFERSIZE);
    HAL_Delay(400);
    HAL_UART_Transmit(&huart2,USART2_Spec_Data,sizeof(USART2_Spec_Data),0xFFFF);
    while (memcmp(DATA_CACHE2,USART2_Spec_FormData,9) != RESET)
    {
      HAL_Delay(20);
      if (memcmp(DATA_CACHE1,USART1_Tsimen_GlobalReset,sizeof(USART1_Tsimen_GlobalReset)) == RESET)
      {
        CRC16 = ModBus_CRC16(DATA_CACHE1,6);
        if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
        {
          CRC16 = 0;
          // HAL_UART_Transmit(&huart1,USART1_Tsimen_OK,sizeof(USART1_Tsimen_OK),0xFFFF);
          // break;
          printf("Will restart the MCU in 1s ...");
          HAL_Delay(1000);
          __set_PRIMASK(1);
          HAL_NVIC_SystemReset();
        }
      }
    }
    HAL_Delay(300);
    // To calculate the Reference Signal
    for (i=0;i<1024;i++)
    {
      // RefData[i] = ((DATA_CACHE2[i*2+9]<<8)|DATA_CACHE2[i*2+10])*Energy_Coefficient[i*2] + Energy_Coefficient[i*2+1];
      RefData[i] = (((DATA_CACHE2[i*2+9]<<8)|DATA_CACHE2[i*2+10]) - DarkData[i]) * Energy_Coefficient[i*2] + Energy_Coefficient[i*2+1] ;
      // printf("Ref%d:%f\r\n",i,RefData[i]);
    }
    Send_Float(RefData,1024);
    // printf("OK\r\n");
    USART_RX2_LENDEMO = 0;
    memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
    memset(DATA_CACHE2,0,RX2BUFFERSIZE);
  }

  // ----- Under Sample Signal -----
  HAL_UART_Transmit(&huart2,xenon3,sizeof(xenon3),0xFFFF);
	HAL_Delay(30);
  if (memcmp(DATA_CACHE2,USART2_Spec_OK,sizeof(USART2_Spec_OK)) == RESET)
  {
    PWM_PulseWidth(sam);
    memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
    memset(DATA_CACHE2,0,RX2BUFFERSIZE);
    HAL_Delay(400);
    HAL_UART_Transmit(&huart2,USART2_Spec_Data,sizeof(USART2_Spec_Data),0xFFFF);
    while (memcmp(DATA_CACHE2,USART2_Spec_FormData,9) != RESET)
    {
      HAL_Delay(20);
      if (memcmp(DATA_CACHE1,USART1_Tsimen_GlobalReset,sizeof(USART1_Tsimen_GlobalReset)) == RESET)
      {
        CRC16 = ModBus_CRC16(DATA_CACHE1,6);
        if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
        {
          CRC16 = 0;
          // HAL_UART_Transmit(&huart1,USART1_Tsimen_OK,sizeof(USART1_Tsimen_OK),0xFFFF);
          // break;
          printf("Will restart the MCU in 1s ...");
          HAL_Delay(1000);
          __set_PRIMASK(1);
          HAL_NVIC_SystemReset();
        }
      }
    }
    HAL_Delay(300);
    // HAL_UART_Transmit(&huart1,DATA_CACHE2,2063,0xFFFF);
    for (i=0;i<1024;i++)
    {
      // SamData[i] = ((DATA_CACHE2[i*2+9]<<8) | DATA_CACHE2[i*2+10]);
      SamData[i] = ((DATA_CACHE2[i*2+9]<<8) | DATA_CACHE2[i*2+10]) - DarkData[i];
      // printf("Sam%d:%f\r\n",i,SamData[i]);
      // HAL_Delay(20);
    }
    Send_Float(SamData,1024);
    // printf("OK\r\n");
    USART_RX2_LENDEMO = 0;
    memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
    memset(DATA_CACHE2,0,RX2BUFFERSIZE);
  }

  HAL_Delay(100);

  // ----- calculate ABS -----
  // for (i=0;i<1024;i++)
  // {
  //   if ((RefData[i]-DarkData[i])<0||(SamData[i]-DarkData[i])<0)
  //   {
  //     ABS[i] = 0;
  //     continue;
  //   }
  //   // ABS[i] = log10((RefData[i]-DarkData[i])/SamData[i]-DarkData[i])/5;
  //   ABS[i] = log10((RefData[i]-DarkData[i])/(SamData[i]-DarkData[i]));
  //   if (ABS[i] < 0)
  //   {
  //     ABS[i] = 0;
  //   }
  //   // printf("%d %f\r\n ",i,ABS[i]);
  //   // HAL_Delay(20);
  // }
  // printf("%f",Optical_path);


  // ----- calculate ABS -----
  for (i=0;i<1024;i++)
  {
    if ((RefData[i])<0||(SamData[i])<0)
    {
      ABS[i] = 0;
      continue;
    }
    ABS[i] = log10((RefData[i])/(SamData[i]))/Optical_path;
    // if (ABS[i] < 0)
    // {
    //   ABS[i] = 0;
    // }

    // printf("%d %f\r\n ",i,ABS[i]);
    // HAL_Delay(20);
  }
  Send_Float(ABS,1024);
}


/**
 * @brief Read Everything
 * 
 * @param xenon1 
 * @param xenon2 
 * @param xenon3 
 * @param dark 
 * @param ref 
 * @param sam 
 */
void UnderCoefficient_WithoutLightpath(uint8_t *xenon1,uint8_t *xenon2, uint8_t *xenon3,int dark,int ref,int sam)
{
	// ----- Under Dark Signal -----
	HAL_UART_Transmit(&huart2,xenon1,sizeof(xenon1),0xFFFF);
	HAL_Delay(30);
  if (memcmp(DATA_CACHE2,USART2_Spec_OK,sizeof(USART2_Spec_OK)) == RESET)
  {
    PWM_PulseWidth(dark);
    memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
    memset(DATA_CACHE2,0,RX2BUFFERSIZE);
    HAL_Delay(400);
    HAL_UART_Transmit(&huart2,USART2_Spec_Data,sizeof(USART2_Spec_Data),0xFFFF);
    while (memcmp(DATA_CACHE2,USART2_Spec_FormData,9) != RESET)
    {
      HAL_Delay(20);
      if (memcmp(DATA_CACHE1,USART1_Tsimen_GlobalReset,sizeof(USART1_Tsimen_GlobalReset)) == RESET)
      {
        CRC16 = ModBus_CRC16(DATA_CACHE1,6);
        if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
        {
          CRC16 = 0;
          // HAL_UART_Transmit(&huart1,USART1_Tsimen_OK,sizeof(USART1_Tsimen_OK),0xFFFF);
          // break;
          printf("Will restart the MCU in 1s ...");
          HAL_Delay(1000);
          __set_PRIMASK(1);
          HAL_NVIC_SystemReset();
        }
      }
    }
    HAL_Delay(300);
    for (i=0;i<1024;i++)
    {
      DarkData[i] = ((DATA_CACHE2[i*2+9]<<8) | DATA_CACHE2[i*2+10]);
      // printf("Dark%d:%f\r\n",i,DarkData[i]);
    }
    Send_Float(DarkData,1024);
    // printf("OK\r\n");
    USART_RX2_LENDEMO = 0;
    memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
    memset(DATA_CACHE2,0,RX2BUFFERSIZE);
  }

  // ----- Under Reference Signal -----
	HAL_UART_Transmit(&huart2,xenon2,sizeof(xenon2),0xFFFF);
	HAL_Delay(30);
  if (memcmp(DATA_CACHE2,USART2_Spec_OK,sizeof(USART2_Spec_OK)) == RESET)
  {
    PWM_PulseWidth(ref);
    memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
    memset(DATA_CACHE2,0,RX2BUFFERSIZE);
    HAL_Delay(400);
    HAL_UART_Transmit(&huart2,USART2_Spec_Data,sizeof(USART2_Spec_Data),0xFFFF);
    while (memcmp(DATA_CACHE2,USART2_Spec_FormData,9) != RESET)
    {
      HAL_Delay(20);
      if (memcmp(DATA_CACHE1,USART1_Tsimen_GlobalReset,sizeof(USART1_Tsimen_GlobalReset)) == RESET)
      {
        CRC16 = ModBus_CRC16(DATA_CACHE1,6);
        if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
        {
          CRC16 = 0;
          // HAL_UART_Transmit(&huart1,USART1_Tsimen_OK,sizeof(USART1_Tsimen_OK),0xFFFF);
          // break;
          printf("Will restart the MCU in 1s ...");
          HAL_Delay(1000);
          __set_PRIMASK(1);
          HAL_NVIC_SystemReset();
        }
      }
    }
    HAL_Delay(300);
    // To calculate the Reference Signal
    for (i=0;i<1024;i++)
    {
      // RefData[i] = ((DATA_CACHE2[i*2+9]<<8)|DATA_CACHE2[i*2+10])*Energy_Coefficient[i*2] + Energy_Coefficient[i*2+1];
      RefData[i] = (((DATA_CACHE2[i*2+9]<<8)|DATA_CACHE2[i*2+10]) - DarkData[i]) * Energy_Coefficient[i*2] + Energy_Coefficient[i*2+1] ;
      // printf("Ref%d:%f\r\n",i,RefData[i]);
    }
    Send_Float(RefData,1024);
    // printf("OK\r\n");
    USART_RX2_LENDEMO = 0;
    memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
    memset(DATA_CACHE2,0,RX2BUFFERSIZE);
  }

  // ----- Under Sample Signal -----
  HAL_UART_Transmit(&huart2,xenon3,sizeof(xenon3),0xFFFF);
	HAL_Delay(30);
  if (memcmp(DATA_CACHE2,USART2_Spec_OK,sizeof(USART2_Spec_OK)) == RESET)
  {
    PWM_PulseWidth(sam);
    memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
    memset(DATA_CACHE2,0,RX2BUFFERSIZE);
    HAL_Delay(400);
    HAL_UART_Transmit(&huart2,USART2_Spec_Data,sizeof(USART2_Spec_Data),0xFFFF);
    while (memcmp(DATA_CACHE2,USART2_Spec_FormData,9) != RESET)
    {
      HAL_Delay(20);
      if (memcmp(DATA_CACHE1,USART1_Tsimen_GlobalReset,sizeof(USART1_Tsimen_GlobalReset)) == RESET)
      {
        CRC16 = ModBus_CRC16(DATA_CACHE1,6);
        if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
        {
          CRC16 = 0;
          // HAL_UART_Transmit(&huart1,USART1_Tsimen_OK,sizeof(USART1_Tsimen_OK),0xFFFF);
          // break;
          printf("Will restart the MCU in 1s ...");
          HAL_Delay(1000);
          __set_PRIMASK(1);
          HAL_NVIC_SystemReset();
        }
      }
    }
    HAL_Delay(300);
    // HAL_UART_Transmit(&huart1,DATA_CACHE2,2063,0xFFFF);
    for (i=0;i<1024;i++)
    {
      // SamData[i] = ((DATA_CACHE2[i*2+9]<<8) | DATA_CACHE2[i*2+10]);
      SamData[i] = ((DATA_CACHE2[i*2+9]<<8) | DATA_CACHE2[i*2+10]) - DarkData[i];
      // printf("Sam%d:%f\r\n",i,SamData[i]);
      // HAL_Delay(20);
    }
    Send_Float(SamData,1024);
    // printf("OK\r\n");
    USART_RX2_LENDEMO = 0;
    memset(DATA_CACHE1,0,sizeof(DATA_CACHE1));
    memset(DATA_CACHE2,0,RX2BUFFERSIZE);
  }

  HAL_Delay(100);

  // ----- calculate ABS -----
  // for (i=0;i<1024;i++)
  // {
  //   if ((RefData[i]-DarkData[i])<0||(SamData[i]-DarkData[i])<0)
  //   {
  //     ABS[i] = 0;
  //     continue;
  //   }
  //   // ABS[i] = log10((RefData[i]-DarkData[i])/SamData[i]-DarkData[i])/5;
  //   ABS[i] = log10((RefData[i]-DarkData[i])/(SamData[i]-DarkData[i]));
  //   if (ABS[i] < 0)
  //   {
  //     ABS[i] = 0;
  //   }
  //   // printf("%d %f\r\n ",i,ABS[i]);
  //   // HAL_Delay(20);
  // }
  // printf("%f",Optical_path);


  // ----- calculate ABS -----
  for (i=0;i<1024;i++)
  {
    if ((RefData[i])<0||(SamData[i])<0)
    {
      ABS[i] = 0;
      continue;
    }
    ABS[i] = log10((RefData[i])/(SamData[i]));
    // if (ABS[i] < 0)
    // {
    //   ABS[i] = 0;
    // }

    // printf("%d %f\r\n ",i,ABS[i]);
    // HAL_Delay(20);
  }
  Send_Float(ABS,1024);
}

/**
 * @brief TimerCallbackHandFunction
 * 
 * @param htim 
 */
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//   if(htim == &htim3 )  //Judge if the timer3
//   {
//     HAL_GPIO_TogglePin(GPIOB,LED_Pin);
    
//     // if (memcmp(DATA_CACHE1,USART1_Tsimen_GlobalReset,6) == RESET )
//     // {
//     //   CRC16 = ModBus_CRC16(DATA_CACHE1,6);
//     //   if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
//     //   {
//     //     printf("Will restart the MCU in 1s ...");
//     //     HAL_Delay(1000);
//     //     __set_PRIMASK(1);
//     //     HAL_NVIC_SystemReset();
//     //   }
//     // }
//   }
// }

// /**
//  * @brief Config the wait time
//  * 
//  */
// void Config_WaitTime(void)
// {
//   WAIT_TIME = 0;
//   WAIT_TIME = (USART2_Spec_Int[1]<<24) | (USART2_Spec_Int[2]<<16) | (USART2_Spec_Int[3]<<8) | USART2_Spec_Int[4] ;
//   WAIT_TIME = WAIT_TIME * (uint32_t)((USART2_Spec_Ave[1]<<8) | USART2_Spec_Ave[2]) ;
//   WAIT_TIME = WAIT_TIME / 1000 ;
// }
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

