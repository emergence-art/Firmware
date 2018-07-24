
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "lwip.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include <errno.h>
#include <sys/unistd.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART5_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int _read(int file, char *data, int len)
{
  if (file != STDIN_FILENO) {
    errno = EBADF;
    return -1;
  }

  /* Receive data through UART */
  HAL_StatusTypeDef status;
  status = HAL_UART_Receive(&huart5, (uint8_t*)data, len, 1000);

  /* Return # of bytes read - as best we can tell */
  return ( status == HAL_OK ? len : 0 );
}

int _write(int file, char *data, int len)
{
  if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
    errno = EBADF;
    return -1;
  }

  /* Transmit data through UART */
  HAL_StatusTypeDef status;
  status = HAL_UART_Transmit(&huart5, (uint8_t*)data, len, 1000);

  /* Return # of bytes written - as best we can tell */
  return ( status == HAL_OK ? len : 0 );
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache-------------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache-------------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_UART5_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_DAC_Init();
  MX_LWIP_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Read a received packet and send it to the lwIP stack for handling */
    MX_LWIP_Process();
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 96;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_HSI;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT2 config 
    */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ETRF;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim5);

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DRV_DIR_B_1_Pin|DRV_STP_B_1_Pin|DRV_DIR_B_2_Pin|DRV_STP_B_2_Pin 
                          |DRV_DIR_B_3_Pin|DRV_STP_B_3_Pin|DRV_DIR_B_4_Pin|DRV_STP_B_4_Pin 
                          |DRV_DIR_B_5_Pin|DRV_STP_B_5_Pin|DRV_DIR_B_6_Pin|DRV_STP_B_6_Pin 
                          |DRV_DIR_B_7_Pin|DRV_STP_B_7_Pin|DRV_DIR_B_0_Pin|DRV_STP_B_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, LED_SDO_B_0__Pin|LED_SDO_B_1_Pin|LED_SDO_B_2_Pin|LED_SDO_B_3_Pin 
                          |LED_SDO_B_4_Pin|LED_SDO_B_5_Pin|LED_SDO_B_6_Pin|LED_SDO_B_7_Pin 
                          |LED_SDO_A_0_Pin|LED_SDO_A_1_Pin|LED_SDO_A_2_Pin|LED_SDO_A_3_Pin 
                          |LED_SDO_A_4_Pin|LED_SDO_A_5_Pin|LED_SDO_A_6_Pin|LED_SDO_A_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, DRV_DIR_C_0_Pin|DRV_STP_C_0_Pin|DRV_DIR_C_1_Pin|DRV_STP_C_1_Pin 
                          |DRV_DIR_C_2_Pin|DRV_STP_C_2_Pin|DRV_DIR_C_3_Pin|DRV_STP_C_3_Pin 
                          |DRV_DIR_C_4_Pin|DRV_STP_C_4_Pin|DRV_DIR_C_5_Pin|DRV_STP_C_5_Pin 
                          |DRV_DIR_C_6_Pin|DRV_STP_C_6_Pin|DRV_DIR_C_7_Pin|DRV_STP_C_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, DRV_MODE_0_Pin|DRV_MODE_1_Pin|DRV_MODE_2_Pin|DRV_nENBL_A_Pin 
                          |DRV_nENBL_B_Pin|DRV_nENBL_C_Pin|DRV_nENBL_D_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, DRV_DECAY_Pin|DRV_nRESET_A_Pin|DRV_nRESET_B_Pin|DRV_nRESET_C_Pin 
                          |DRV_nRESET_D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BRD_LED1_G_Pin|BRD_LED2_R_Pin|BRD_LED2_G_Pin|BRD_LED1_R_Pin 
                          |BRD_LED2_B_Pin|BRD_LED1_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOJ, LED_SDO_C_0_Pin|LED_SDO_C_1_Pin|LED_SDO_C_2_Pin|LED_SDO_C_3_Pin 
                          |LED_SDO_C_4_Pin|LED_SDO_C_5_Pin|LED_SDO_C_6_Pin|LED_SDO_C_7_Pin 
                          |LED_SDO_D_0_Pin|LED_SDO_D_1_Pin|LED_SDO_D_2_Pin|LED_SDO_D_3_Pin 
                          |LED_SDO_D_4_Pin|LED_SDO_D_5_Pin|LED_SDO_D_6_Pin|LED_SDO_D_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, DRV_DIR_D_0_Pin|DRV_STP_D_0_Pin|DRV_DIR_D_1_Pin|DRV_STP_D_1_Pin 
                          |DRV_DIR_D_2_Pin|DRV_STP_D_2_Pin|DRV_DIR_D_3_Pin|DRV_STP_D_3_Pin 
                          |DRV_DIR_D_4_Pin|DRV_STP_D_4_Pin|DRV_DIR_D_5_Pin|DRV_STP_D_5_Pin 
                          |DRV_DIR_D_6_Pin|DRV_STP_D_6_Pin|DRV_DIR_D_7_Pin|DRV_STP_D_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DRV_DIR_A_4_Pin|DRV_STP_A_4_Pin|DRV_DIR_A_5_Pin|DRV_STP_A_5_Pin 
                          |DRV_DIR_A_6_Pin|DRV_STP_A_6_Pin|DRV_DIR_A_7_Pin|DRV_STP_A_7_Pin 
                          |DRV_DIR_A_0_Pin|DRV_STP_A_0_Pin|DRV_DIR_A_1_Pin|DRV_STP_A_1_Pin 
                          |DRV_DIR_A_2_Pin|DRV_STP_A_2_Pin|DRV_DIR_A_3_Pin|DRV_STP_A_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_nOE_A_Pin|LED_nOE_B_Pin|LED_nOE_C_Pin|LED_nOE_D_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DRV_DIR_B_1_Pin DRV_STP_B_1_Pin DRV_DIR_B_2_Pin DRV_STP_B_2_Pin 
                           DRV_DIR_B_3_Pin DRV_STP_B_3_Pin DRV_DIR_B_4_Pin DRV_STP_B_4_Pin 
                           DRV_DIR_B_5_Pin DRV_STP_B_5_Pin DRV_DIR_B_6_Pin DRV_STP_B_6_Pin 
                           DRV_DIR_B_7_Pin DRV_STP_B_7_Pin DRV_DIR_B_0_Pin DRV_STP_B_0_Pin */
  GPIO_InitStruct.Pin = DRV_DIR_B_1_Pin|DRV_STP_B_1_Pin|DRV_DIR_B_2_Pin|DRV_STP_B_2_Pin 
                          |DRV_DIR_B_3_Pin|DRV_STP_B_3_Pin|DRV_DIR_B_4_Pin|DRV_STP_B_4_Pin 
                          |DRV_DIR_B_5_Pin|DRV_STP_B_5_Pin|DRV_DIR_B_6_Pin|DRV_STP_B_6_Pin 
                          |DRV_DIR_B_7_Pin|DRV_STP_B_7_Pin|DRV_DIR_B_0_Pin|DRV_STP_B_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_SDO_B_0__Pin LED_SDO_B_1_Pin LED_SDO_B_2_Pin LED_SDO_B_3_Pin 
                           LED_SDO_B_4_Pin LED_SDO_B_5_Pin LED_SDO_B_6_Pin LED_SDO_B_7_Pin 
                           LED_SDO_A_0_Pin LED_SDO_A_1_Pin LED_SDO_A_2_Pin LED_SDO_A_3_Pin 
                           LED_SDO_A_4_Pin LED_SDO_A_5_Pin LED_SDO_A_6_Pin LED_SDO_A_7_Pin */
  GPIO_InitStruct.Pin = LED_SDO_B_0__Pin|LED_SDO_B_1_Pin|LED_SDO_B_2_Pin|LED_SDO_B_3_Pin 
                          |LED_SDO_B_4_Pin|LED_SDO_B_5_Pin|LED_SDO_B_6_Pin|LED_SDO_B_7_Pin 
                          |LED_SDO_A_0_Pin|LED_SDO_A_1_Pin|LED_SDO_A_2_Pin|LED_SDO_A_3_Pin 
                          |LED_SDO_A_4_Pin|LED_SDO_A_5_Pin|LED_SDO_A_6_Pin|LED_SDO_A_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : DRV_nFAULT_D_Pin DRV_nFAULT_A_Pin DRV_nFAULT_B_Pin DRV_nFAULT_C_Pin */
  GPIO_InitStruct.Pin = DRV_nFAULT_D_Pin|DRV_nFAULT_A_Pin|DRV_nFAULT_B_Pin|DRV_nFAULT_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DRV_DIR_C_0_Pin DRV_STP_C_0_Pin DRV_DIR_C_1_Pin DRV_STP_C_1_Pin 
                           DRV_DIR_C_2_Pin DRV_STP_C_2_Pin DRV_DIR_C_3_Pin DRV_STP_C_3_Pin 
                           DRV_DIR_C_4_Pin DRV_STP_C_4_Pin DRV_DIR_C_5_Pin DRV_STP_C_5_Pin 
                           DRV_DIR_C_6_Pin DRV_STP_C_6_Pin DRV_DIR_C_7_Pin DRV_STP_C_7_Pin */
  GPIO_InitStruct.Pin = DRV_DIR_C_0_Pin|DRV_STP_C_0_Pin|DRV_DIR_C_1_Pin|DRV_STP_C_1_Pin 
                          |DRV_DIR_C_2_Pin|DRV_STP_C_2_Pin|DRV_DIR_C_3_Pin|DRV_STP_C_3_Pin 
                          |DRV_DIR_C_4_Pin|DRV_STP_C_4_Pin|DRV_DIR_C_5_Pin|DRV_STP_C_5_Pin 
                          |DRV_DIR_C_6_Pin|DRV_STP_C_6_Pin|DRV_DIR_C_7_Pin|DRV_STP_C_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : BRD_USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = BRD_USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BRD_USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DRV_MODE_0_Pin DRV_MODE_1_Pin DRV_MODE_2_Pin DRV_nENBL_A_Pin 
                           DRV_nENBL_B_Pin DRV_nENBL_C_Pin DRV_nENBL_D_Pin */
  GPIO_InitStruct.Pin = DRV_MODE_0_Pin|DRV_MODE_1_Pin|DRV_MODE_2_Pin|DRV_nENBL_A_Pin 
                          |DRV_nENBL_B_Pin|DRV_nENBL_C_Pin|DRV_nENBL_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : DRV_DECAY_Pin */
  GPIO_InitStruct.Pin = DRV_DECAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DRV_DECAY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BRD_LED1_G_Pin BRD_LED2_R_Pin BRD_LED2_G_Pin BRD_LED1_R_Pin 
                           BRD_LED2_B_Pin BRD_LED1_B_Pin */
  GPIO_InitStruct.Pin = BRD_LED1_G_Pin|BRD_LED2_R_Pin|BRD_LED2_G_Pin|BRD_LED1_R_Pin 
                          |BRD_LED2_B_Pin|BRD_LED1_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_SDO_C_0_Pin LED_SDO_C_1_Pin LED_SDO_C_2_Pin LED_SDO_C_3_Pin 
                           LED_SDO_C_4_Pin LED_SDO_C_5_Pin LED_SDO_C_6_Pin LED_SDO_C_7_Pin 
                           LED_SDO_D_0_Pin LED_SDO_D_1_Pin LED_SDO_D_2_Pin LED_SDO_D_3_Pin 
                           LED_SDO_D_4_Pin LED_SDO_D_5_Pin LED_SDO_D_6_Pin LED_SDO_D_7_Pin */
  GPIO_InitStruct.Pin = LED_SDO_C_0_Pin|LED_SDO_C_1_Pin|LED_SDO_C_2_Pin|LED_SDO_C_3_Pin 
                          |LED_SDO_C_4_Pin|LED_SDO_C_5_Pin|LED_SDO_C_6_Pin|LED_SDO_C_7_Pin 
                          |LED_SDO_D_0_Pin|LED_SDO_D_1_Pin|LED_SDO_D_2_Pin|LED_SDO_D_3_Pin 
                          |LED_SDO_D_4_Pin|LED_SDO_D_5_Pin|LED_SDO_D_6_Pin|LED_SDO_D_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pins : DRV_DIR_D_0_Pin DRV_STP_D_0_Pin DRV_DIR_D_1_Pin DRV_STP_D_1_Pin 
                           DRV_DIR_D_2_Pin DRV_STP_D_2_Pin DRV_DIR_D_3_Pin DRV_STP_D_3_Pin 
                           DRV_DIR_D_4_Pin DRV_STP_D_4_Pin DRV_DIR_D_5_Pin DRV_STP_D_5_Pin 
                           DRV_DIR_D_6_Pin DRV_STP_D_6_Pin DRV_DIR_D_7_Pin DRV_STP_D_7_Pin */
  GPIO_InitStruct.Pin = DRV_DIR_D_0_Pin|DRV_STP_D_0_Pin|DRV_DIR_D_1_Pin|DRV_STP_D_1_Pin 
                          |DRV_DIR_D_2_Pin|DRV_STP_D_2_Pin|DRV_DIR_D_3_Pin|DRV_STP_D_3_Pin 
                          |DRV_DIR_D_4_Pin|DRV_STP_D_4_Pin|DRV_DIR_D_5_Pin|DRV_STP_D_5_Pin 
                          |DRV_DIR_D_6_Pin|DRV_STP_D_6_Pin|DRV_DIR_D_7_Pin|DRV_STP_D_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : DRV_nRESET_A_Pin DRV_nRESET_B_Pin DRV_nRESET_C_Pin DRV_nRESET_D_Pin */
  GPIO_InitStruct.Pin = DRV_nRESET_A_Pin|DRV_nRESET_B_Pin|DRV_nRESET_C_Pin|DRV_nRESET_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : DRV_DIR_A_4_Pin DRV_STP_A_4_Pin DRV_DIR_A_5_Pin DRV_STP_A_5_Pin 
                           DRV_DIR_A_6_Pin DRV_STP_A_6_Pin DRV_DIR_A_7_Pin DRV_STP_A_7_Pin 
                           DRV_DIR_A_0_Pin DRV_STP_A_0_Pin DRV_DIR_A_1_Pin DRV_STP_A_1_Pin 
                           DRV_DIR_A_2_Pin DRV_STP_A_2_Pin DRV_DIR_A_3_Pin DRV_STP_A_3_Pin */
  GPIO_InitStruct.Pin = DRV_DIR_A_4_Pin|DRV_STP_A_4_Pin|DRV_DIR_A_5_Pin|DRV_STP_A_5_Pin 
                          |DRV_DIR_A_6_Pin|DRV_STP_A_6_Pin|DRV_DIR_A_7_Pin|DRV_STP_A_7_Pin 
                          |DRV_DIR_A_0_Pin|DRV_STP_A_0_Pin|DRV_DIR_A_1_Pin|DRV_STP_A_1_Pin 
                          |DRV_DIR_A_2_Pin|DRV_STP_A_2_Pin|DRV_DIR_A_3_Pin|DRV_STP_A_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : BRD_ID_0_Pin BRD_ID_1_Pin BRD_ID_2_Pin BRD_ID_3_Pin 
                           BRD_ID_4_Pin BRD_ID_5_Pin BRD_ID_6_Pin BRD_ID_7_Pin */
  GPIO_InitStruct.Pin = BRD_ID_0_Pin|BRD_ID_1_Pin|BRD_ID_2_Pin|BRD_ID_3_Pin 
                          |BRD_ID_4_Pin|BRD_ID_5_Pin|BRD_ID_6_Pin|BRD_ID_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_nOE_A_Pin LED_nOE_B_Pin LED_nOE_C_Pin LED_nOE_D_Pin */
  GPIO_InitStruct.Pin = LED_nOE_A_Pin|LED_nOE_B_Pin|LED_nOE_C_Pin|LED_nOE_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
