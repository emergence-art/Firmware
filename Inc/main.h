/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define DRV_DIR_B_1_Pin GPIO_PIN_2
#define DRV_DIR_B_1_GPIO_Port GPIOE
#define DRV_STP_B_1_Pin GPIO_PIN_3
#define DRV_STP_B_1_GPIO_Port GPIOE
#define DRV_DIR_B_2_Pin GPIO_PIN_4
#define DRV_DIR_B_2_GPIO_Port GPIOE
#define DRV_STP_B_2_Pin GPIO_PIN_5
#define DRV_STP_B_2_GPIO_Port GPIOE
#define DRV_DIR_B_3_Pin GPIO_PIN_6
#define DRV_DIR_B_3_GPIO_Port GPIOE
#define LED_SDO_B_0__Pin GPIO_PIN_8
#define LED_SDO_B_0__GPIO_Port GPIOI
#define DRV_nFAULT_D_Pin GPIO_PIN_13
#define DRV_nFAULT_D_GPIO_Port GPIOC
#define DRV_nFAULT_D_EXTI_IRQn EXTI15_10_IRQn
#define LED_SDO_B_1_Pin GPIO_PIN_9
#define LED_SDO_B_1_GPIO_Port GPIOI
#define LED_SDO_B_2_Pin GPIO_PIN_10
#define LED_SDO_B_2_GPIO_Port GPIOI
#define LED_SDO_B_3_Pin GPIO_PIN_11
#define LED_SDO_B_3_GPIO_Port GPIOI
#define DRV_DIR_C_0_Pin GPIO_PIN_0
#define DRV_DIR_C_0_GPIO_Port GPIOF
#define DRV_STP_C_0_Pin GPIO_PIN_1
#define DRV_STP_C_0_GPIO_Port GPIOF
#define DRV_DIR_C_1_Pin GPIO_PIN_2
#define DRV_DIR_C_1_GPIO_Port GPIOF
#define LED_SDO_B_4_Pin GPIO_PIN_12
#define LED_SDO_B_4_GPIO_Port GPIOI
#define LED_SDO_B_5_Pin GPIO_PIN_13
#define LED_SDO_B_5_GPIO_Port GPIOI
#define LED_SDO_B_6_Pin GPIO_PIN_14
#define LED_SDO_B_6_GPIO_Port GPIOI
#define DRV_STP_C_1_Pin GPIO_PIN_3
#define DRV_STP_C_1_GPIO_Port GPIOF
#define DRV_DIR_C_2_Pin GPIO_PIN_4
#define DRV_DIR_C_2_GPIO_Port GPIOF
#define DRV_STP_C_2_Pin GPIO_PIN_5
#define DRV_STP_C_2_GPIO_Port GPIOF
#define DRV_DIR_C_3_Pin GPIO_PIN_6
#define DRV_DIR_C_3_GPIO_Port GPIOF
#define DRV_STP_C_3_Pin GPIO_PIN_7
#define DRV_STP_C_3_GPIO_Port GPIOF
#define DRV_DIR_C_4_Pin GPIO_PIN_8
#define DRV_DIR_C_4_GPIO_Port GPIOF
#define DRV_STP_C_4_Pin GPIO_PIN_9
#define DRV_STP_C_4_GPIO_Port GPIOF
#define DRV_DIR_C_5_Pin GPIO_PIN_10
#define DRV_DIR_C_5_GPIO_Port GPIOF
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOH
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOH
#define BRD_USER_BUTTON_Pin GPIO_PIN_0
#define BRD_USER_BUTTON_GPIO_Port GPIOC
#define BRD_USER_BUTTON_EXTI_IRQn EXTI0_IRQn
#define ETH_MDC_Pin GPIO_PIN_1
#define ETH_MDC_GPIO_Port GPIOC
#define SPI_MISO_Pin GPIO_PIN_2
#define SPI_MISO_GPIO_Port GPIOC
#define SPI_MOSI_Pin GPIO_PIN_3
#define SPI_MOSI_GPIO_Port GPIOC
#define SSYNC_Pin GPIO_PIN_0
#define SSYNC_GPIO_Port GPIOA
#define ETH_REF_CLK_Pin GPIO_PIN_1
#define ETH_REF_CLK_GPIO_Port GPIOA
#define ETH_MDIO_Pin GPIO_PIN_2
#define ETH_MDIO_GPIO_Port GPIOA
#define DRV_MODE_0_Pin GPIO_PIN_2
#define DRV_MODE_0_GPIO_Port GPIOH
#define DRV_MODE_1_Pin GPIO_PIN_3
#define DRV_MODE_1_GPIO_Port GPIOH
#define DRV_MODE_2_Pin GPIO_PIN_4
#define DRV_MODE_2_GPIO_Port GPIOH
#define DRV_DECAY_Pin GPIO_PIN_5
#define DRV_DECAY_GPIO_Port GPIOH
#define MSYNC_Pin GPIO_PIN_3
#define MSYNC_GPIO_Port GPIOA
#define DRV_VREF1_Pin GPIO_PIN_4
#define DRV_VREF1_GPIO_Port GPIOA
#define DRV_VREF2_Pin GPIO_PIN_5
#define DRV_VREF2_GPIO_Port GPIOA
#define ETH_CRS_DV_Pin GPIO_PIN_7
#define ETH_CRS_DV_GPIO_Port GPIOA
#define ETH_RXD0_Pin GPIO_PIN_4
#define ETH_RXD0_GPIO_Port GPIOC
#define ETH_RXD1_Pin GPIO_PIN_5
#define ETH_RXD1_GPIO_Port GPIOC
#define BRD_LED1_G_Pin GPIO_PIN_0
#define BRD_LED1_G_GPIO_Port GPIOB
#define BRD_LED2_R_Pin GPIO_PIN_1
#define BRD_LED2_R_GPIO_Port GPIOB
#define BRD_LED2_G_Pin GPIO_PIN_2
#define BRD_LED2_G_GPIO_Port GPIOB
#define LED_SDO_B_7_Pin GPIO_PIN_15
#define LED_SDO_B_7_GPIO_Port GPIOI
#define LED_SDO_C_0_Pin GPIO_PIN_0
#define LED_SDO_C_0_GPIO_Port GPIOJ
#define LED_SDO_C_1_Pin GPIO_PIN_1
#define LED_SDO_C_1_GPIO_Port GPIOJ
#define LED_SDO_C_2_Pin GPIO_PIN_2
#define LED_SDO_C_2_GPIO_Port GPIOJ
#define LED_SDO_C_3_Pin GPIO_PIN_3
#define LED_SDO_C_3_GPIO_Port GPIOJ
#define LED_SDO_C_4_Pin GPIO_PIN_4
#define LED_SDO_C_4_GPIO_Port GPIOJ
#define DRV_STP_C_5_Pin GPIO_PIN_11
#define DRV_STP_C_5_GPIO_Port GPIOF
#define DRV_DIR_C_6_Pin GPIO_PIN_12
#define DRV_DIR_C_6_GPIO_Port GPIOF
#define DRV_STP_C_6_Pin GPIO_PIN_13
#define DRV_STP_C_6_GPIO_Port GPIOF
#define DRV_DIR_C_7_Pin GPIO_PIN_14
#define DRV_DIR_C_7_GPIO_Port GPIOF
#define DRV_STP_C_7_Pin GPIO_PIN_15
#define DRV_STP_C_7_GPIO_Port GPIOF
#define DRV_DIR_D_0_Pin GPIO_PIN_0
#define DRV_DIR_D_0_GPIO_Port GPIOG
#define DRV_STP_D_0_Pin GPIO_PIN_1
#define DRV_STP_D_0_GPIO_Port GPIOG
#define DRV_STP_B_3_Pin GPIO_PIN_7
#define DRV_STP_B_3_GPIO_Port GPIOE
#define DRV_DIR_B_4_Pin GPIO_PIN_8
#define DRV_DIR_B_4_GPIO_Port GPIOE
#define DRV_STP_B_4_Pin GPIO_PIN_9
#define DRV_STP_B_4_GPIO_Port GPIOE
#define DRV_DIR_B_5_Pin GPIO_PIN_10
#define DRV_DIR_B_5_GPIO_Port GPIOE
#define DRV_STP_B_5_Pin GPIO_PIN_11
#define DRV_STP_B_5_GPIO_Port GPIOE
#define DRV_DIR_B_6_Pin GPIO_PIN_12
#define DRV_DIR_B_6_GPIO_Port GPIOE
#define DRV_STP_B_6_Pin GPIO_PIN_13
#define DRV_STP_B_6_GPIO_Port GPIOE
#define DRV_DIR_B_7_Pin GPIO_PIN_14
#define DRV_DIR_B_7_GPIO_Port GPIOE
#define DRV_STP_B_7_Pin GPIO_PIN_15
#define DRV_STP_B_7_GPIO_Port GPIOE
#define SPI_SCK_Pin GPIO_PIN_10
#define SPI_SCK_GPIO_Port GPIOB
#define ETH_TX_EN_Pin GPIO_PIN_11
#define ETH_TX_EN_GPIO_Port GPIOB
#define LED_SDO_C_5_Pin GPIO_PIN_5
#define LED_SDO_C_5_GPIO_Port GPIOJ
#define DRV_nRESET_A_Pin GPIO_PIN_8
#define DRV_nRESET_A_GPIO_Port GPIOH
#define DRV_nRESET_B_Pin GPIO_PIN_9
#define DRV_nRESET_B_GPIO_Port GPIOH
#define DRV_nRESET_C_Pin GPIO_PIN_10
#define DRV_nRESET_C_GPIO_Port GPIOH
#define DRV_nRESET_D_Pin GPIO_PIN_11
#define DRV_nRESET_D_GPIO_Port GPIOH
#define DRV_nENBL_A_Pin GPIO_PIN_12
#define DRV_nENBL_A_GPIO_Port GPIOH
#define ETH_TXD0_Pin GPIO_PIN_12
#define ETH_TXD0_GPIO_Port GPIOB
#define ETH_TXD1_Pin GPIO_PIN_13
#define ETH_TXD1_GPIO_Port GPIOB
#define BRD_LED1_R_Pin GPIO_PIN_14
#define BRD_LED1_R_GPIO_Port GPIOB
#define BRD_LED2_B_Pin GPIO_PIN_15
#define BRD_LED2_B_GPIO_Port GPIOB
#define DRV_DIR_A_4_Pin GPIO_PIN_8
#define DRV_DIR_A_4_GPIO_Port GPIOD
#define DRV_STP_A_4_Pin GPIO_PIN_9
#define DRV_STP_A_4_GPIO_Port GPIOD
#define DRV_DIR_A_5_Pin GPIO_PIN_10
#define DRV_DIR_A_5_GPIO_Port GPIOD
#define DRV_STP_A_5_Pin GPIO_PIN_11
#define DRV_STP_A_5_GPIO_Port GPIOD
#define DRV_DIR_A_6_Pin GPIO_PIN_12
#define DRV_DIR_A_6_GPIO_Port GPIOD
#define DRV_STP_A_6_Pin GPIO_PIN_13
#define DRV_STP_A_6_GPIO_Port GPIOD
#define DRV_DIR_A_7_Pin GPIO_PIN_14
#define DRV_DIR_A_7_GPIO_Port GPIOD
#define DRV_STP_A_7_Pin GPIO_PIN_15
#define DRV_STP_A_7_GPIO_Port GPIOD
#define LED_SDO_C_6_Pin GPIO_PIN_6
#define LED_SDO_C_6_GPIO_Port GPIOJ
#define LED_SDO_C_7_Pin GPIO_PIN_7
#define LED_SDO_C_7_GPIO_Port GPIOJ
#define LED_SDO_D_0_Pin GPIO_PIN_8
#define LED_SDO_D_0_GPIO_Port GPIOJ
#define LED_SDO_D_1_Pin GPIO_PIN_9
#define LED_SDO_D_1_GPIO_Port GPIOJ
#define LED_SDO_D_2_Pin GPIO_PIN_10
#define LED_SDO_D_2_GPIO_Port GPIOJ
#define LED_SDO_D_3_Pin GPIO_PIN_11
#define LED_SDO_D_3_GPIO_Port GPIOJ
#define BRD_ID_0_Pin GPIO_PIN_0
#define BRD_ID_0_GPIO_Port GPIOK
#define BRD_ID_1_Pin GPIO_PIN_1
#define BRD_ID_1_GPIO_Port GPIOK
#define BRD_ID_2_Pin GPIO_PIN_2
#define BRD_ID_2_GPIO_Port GPIOK
#define DRV_DIR_D_1_Pin GPIO_PIN_2
#define DRV_DIR_D_1_GPIO_Port GPIOG
#define DRV_STP_D_1_Pin GPIO_PIN_3
#define DRV_STP_D_1_GPIO_Port GPIOG
#define DRV_DIR_D_2_Pin GPIO_PIN_4
#define DRV_DIR_D_2_GPIO_Port GPIOG
#define DRV_STP_D_2_Pin GPIO_PIN_5
#define DRV_STP_D_2_GPIO_Port GPIOG
#define DRV_DIR_D_3_Pin GPIO_PIN_6
#define DRV_DIR_D_3_GPIO_Port GPIOG
#define DRV_STP_D_3_Pin GPIO_PIN_7
#define DRV_STP_D_3_GPIO_Port GPIOG
#define DRV_DIR_D_4_Pin GPIO_PIN_8
#define DRV_DIR_D_4_GPIO_Port GPIOG
#define LED_nOE_A_Pin GPIO_PIN_6
#define LED_nOE_A_GPIO_Port GPIOC
#define LED_nOE_B_Pin GPIO_PIN_7
#define LED_nOE_B_GPIO_Port GPIOC
#define LED_nOE_C_Pin GPIO_PIN_8
#define LED_nOE_C_GPIO_Port GPIOC
#define LED_nOE_D_Pin GPIO_PIN_9
#define LED_nOE_D_GPIO_Port GPIOC
#define USB_FS_SOF_Pin GPIO_PIN_8
#define USB_FS_SOF_GPIO_Port GPIOA
#define USB_FS_VBUS_Pin GPIO_PIN_9
#define USB_FS_VBUS_GPIO_Port GPIOA
#define USB_FS_ID_Pin GPIO_PIN_10
#define USB_FS_ID_GPIO_Port GPIOA
#define USB_FS_DN_Pin GPIO_PIN_11
#define USB_FS_DN_GPIO_Port GPIOA
#define USB_FS_DP_Pin GPIO_PIN_12
#define USB_FS_DP_GPIO_Port GPIOA
#define DRV_nENBL_B_Pin GPIO_PIN_13
#define DRV_nENBL_B_GPIO_Port GPIOH
#define DRV_nENBL_C_Pin GPIO_PIN_14
#define DRV_nENBL_C_GPIO_Port GPIOH
#define DRV_nENBL_D_Pin GPIO_PIN_15
#define DRV_nENBL_D_GPIO_Port GPIOH
#define LED_SDO_A_0_Pin GPIO_PIN_0
#define LED_SDO_A_0_GPIO_Port GPIOI
#define LED_SDO_A_1_Pin GPIO_PIN_1
#define LED_SDO_A_1_GPIO_Port GPIOI
#define LED_SDO_A_2_Pin GPIO_PIN_2
#define LED_SDO_A_2_GPIO_Port GPIOI
#define LED_SDO_A_3_Pin GPIO_PIN_3
#define LED_SDO_A_3_GPIO_Port GPIOI
#define DRV_nFAULT_A_Pin GPIO_PIN_10
#define DRV_nFAULT_A_GPIO_Port GPIOC
#define DRV_nFAULT_A_EXTI_IRQn EXTI15_10_IRQn
#define DRV_nFAULT_B_Pin GPIO_PIN_11
#define DRV_nFAULT_B_GPIO_Port GPIOC
#define DRV_nFAULT_B_EXTI_IRQn EXTI15_10_IRQn
#define DRV_nFAULT_C_Pin GPIO_PIN_12
#define DRV_nFAULT_C_GPIO_Port GPIOC
#define DRV_nFAULT_C_EXTI_IRQn EXTI15_10_IRQn
#define DRV_DIR_A_0_Pin GPIO_PIN_0
#define DRV_DIR_A_0_GPIO_Port GPIOD
#define DRV_STP_A_0_Pin GPIO_PIN_1
#define DRV_STP_A_0_GPIO_Port GPIOD
#define DRV_DIR_A_1_Pin GPIO_PIN_2
#define DRV_DIR_A_1_GPIO_Port GPIOD
#define DRV_STP_A_1_Pin GPIO_PIN_3
#define DRV_STP_A_1_GPIO_Port GPIOD
#define DRV_DIR_A_2_Pin GPIO_PIN_4
#define DRV_DIR_A_2_GPIO_Port GPIOD
#define DRV_STP_A_2_Pin GPIO_PIN_5
#define DRV_STP_A_2_GPIO_Port GPIOD
#define DRV_DIR_A_3_Pin GPIO_PIN_6
#define DRV_DIR_A_3_GPIO_Port GPIOD
#define DRV_STP_A_3_Pin GPIO_PIN_7
#define DRV_STP_A_3_GPIO_Port GPIOD
#define LED_SDO_D_4_Pin GPIO_PIN_12
#define LED_SDO_D_4_GPIO_Port GPIOJ
#define LED_SDO_D_5_Pin GPIO_PIN_13
#define LED_SDO_D_5_GPIO_Port GPIOJ
#define LED_SDO_D_6_Pin GPIO_PIN_14
#define LED_SDO_D_6_GPIO_Port GPIOJ
#define LED_SDO_D_7_Pin GPIO_PIN_15
#define LED_SDO_D_7_GPIO_Port GPIOJ
#define DRV_STP_D_4_Pin GPIO_PIN_9
#define DRV_STP_D_4_GPIO_Port GPIOG
#define DRV_DIR_D_5_Pin GPIO_PIN_10
#define DRV_DIR_D_5_GPIO_Port GPIOG
#define DRV_STP_D_5_Pin GPIO_PIN_11
#define DRV_STP_D_5_GPIO_Port GPIOG
#define DRV_DIR_D_6_Pin GPIO_PIN_12
#define DRV_DIR_D_6_GPIO_Port GPIOG
#define DRV_STP_D_6_Pin GPIO_PIN_13
#define DRV_STP_D_6_GPIO_Port GPIOG
#define DRV_DIR_D_7_Pin GPIO_PIN_14
#define DRV_DIR_D_7_GPIO_Port GPIOG
#define BRD_ID_3_Pin GPIO_PIN_3
#define BRD_ID_3_GPIO_Port GPIOK
#define BRD_ID_4_Pin GPIO_PIN_4
#define BRD_ID_4_GPIO_Port GPIOK
#define BRD_ID_5_Pin GPIO_PIN_5
#define BRD_ID_5_GPIO_Port GPIOK
#define BRD_ID_6_Pin GPIO_PIN_6
#define BRD_ID_6_GPIO_Port GPIOK
#define BRD_ID_7_Pin GPIO_PIN_7
#define BRD_ID_7_GPIO_Port GPIOK
#define DRV_STP_D_7_Pin GPIO_PIN_15
#define DRV_STP_D_7_GPIO_Port GPIOG
#define SPI_NSS_Pin GPIO_PIN_4
#define SPI_NSS_GPIO_Port GPIOB
#define UART_RX_Pin GPIO_PIN_5
#define UART_RX_GPIO_Port GPIOB
#define UART_TX_Pin GPIO_PIN_6
#define UART_TX_GPIO_Port GPIOB
#define BRD_LED1_B_Pin GPIO_PIN_7
#define BRD_LED1_B_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_8
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_9
#define I2C_SDA_GPIO_Port GPIOB
#define DRV_DIR_B_0_Pin GPIO_PIN_0
#define DRV_DIR_B_0_GPIO_Port GPIOE
#define DRV_STP_B_0_Pin GPIO_PIN_1
#define DRV_STP_B_0_GPIO_Port GPIOE
#define LED_SDO_A_4_Pin GPIO_PIN_4
#define LED_SDO_A_4_GPIO_Port GPIOI
#define LED_SDO_A_5_Pin GPIO_PIN_5
#define LED_SDO_A_5_GPIO_Port GPIOI
#define LED_SDO_A_6_Pin GPIO_PIN_6
#define LED_SDO_A_6_GPIO_Port GPIOI
#define LED_SDO_A_7_Pin GPIO_PIN_7
#define LED_SDO_A_7_GPIO_Port GPIOI

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
 #define USE_FULL_ASSERT    1U 

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
