/*******************************************************************************
 *
 *   EMERGENCE PROJECT (http://emergencejourney.org)
 *   Copyright (C) 2018 - David Marchaland (david.marchaland@gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "obj.h"
#include "pio.h"
#include "motor.h"

#include <math.h>

/* Private variables ---------------------------------------------------------*/

PIO_HandleTypeDef hpioEnableBankA;
PIO_HandleTypeDef hpioEnableBankB;
PIO_HandleTypeDef hpioEnableBankC;
PIO_HandleTypeDef hpioEnableBankD;

MOTOR_HandleTypeDef hmotorBankA;
MOTOR_HandleTypeDef hmotorBankB;
MOTOR_HandleTypeDef hmotorBankC;
MOTOR_HandleTypeDef hmotorBankD;

/* From main.c */
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_tim8_ch1;
extern DMA_HandleTypeDef hdma_tim8_ch2;
extern DMA_HandleTypeDef hdma_tim8_ch3;
extern DMA_HandleTypeDef hdma_tim8_ch4_trig_com;

/* Exported functions --------------------------------------------------------*/

void EX_MOTORS_Init(void)
{
  MOTOR_Channel_ConfigTypeDef ConfigChannel;

  /* Initialize PIO drivers */

  hpioEnableBankA.Init.Pin      = DRV_nENBL_A_Pin;
  hpioEnableBankA.Init.Port     = DRV_nENBL_A_GPIO_Port;
  hpioEnableBankA.Init.Polarity = PIO_POLARITY_LOW;
  if (PIO_Init(&hpioEnableBankA) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hpioEnableBankB.Init.Pin      = DRV_nENBL_B_Pin;
  hpioEnableBankB.Init.Port     = DRV_nENBL_B_GPIO_Port;
  hpioEnableBankB.Init.Polarity = PIO_POLARITY_LOW;
  if (PIO_Init(&hpioEnableBankB) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hpioEnableBankC.Init.Pin      = DRV_nENBL_C_Pin;
  hpioEnableBankC.Init.Port     = DRV_nENBL_C_GPIO_Port;
  hpioEnableBankC.Init.Polarity = PIO_POLARITY_LOW;
  if (PIO_Init(&hpioEnableBankC) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hpioEnableBankD.Init.Pin      = DRV_nENBL_D_Pin;
  hpioEnableBankD.Init.Port     = DRV_nENBL_D_GPIO_Port;
  hpioEnableBankD.Init.Polarity = PIO_POLARITY_LOW;
  if (PIO_Init(&hpioEnableBankD) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Initialize MOTOR drivers */

  hmotorBankA.Init.Port    = GPIOD;
  hmotorBankA.Init.htim    = &htim8;
  hmotorBankA.Init.hdma    = &hdma_tim8_ch1;
  hmotorBankA.Init.Channel = TIM_CHANNEL_1;
  if (MOTOR_Init(&hmotorBankA) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hmotorBankB.Init.Port    = GPIOE;
  hmotorBankB.Init.htim    = &htim8;
  hmotorBankB.Init.hdma    = &hdma_tim8_ch2;
  hmotorBankB.Init.Channel = TIM_CHANNEL_2;
  if (MOTOR_Init(&hmotorBankB) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hmotorBankC.Init.Port    = GPIOF;
  hmotorBankC.Init.htim    = &htim8;
  hmotorBankC.Init.hdma    = &hdma_tim8_ch3;
  hmotorBankC.Init.Channel = TIM_CHANNEL_3;
  if (MOTOR_Init(&hmotorBankC) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hmotorBankD.Init.Port    = GPIOG;
  hmotorBankD.Init.htim    = &htim8;
  hmotorBankD.Init.hdma    = &hdma_tim8_ch4_trig_com;
  hmotorBankD.Init.Channel = TIM_CHANNEL_4;
  if (MOTOR_Init(&hmotorBankD) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Configure MOTOR drivers channels */

  ConfigChannel.hpioEnable = &hpioEnableBankA;
  if (MOTOR_Config(&hmotorBankA, &ConfigChannel, MOTOR_CHANNEL_8B) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  ConfigChannel.hpioEnable = &hpioEnableBankB;
  if (MOTOR_Config(&hmotorBankB, &ConfigChannel, MOTOR_CHANNEL_8B) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  ConfigChannel.hpioEnable = &hpioEnableBankC;
  if (MOTOR_Config(&hmotorBankC, &ConfigChannel, MOTOR_CHANNEL_8B) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  ConfigChannel.hpioEnable = &hpioEnableBankD;
  if (MOTOR_Config(&hmotorBankD, &ConfigChannel, MOTOR_CHANNEL_8B) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Allocate MOTOR drivers memory */

  if (MOTOR_Alloc(&hmotorBankA) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (MOTOR_Alloc(&hmotorBankB) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (MOTOR_Alloc(&hmotorBankC) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (MOTOR_Alloc(&hmotorBankD) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Success! */
  printf("EX: MOTOR's successfully initialized!\n");

#ifdef USE_MOTOR_PATTERN

  /* Enable LED1 Green for status check */
  HAL_GPIO_WritePin(BRD_LED1_G_GPIO_Port, BRD_LED1_G_Pin, GPIO_PIN_SET);

  /* Set drivers mode - 4 microsteps/step */
  HAL_GPIO_WritePin(DRV_MODE_0_GPIO_Port, DRV_MODE_0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DRV_MODE_1_GPIO_Port, DRV_MODE_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DRV_MODE_2_GPIO_Port, DRV_MODE_2_Pin, GPIO_PIN_RESET);

  /* Set drivers Vref value */
  uint32_t vout = 1024; // 4096 = VREF (12 bits)
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, vout);
  // HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);  // Required?

  /* Deassert drivers resets */
  HAL_GPIO_WritePin(DRV_nRESET_A_GPIO_Port, DRV_nRESET_A_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DRV_nRESET_B_GPIO_Port, DRV_nRESET_B_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DRV_nRESET_C_GPIO_Port, DRV_nRESET_C_Pin, GPIO_PIN_SET);

  /* Enable all channels */
  MOTOR_Enable(&hmotorBankA, MOTOR_CHANNEL_8B);
  MOTOR_Enable(&hmotorBankB, MOTOR_CHANNEL_8B);
  MOTOR_Enable(&hmotorBankC, MOTOR_CHANNEL_8B);

  motion_t motion = {.timestamp=0.0, .position=0.0, .velocity=0.0, .acceleration=0.0};
  while (1)
  {
    /* Enable LED2 Blue for frame status check */
    HAL_GPIO_WritePin(BRD_LED2_B_GPIO_Port, BRD_LED2_B_Pin, GPIO_PIN_SET);
    /* Set all motors with same motion for all channels */
    motion.timestamp += 0.010;
    motion.velocity = 10.0*sin(motion.timestamp);
    MOTOR_SetMotion(&hmotorBankA, motion, MOTOR_CHANNEL_8B);
    MOTOR_SetMotion(&hmotorBankB, motion, MOTOR_CHANNEL_8B);
    MOTOR_SetMotion(&hmotorBankC, motion, MOTOR_CHANNEL_8B);
    /* Disable LED2 Blue */
    HAL_GPIO_WritePin(BRD_LED2_B_GPIO_Port, BRD_LED2_B_Pin, GPIO_PIN_RESET);
    /* Wait 10ms for the next frame */
    HAL_Delay(10);
  }

#endif /* USE_MOTOR_PATTERN */

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BRD_USER_BUTTON_Pin)
  {
    /* Disable all channels */
    MOTOR_Disable(&hmotorBankA, MOTOR_CHANNEL_8B);
    MOTOR_Disable(&hmotorBankB, MOTOR_CHANNEL_8B);
    MOTOR_Disable(&hmotorBankC, MOTOR_CHANNEL_8B);
  }
}
