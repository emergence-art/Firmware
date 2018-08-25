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
#include "ex_motors.h"

#include <math.h>

/* Private defines -----------------------------------------------------------*/

#define MOTOR_CHANNELS_BANK_A  MOTOR_CHANNEL_8B
#define MOTOR_CHANNELS_BANK_B  MOTOR_CHANNEL_8B
#define MOTOR_CHANNELS_BANK_C  MOTOR_CHANNEL_8B
#define MOTOR_CHANNELS_BANK_D  MOTOR_CHANNEL_8B

/* Private variables ---------------------------------------------------------*/

static PIO_HandleTypeDef hpioMotorEnableBankA;
static PIO_HandleTypeDef hpioMotorEnableBankB;
static PIO_HandleTypeDef hpioMotorEnableBankC;
static PIO_HandleTypeDef hpioMotorEnableBankD;

static MOTOR_HandleTypeDef hmotorBankA;
static MOTOR_HandleTypeDef hmotorBankB;
static MOTOR_HandleTypeDef hmotorBankC;
// static MOTOR_HandleTypeDef hmotorBankD;

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

  hpioMotorEnableBankA.Init.Pin      = DRV_nENBL_A_Pin;
  hpioMotorEnableBankA.Init.Port     = DRV_nENBL_A_GPIO_Port;
  hpioMotorEnableBankA.Init.Polarity = PIO_POLARITY_LOW;
  if (PIO_Init(&hpioMotorEnableBankA) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hpioMotorEnableBankB.Init.Pin      = DRV_nENBL_B_Pin;
  hpioMotorEnableBankB.Init.Port     = DRV_nENBL_B_GPIO_Port;
  hpioMotorEnableBankB.Init.Polarity = PIO_POLARITY_LOW;
  if (PIO_Init(&hpioMotorEnableBankB) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hpioMotorEnableBankC.Init.Pin      = DRV_nENBL_C_Pin;
  hpioMotorEnableBankC.Init.Port     = DRV_nENBL_C_GPIO_Port;
  hpioMotorEnableBankC.Init.Polarity = PIO_POLARITY_LOW;
  if (PIO_Init(&hpioMotorEnableBankC) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hpioMotorEnableBankD.Init.Pin      = DRV_nENBL_D_Pin;
  hpioMotorEnableBankD.Init.Port     = DRV_nENBL_D_GPIO_Port;
  hpioMotorEnableBankD.Init.Polarity = PIO_POLARITY_LOW;
  if (PIO_Init(&hpioMotorEnableBankD) != OBJ_OK)
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

  // hmotorBankD.Init.Port    = GPIOG;
  // hmotorBankD.Init.htim    = &htim8;
  // hmotorBankD.Init.hdma    = &hdma_tim8_ch4_trig_com;
  // hmotorBankD.Init.Channel = TIM_CHANNEL_4;
  // if (MOTOR_Init(&hmotorBankD) != OBJ_OK)
  // {
  //   _Error_Handler(__FILE__, __LINE__);
  // }

  /* Configure MOTOR drivers channels */

  ConfigChannel.hpioEnable = &hpioMotorEnableBankA;
  if (MOTOR_Config(&hmotorBankA, &ConfigChannel, MOTOR_CHANNELS_BANK_A) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  ConfigChannel.hpioEnable = &hpioMotorEnableBankB;
  if (MOTOR_Config(&hmotorBankB, &ConfigChannel, MOTOR_CHANNELS_BANK_B) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  ConfigChannel.hpioEnable = &hpioMotorEnableBankC;
  if (MOTOR_Config(&hmotorBankC, &ConfigChannel, MOTOR_CHANNELS_BANK_C) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  // ConfigChannel.hpioEnable = &hpioMotorEnableBankD;
  // if (MOTOR_Config(&hmotorBankD, &ConfigChannel, MOTOR_CHANNELS_BANK_D) != OBJ_OK)
  // {
  //   _Error_Handler(__FILE__, __LINE__);
  // }

  /* Allocate MOTOR drivers memory */

  #define BUFFER_SIZE  8192  // MOTOR_STEPS_BUFFER_DEPTH

  if (MOTOR_Alloc(&hmotorBankA) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  static uint32_t A0[BUFFER_SIZE];
  static uint32_t A1[BUFFER_SIZE];
  hmotorBankA.BufferSize = BUFFER_SIZE;
  hmotorBankA.BufferData0 = A0;
  hmotorBankA.BufferData1 = A1;

  if (MOTOR_Alloc(&hmotorBankB) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  static uint32_t B0[BUFFER_SIZE];
  static uint32_t B1[BUFFER_SIZE];
  hmotorBankB.BufferSize = BUFFER_SIZE;
  hmotorBankB.BufferData0 = B0;
  hmotorBankB.BufferData1 = B1;

  if (MOTOR_Alloc(&hmotorBankC) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  static uint32_t C0[BUFFER_SIZE];
  static uint32_t C1[BUFFER_SIZE];
  hmotorBankC.BufferSize = BUFFER_SIZE;
  hmotorBankC.BufferData0 = C0;
  hmotorBankC.BufferData1 = C1;

  // if (MOTOR_Alloc(&hmotorBankD) != OBJ_OK)
  // {
  //   _Error_Handler(__FILE__, __LINE__);
  // }

  /* Success! */
  printf("EX: MOTOR's successfully initialized!\n");
}

void EX_MOTORS_Setup(void)
{
  /* Local variables */
  HAL_StatusTypeDef hal_status = HAL_OK;
  OBJ_StatusTypeDef obj_status = OBJ_OK;

  /* Set drivers mode - 4 microsteps/step */
  HAL_GPIO_WritePin(DRV_MODE_0_GPIO_Port, DRV_MODE_0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DRV_MODE_1_GPIO_Port, DRV_MODE_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DRV_MODE_2_GPIO_Port, DRV_MODE_2_Pin, GPIO_PIN_RESET);

  /* Set drivers Vref value */
  uint32_t vout = 1024; // 4096 = VREF (12 bits)
  hal_status |= HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  hal_status |= HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, vout);
  // hal_status |= HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);  // Required?

  /* Deassert drivers resets */
  HAL_GPIO_WritePin(DRV_nRESET_A_GPIO_Port, DRV_nRESET_A_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DRV_nRESET_B_GPIO_Port, DRV_nRESET_B_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DRV_nRESET_C_GPIO_Port, DRV_nRESET_C_Pin, GPIO_PIN_SET);
  // HAL_GPIO_WritePin(DRV_nRESET_D_GPIO_Port, DRV_nRESET_D_Pin, GPIO_PIN_SET);

  /* Check status */
  if (hal_status == HAL_OK && obj_status == OBJ_OK)
  {
    printf("EX: MOTOR's set up\n");
  }
  else
  {
    printf("EX: Cannot set up MOTOR's\n");
    HAL_GPIO_WritePin(BRD_LED1_R_GPIO_Port, BRD_LED1_R_Pin, GPIO_PIN_SET);
  }
}

void EX_MOTORS_Enable(void)
{
  /* Local variables */
  OBJ_StatusTypeDef status = OBJ_OK;

  /* Enable motors */
  status |= MOTOR_Enable(&hmotorBankA, MOTOR_CHANNELS_BANK_A);
  status |= MOTOR_Enable(&hmotorBankB, MOTOR_CHANNELS_BANK_B);
  status |= MOTOR_Enable(&hmotorBankC, MOTOR_CHANNELS_BANK_C);
  // status |= MOTOR_Enable(&hmotorBankD, MOTOR_CHANNELS_BANK_D);

  /* Check status */
  if (status == OBJ_OK)
  {
    printf("EX: MOTOR's enabled\n");
  }
  else
  {
    printf("EX: Cannot enable MOTOR's\n");
    HAL_GPIO_WritePin(BRD_LED1_R_GPIO_Port, BRD_LED1_R_Pin, GPIO_PIN_SET);
  }
}

void EX_MOTORS_Disable(void)
{
  /* Local variables */
  OBJ_StatusTypeDef status = OBJ_OK;

  /* Disable motors */
  status |= MOTOR_Disable(&hmotorBankA, MOTOR_CHANNELS_BANK_A);
  status |= MOTOR_Disable(&hmotorBankB, MOTOR_CHANNELS_BANK_B);
  status |= MOTOR_Disable(&hmotorBankC, MOTOR_CHANNELS_BANK_C);
  // status |= MOTOR_Disable(&hmotorBankD, MOTOR_CHANNELS_BANK_D);

  /* Check status */
  if (status == OBJ_OK)
  {
    printf("EX: MOTOR's disabled\n");
  }
  else
  {
    printf("EX: Cannot disable MOTOR's\n");
    HAL_GPIO_WritePin(BRD_LED1_R_GPIO_Port, BRD_LED1_R_Pin, GPIO_PIN_SET);
  }
}

void EX_MOTORS_SetMotion(uint64_t timestamp, int32_t position, int32_t velocity, uint32_t channel)
{
  /* Local variables */
  OBJ_StatusTypeDef status = OBJ_OK;

  /* Set motion */
  if (channel < 8)
  {
    status = MOTOR_SetMotion(&hmotorBankA, timestamp, position, velocity, 1<<channel);
  }
  else if (channel < 16)
  {
    status = MOTOR_SetMotion(&hmotorBankB, timestamp, position, velocity, 1<<(channel-8));
  }
  else if (channel < 24)
  {
    status = MOTOR_SetMotion(&hmotorBankC, timestamp, position, velocity, 1<<(channel-16));
  }
  // else if (channel < 32)
  // {
  //   status = MOTOR_SetMotion(&hmotorBankD, timestamp, position, velocity, 1<<(channel-24));
  // }
  else
  {
    status = OBJ_ERROR;
  }

  /* Check status */
  if (status != OBJ_OK)
  {
    printf("EX: Cannot set MOTOR's position (channel %lu)\n", channel);
    HAL_GPIO_WritePin(BRD_LED1_R_GPIO_Port, BRD_LED1_R_Pin, GPIO_PIN_SET);
  }
}

void EX_MOTORS_RunTestMode(_Bool loop, uint32_t delay)
{
  /* Enable LED1 Green for status check */
  HAL_GPIO_WritePin(BRD_LED1_G_GPIO_Port, BRD_LED1_G_Pin, GPIO_PIN_SET);

  static uint64_t timestamp = (uint64_t)(10.0*M_PI);
  do
  {
    /* Enable LED2 Blue for frame status check */
    HAL_GPIO_WritePin(BRD_LED2_B_GPIO_Port, BRD_LED2_B_Pin, GPIO_PIN_SET);
    /* Set all motors with same motion for all channels */
    int32_t position = 5.0*(1.0+sin(1.0*timestamp/10.0));
    int32_t velocity = 5.0*(1.0+cos(1.0*timestamp/10.0));
    timestamp += 1;
    MOTOR_SetMotion(&hmotorBankA, timestamp, position, velocity, MOTOR_CHANNEL_8B);
    MOTOR_SetMotion(&hmotorBankB, timestamp, position, velocity, MOTOR_CHANNEL_8B);
    MOTOR_SetMotion(&hmotorBankC, timestamp, position, velocity, MOTOR_CHANNEL_8B);
    // MOTOR_SetMotion(&hmotorBankD, timestamp, position, velocity, MOTOR_CHANNEL_8B);
    /* Disable LED2 Blue */
    HAL_GPIO_WritePin(BRD_LED2_B_GPIO_Port, BRD_LED2_B_Pin, GPIO_PIN_RESET);
    /* Wait for the next frame */
    if (delay > 0)
    {
      HAL_Delay(delay);
    }
  }
  while (loop);
}
