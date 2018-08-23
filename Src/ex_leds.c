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
#include "led.h"
#include "ex_leds.h"

/* Private constants ---------------------------------------------------------*/

#define EX_LEDS_PER_STRIP   (25)
#define EX_LEDS_BRIGHTNESS  (255)

/* Private defines -----------------------------------------------------------*/

#define LED_CHANNELS_BANK_A  LED_CHANNEL_8BL
#define LED_CHANNELS_BANK_B  LED_CHANNEL_8BH
#define LED_CHANNELS_BANK_C  LED_CHANNEL_8BL
#define LED_CHANNELS_BANK_D  LED_CHANNEL_8BH

/* Private variables ---------------------------------------------------------*/

static PIO_HandleTypeDef hpioEnableBankA;
static PIO_HandleTypeDef hpioEnableBankB;
static PIO_HandleTypeDef hpioEnableBankC;
static PIO_HandleTypeDef hpioEnableBankD;

static LED_HandleTypeDef hledBankAB;
static LED_HandleTypeDef hledBankCD;

/* From main.c */
extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_tim1_ch1;
extern DMA_HandleTypeDef hdma_tim1_ch3;

/* Exported functions --------------------------------------------------------*/

void EX_LEDS_Init(void)
{
  LED_Channel_ConfigTypeDef ConfigChannel;

  /* Initialize PIO drivers */

  hpioEnableBankA.Init.Pin      = LED_nOE_A_Pin;
  hpioEnableBankA.Init.Port     = LED_nOE_A_GPIO_Port;
  hpioEnableBankA.Init.Polarity = PIO_POLARITY_LOW;
  if (PIO_Init(&hpioEnableBankA) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hpioEnableBankB.Init.Pin      = LED_nOE_B_Pin;
  hpioEnableBankB.Init.Port     = LED_nOE_B_GPIO_Port;
  hpioEnableBankB.Init.Polarity = PIO_POLARITY_LOW;
  if (PIO_Init(&hpioEnableBankB) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hpioEnableBankC.Init.Pin      = LED_nOE_C_Pin;
  hpioEnableBankC.Init.Port     = LED_nOE_C_GPIO_Port;
  hpioEnableBankC.Init.Polarity = PIO_POLARITY_LOW;
  if (PIO_Init(&hpioEnableBankC) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hpioEnableBankD.Init.Pin      = LED_nOE_D_Pin;
  hpioEnableBankD.Init.Port     = LED_nOE_D_GPIO_Port;
  hpioEnableBankD.Init.Polarity = PIO_POLARITY_LOW;
  if (PIO_Init(&hpioEnableBankD) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Initialize LED drivers */

  hledBankAB.Init.Port    = GPIOI;
  hledBankAB.Init.htim    = &htim1;
  hledBankAB.Init.hdma    = &hdma_tim1_ch1;
  hledBankAB.Init.Channel = TIM_CHANNEL_1;
  if (LED_Init(&hledBankAB) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hledBankCD.Init.Port    = GPIOJ;
  hledBankCD.Init.htim    = &htim1;
  hledBankCD.Init.hdma    = &hdma_tim1_ch3;
  hledBankCD.Init.Channel = TIM_CHANNEL_3;
  if (LED_Init(&hledBankCD) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Configure LED drivers channels */

  ConfigChannel.Length     = EX_LEDS_PER_STRIP;
  ConfigChannel.Brightness = EX_LEDS_BRIGHTNESS;

  ConfigChannel.hpioEnable = &hpioEnableBankA;
  if (LED_Config(&hledBankAB, &ConfigChannel, LED_CHANNELS_BANK_A) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  ConfigChannel.hpioEnable = &hpioEnableBankB;
  if (LED_Config(&hledBankAB, &ConfigChannel, LED_CHANNELS_BANK_B) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  ConfigChannel.hpioEnable = &hpioEnableBankC;
  if (LED_Config(&hledBankCD, &ConfigChannel, LED_CHANNELS_BANK_C) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  ConfigChannel.hpioEnable = &hpioEnableBankD;
  if (LED_Config(&hledBankCD, &ConfigChannel, LED_CHANNELS_BANK_D) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Allocate LED drivers memory */

  if (LED_Alloc(&hledBankAB) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (LED_Alloc(&hledBankCD) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Success! */
  printf("EX: LED's successfully initialized!\n");
}

void EX_LEDS_Setup(void)
{
  /* Local variables */
  HAL_StatusTypeDef hal_status = HAL_OK;
  OBJ_StatusTypeDef obj_status = OBJ_OK;

  /* Check status */
  if (hal_status == HAL_OK && obj_status == OBJ_OK)
  {
    printf("EX: LED's set up\n");
  }
  else
  {
    printf("EX: Cannot set up LED's\n");
    HAL_GPIO_WritePin(BRD_LED1_R_GPIO_Port, BRD_LED1_R_Pin, GPIO_PIN_SET);
  }
}

void EX_LEDS_Enable(void)
{
  /* Local variables */
  OBJ_StatusTypeDef status = OBJ_OK;

  /* Enable leds */
  status |= LED_Enable(&hledBankAB, LED_CHANNELS_BANK_A);
  status |= LED_Enable(&hledBankAB, LED_CHANNELS_BANK_B);
  status |= LED_Enable(&hledBankCD, LED_CHANNELS_BANK_C);
  status |= LED_Enable(&hledBankCD, LED_CHANNELS_BANK_D);

  /* Check status */
  if (status == OBJ_OK)
  {
    printf("EX: LED's enabled\n");
  }
  else
  {
    printf("EX: Cannot enable LED's\n");
    HAL_GPIO_WritePin(BRD_LED1_R_GPIO_Port, BRD_LED1_R_Pin, GPIO_PIN_SET);
  }
}

void EX_LEDS_Disable(void)
{
  /* Local variables */
  OBJ_StatusTypeDef status = OBJ_OK;

  /* Disable leds */
  status |= LED_Disable(&hledBankAB, LED_CHANNELS_BANK_A);
  status |= LED_Disable(&hledBankAB, LED_CHANNELS_BANK_B);
  status |= LED_Disable(&hledBankCD, LED_CHANNELS_BANK_C);
  status |= LED_Disable(&hledBankCD, LED_CHANNELS_BANK_D);

  /* Check status */
  if (status == OBJ_OK)
  {
    printf("EX: LED's disabled\n");
  }
  else
  {
    printf("EX: Cannot disable LED's\n");
    HAL_GPIO_WritePin(BRD_LED1_R_GPIO_Port, BRD_LED1_R_Pin, GPIO_PIN_SET);
  }
}

void EX_LEDS_BlackoutPixels(void)
{
  /* Local variables */
  OBJ_StatusTypeDef status = OBJ_OK;

  /* Set pixels to black */
  for (uint32_t i=0; i<EX_LEDS_PER_STRIP; i++)
  {
    status |= LED_SetPixels(&hledBankAB, 0, i, LED_CHANNEL_16B);
    status |= LED_SetPixels(&hledBankCD, 0, i, LED_CHANNEL_16B);
  }

  /* refresh pixels */
  status |= LED_RefreshPixels(&hledBankAB);
  status |= LED_RefreshPixels(&hledBankCD);

  /* Check status */
  if (status != OBJ_OK)
  {
    printf("EX: Cannot blackout LED's pixels\n");
    HAL_GPIO_WritePin(BRD_LED1_R_GPIO_Port, BRD_LED1_R_Pin, GPIO_PIN_SET);
  }
}

void EX_LEDS_SetPixel(uint32_t argb, uint32_t position, uint32_t channel)
{
  /* Local variables */
  OBJ_StatusTypeDef status = OBJ_OK;

  /* Set pixel */
  if (channel < 16)
  {
    status = LED_SetPixels(&hledBankAB, argb, position, 1<<channel);
  }
  else if (channel < 32)
  {
    status = LED_SetPixels(&hledBankCD, argb, position, 1<<(channel-16));
  }
  else
  {
    status = OBJ_ERROR;
  }

  /* Check status */
  if (status != OBJ_OK)
  {
    printf("EX: Cannot set LED's pixel (position %lu, channel %lu)\n", position, channel);
    HAL_GPIO_WritePin(BRD_LED1_R_GPIO_Port, BRD_LED1_R_Pin, GPIO_PIN_SET);
  }
}

void EX_LEDS_RefreshPixels(void)
{
  /* Local variables */
  OBJ_StatusTypeDef status = OBJ_OK;

  /* refresh pixels */
  status |= LED_RefreshPixels(&hledBankAB);
  status |= LED_RefreshPixels(&hledBankCD);

  /* Check status */
  if (status != OBJ_OK)
  {
    printf("EX: Cannot refresh LED's pixels\n");
    HAL_GPIO_WritePin(BRD_LED1_R_GPIO_Port, BRD_LED1_R_Pin, GPIO_PIN_SET);
  }
}

void EX_LEDS_RunTestMode(_Bool loop, uint32_t delay)
{
  static uint32_t pattern[EX_LEDS_PER_STRIP] = {
    0x00000F, 0x00000F, 0x0F0000, 0x00000F, 0x00000F,
    0x00000F, 0x000F00, 0x000F00, 0x000F00, 0x00000F,
    0x0F0000, 0x000F00, 0xFFFFFF, 0x000F00, 0x0F0000,
    0x00000F, 0x000F00, 0x000F00, 0x000F00, 0x00000F,
    0x00000F, 0x00000F, 0x0F0000, 0x00000F, 0x00000F,
  };

  /* Enable LED1 Green for status check */
  HAL_GPIO_WritePin(BRD_LED1_G_GPIO_Port, BRD_LED1_G_Pin, GPIO_PIN_SET);

  /* Main workloop */
  static size_t offset = 0;
  do
  {
    /* Enable LED2 Blue for frame status check */
    HAL_GPIO_WritePin(BRD_LED2_B_GPIO_Port, BRD_LED2_B_Pin, GPIO_PIN_SET);
    /* Set all pixels with same pattern data for all channels */
    for (size_t i=0; i<EX_LEDS_PER_STRIP; i++)
    {
      uint32_t argb = pattern[(i+offset)%EX_LEDS_PER_STRIP];
      LED_SetPixels(&hledBankAB, argb, i, LED_CHANNEL_16B);
      LED_SetPixels(&hledBankCD, argb, i, LED_CHANNEL_16B);
    }
    offset++; // Offset pattern to create a visual update
    /* Refresh all channels */
    LED_RefreshPixels(&hledBankAB);
    LED_RefreshPixels(&hledBankCD);
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
