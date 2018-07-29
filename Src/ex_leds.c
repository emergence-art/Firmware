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

/* Private constants ---------------------------------------------------------*/

#define EX_LEDS_PER_STRIP   (25)
#define EX_LEDS_BRIGHTNESS  (255)

/* Private variables ---------------------------------------------------------*/

PIO_HandleTypeDef hpioEnableBankA;
PIO_HandleTypeDef hpioEnableBankB;
PIO_HandleTypeDef hpioEnableBankC;
PIO_HandleTypeDef hpioEnableBankD;

LED_HandleTypeDef hledBankAB;
LED_HandleTypeDef hledBankCD;

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
  if (LED_Config(&hledBankAB, &ConfigChannel, LED_CHANNEL_8BL) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  ConfigChannel.hpioEnable = &hpioEnableBankB;
  if (LED_Config(&hledBankAB, &ConfigChannel, LED_CHANNEL_8BH) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  ConfigChannel.hpioEnable = &hpioEnableBankC;
  if (LED_Config(&hledBankCD, &ConfigChannel, LED_CHANNEL_8BL) != OBJ_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  ConfigChannel.hpioEnable = &hpioEnableBankD;
  if (LED_Config(&hledBankCD, &ConfigChannel, LED_CHANNEL_8BH) != OBJ_OK)
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
